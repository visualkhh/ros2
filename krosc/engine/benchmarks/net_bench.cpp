#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "krosc/context.hpp"
#include "krosc/msgs.hpp"
#include "krosc/net/net_peer.hpp"
#include "krosc/qos.hpp"

namespace {

using Clock = std::chrono::steady_clock;

double micros(Clock::duration duration) {
  return std::chrono::duration<double, std::micro>(duration).count();
}

void wait_connected(krosc::Context& client, int count = 1) {
  const auto deadline = Clock::now() + std::chrono::seconds(3);
  while (Clock::now() < deadline) {
    if (static_cast<int>(client.peer_network()->peer_endpoints().size()) >= count) {
      return;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }
}

std::unique_ptr<krosc::Context> make_listener(const std::string& name) {
  krosc::Context::Options options;
  options.network.name = name;
  options.network.host = "127.0.0.1";
  options.network.port = 0;
  return std::make_unique<krosc::Context>(options);
}

std::unique_ptr<krosc::Context> make_client(const std::string& name, std::uint16_t port) {
  krosc::Context::Options options;
  options.network.name = name;
  options.network.servers = {"127.0.0.1:" + std::to_string(port)};
  options.network.rpc_timeout_sec = 3.0;
  return std::make_unique<krosc::Context>(options);
}

void print_latency(const char* label, std::vector<double> samples) {
  if (samples.empty()) {
    std::printf("%-20s no samples\n", label);
    return;
  }
  std::sort(samples.begin(), samples.end());
  const auto at = [&](double ratio) {
    return samples[std::min(samples.size() - 1, static_cast<std::size_t>(ratio * samples.size()))];
  };
  std::printf("%-20s n=%zu p50=%.1fus p95=%.1fus p99=%.1fus max=%.1fus\n", label,
              samples.size(), at(0.50), at(0.95), at(0.99), samples.back());
}

void bench_latency() {
  auto server_holder = make_listener("bench_latency_server");
  krosc::Context& server = *server_holder;
  const std::uint16_t port = server.peer_network()->port();
  auto client_holder = make_client("bench_latency_client", port);
  krosc::Context& client = *client_holder;
  wait_connected(client);
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  std::mutex mutex;
  std::condition_variable cv;
  std::uint64_t received = 0;
  server.topics().subscribe<krosc::msgs::String>(
      "ping", [&](const krosc::msgs::String& message) {
        server.topics().publish("pong", krosc::msgs::String{message.data}, krosc::QOS_DEFAULT);
      });
  client.topics().subscribe<krosc::msgs::String>(
      "pong", [&](const krosc::msgs::String&) {
        {
          std::lock_guard<std::mutex> lock(mutex);
          ++received;
        }
        cv.notify_all();
      });
  std::this_thread::sleep_for(std::chrono::milliseconds(200));

  std::vector<double> samples;
  samples.reserve(300);
  for (std::uint64_t i = 0; i < 300; ++i) {
    const auto start = Clock::now();
    client.topics().publish("ping", krosc::msgs::String{std::to_string(i)}, krosc::QOS_DEFAULT);
    std::unique_lock<std::mutex> lock(mutex);
    if (!cv.wait_for(lock, std::chrono::seconds(1), [&] { return received == i + 1; })) {
      std::printf("latency timeout at %llu\n", static_cast<unsigned long long>(i));
      break;
    }
    samples.push_back(micros(Clock::now() - start));
  }
  print_latency("topic roundtrip", std::move(samples));
}

// 처리량은 손실 없이 재야 하므로 KEEP_ALL (기본 KEEP_LAST 10이면 몰아 보낸 앞부분이 버려진다).
void bench_throughput() {
  auto server_holder = make_listener("bench_bulk_server");
  krosc::Context& server = *server_holder;
  const std::uint16_t port = server.peer_network()->port();
  auto client_holder = make_client("bench_bulk_client", port);
  krosc::Context& client = *client_holder;
  wait_connected(client);
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  std::atomic<std::uint64_t> received{0};
  const std::string payload(1024, 'x');
  server.topics().subscribe<krosc::msgs::String>(
      "bulk", [&](const krosc::msgs::String&) { received.fetch_add(1); },
      krosc::QOS_KEEP_ALL);

  constexpr std::uint64_t kTotal = 5000;
  constexpr std::uint64_t kBatch = 256;
  std::this_thread::sleep_for(std::chrono::milliseconds(200));
  const auto start = Clock::now();
  for (std::uint64_t sent = 0; sent < kTotal; sent += kBatch) {
    const std::uint64_t end = std::min(kTotal, sent + kBatch);
    for (std::uint64_t i = sent; i < end; ++i) {
      client.topics().publish("bulk", krosc::msgs::String{payload}, krosc::QOS_KEEP_ALL);
    }
    const auto deadline = Clock::now() + std::chrono::seconds(3);
    while (received.load() < end && Clock::now() < deadline) {
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
  }
  const double seconds = std::chrono::duration<double>(Clock::now() - start).count();
  const std::uint64_t got = received.load();
  std::printf("topic bulk 1KiB    sent=%llu recv=%llu (%.1f%%) %.0f msg/s %.1f MiB/s\n",
              static_cast<unsigned long long>(kTotal), static_cast<unsigned long long>(got),
              100.0 * static_cast<double>(got) / static_cast<double>(kTotal),
              static_cast<double>(got) / seconds,
              static_cast<double>(got) * 1024.0 / seconds / (1024.0 * 1024.0));
}

void bench_fanout(int clients_count) {
  auto server_holder = make_listener("bench_fanout_server");
  krosc::Context& server = *server_holder;
  const std::uint16_t port = server.peer_network()->port();
  std::vector<std::unique_ptr<krosc::Context>> clients;
  std::vector<std::atomic<std::uint64_t>> received(static_cast<std::size_t>(clients_count));
  for (int i = 0; i < clients_count; ++i) {
    clients.push_back(make_client("bench_fanout_" + std::to_string(i), port));
    clients.back()->topics().subscribe<krosc::msgs::String>(
        "fanout", [&, i](const krosc::msgs::String&) { received[static_cast<std::size_t>(i)].fetch_add(1); },
        krosc::QOS_KEEP_ALL);
  }
  for (auto& client : clients) {
    wait_connected(*client);
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(150));

  constexpr std::uint64_t kTotal = 2000;
  constexpr std::uint64_t kBatch = 128;
  std::this_thread::sleep_for(std::chrono::milliseconds(200));
  const auto start = Clock::now();
  for (std::uint64_t sent = 0; sent < kTotal; sent += kBatch) {
    const std::uint64_t end = std::min(kTotal, sent + kBatch);
    for (std::uint64_t i = sent; i < end; ++i) {
      server.topics().publish("fanout", krosc::msgs::String{std::to_string(i)},
                              krosc::QOS_KEEP_ALL);
    }
    const auto deadline = Clock::now() + std::chrono::seconds(3);
    for (;;) {
      std::uint64_t minimum = kTotal;
      for (const auto& count : received) {
        minimum = std::min(minimum, count.load());
      }
      if (minimum >= end || Clock::now() >= deadline) {
        break;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
  }
  const double seconds = std::chrono::duration<double>(Clock::now() - start).count();
  std::uint64_t minimum = kTotal;
  for (const auto& count : received) {
    minimum = std::min(minimum, count.load());
  }
  std::uint64_t total = 0;
  for (const auto& count : received) {
    total += count.load();
  }
  std::printf("fanout %d clients   min=%llu/%llu total=%llu  %.0f deliveries/s\n",
              clients_count, static_cast<unsigned long long>(minimum),
              static_cast<unsigned long long>(kTotal), static_cast<unsigned long long>(total),
              static_cast<double>(total) / seconds);
}

void bench_rpc() {
  auto server_holder = make_listener("bench_rpc_server");
  krosc::Context& server = *server_holder;
  const std::uint16_t port = server.peer_network()->port();
  auto client_holder = make_client("bench_rpc_client", port);
  krosc::Context& client = *client_holder;
  wait_connected(client);
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  auto service = server.services().create_service<krosc::msgs::String, krosc::msgs::String>(
      "bench_echo", [](const krosc::msgs::String& request) { return request; });

  std::vector<double> samples;
  samples.reserve(500);
  for (int i = 0; i < 500; ++i) {
    const auto start = Clock::now();
    const auto response = client.services().call<krosc::msgs::String, krosc::msgs::String>(
        "bench_echo", krosc::msgs::String{std::to_string(i)});
    (void)response;
    samples.push_back(micros(Clock::now() - start));
  }
  print_latency("rpc roundtrip", std::move(samples));
  service.destroy();
}

}  // namespace

int main(int argc, char** argv) {
  std::printf("krosc net benchmark (localhost loopback)\n");
  bench_latency();
  bench_throughput();
  int clients = 3;
  if (argc > 1) {
    clients = std::max(1, std::min(32, std::atoi(argv[1])));
  }
  bench_fanout(clients);
  bench_rpc();
  return 0;
}
