#include <fcntl.h>
#include <poll.h>
#include <unistd.h>

#ifdef __APPLE__
#include <util.h>
#else
#include <pty.h>
#endif

#include <chrono>
#include <cstring>
#include <string>
#include <thread>
#include <vector>

#include "hardwares/controls/opencr/opencr_serial.hpp"

#include "test_framework.hpp"

using namespace std::chrono_literals;

namespace {
bool write_all_fd(int fd, const std::string& data) {
  std::size_t offset = 0;
  while (offset < data.size()) {
    const ssize_t sent = ::write(fd, data.data() + offset, data.size() - offset);
    if (sent <= 0) {
      return false;
    }
    offset += static_cast<std::size_t>(sent);
  }
  return true;
}

std::string read_available(int fd, int timeout_ms) {
  std::string result;
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);
  while (std::chrono::steady_clock::now() < deadline) {
    pollfd descriptor{fd, POLLIN, 0};
    const int ready = ::poll(&descriptor, 1, 20);
    if (ready > 0) {
      char buffer[256];
      const ssize_t count = ::read(fd, buffer, sizeof(buffer));
      if (count > 0) {
        result.append(buffer, static_cast<std::size_t>(count));
      }
    }
  }
  return result;
}
}  // namespace

KROSC_TEST(opencr_serial_pty_roundtrip) {
  int master = -1;
  int slave = -1;
  char name[256] = {};
  if (::openpty(&master, &slave, name, nullptr, nullptr) != 0) {
    return;  // PTY를 못 만들면 건너뛴다 (플랫폼 제한).
  }
  const int flags = ::fcntl(master, F_GETFL, 0);
  (void)::fcntl(master, F_SETFL, flags | O_NONBLOCK);
  ::close(slave);

  krosc_ws::OpenCrSerial opencr(0.033, 0.287, name, 115200, 0.05, 0.5, -1.0, -1.0, 7.8);
  opencr.start();

  const auto connected_deadline = std::chrono::steady_clock::now() + 2s;
  while (!opencr.connected() && std::chrono::steady_clock::now() < connected_deadline) {
    std::this_thread::sleep_for(10ms);
  }
  CHECK(opencr.connected());

  CHECK(write_all_fd(master, "S,1.0,0.5,2.0,0.25,1000,0.1,0.2,30,31,12.0\n"));
  CHECK(write_all_fd(master, "I,0.1,0.2,0.3,0.0,0.0,9.81,1.0,0.0,0.0,0.0\n"));
  const auto status_deadline = std::chrono::steady_clock::now() + 2s;
  while (std::chrono::steady_clock::now() < status_deadline) {
    if (opencr.left_position() != 0.0 || opencr.status().has_value()) {
      break;
    }
    std::this_thread::sleep_for(10ms);
  }
  CHECK_NEAR(-1.0, opencr.left_position(), 1e-6);
  CHECK_NEAR(-2.0, opencr.right_position(), 1e-6);
  const auto status = opencr.status();
  CHECK(status.has_value());
  if (status) {
    CHECK_NEAR(12.0, status->battery_voltage, 1e-6);
  }
  const auto imu = opencr.consume_imu();
  CHECK(imu.has_value());
  if (imu) {
    CHECK_NEAR(0.1, imu->angular_velocity.x, 1e-9);
    CHECK_NEAR(1.0, imu->orientation.w, 1e-9);
  }

  opencr.write(krosc::msgs::Twist{{0.1, 0.0, 0.0}, {0.0, 0.0, 0.5}});
  std::string commands;
  for (int i = 0; i < 20 && commands.find("C,") == std::string::npos; ++i) {
    opencr.tick();
    commands += read_available(master, 20);
  }
  CHECK(commands.find("C,") != std::string::npos);

  opencr.stop();
  ::close(master);
}
