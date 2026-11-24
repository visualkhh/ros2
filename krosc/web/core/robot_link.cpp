#include "krosc/web/robot_link.hpp"

#include <arpa/inet.h>
#include <netdb.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cerrno>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <utility>

#ifndef MSG_NOSIGNAL
#define MSG_NOSIGNAL 0
#endif

namespace krosc::web {

namespace {

constexpr std::uint32_t kMaxFrame = 4u * 1024u * 1024u;

bool recv_exact(int fd, std::uint8_t* buffer, std::size_t size) {
  std::size_t offset = 0;
  while (offset < size) {
    const ssize_t received = ::recv(fd, buffer + offset, size - offset, 0);
    if (received == 0) {
      return false;
    }
    if (received < 0) {
      if (errno == EINTR) {
        continue;
      }
      return false;
    }
    offset += static_cast<std::size_t>(received);
  }
  return true;
}

bool send_all(int fd, const std::uint8_t* data, std::size_t size) {
  std::size_t offset = 0;
  while (offset < size) {
    const ssize_t sent = ::send(fd, data + offset, size - offset, MSG_NOSIGNAL);
    if (sent < 0) {
      if (errno == EINTR) {
        continue;
      }
      return false;
    }
    offset += static_cast<std::size_t>(sent);
  }
  return true;
}

int connect_to(const std::string& host, std::uint16_t port) {
  addrinfo hints{};
  hints.ai_family = AF_UNSPEC;
  hints.ai_socktype = SOCK_STREAM;
  addrinfo* result = nullptr;
  const std::string service = std::to_string(port);
  if (::getaddrinfo(host.c_str(), service.c_str(), &hints, &result) != 0) {
    return -1;
  }
  int fd = -1;
  for (addrinfo* entry = result; entry != nullptr; entry = entry->ai_next) {
    fd = ::socket(entry->ai_family, entry->ai_socktype, entry->ai_protocol);
    if (fd < 0) {
      continue;
    }
    if (::connect(fd, entry->ai_addr, entry->ai_addrlen) == 0) {
      break;
    }
    ::close(fd);
    fd = -1;
  }
  ::freeaddrinfo(result);
  if (fd >= 0) {
    int enabled = 1;
    (void)::setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &enabled, sizeof(enabled));
#ifdef SO_NOSIGPIPE
    (void)::setsockopt(fd, SOL_SOCKET, SO_NOSIGPIPE, &enabled, sizeof(enabled));
#endif
    timeval timeout{};
    timeout.tv_sec = 1;
    (void)::setsockopt(fd, SOL_SOCKET, SO_SNDTIMEO, &timeout, sizeof(timeout));
  }
  return fd;
}

}  // namespace

RobotLink::RobotLink(std::string host, std::uint16_t port)
    : host_(std::move(host)), port_(port) {}

RobotLink::~RobotLink() { shutdown(); }

void RobotLink::start() {
  if (thread_.joinable()) {
    return;
  }
  stopping_.store(false);
  thread_ = std::thread([this] { run(); });
}

void RobotLink::shutdown() {
  if (stopping_.exchange(true)) {
    return;
  }
  const int fd = fd_.exchange(-1);
  if (fd >= 0) {
    ::shutdown(fd, SHUT_RDWR);
    ::close(fd);
  }
  if (thread_.joinable()) {
    thread_.join();
  }
}

bool RobotLink::send(const std::vector<std::uint8_t>& body) {
  if (!connected_.load() || body.empty()) {
    return false;
  }
  const int fd = fd_.load();
  if (fd < 0) {
    return false;
  }
  const std::uint32_t length = static_cast<std::uint32_t>(body.size());
  std::uint8_t header[4] = {
      static_cast<std::uint8_t>(length & 0xFFu),
      static_cast<std::uint8_t>((length >> 8) & 0xFFu),
      static_cast<std::uint8_t>((length >> 16) & 0xFFu),
      static_cast<std::uint8_t>((length >> 24) & 0xFFu),
  };
  std::lock_guard<std::mutex> lock(send_mutex_);
  if (fd_.load() != fd) {
    return false;
  }
  return send_all(fd, header, sizeof(header)) && send_all(fd, body.data(), body.size());
}

void RobotLink::run() {
  while (!stopping_.load()) {
    const int fd = connect_to(host_, port_);
    if (fd < 0) {
      std::this_thread::sleep_for(std::chrono::seconds(1));
      continue;
    }
    fd_.store(fd);
    connected_.store(true);
    std::printf("[web] robot connected: %s:%u\n", host_.c_str(), static_cast<unsigned>(port_));
    std::fflush(stdout);
    if (status_handler_) {
      status_handler_(true);
    }

    while (!stopping_.load()) {
      std::uint8_t length_buffer[4] = {};
      if (!recv_exact(fd, length_buffer, sizeof(length_buffer))) {
        break;
      }
      const std::uint32_t length = static_cast<std::uint32_t>(length_buffer[0]) |
                                   (static_cast<std::uint32_t>(length_buffer[1]) << 8) |
                                   (static_cast<std::uint32_t>(length_buffer[2]) << 16) |
                                   (static_cast<std::uint32_t>(length_buffer[3]) << 24);
      if (length == 0 || length > kMaxFrame) {
        break;
      }
      std::vector<std::uint8_t> body(length);
      if (!recv_exact(fd, body.data(), body.size())) {
        break;
      }
      if (frame_handler_) {
        frame_handler_(body);
      }
    }

    connected_.store(false);
    fd_.store(-1);
    ::close(fd);
    std::printf("[web] robot disconnected: %s:%u\n", host_.c_str(), static_cast<unsigned>(port_));
    std::fflush(stdout);
    if (status_handler_) {
      status_handler_(false);
    }
    if (!stopping_.load()) {
      std::this_thread::sleep_for(std::chrono::seconds(1));
    }
  }
}

}  // namespace krosc::web
