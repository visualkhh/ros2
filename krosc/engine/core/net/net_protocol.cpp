#include "krosc/net/net_protocol.hpp"

#include <arpa/inet.h>
#include <netdb.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cerrno>
#include <chrono>
#include <thread>
#include <utility>

#ifndef MSG_NOSIGNAL
#define MSG_NOSIGNAL 0
#endif

namespace krosc::net {

namespace {

bool read_exact(int fd, std::uint8_t* buffer, std::size_t size) {
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


}  // namespace

std::vector<std::uint8_t> encode_body(const Frame& frame) {
  if (frame.topic.size() > 0xFFFFu) {
    return {};
  }
  std::vector<std::uint8_t> body;
  body.reserve(3 + frame.topic.size() + 1 + frame.payload.size());
  body.push_back(frame.type);
  const std::uint16_t topic_size = static_cast<std::uint16_t>(frame.topic.size());
  body.push_back(static_cast<std::uint8_t>(topic_size & 0xFFu));
  body.push_back(static_cast<std::uint8_t>((topic_size >> 8) & 0xFFu));
  body.insert(body.end(), frame.topic.begin(), frame.topic.end());
  body.push_back(static_cast<std::uint8_t>(frame.kind));
  body.insert(body.end(), frame.payload.begin(), frame.payload.end());
  return body;
}

std::optional<Frame> decode_body(const std::vector<std::uint8_t>& body) {
  if (body.size() < 4) {
    return std::nullopt;
  }
  Frame frame;
  std::size_t position = 0;
  frame.type = body[position++];
  const std::uint16_t topic_size =
      static_cast<std::uint16_t>(body[position]) |
      static_cast<std::uint16_t>(static_cast<std::uint16_t>(body[position + 1]) << 8);
  position += 2;
  if (position + topic_size + 1 > body.size()) {
    return std::nullopt;
  }
  frame.topic.assign(reinterpret_cast<const char*>(body.data() + position), topic_size);
  position += topic_size;
  frame.kind = static_cast<Kind>(body[position++]);
  frame.payload.assign(body.begin() + static_cast<std::ptrdiff_t>(position), body.end());
  return frame;
}

std::vector<std::uint8_t> encode_frame(const Frame& frame) {
  const std::vector<std::uint8_t> body = encode_body(frame);
  if (body.empty()) {
    return {};
  }
  std::vector<std::uint8_t> encoded;
  encoded.reserve(4 + body.size());
  const std::uint32_t size = static_cast<std::uint32_t>(body.size());
  encoded.push_back(static_cast<std::uint8_t>(size & 0xFFu));
  encoded.push_back(static_cast<std::uint8_t>((size >> 8) & 0xFFu));
  encoded.push_back(static_cast<std::uint8_t>((size >> 16) & 0xFFu));
  encoded.push_back(static_cast<std::uint8_t>((size >> 24) & 0xFFu));
  encoded.insert(encoded.end(), body.begin(), body.end());
  return encoded;
}

bool read_frame(int fd, Frame& frame) {
  std::uint8_t length_buffer[4] = {};
  if (!read_exact(fd, length_buffer, sizeof(length_buffer))) {
    return false;
  }
  const std::uint32_t length = static_cast<std::uint32_t>(length_buffer[0]) |
                               (static_cast<std::uint32_t>(length_buffer[1]) << 8) |
                               (static_cast<std::uint32_t>(length_buffer[2]) << 16) |
                               (static_cast<std::uint32_t>(length_buffer[3]) << 24);
  if (length == 0 || length > kMaxFrame) {
    return false;
  }
  std::vector<std::uint8_t> body(length);
  if (!read_exact(fd, body.data(), body.size())) {
    return false;
  }
  auto decoded = decode_body(body);
  if (!decoded) {
    return false;
  }
  frame = std::move(*decoded);
  return true;
}

bool write_frame(int fd, const Frame& frame) {
  const std::vector<std::uint8_t> encoded = encode_frame(frame);
  if (encoded.empty()) {
    return false;
  }
  return send_all(fd, encoded.data(), encoded.size());
}

std::optional<std::string> frame_topic(const std::vector<std::uint8_t>& body) {
  const auto frame = decode_body(body);
  return frame ? std::optional<std::string>(frame->topic) : std::nullopt;
}


}  // namespace krosc::net
