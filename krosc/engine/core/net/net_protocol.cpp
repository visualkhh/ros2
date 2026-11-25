#include "krosc/net/net_protocol.hpp"

#include <arpa/inet.h>
#include <netdb.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <sys/socket.h>
#include <sys/uio.h>
#include <unistd.h>

#include <array>
#include <cerrno>
#include <cstring>
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
  if (frame.topic.size() > 0xFFFFu || frame.payload.size() > kMaxFrame) {
    return false;
  }
  const std::size_t body_size = 3 + frame.topic.size() + 1 + frame.payload.size();
  if (body_size > kMaxFrame) {
    return false;
  }
  std::array<std::uint8_t, 512> stack_header{};
  std::vector<std::uint8_t> heap_header;
  const std::size_t header_size = 4 + 3 + frame.topic.size() + 1;
  std::uint8_t* header = stack_header.data();
  if (header_size > stack_header.size()) {
    heap_header.resize(header_size);
    header = heap_header.data();
  }
  const std::uint32_t length = static_cast<std::uint32_t>(body_size);
  header[0] = static_cast<std::uint8_t>(length & 0xFF);
  header[1] = static_cast<std::uint8_t>((length >> 8) & 0xFF);
  header[2] = static_cast<std::uint8_t>((length >> 16) & 0xFF);
  header[3] = static_cast<std::uint8_t>((length >> 24) & 0xFF);
  header[4] = frame.type;
  const std::uint16_t topic_size = static_cast<std::uint16_t>(frame.topic.size());
  header[5] = static_cast<std::uint8_t>(topic_size & 0xFF);
  header[6] = static_cast<std::uint8_t>((topic_size >> 8) & 0xFF);
  std::memcpy(header + 7, frame.topic.data(), frame.topic.size());
  header[7 + frame.topic.size()] = static_cast<std::uint8_t>(frame.kind);
  iovec vectors[2] = {
      {header, header_size},
      {const_cast<std::uint8_t*>(frame.payload.data()), frame.payload.size()},
  };
  msghdr message{};
  message.msg_iov = vectors;
  message.msg_iovlen = frame.payload.empty() ? 1 : 2;
  while (message.msg_iovlen > 0) {
    const ssize_t sent = ::sendmsg(fd, &message, MSG_NOSIGNAL);
    if (sent < 0) {
      if (errno == EINTR) {
        continue;
      }
      return false;
    }
    if (sent == 0) {
      return false;
    }
    std::size_t remaining = static_cast<std::size_t>(sent);
    while (remaining > 0 && message.msg_iovlen > 0) {
      iovec& current = message.msg_iov[0];
      if (remaining >= current.iov_len) {
        remaining -= current.iov_len;
        ++message.msg_iov;
        --message.msg_iovlen;
      } else {
        current.iov_base = static_cast<std::uint8_t*>(current.iov_base) + remaining;
        current.iov_len -= remaining;
        remaining = 0;
      }
    }
  }
  return true;
}

std::optional<std::string> frame_topic(const std::vector<std::uint8_t>& body) {
  // payload를 디코드/복사하지 않고 topic만 읽는다 (웹 게이트웨이의 고주기 경로).
  if (body.size() < 4) {
    return std::nullopt;
  }
  const std::uint16_t topic_size =
      static_cast<std::uint16_t>(body[1]) |
      static_cast<std::uint16_t>(static_cast<std::uint16_t>(body[2]) << 8);
  if (3u + topic_size > body.size()) {
    return std::nullopt;
  }
  return std::string(reinterpret_cast<const char*>(body.data() + 3), topic_size);
}


}  // namespace krosc::net
