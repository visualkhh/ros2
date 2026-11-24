#include "krosc/web/websocket.hpp"

#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>

#include <array>
#include <cerrno>
#include <cstring>
#include <vector>

#ifndef MSG_NOSIGNAL
#define MSG_NOSIGNAL 0
#endif

namespace krosc::web {

namespace {

constexpr std::size_t kMaxPayload = 4u * 1024u * 1024u;

std::uint32_t rotate_left(std::uint32_t value, int bits) {
  return (value << bits) | (value >> (32 - bits));
}

std::array<std::uint8_t, 20> sha1(const std::uint8_t* data, std::size_t size) {
  std::uint32_t h[5] = {0x67452301u, 0xEFCDAB89u, 0x98BADCFEu, 0x10325476u, 0xC3D2E1F0u};
  std::vector<std::uint8_t> message(data, data + size);
  const std::uint64_t bit_length = static_cast<std::uint64_t>(size) * 8u;
  message.push_back(0x80);
  while ((message.size() % 64) != 56) {
    message.push_back(0);
  }
  for (int index = 7; index >= 0; --index) {
    message.push_back(static_cast<std::uint8_t>((bit_length >> (index * 8)) & 0xFFu));
  }

  for (std::size_t offset = 0; offset < message.size(); offset += 64) {
    std::uint32_t w[80] = {};
    for (int index = 0; index < 16; ++index) {
      const std::size_t base = offset + static_cast<std::size_t>(index) * 4;
      w[index] = (static_cast<std::uint32_t>(message[base]) << 24) |
                 (static_cast<std::uint32_t>(message[base + 1]) << 16) |
                 (static_cast<std::uint32_t>(message[base + 2]) << 8) |
                 static_cast<std::uint32_t>(message[base + 3]);
    }
    for (int index = 16; index < 80; ++index) {
      w[index] = rotate_left(w[index - 3] ^ w[index - 8] ^ w[index - 14] ^ w[index - 16], 1);
    }
    std::uint32_t a = h[0];
    std::uint32_t b = h[1];
    std::uint32_t c = h[2];
    std::uint32_t d = h[3];
    std::uint32_t e = h[4];
    for (int index = 0; index < 80; ++index) {
      std::uint32_t f = 0;
      std::uint32_t k = 0;
      if (index < 20) {
        f = (b & c) | ((~b) & d);
        k = 0x5A827999u;
      } else if (index < 40) {
        f = b ^ c ^ d;
        k = 0x6ED9EBA1u;
      } else if (index < 60) {
        f = (b & c) | (b & d) | (c & d);
        k = 0x8F1BBCDCu;
      } else {
        f = b ^ c ^ d;
        k = 0xCA62C1D6u;
      }
      const std::uint32_t temp = rotate_left(a, 5) + f + e + k + w[index];
      e = d;
      d = c;
      c = rotate_left(b, 30);
      b = a;
      a = temp;
    }
    h[0] += a;
    h[1] += b;
    h[2] += c;
    h[3] += d;
    h[4] += e;
  }

  std::array<std::uint8_t, 20> digest{};
  for (int index = 0; index < 5; ++index) {
    digest[static_cast<std::size_t>(index) * 4] = static_cast<std::uint8_t>((h[index] >> 24) & 0xFFu);
    digest[static_cast<std::size_t>(index) * 4 + 1] = static_cast<std::uint8_t>((h[index] >> 16) & 0xFFu);
    digest[static_cast<std::size_t>(index) * 4 + 2] = static_cast<std::uint8_t>((h[index] >> 8) & 0xFFu);
    digest[static_cast<std::size_t>(index) * 4 + 3] = static_cast<std::uint8_t>(h[index] & 0xFFu);
  }
  return digest;
}

std::string base64_encode(const std::uint8_t* data, std::size_t size) {
  static constexpr char kTable[] =
      "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
  std::string output;
  output.reserve((size + 2) / 3 * 4);
  for (std::size_t index = 0; index < size; index += 3) {
    const std::uint32_t a = data[index];
    const std::uint32_t b = index + 1 < size ? data[index + 1] : 0;
    const std::uint32_t c = index + 2 < size ? data[index + 2] : 0;
    const std::uint32_t value = (a << 16) | (b << 8) | c;
    output.push_back(kTable[(value >> 18) & 0x3F]);
    output.push_back(kTable[(value >> 12) & 0x3F]);
    output.push_back(index + 1 < size ? kTable[(value >> 6) & 0x3F] : '=');
    output.push_back(index + 2 < size ? kTable[value & 0x3F] : '=');
  }
  return output;
}

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

}  // namespace

std::string websocket_accept(const std::string& key) {
  const std::string magic = key + "258EAFA5-E914-47DA-95CA-C5AB0DC85B11";
  const auto digest = sha1(reinterpret_cast<const std::uint8_t*>(magic.data()), magic.size());
  return base64_encode(digest.data(), digest.size());
}

bool websocket_read_frame(int fd, std::uint8_t& opcode, std::vector<std::uint8_t>& payload) {
  std::uint8_t header[2] = {};
  if (!recv_exact(fd, header, sizeof(header))) {
    return false;
  }
  const bool final = (header[0] & 0x80u) != 0;
  opcode = header[0] & 0x0Fu;
  const bool masked = (header[1] & 0x80u) != 0;
  std::uint64_t length = header[1] & 0x7Fu;
  if (!final || !masked) {
    return false;
  }
  if (length == 126) {
    std::uint8_t extended[2] = {};
    if (!recv_exact(fd, extended, sizeof(extended))) {
      return false;
    }
    length = (static_cast<std::uint64_t>(extended[0]) << 8) | extended[1];
  } else if (length == 127) {
    std::uint8_t extended[8] = {};
    if (!recv_exact(fd, extended, sizeof(extended))) {
      return false;
    }
    length = 0;
    for (std::uint8_t byte : extended) {
      length = (length << 8) | byte;
    }
  }
  if (length > kMaxPayload) {
    return false;
  }
  std::uint8_t mask[4] = {};
  if (!recv_exact(fd, mask, sizeof(mask))) {
    return false;
  }
  payload.resize(static_cast<std::size_t>(length));
  if (!payload.empty() && !recv_exact(fd, payload.data(), payload.size())) {
    return false;
  }
  for (std::size_t index = 0; index < payload.size(); ++index) {
    payload[index] ^= mask[index % 4];
  }
  return true;
}

bool websocket_send_frame(int fd, std::uint8_t opcode, const std::uint8_t* data, std::size_t size) {
  std::uint8_t header[10] = {};
  std::size_t header_size = 2;
  header[0] = static_cast<std::uint8_t>(0x80u | (opcode & 0x0Fu));
  if (size < 126) {
    header[1] = static_cast<std::uint8_t>(size);
  } else if (size <= 0xFFFF) {
    header[1] = 126;
    header[2] = static_cast<std::uint8_t>((size >> 8) & 0xFFu);
    header[3] = static_cast<std::uint8_t>(size & 0xFFu);
    header_size = 4;
  } else {
    header[1] = 127;
    for (int index = 0; index < 8; ++index) {
      header[2 + index] = static_cast<std::uint8_t>((static_cast<std::uint64_t>(size) >> ((7 - index) * 8)) & 0xFFu);
    }
    header_size = 10;
  }
  if (!send_all(fd, header, header_size)) {
    return false;
  }
  return size == 0 || send_all(fd, data, size);
}

bool websocket_send_frame(int fd, std::uint8_t opcode, const std::vector<std::uint8_t>& payload) {
  return websocket_send_frame(fd, opcode, payload.data(), payload.size());
}

}  // namespace krosc::web
