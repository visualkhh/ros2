#include "http_client.hpp"

#include <arpa/inet.h>
#include <fcntl.h>
#include <netdb.h>
#include <poll.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <unistd.h>

#include <algorithm>
#include <cctype>
#include <cerrno>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <string>
#include <string_view>

namespace krosc_ws {

namespace {

struct Endpoint {
  bool ok = false;
  std::string host;
  std::uint16_t port = 80;
  std::string path;
  std::string error;
};

Endpoint parse_endpoint(const std::string& url) {
  constexpr std::string_view kScheme = "http://";
  Endpoint endpoint;
  if (url.rfind(kScheme, 0) != 0) {
    endpoint.error = "http:// URL만 지원한다: " + url;
    return endpoint;
  }
  std::string rest = url.substr(kScheme.size());
  const std::size_t slash = rest.find('/');
  std::string authority = slash == std::string::npos ? rest : rest.substr(0, slash);
  endpoint.path = slash == std::string::npos ? "/" : rest.substr(slash);
  const std::size_t colon = authority.rfind(':');
  if (colon != std::string::npos) {
    endpoint.host = authority.substr(0, colon);
    const std::string port = authority.substr(colon + 1);
    char* end = nullptr;
    const unsigned long value = std::strtoul(port.c_str(), &end, 10);
    if (end == port.c_str() || value == 0 || value > 65535) {
      endpoint.error = "포트가 잘못됐다: " + port;
      return endpoint;
    }
    endpoint.port = static_cast<std::uint16_t>(value);
  } else {
    endpoint.host = authority;
  }
  if (endpoint.host.empty()) {
    endpoint.error = "host가 비어 있다";
    return endpoint;
  }
  endpoint.ok = true;
  return endpoint;
}

int timeout_ms(double seconds) {
  return static_cast<int>(seconds * 1000.0);
}

void set_timeouts(int fd, double timeout_s) {
  struct timeval timeout {};
  timeout.tv_sec = static_cast<long>(timeout_s);
  timeout.tv_usec = static_cast<long>((timeout_s - static_cast<double>(timeout.tv_sec)) * 1e6);
  setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));
  setsockopt(fd, SOL_SOCKET, SO_SNDTIMEO, &timeout, sizeof(timeout));
}

int connect_socket(const Endpoint& endpoint, double timeout_s, std::string& error) {
  struct addrinfo hints {};
  hints.ai_family = AF_UNSPEC;
  hints.ai_socktype = SOCK_STREAM;
  struct addrinfo* result = nullptr;
  const std::string port = std::to_string(endpoint.port);
  const int lookup = getaddrinfo(endpoint.host.c_str(), port.c_str(), &hints, &result);
  if (lookup != 0) {
    error = std::string("DNS 실패: ") + gai_strerror(lookup);
    return -1;
  }
  int fd = -1;
  for (struct addrinfo* address = result; address != nullptr; address = address->ai_next) {
    fd = socket(address->ai_family, address->ai_socktype, address->ai_protocol);
    if (fd < 0) {
      continue;
    }
#ifdef SO_NOSIGPIPE
    const int enabled = 1;
    setsockopt(fd, SOL_SOCKET, SO_NOSIGPIPE, &enabled, sizeof(enabled));
#endif
    const int flags = fcntl(fd, F_GETFL, 0);
    fcntl(fd, F_SETFL, flags | O_NONBLOCK);

    int result_code = connect(fd, address->ai_addr, address->ai_addrlen);
    if (result_code < 0 && (errno == EINPROGRESS || errno == EWOULDBLOCK)) {
      struct pollfd descriptor {fd, POLLOUT, 0};
      if (poll(&descriptor, 1, timeout_ms(timeout_s)) == 1) {
        int socket_error = 0;
        socklen_t length = sizeof(socket_error);
        if (getsockopt(fd, SOL_SOCKET, SO_ERROR, &socket_error, &length) == 0 &&
            socket_error == 0) {
          result_code = 0;
        }
      }
    }
    if (result_code == 0) {
      fcntl(fd, F_SETFL, flags);
      set_timeouts(fd, timeout_s);
      break;
    }
    close(fd);
    fd = -1;
  }
  freeaddrinfo(result);
  if (fd < 0) {
    error = "연결 실패: " + endpoint.host + ":" + port;
  }
  return fd;
}

bool send_all(int fd, std::string_view data, std::string& error) {
  std::size_t sent = 0;
  while (sent < data.size()) {
    const ssize_t count = send(fd, data.data() + sent, data.size() - sent, 0);
    if (count > 0) {
      sent += static_cast<std::size_t>(count);
      continue;
    }
    if (count < 0 && errno == EINTR) {
      continue;
    }
    error = "send 실패: ";
    error += std::strerror(errno);
    return false;
  }
  return true;
}

bool receive_all(int fd, std::string& out, std::string& error) {
  char buffer[8192];
  while (true) {
    const ssize_t count = recv(fd, buffer, sizeof(buffer), 0);
    if (count > 0) {
      out.append(buffer, static_cast<std::size_t>(count));
      if (out.size() > 16U * 1024U * 1024U) {
        error = "응답이 너무 크다";
        return false;
      }
      continue;
    }
    if (count == 0) {
      return true;
    }
    if (errno == EINTR) {
      continue;
    }
    if (errno == EAGAIN || errno == EWOULDBLOCK) {
      error = "응답 타임아웃";
    } else {
      error = std::string("recv 실패: ") + std::strerror(errno);
    }
    return false;
  }
}

std::string decode_chunked(std::string_view body) {
  std::string out;
  std::size_t position = 0;
  while (position < body.size()) {
    const std::size_t line_end = body.find("\r\n", position);
    if (line_end == std::string_view::npos) {
      break;
    }
    std::string size_text(body.substr(position, line_end - position));
    const std::size_t extension = size_text.find(';');
    if (extension != std::string::npos) {
      size_text.resize(extension);
    }
    char* end = nullptr;
    const unsigned long size = std::strtoul(size_text.c_str(), &end, 16);
    if (end == size_text.c_str() || size == 0) {
      break;
    }
    position = line_end + 2;
    if (position + size > body.size()) {
      break;
    }
    out.append(body.substr(position, static_cast<std::size_t>(size)));
    position += static_cast<std::size_t>(size) + 2;
  }
  return out;
}

int parse_status(std::string_view status_line) {
  const std::size_t space = status_line.find(' ');
  if (space == std::string_view::npos) {
    return 0;
  }
  const std::string_view code = status_line.substr(space + 1, 3);
  return std::atoi(std::string(code).c_str());
}

bool has_header_value(std::string_view headers, std::string_view name,
                      std::string_view value) {
  std::size_t position = 0;
  while (position < headers.size()) {
    const std::size_t line_end = headers.find("\r\n", position);
    std::string_view line =
        line_end == std::string_view::npos ? headers.substr(position)
                                           : headers.substr(position, line_end - position);
    if (line.size() >= name.size() &&
        std::equal(name.begin(), name.end(), line.begin(),
                   [](char left, char right) {
                     return std::tolower(static_cast<unsigned char>(left)) ==
                            std::tolower(static_cast<unsigned char>(right));
                   })) {
      const std::string_view rest = line.substr(name.size());
      if (rest.find(value) != std::string_view::npos) {
        return true;
      }
    }
    if (line_end == std::string_view::npos) {
      break;
    }
    position = line_end + 2;
  }
  return false;
}

}  // namespace

HttpResponse http_post_json(const std::string& url, const std::string& body,
                            double timeout_s) {
  HttpResponse response;
  const Endpoint endpoint = parse_endpoint(url);
  if (!endpoint.ok) {
    response.error = endpoint.error;
    return response;
  }
  const std::string request =
      "POST " + endpoint.path + " HTTP/1.1\r\nHost: " + endpoint.host +
      "\r\nContent-Type: application/json\r\nContent-Length: " +
      std::to_string(body.size()) + "\r\nConnection: close\r\n\r\n" + body;
  std::string error;
  const int fd = connect_socket(endpoint, timeout_s, error);
  if (fd < 0) {
    response.error = std::move(error);
    return response;
  }
  std::string raw;
  const bool sent = send_all(fd, request, error);
  const bool received = sent && receive_all(fd, raw, error);
  close(fd);
  if (!sent || !received) {
    response.error = std::move(error);
    return response;
  }
  const std::size_t header_end = raw.find("\r\n\r\n");
  if (header_end == std::string::npos) {
    response.error = "HTTP 헤더를 찾지 못했다";
    return response;
  }
  const std::size_t status_end = raw.find("\r\n");
  response.status = parse_status(raw.substr(0, status_end));
  const std::string headers = raw.substr(0, header_end);
  std::string payload = raw.substr(header_end + 4);
  if (has_header_value(headers, "Transfer-Encoding", "chunked")) {
    payload = decode_chunked(payload);
  }
  response.body = std::move(payload);
  if (response.status < 200 || response.status >= 300) {
    response.error = "HTTP " + std::to_string(response.status) + ": " +
                     response.body.substr(0, 200);
    return response;
  }
  response.ok = true;
  return response;
}

}  // namespace krosc_ws
