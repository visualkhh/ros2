#include "serial.hpp"

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include <cerrno>
#include <cstring>

#ifdef __linux__
#include <sys/ioctl.h>
#endif

namespace srf_recycling_ws {

namespace {
speed_t baud_constant(int baud) {
  switch (baud) {
    case 9600:
      return B9600;
    case 57600:
      return B57600;
    case 115200:
      return B115200;
#ifdef B230400
    case 230400:
      return B230400;
#endif
    default:
      return 0;
  }
}
}  // namespace

int open_serial(const std::string& port, int baud) {
  const speed_t speed = baud_constant(baud);
  if (speed == 0) {
    return -1;
  }
  const int fd = ::open(port.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (fd < 0) {
    return -1;
  }
  termios attributes{};
  if (::tcgetattr(fd, &attributes) != 0) {
    ::close(fd);
    return -1;
  }
  ::cfmakeraw(&attributes);
  attributes.c_cflag |= CLOCAL | CREAD;
  attributes.c_cflag &= ~CSTOPB;
  attributes.c_cflag &= ~PARENB;
  attributes.c_cflag &= ~CRTSCTS;
  attributes.c_cflag = (attributes.c_cflag & ~CSIZE) | CS8;
  attributes.c_cc[VMIN] = 0;
  attributes.c_cc[VTIME] = 0;
  ::cfsetispeed(&attributes, speed);
  ::cfsetospeed(&attributes, speed);
  if (::tcsetattr(fd, TCSANOW, &attributes) != 0) {
    ::close(fd);
    return -1;
  }
  ::tcflush(fd, TCIOFLUSH);
  return fd;
}

void close_serial(int fd) {
  if (fd >= 0) {
    ::close(fd);
  }
}

bool write_bytes(int fd, const void* data, std::size_t size) {
  const auto* bytes = static_cast<const std::uint8_t*>(data);
  std::size_t offset = 0;
  while (offset < size) {
    const ssize_t sent = ::write(fd, bytes + offset, size - offset);
    if (sent < 0) {
      if (errno == EINTR) {
        continue;
      }
      return false;
    }
    if (sent == 0) {
      return false;
    }
    offset += static_cast<std::size_t>(sent);
  }
  return true;
}

bool write_line(int fd, const std::string& line) {
  const std::string data = line + "\n";
  return write_bytes(fd, data.data(), data.size());
}

void set_dtr_low(int fd) {
#ifdef __linux__
  int status = TIOCM_DTR;
  (void)::ioctl(fd, TIOCMBIC, &status);
#else
  (void)fd;
#endif
}

bool port_exists(const std::string& path) { return ::access(path.c_str(), R_OK | W_OK) == 0; }

}  // namespace srf_recycling_ws
