#pragma once

#include <cstddef>
#include <cstdint>
#include <string>

namespace srf_recycling_ws {

int open_serial(const std::string& port, int baud);
void close_serial(int fd);
bool write_bytes(int fd, const void* data, std::size_t size);
bool write_line(int fd, const std::string& line);
void set_dtr_low(int fd);
bool port_exists(const std::string& path);

}  // namespace srf_recycling_ws
