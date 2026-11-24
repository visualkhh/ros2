#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace krosc::web {

// RFC 6455 최소 구현: 텍스트/바이너리/핑/클로즈, 프래그먼트 없음, 마스킹 필수(클라이언트).
std::string websocket_accept(const std::string& key);

bool websocket_read_frame(int fd, std::uint8_t& opcode, std::vector<std::uint8_t>& payload);
bool websocket_send_frame(int fd, std::uint8_t opcode, const std::uint8_t* data, std::size_t size);
bool websocket_send_frame(int fd, std::uint8_t opcode, const std::vector<std::uint8_t>& payload);

}  // namespace krosc::web
