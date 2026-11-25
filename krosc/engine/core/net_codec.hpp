#pragma once

// krosc net/RPC 공용 바이너리 리더/라이터 (내부 헤더).
// 프로토콜은 little-endian 고정폭 + u16 길이 문자열이다.

#include <algorithm>
#include <bit>
#include <cstdint>
#include <string>
#include <vector>

namespace krosc::net::detail {

class Writer {
 public:
  std::vector<std::uint8_t> data;

  void u8(std::uint8_t value) { data.push_back(value); }
  void u16(std::uint16_t value) {
    data.push_back(static_cast<std::uint8_t>(value & 0xFF));
    data.push_back(static_cast<std::uint8_t>((value >> 8) & 0xFF));
  }
  void u32(std::uint32_t value) {
    for (int i = 0; i < 4; ++i) {
      data.push_back(static_cast<std::uint8_t>((value >> (8 * i)) & 0xFF));
    }
  }
  void u64(std::uint64_t value) {
    for (int i = 0; i < 8; ++i) {
      data.push_back(static_cast<std::uint8_t>((value >> (8 * i)) & 0xFF));
    }
  }
  void i64(std::int64_t value) { u64(static_cast<std::uint64_t>(value)); }
  void f32(float value) { u32(std::bit_cast<std::uint32_t>(value)); }
  void f64(double value) { u64(std::bit_cast<std::uint64_t>(value)); }
  void str(const std::string& value) {
    const auto length = static_cast<std::uint16_t>(std::min<std::size_t>(value.size(), 65535));
    u16(length);
    data.insert(data.end(), value.begin(), value.begin() + static_cast<std::ptrdiff_t>(length));
  }
  void bytes(const std::uint8_t* pointer, std::size_t count) {
    data.insert(data.end(), pointer, pointer + count);
  }
  void bytes(const std::vector<std::uint8_t>& value) { data.insert(data.end(), value.begin(), value.end()); }
};

class Reader {
 public:
  explicit Reader(const std::vector<std::uint8_t>& data) : pointer_(data.data()), size_(data.size()) {}
  Reader(const std::uint8_t* data, std::size_t size) : pointer_(data), size_(size) {}

  [[nodiscard]] bool ok() const { return ok_; }
  [[nodiscard]] std::size_t remaining() const { return ok_ ? size_ - position_ : 0; }
  [[nodiscard]] const std::uint8_t* rest() const { return pointer_ + position_; }

  std::uint8_t u8() {
    if (!need(1)) {
      return 0;
    }
    return pointer_[position_++];
  }
  std::uint16_t u16() {
    if (!need(2)) {
      return 0;
    }
    const std::uint16_t value = static_cast<std::uint16_t>(pointer_[position_]) |
                                static_cast<std::uint16_t>(pointer_[position_ + 1] << 8);
    position_ += 2;
    return value;
  }
  std::uint32_t u32() {
    if (!need(4)) {
      return 0;
    }
    std::uint32_t value = 0;
    for (int i = 0; i < 4; ++i) {
      value |= static_cast<std::uint32_t>(pointer_[position_ + static_cast<std::size_t>(i)]) << (8 * i);
    }
    position_ += 4;
    return value;
  }
  std::uint64_t u64() {
    if (!need(8)) {
      return 0;
    }
    std::uint64_t value = 0;
    for (int i = 0; i < 8; ++i) {
      value |= static_cast<std::uint64_t>(pointer_[position_ + static_cast<std::size_t>(i)]) << (8 * i);
    }
    position_ += 8;
    return value;
  }
  std::int64_t i64() { return static_cast<std::int64_t>(u64()); }
  float f32() { return std::bit_cast<float>(u32()); }
  double f64() { return std::bit_cast<double>(u64()); }
  std::string str() {
    const std::uint16_t length = u16();
    if (!ok_ || !need(length)) {
      return {};
    }
    std::string value(reinterpret_cast<const char*>(pointer_ + position_), length);
    position_ += length;
    return value;
  }
  void skip(std::size_t count) {
    if (need(count)) {
      position_ += count;
    }
  }

 private:
  bool need(std::size_t count) {
    if (!ok_ || count > size_ - position_) {
      ok_ = false;
      return false;
    }
    return true;
  }

  const std::uint8_t* pointer_;
  std::size_t size_;
  std::size_t position_ = 0;
  bool ok_ = true;
};

}  // namespace krosc::net::detail
