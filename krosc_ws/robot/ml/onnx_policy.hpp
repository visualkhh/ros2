#pragma once

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace Ort {
struct Env;
struct Session;
}  // namespace Ort

namespace krosc_ws::ml {

// 학습한 정책(policy.onnx + policy.onnx.data)을 ONNX Runtime으로 추론한다.
// 입력 [1, n_in] float, 출력 [1, n_out] float. 스케일/clamp는 호출 쪽에서 한다.
class OnnxPolicy {
 public:
  OnnxPolicy();
  ~OnnxPolicy();
  OnnxPolicy(OnnxPolicy&&) noexcept;
  OnnxPolicy& operator=(OnnxPolicy&&) noexcept;

  OnnxPolicy(const OnnxPolicy&) = delete;
  OnnxPolicy& operator=(const OnnxPolicy&) = delete;

  void load(const std::string& path);  // 실패하면 std::runtime_error
  void unload();
  [[nodiscard]] bool loaded() const { return session_ != nullptr; }
  [[nodiscard]] std::int64_t inputs() const { return n_in_; }
  [[nodiscard]] std::int64_t outputs() const { return n_out_; }
  [[nodiscard]] const std::string& path() const { return path_; }

  [[nodiscard]] std::vector<float> run(const std::vector<float>& observation) const;

 private:
  std::unique_ptr<Ort::Session> session_;
  std::string path_;
  std::string input_name_;
  std::string output_name_;
  std::int64_t n_in_ = 0;
  std::int64_t n_out_ = 0;
};

}  // namespace krosc_ws::ml
