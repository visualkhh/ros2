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
// 실행 장치는 CUDA가 있으면 CUDAExecutionProvider, 없으면 CPUExecutionProvider다.
// ONNX Runtime 없이 빌드하면 load()가 실패하고 호출 쪽(nav2 controller)이 fallback한다.
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

  // diagnostics용: 실제로 고른 실행 장치와, 고를 수 있었던 장치들.
  [[nodiscard]] const std::string& provider() const { return provider_; }
  [[nodiscard]] const std::string& provider_error() const { return provider_error_; }

  [[nodiscard]] static bool runtime_available();
  [[nodiscard]] static std::string runtime_version();
  [[nodiscard]] static std::vector<std::string> available_providers();

  [[nodiscard]] std::vector<float> run(const std::vector<float>& observation) const;

 private:
  std::shared_ptr<Ort::Session> session_;
  std::string path_;
  std::string input_name_;
  std::string output_name_;
  std::string provider_;
  std::string provider_error_;
  std::vector<std::string> available_providers_;
  std::int64_t n_in_ = 0;
  std::int64_t n_out_ = 0;
};

}  // namespace krosc_ws::ml
