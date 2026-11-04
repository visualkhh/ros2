// OnnxPolicy — 학습한 정책(play.py가 내보낸 policy.onnx, 옆에 policy.onnx.data)을 ONNX Runtime으로 한 번에 하나씩 추론.
// 입력 [1, n_in] float · 출력 [1, n_out] float (rsl_rl actor 그대로 — 스케일 · clamp는 쓰는 쪽에서).
#pragma once

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace Ort {struct Env; struct Session;}   // NOLINT — onnxruntime 헤더를 쓰는 쪽에 퍼뜨리지 않는다

namespace rl_policy
{

class OnnxPolicy
{
public:
  OnnxPolicy();
  ~OnnxPolicy();
  OnnxPolicy(OnnxPolicy &&) noexcept;
  OnnxPolicy & operator=(OnnxPolicy &&) noexcept;

  void load(const std::string & path);   // 실패하면 std::runtime_error (이전 모델은 그대로)
  void unload();
  bool loaded() const {return session_ != nullptr;}
  int64_t inputs() const {return n_in_;}
  int64_t outputs() const {return n_out_;}
  const std::string & path() const {return path_;}
  std::vector<float> run(const std::vector<float> & obs) const;   // obs.size() == inputs()

private:
  std::unique_ptr<Ort::Session> session_;
  std::string path_, in_name_, out_name_;
  int64_t n_in_ = 0, n_out_ = 0;
};

}  // namespace rl_policy
