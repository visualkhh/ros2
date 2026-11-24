#include "onnx_policy.hpp"

#include <onnxruntime_cxx_api.h>

#include <stdexcept>
#include <utility>

namespace krosc_ws::ml {

namespace {
Ort::Env& environment() {
  static Ort::Env env(ORT_LOGGING_LEVEL_WARNING, "krosc_rl_nav");
  return env;
}
}  // namespace

OnnxPolicy::OnnxPolicy() = default;
OnnxPolicy::~OnnxPolicy() = default;
OnnxPolicy::OnnxPolicy(OnnxPolicy&&) noexcept = default;
OnnxPolicy& OnnxPolicy::operator=(OnnxPolicy&&) noexcept = default;

void OnnxPolicy::load(const std::string& path) {
  try {
    Ort::SessionOptions options;
    options.SetIntraOpNumThreads(1);  // 20Hz 작은 MLP — 한 스레드면 충분
    auto session = std::make_unique<Ort::Session>(environment(), path.c_str(), options);
    if (session->GetInputCount() != 1 || session->GetOutputCount() != 1) {
      throw std::runtime_error("입력/출력이 하나씩이 아니다");
    }
    Ort::AllocatorWithDefaultOptions allocator;
    const auto input_shape = session->GetInputTypeInfo(0).GetTensorTypeAndShapeInfo().GetShape();
    const auto output_shape = session->GetOutputTypeInfo(0).GetTensorTypeAndShapeInfo().GetShape();
    if (input_shape.size() != 2 || output_shape.size() != 2 || input_shape[1] <= 0 || output_shape[1] <= 0) {
      throw std::runtime_error("입출력 모양이 [1, n]이 아니다");
    }
    input_name_ = session->GetInputNameAllocated(0, allocator).get();
    output_name_ = session->GetOutputNameAllocated(0, allocator).get();
    n_in_ = input_shape[1];
    n_out_ = output_shape[1];
    session_ = std::move(session);
    path_ = path;
  } catch (const Ort::Exception& error) {
    throw std::runtime_error("ONNX 열기 실패: " + path + " (" + error.what() + ")");
  } catch (const std::runtime_error& error) {
    throw std::runtime_error("ONNX 형식: " + path + " — " + error.what());
  }
}

void OnnxPolicy::unload() {
  session_.reset();
  path_.clear();
  n_in_ = 0;
  n_out_ = 0;
}

std::vector<float> OnnxPolicy::run(const std::vector<float>& observation) const {
  if (!session_) {
    throw std::runtime_error("정책이 로드되지 않았다");
  }
  if (static_cast<std::int64_t>(observation.size()) != n_in_) {
    throw std::runtime_error("관측 크기 불일치");
  }
  const std::int64_t shape[2] = {1, n_in_};
  auto memory = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);
  auto input = Ort::Value::CreateTensor<float>(memory, const_cast<float*>(observation.data()), observation.size(),
                                               shape, 2);
  const char* input_names[] = {input_name_.c_str()};
  const char* output_names[] = {output_name_.c_str()};
  auto output = session_->Run(Ort::RunOptions{nullptr}, input_names, &input, 1, output_names, 1);
  const float* data = output[0].GetTensorData<float>();
  return std::vector<float>(data, data + n_out_);
}

}  // namespace krosc_ws::ml
