#include "onnx_policy.hpp"

#include <algorithm>
#include <cstdio>
#include <stdexcept>
#include <utility>

#ifdef KROSC_WS_HAS_ONNXRUNTIME
#include <onnxruntime_cxx_api.h>
#endif

namespace krosc_ws::ml {

namespace {

#ifdef KROSC_WS_HAS_ONNXRUNTIME
constexpr const char* kCpuProvider = "CPUExecutionProvider";

Ort::Env& environment() {
  static Ort::Env env(ORT_LOGGING_LEVEL_WARNING, "krosc_rl_nav");
  return env;
}

// GPU가 있으면 GPU를, 없으면 CPU를 쓴다. 순서가 우선순위다.
const std::vector<std::string>& preferred_providers() {
  static const std::vector<std::string> kPreferred{
      "CUDAExecutionProvider", "CoreMLExecutionProvider", kCpuProvider};
  return kPreferred;
}

std::string choose_provider(const std::vector<std::string>& available) {
  for (const auto& candidate : preferred_providers()) {
    if (candidate == kCpuProvider) {
      return candidate;
    }
    if (std::find(available.begin(), available.end(), candidate) != available.end()) {
      return candidate;
    }
  }
  return kCpuProvider;
}
#else
constexpr const char* kUnavailable =
    "ONNX Runtime 없이 빌드됐다 (RL은 fallback 컨트롤러로 주행한다)";
#endif

}  // namespace

#ifdef KROSC_WS_HAS_ONNXRUNTIME

OnnxPolicy::OnnxPolicy() {
  try {
    available_providers_ = Ort::GetAvailableProviders();
  } catch (const std::exception& error) {
    provider_error_ = std::string("ONNX Runtime provider 목록 실패: ") + error.what();
  }
  provider_ = choose_provider(available_providers_);
}

OnnxPolicy::~OnnxPolicy() = default;
OnnxPolicy::OnnxPolicy(OnnxPolicy&&) noexcept = default;
OnnxPolicy& OnnxPolicy::operator=(OnnxPolicy&&) noexcept = default;

bool OnnxPolicy::runtime_available() { return true; }

std::string OnnxPolicy::runtime_version() {
  try {
    return Ort::GetVersionString();
  } catch (const std::exception&) {
    return {};
  }
}

std::vector<std::string> OnnxPolicy::available_providers() {
  try {
    return Ort::GetAvailableProviders();
  } catch (const std::exception&) {
    return {};
  }
}

void OnnxPolicy::load(const std::string& path) {
  provider_error_.clear();
  try {
    // CUDA provider 초기화가 DefaultLogger를 찾으므로 Env를 먼저 만든다.
    Ort::Env& env = environment();
    std::string provider = provider_;
    std::shared_ptr<Ort::Session> session;
    while (!session) {
      try {
        Ort::SessionOptions options;
        options.SetIntraOpNumThreads(1);  // 20Hz 작은 MLP — 한 스레드면 충분
        if (provider == "CUDAExecutionProvider") {
          // CUDA는 shared provider라 문자열 API가 아니라 명시적 옵션으로 붙여야 한다.
          OrtCUDAProviderOptions cuda_options{};
          cuda_options.device_id = 0;
          options.AppendExecutionProvider_CUDA(cuda_options);
        } else if (provider != kCpuProvider) {
          options.AppendExecutionProvider(provider);
        }
        session = std::make_shared<Ort::Session>(env, path.c_str(), options);
      } catch (const std::exception& error) {
        if (provider == kCpuProvider) {
          throw;
        }
        // CUDA/CoreML이 목록에는 있어도 실제 장치가 없을 수 있다. CPU로 내린다.
        provider_error_ = provider + " 실패 → CPU fallback: " + error.what();
        std::fprintf(stderr, "[onnx] %s\n", provider_error_.c_str());
        provider = kCpuProvider;
      }
    }
    provider_ = provider;
    if (session->GetInputCount() != 1 || session->GetOutputCount() != 1) {
      throw std::runtime_error("입력/출력이 하나씩이 아니다");
    }
    Ort::AllocatorWithDefaultOptions allocator;
    const auto input_shape = session->GetInputTypeInfo(0).GetTensorTypeAndShapeInfo().GetShape();
    const auto output_shape = session->GetOutputTypeInfo(0).GetTensorTypeAndShapeInfo().GetShape();
    if (input_shape.size() != 2 || output_shape.size() != 2 || input_shape[1] <= 0 ||
        output_shape[1] <= 0) {
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
  provider_ = choose_provider(available_providers_);
  provider_error_.clear();
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
  auto input = Ort::Value::CreateTensor<float>(memory, const_cast<float*>(observation.data()),
                                               observation.size(), shape, 2);
  const char* input_names[] = {input_name_.c_str()};
  const char* output_names[] = {output_name_.c_str()};
  auto output = session_->Run(Ort::RunOptions{nullptr}, input_names, &input, 1, output_names, 1);
  const float* data = output[0].GetTensorData<float>();
  return std::vector<float>(data, data + n_out_);
}

#else  // KROSC_WS_HAS_ONNXRUNTIME

OnnxPolicy::OnnxPolicy() : provider_("-"), provider_error_(kUnavailable) {}

OnnxPolicy::~OnnxPolicy() = default;
OnnxPolicy::OnnxPolicy(OnnxPolicy&&) noexcept = default;
OnnxPolicy& OnnxPolicy::operator=(OnnxPolicy&&) noexcept = default;

bool OnnxPolicy::runtime_available() { return false; }
std::string OnnxPolicy::runtime_version() { return {}; }
std::vector<std::string> OnnxPolicy::available_providers() { return {}; }

void OnnxPolicy::load(const std::string&) { throw std::runtime_error(kUnavailable); }

void OnnxPolicy::unload() {
  session_.reset();
  path_.clear();
  n_in_ = 0;
  n_out_ = 0;
  provider_ = "-";
  provider_error_ = kUnavailable;
}

std::vector<float> OnnxPolicy::run(const std::vector<float>&) const {
  throw std::runtime_error(kUnavailable);
}

#endif  // KROSC_WS_HAS_ONNXRUNTIME

}  // namespace krosc_ws::ml
