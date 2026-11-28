#pragma once

#include <algorithm>
#include <filesystem>
#include <string>
#include <vector>

// RL 정책 폴더 스캔 (ONNX Runtime 없이도 테스트 가능한 순수 파일 시스템 코드).
//   artifacts/models/rl_nav_controller/<name>/*.onnx
namespace srf_recycling_ws::ml {

inline bool safe_model_name(const std::string& name) {
  return !name.empty() && name != "." && name != ".." && name.find('/') == std::string::npos &&
         name.find('\\') == std::string::npos;
}

inline std::vector<std::string> list_models(const std::string& directory) {
  std::vector<std::string> found;
  std::error_code error;
  const std::filesystem::path base(directory);
  if (!std::filesystem::exists(base, error)) {
    return found;
  }
  for (const auto& entry : std::filesystem::directory_iterator(base, error)) {
    if (error) {
      break;
    }
    if (!entry.is_directory(error)) {
      continue;  // README.md 같은 파일이 먼저 나와도 계속 스캔한다
    }
    bool has_onnx = false;
    for (const auto& file : std::filesystem::directory_iterator(entry.path(), error)) {
      if (error) {
        break;
      }
      if (file.is_regular_file(error) && file.path().extension() == ".onnx") {
        has_onnx = true;
        break;
      }
    }
    if (error) {
      error.clear();
      continue;
    }
    if (has_onnx) {
      found.push_back(entry.path().filename().string());
    }
  }
  std::sort(found.begin(), found.end());
  return found;
}

inline std::string model_file(const std::string& directory, const std::string& model) {
  if (!safe_model_name(model)) {
    return {};
  }
  const std::filesystem::path base = std::filesystem::path(directory) / model;
  std::error_code error;
  const std::filesystem::path preferred = base / "policy.onnx";
  if (std::filesystem::exists(preferred, error)) {
    return preferred.string();
  }
  for (const auto& entry : std::filesystem::directory_iterator(base, error)) {
    if (error) {
      break;
    }
    if (entry.is_regular_file(error) && entry.path().extension() == ".onnx") {
      return entry.path().string();
    }
  }
  return {};
}

}  // namespace srf_recycling_ws::ml
