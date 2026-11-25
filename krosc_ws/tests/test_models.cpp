#include <unistd.h>

#include <algorithm>
#include <exception>
#include <filesystem>
#include <fstream>
#include <string>
#include <vector>

#include "ml/model_catalog.hpp"
#include "ml/onnx_policy.hpp"

#include "test_framework.hpp"

namespace {

std::filesystem::path make_tree() {
  const auto base = std::filesystem::temp_directory_path() /
                    ("krosc_models_" + std::to_string(::getpid()));
  std::filesystem::remove_all(base);
  std::filesystem::create_directories(base);
  std::ofstream(base / "README.md") << "top-level file must not stop the scan";
  std::filesystem::create_directories(base / "alpha");
  std::filesystem::create_directories(base / "beta");
  std::filesystem::create_directories(base / "gamma");
  {
    std::ofstream(base / "alpha" / "policy.onnx") << "onnx";
    std::ofstream(base / "alpha" / "policy.onnx.data") << "data";
    std::ofstream(base / "beta" / "maze.onnx") << "onnx";
    std::ofstream(base / "gamma" / "readme.txt") << "not a model";
  }
  return base;
}

}  // namespace

KROSC_TEST(rl_model_catalog_lists_and_resolves) {
  const auto base = make_tree();
  const auto names = krosc_ws::ml::list_models(base.string());
  CHECK_EQ(2u, names.size());
  CHECK_EQ(std::string("alpha"), names[0]);
  CHECK_EQ(std::string("beta"), names[1]);

  CHECK(krosc_ws::ml::model_file(base.string(), "alpha").find("alpha/policy.onnx") !=
        std::string::npos);
  CHECK(krosc_ws::ml::model_file(base.string(), "beta").find("beta/maze.onnx") !=
        std::string::npos);
  CHECK(krosc_ws::ml::model_file(base.string(), "gamma").empty());
  CHECK(krosc_ws::ml::model_file(base.string(), "../alpha").empty());
  CHECK(!krosc_ws::ml::safe_model_name("a/b"));
  CHECK(krosc_ws::ml::safe_model_name("rl_maze"));
  std::filesystem::remove_all(base);
}

KROSC_TEST(onnx_policy_reports_runtime_and_provider) {
  using krosc_ws::ml::OnnxPolicy;
  OnnxPolicy policy;
#ifdef KROSC_WS_HAS_ONNXRUNTIME
  CHECK(OnnxPolicy::runtime_available());
  CHECK(!OnnxPolicy::runtime_version().empty());
  const auto providers = OnnxPolicy::available_providers();
  CHECK(!providers.empty());
  CHECK(!policy.provider().empty());
  CHECK(policy.provider() == "CPUExecutionProvider" ||
        std::find(providers.begin(), providers.end(), policy.provider()) != providers.end());

  const std::string model = std::string(KROSC_WS_MODELS_DIR) + "/rl_nav_controller/rl_maze/policy.onnx";
  policy.load(model);
  CHECK(policy.loaded());
  CHECK_EQ(52, policy.inputs());
  CHECK_EQ(2, policy.outputs());
  const auto action = policy.run(std::vector<float>(52, 0.0F));
  CHECK_EQ(2u, action.size());
  CHECK(policy.provider() != "-");
#else
  CHECK(!OnnxPolicy::runtime_available());
  CHECK(OnnxPolicy::runtime_version().empty());
  CHECK(OnnxPolicy::available_providers().empty());
  CHECK_EQ(std::string("-"), policy.provider());
  bool threw = false;
  try {
    policy.load("no-such.onnx");
  } catch (const std::exception&) {
    threw = true;
  }
  CHECK(threw);
#endif
}
