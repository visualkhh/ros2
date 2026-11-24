#include <unistd.h>

#include <filesystem>
#include <fstream>
#include <string>
#include <vector>

#include "ml/model_catalog.hpp"

#include "test_framework.hpp"

namespace {

std::filesystem::path make_tree() {
  const auto base = std::filesystem::temp_directory_path() /
                    ("krosc_models_" + std::to_string(::getpid()));
  std::filesystem::remove_all(base);
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
