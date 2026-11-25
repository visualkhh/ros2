#pragma once

#include <string>
#include <utility>
#include <vector>

namespace krosc::viz_web {

// 웹 Viz의 JS 플러그인 한 개. module은 브라우저가 import할 경로다.
struct VizWebPluginInfo {
  std::string name;
  std::string title;
  std::string module;
};

// main이 등록한 플러그인만 /plugins.json에 노출된다. 기본은 빈 레지스트리다.
class VizWebPluginRegistry {
 public:
  void add(VizWebPluginInfo info) {
    for (auto& existing : plugins_) {
      if (existing.name == info.name) {
        existing = std::move(info);
        return;
      }
    }
    plugins_.push_back(std::move(info));
  }

  void add(std::string name, std::string title, std::string module) {
    add(VizWebPluginInfo{std::move(name), std::move(title), std::move(module)});
  }

  [[nodiscard]] const std::vector<VizWebPluginInfo>& plugins() const { return plugins_; }

  // 브라우저가 fetch하는 plugins.json 본문을 만든다.
  [[nodiscard]] std::string manifest_json() const;

 private:
  std::vector<VizWebPluginInfo> plugins_;
};

}  // namespace krosc::viz_web
