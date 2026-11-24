#pragma once

#include <QString>
#include <Qt>

#include <functional>
#include <map>
#include <memory>
#include <string>
#include <vector>

class QDockWidget;
class QMainWindow;
class QWidget;

namespace krosc::viz {

class MapView2D;
class SceneView;
class VizModel;

struct VizPluginInfo {
  std::string id;
  std::string name;
  std::string version = "1.0";
  std::vector<std::string> requirements;  // 예: {"tf", "robot_description", "moveit_state"}
};

// 코어 Viz가 플러그인에 제공하는 최소 API. 플러그인은 이 인터페이스만 알고,
// VizWindow 내부 구현에는 의존하지 않는다.
class VizHost {
 public:
  virtual ~VizHost() = default;

  virtual VizModel& model() = 0;
  virtual SceneView& scene() = 0;
  virtual MapView2D& map_view() = 0;
  virtual QMainWindow& window() = 0;

  virtual void add_dock(QDockWidget* dock, Qt::DockWidgetArea area) = 0;
  virtual void add_tab(QWidget* tab, const QString& title, bool make_current = false) = 0;
};

class IVizPlugin {
 public:
  virtual ~IVizPlugin() = default;
  [[nodiscard]] virtual VizPluginInfo info() const = 0;
  virtual void initialize(VizHost& host) = 0;
  virtual void shutdown() {}
};

using VizPluginFactory = std::function<std::unique_ptr<IVizPlugin>()>;

class VizPluginRegistry {
 public:
  void add(const std::string& id, VizPluginFactory factory) { factories_[id] = std::move(factory); }

  [[nodiscard]] std::unique_ptr<IVizPlugin> create(const std::string& id) const {
    const auto found = factories_.find(id);
    if (found == factories_.end()) {
      return nullptr;
    }
    return found->second();
  }

  [[nodiscard]] std::vector<std::string> ids() const {
    std::vector<std::string> result;
    result.reserve(factories_.size());
    for (const auto& [id, factory] : factories_) {
      (void)factory;
      result.push_back(id);
    }
    return result;
  }

 private:
  std::map<std::string, VizPluginFactory> factories_;
};

}  // namespace krosc::viz
