#pragma once

#include <cstdint>
#include <filesystem>
#include <map>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include "krosc/node.hpp"

namespace krosc::nodes {

// nav2 map_server + map_saver + map_catalog 흉내 (토픽 기반).
// - map_change (String): 지도 이름을 불러오고 localization 모드로. 빈 이름이면 지금 지도를 내리고 그 폴더를 지운다.
// - map_save (String): 지금 map 토픽의 지도를 maps_directory/<이름>/map.yaml + map.pgm으로 저장. 빈 이름이면 날짜/시각.
// - map_list (String, latch): maps_directory 아래 지도 이름들 (한 줄에 하나)
// - map_status (String, latch): 마지막 결과
// - slam_mode (String, latch): map_change가 localization/idle을 알려 Cartographer가 따라간다.
class MapCatalog final : public Node {
 public:
  explicit MapCatalog(Context& context, std::string name = "map_catalog",
                      std::map<std::string, ParamValue> parameters = {});
  ~MapCatalog() override = default;

  MapCatalog(const MapCatalog&) = delete;
  MapCatalog& operator=(const MapCatalog&) = delete;

  [[nodiscard]] std::string loaded_name() const;
  [[nodiscard]] std::vector<std::string> names() const;
  [[nodiscard]] std::string status() const;

 private:
  void refresh();
  void on_map(const msgs::OccupancyGrid& grid);
  void on_map_change(const msgs::String& message);
  void on_map_save(const msgs::String& message);
  void on_map_unload(const msgs::Empty& message);
  void on_slam_status(const msgs::String& message);
  void publish_status(const std::string& text);
  void publish_list();
  void publish_slam_mode(const std::string& mode);
  void publish_current_map(const std::string& name);
  [[nodiscard]] std::filesystem::path directory() const;
  [[nodiscard]] std::filesystem::path map_path(const std::string& name) const;
  [[nodiscard]] static bool safe_name(const std::string& name);

  Parameter<std::string> maps_directory_;
  Parameter<std::string> map_frame_;
  Parameter<double> list_period_;
  Parameter<std::string> map_topic_;
  Parameter<std::string> map_change_topic_;
  Parameter<std::string> map_save_topic_;
  Parameter<std::string> map_unload_topic_;
  Parameter<std::string> map_list_topic_;
  Parameter<std::string> map_status_topic_;
  Parameter<std::string> slam_mode_topic_;
  Parameter<std::string> slam_status_topic_;
  Parameter<std::string> current_map_topic_;

  mutable std::mutex mutex_;
  std::optional<msgs::OccupancyGrid> map_;
  std::string loaded_name_;
  std::vector<std::string> names_;
  std::string status_ = "no map loaded";
  std::string slam_status_ = "idle";
  std::string current_map_;
};

}  // namespace krosc::nodes
