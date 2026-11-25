#pragma once

#include <cstdint>
#include <filesystem>
#include <map>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include "krosc/msgs/nav2_srv.hpp"
#include "krosc/node.hpp"

namespace krosc::nodes {

// krosc map_catalog: 지도 목록/선택 UI를 담당하고, 실제 로드/저장은
// map_server(nav2)와 map_saver(nav2) 서비스에 맡긴다.
// - <name>/load_map (LoadMap): 지도 이름을 불러오고 localization 모드로.
// - <name>/save_map (SaveMap): maps_directory/<이름>/map.yaml + map.pgm 저장. 빈 이름이면 날짜/시각.
// - <name>/delete_map (DeleteMap): maps_directory/<이름> 폴더를 지운다.
// - <name>/unload_map (Empty): 지도만 내리고 폴더는 남긴다.
// - <name>/set_slam_mode (String -> Bool): mapping/localization/idle 전환.
//   Cartographer의 mode 파라미터를 표준 set_parameters 서비스로 바꾼다.
// - map_list (String, latch): maps_directory 아래 지도 이름들 (한 줄에 하나)
// - map_name (String, latch): 지금 불러온 지도 이름. 없으면 빈 문자열
// - map_status (String, latch): 마지막 결과
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
  msgs::nav2::LoadMapResponse on_load(const msgs::nav2::LoadMapRequest& request);
  msgs::nav2::SaveMapResponse on_save(const msgs::nav2::SaveMapRequest& request);
  msgs::nav2::DeleteMapResponse on_delete(const msgs::nav2::DeleteMapRequest& request);
  msgs::Empty on_unload(const msgs::Empty& request);
  msgs::Bool on_set_slam_mode(const msgs::String& request);
  void publish_status(const std::string& text);
  void publish_map_name();
  void publish_list();
  bool set_slam_mode(const std::string& mode);
  [[nodiscard]] std::filesystem::path directory() const;
  [[nodiscard]] std::filesystem::path map_path(const std::string& name) const;
  [[nodiscard]] static bool safe_name(const std::string& name);

  Parameter<std::string> maps_directory_;
  Parameter<std::string> map_frame_;
  Parameter<double> list_period_;
  Parameter<std::string> map_server_;
  Parameter<std::string> map_saver_;
  Parameter<std::string> map_list_topic_;
  Parameter<std::string> map_name_topic_;
  Parameter<std::string> map_status_topic_;
  Parameter<std::string> cartographer_node_;

  mutable std::mutex mutex_;
  std::string loaded_name_;
  std::vector<std::string> names_;
  std::string status_ = "no map loaded";
};

}  // namespace krosc::nodes
