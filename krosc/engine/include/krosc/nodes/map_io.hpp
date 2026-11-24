#pragma once

#include <stdexcept>
#include <string>

#include "krosc/msgs.hpp"

namespace krosc::nodes {

class MapIoError : public std::runtime_error {
 public:
  using std::runtime_error::runtime_error;
};

// nav2 map_server 형식(yaml + PGM P5) 읽기/쓰기. yaml 안의 image는 yaml 파일 기준 상대 경로다.
// origin의 yaw 회전은 읽을 때 무시한다 (map_server 지도는 보통 0).
msgs::OccupancyGrid load_map(const std::string& yaml_path, const std::string& frame_id = "map", double stamp = 0.0);

// path_without_extension에 .pgm + .yaml을 쓴다. 쓴 yaml 경로를 돌려준다.
std::string save_map(const msgs::OccupancyGrid& grid, const std::string& path_without_extension,
                     double free_thresh = 0.196, double occupied_thresh = 0.65);

}  // namespace krosc::nodes
