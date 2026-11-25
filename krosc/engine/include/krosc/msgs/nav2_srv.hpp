#pragma once

#include <string>

// nav2_msgs/srv LoadMap / SaveMap의 축소판.
namespace krosc::msgs::nav2 {

struct LoadMapRequest {
  std::string map_url;
};

struct LoadMapResponse {
  static constexpr int RESULT_SUCCESS = 0;
  static constexpr int RESULT_MAP_DOES_NOT_EXIST = 1;
  static constexpr int RESULT_INVALID_MAP_DATA = 2;
  static constexpr int RESULT_INVALID_MAP_METADATA = 3;
  static constexpr int RESULT_UNDEFINED_FAILURE = 255;

  int result = RESULT_UNDEFINED_FAILURE;
};

struct SaveMapRequest {
  std::string map_topic = "map";
  std::string map_url = "map";
  std::string image_format = "pgm";
  std::string map_mode = "trinary";
  double free_thresh = 0.0;
  double occupied_thresh = 0.0;
};

struct SaveMapResponse {
  bool result = false;
};

struct DeleteMapRequest {
  std::string map_url;
};
struct DeleteMapResponse {
  bool result = false;
  std::string message;
};

}  // namespace krosc::msgs::nav2
