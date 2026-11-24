#pragma once

#include <map>
#include <string>

#include "krosc/msgs.hpp"
#include "krosc/msgs/nav2_srv.hpp"
#include "krosc/node.hpp"
#include "krosc/parameter.hpp"
#include "krosc/service.hpp"
#include "krosc/topic.hpp"

namespace krosc::nodes {

// nav2 map_server 흉내: yaml+pgm을 읽어 map 토픽(latch)으로 내고,
// map_server/load_map, map_server/unload_map 서비스를 제공한다.
class MapServer final : public Node {
 public:
  explicit MapServer(Context& context, std::string name = "map_server",
                     std::map<std::string, ParamValue> parameters = {});

  [[nodiscard]] msgs::nav2::LoadMapResponse load(const std::string& url);
  void unload();

 private:
  [[nodiscard]] std::string resolve(const std::string& url) const;
  void publish_grid(msgs::OccupancyGrid grid);

  Parameter<std::string> yaml_filename_;
  Parameter<std::string> topic_name_;
  Parameter<std::string> frame_id_;
  Parameter<std::string> maps_directory_;
  ServiceServer load_service_;
  ServiceServer unload_service_;
};

}  // namespace krosc::nodes
