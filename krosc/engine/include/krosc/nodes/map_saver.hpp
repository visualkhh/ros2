#pragma once

#include <condition_variable>
#include <map>
#include <memory>
#include <mutex>
#include <string>

#include "krosc/msgs.hpp"
#include "krosc/msgs/nav2_srv.hpp"
#include "krosc/node.hpp"
#include "krosc/parameter.hpp"
#include "krosc/qos.hpp"
#include "krosc/service.hpp"
#include "krosc/topic.hpp"

namespace krosc::nodes {

// nav2 map_saver_server 흉내: map_saver/save_map 서비스로 map 토픽의 지도를 yaml+pgm으로 저장한다.
class MapSaver final : public Node {
 public:
  explicit MapSaver(Context& context, std::string name = "map_saver",
                    std::map<std::string, ParamValue> parameters = {});

  [[nodiscard]] bool save(const msgs::nav2::SaveMapRequest& request);

 private:
  [[nodiscard]] std::string resolve(const std::string& url) const;

  Parameter<double> save_map_timeout_;
  Parameter<double> free_thresh_default_;
  Parameter<double> occupied_thresh_default_;
  Parameter<std::string> maps_directory_;
  ServiceServer save_service_;
};

}  // namespace krosc::nodes
