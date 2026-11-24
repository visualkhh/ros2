#pragma once

#include <map>
#include <mutex>
#include <string>

#include "krosc/msgs.hpp"
#include "krosc/node.hpp"
#include "krosc/parameter.hpp"
#include "krosc/timer.hpp"
#include "krosc/topic.hpp"

namespace krosc::nodes {

// nav2 velocity_smoother의 최소 버전.
// cmd_vel 입력을 가속도 한계로 부드럽게 만들고 cmd_vel_out으로 낸다.
// twist_mux와 collision_monitor 사이에 둔다.
class VelocitySmoother final : public Node {
 public:
  explicit VelocitySmoother(Context& context, std::string name = "velocity_smoother",
                            std::map<std::string, ParamValue> parameters = {});

 private:
  void on_cmd_vel(const msgs::Twist& message);
  void tick();

  Parameter<std::string> cmd_vel_in_topic_;
  Parameter<std::string> cmd_vel_out_topic_;
  Parameter<double> max_linear_accel_;
  Parameter<double> max_angular_accel_;
  Parameter<double> max_linear_speed_;
  Parameter<double> max_angular_speed_;
  Parameter<double> frequency_;
  Parameter<double> input_timeout_;

  std::mutex mutex_;
  double target_linear_ = 0.0;
  double target_angular_ = 0.0;
  double output_linear_ = 0.0;
  double output_angular_ = 0.0;
  double last_input_ = -1e18;
  double last_tick_ = -1.0;
  Subscription<msgs::Twist> subscription_;
  Timer timer_;
};

}  // namespace krosc::nodes
