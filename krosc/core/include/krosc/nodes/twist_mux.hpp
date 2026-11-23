#pragma once

#include <cstdint>
#include <map>
#include <optional>
#include <string>
#include <tuple>
#include <vector>

#include "krosc/msgs.hpp"
#include "krosc/node.hpp"
#include "krosc/parameter.hpp"
#include "krosc/topic.hpp"

namespace krosc::nodes {

// ROS twist_mux 흉내: 여러 속도 명령 중 우선순위가 가장 높고 살아 있는(timeout 안에 받은) 것 하나만
// cmd_vel_out으로 낸다. lock(std_msgs/Bool)이 켜져 있으면 그 priority 이하 입력은 막힌다.
class TwistMux final : public Node {
 public:
  explicit TwistMux(Context& context, std::string name = "twist_mux", std::map<std::string, ParamValue> parameters = {});

  [[nodiscard]] std::string cmd_vel_out() const { return cmd_vel_out_.value(); }
  [[nodiscard]] std::string active() const;
  [[nodiscard]] int lock_priority() const;

 private:
  struct Input {
    std::string topic;
    double timeout = 0.5;
    int priority = 0;
    double last = -1e18;
    bool received = false;
  };
  struct Lock {
    std::string topic;
    double timeout = 0.0;
    int priority = 255;
    double last = 0.0;
    bool state = false;
  };

  void on_twist(const std::string& name, const msgs::Twist& twist);
  void on_lock(const std::string& name, const msgs::Bool& flag);

  Parameter<std::string> cmd_vel_out_;
  Parameter<std::vector<std::string>> topics_;
  Parameter<std::vector<std::string>> locks_;
  std::map<std::string, Input> inputs_;
  std::map<std::string, Lock> lock_inputs_;
  double started_ = 0.0;
  std::vector<Subscription<msgs::Twist>> subscriptions_;
  std::vector<Subscription<msgs::Bool>> lock_subscriptions_;
};

}  // namespace krosc::nodes
