#include "krosc/nodes/velocity_smoother.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

#include "krosc/qos.hpp"

namespace krosc::nodes {

namespace {
double approach(double current, double target, double max_delta) {
  const double delta = target - current;
  if (std::abs(delta) <= max_delta) {
    return target;
  }
  return current + std::copysign(max_delta, delta);
}
}  // namespace

VelocitySmoother::VelocitySmoother(Context& context, std::string name, std::map<std::string, ParamValue> parameters)
    : Node(context, std::move(name), std::move(parameters)),
      cmd_vel_in_topic_(declare_parameter<std::string>("cmd_vel_in_topic", "cmd_vel_mux")),
      cmd_vel_out_topic_(declare_parameter<std::string>("cmd_vel_out_topic", "cmd_vel_smoothed")),
      max_linear_accel_(declare_parameter<double>("max_linear_accel", 0.5,
                                                 [](double value) { return value > 0.0; })),
      max_angular_accel_(declare_parameter<double>("max_angular_accel", 2.0,
                                                  [](double value) { return value > 0.0; })),
      max_linear_speed_(declare_parameter<double>("max_linear_speed", 0.26,
                                                  [](double value) { return value > 0.0; })),
      max_angular_speed_(declare_parameter<double>("max_angular_speed", 1.82,
                                                   [](double value) { return value > 0.0; })),
      frequency_(declare_parameter<double>("frequency", 50.0, [](double value) { return value > 0.0; })),
      input_timeout_(declare_parameter<double>("input_timeout", 0.5, [](double value) { return value > 0.0; })) {
  subscription_ = subscribe<msgs::Twist>(
      cmd_vel_in_topic_.value(), [this](const msgs::Twist& message) { on_cmd_vel(message); }, QOS_COMMAND);
  timer_ = create_timer(1.0 / frequency_.value(), [this] { tick(); });
  logger().info("VelocitySmoother: in=%s out=%s accel=%.2f/%.2f speed=%.2f/%.2f",
                cmd_vel_in_topic_.value().c_str(), cmd_vel_out_topic_.value().c_str(),
                max_linear_accel_.value(), max_angular_accel_.value(), max_linear_speed_.value(),
                max_angular_speed_.value());
}

void VelocitySmoother::on_cmd_vel(const msgs::Twist& message) {
  std::lock_guard<std::mutex> lock(mutex_);
  target_linear_ = std::clamp(message.linear.x, -max_linear_speed_.value(), max_linear_speed_.value());
  target_angular_ = std::clamp(message.angular.z, -max_angular_speed_.value(), max_angular_speed_.value());
  last_input_ = now();
}

void VelocitySmoother::tick() {
  const double current = now();
  double dt = last_tick_ < 0.0 ? 1.0 / frequency_.value() : current - last_tick_;
  last_tick_ = current;
  dt = std::clamp(dt, 0.0, 0.2);

  std::lock_guard<std::mutex> lock(mutex_);
  if (current - last_input_ > input_timeout_.value()) {
    target_linear_ = 0.0;
    target_angular_ = 0.0;
  }
  output_linear_ = approach(output_linear_, target_linear_, max_linear_accel_.value() * dt);
  output_angular_ = approach(output_angular_, target_angular_, max_angular_accel_.value() * dt);
  if (std::abs(output_linear_) < 1e-4 && target_linear_ == 0.0) {
    output_linear_ = 0.0;
  }
  if (std::abs(output_angular_) < 1e-4 && target_angular_ == 0.0) {
    output_angular_ = 0.0;
  }
  msgs::Twist output;
  output.linear.x = output_linear_;
  output.angular.z = output_angular_;
  publish(cmd_vel_out_topic_.value(), output, QOS_COMMAND);
}

}  // namespace krosc::nodes
