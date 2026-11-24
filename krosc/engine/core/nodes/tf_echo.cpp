#include "krosc/nodes/tf_echo.hpp"

#include <utility>

namespace krosc::nodes {

TfEcho::TfEcho(Context& context, std::string name, std::map<std::string, ParamValue> parameters)
    : Node(context, std::move(name), std::move(parameters)),
      target_frame_(declare_parameter<std::string>("target_frame", "")),
      source_frame_(declare_parameter<std::string>("source_frame", "")),
      rate_(declare_parameter<double>("rate", 1.0, [](double value) { return value > 0.0; })),
      listener_(buffer_, *this) {
  timer_ = create_timer(1.0 / rate_.value(), [this] { tick(); });
}

void TfEcho::tick() {
  const std::string target = target_frame_.value();
  const std::string source = source_frame_.value();
  if (target.empty() || source.empty()) {
    return;
  }
  try {
    const msgs::TransformStamped stamped = buffer_.lookup_transform(target, source);
    if (!last_error_.empty()) {
      last_error_.clear();
      logger().info("%s -> %s 조회가 다시 된다", target.c_str(), source.c_str());
    }
    const auto& t = stamped.transform;
    const tfmath::Rpy rpy = tfmath::to_rpy(t.rotation);
    logger().info(
        "At time %.3f\n- Translation: [%.3f, %.3f, %.3f]\n- Rotation: in Quaternion [%.3f, %.3f, %.3f, %.3f]\n- Rotation: in RPY (radian) [%.3f, %.3f, %.3f]",
        stamped.header.stamp, t.translation.x, t.translation.y, t.translation.z, t.rotation.x, t.rotation.y,
        t.rotation.z, t.rotation.w, rpy.roll, rpy.pitch, rpy.yaw);
  } catch (const TransformException& error) {
    if (last_error_ != error.what()) {
      last_error_ = error.what();
      logger().warn("%s", error.what());
    }
  }
}

}  // namespace krosc::nodes
