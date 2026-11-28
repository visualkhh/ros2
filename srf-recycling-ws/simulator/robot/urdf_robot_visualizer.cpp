#include "robot/urdf_robot_visualizer.hpp"

#include <cmath>
#include <functional>
#include <utility>

#include "srf/tf/math.hpp"

namespace srf_recycling_ws::sim {

namespace msgs = srf::msgs;
namespace tfmath = srf::tfmath;

namespace {

msgs::Transform joint_origin(const srf::UrdfJoint& joint) {
  return msgs::Transform{{joint.xyz[0], joint.xyz[1], joint.xyz[2]},
                         tfmath::from_rpy(joint.rpy[0], joint.rpy[1], joint.rpy[2])};
}

msgs::Transform joint_motion(const srf::UrdfJoint& joint, double position) {
  msgs::Transform motion;
  if (joint.type == "revolute" || joint.type == "continuous") {
    const double length = std::hypot(joint.axis[0], joint.axis[1], joint.axis[2]);
    if (length > 1e-12) {
      motion.rotation = tfmath::from_axis_angle(
          {joint.axis[0] / length, joint.axis[1] / length, joint.axis[2] / length}, position);
    }
  } else if (joint.type == "prismatic") {
    motion.translation = {joint.axis[0] * position, joint.axis[1] * position,
                          joint.axis[2] * position};
  }
  return motion;
}

std::array<double, 3> material_color(const srf::UrdfRobot& robot, const std::string& name) {
  const auto found = robot.materials.find(name);
  if (found == robot.materials.end()) {
    return {0.62, 0.64, 0.70};
  }
  return {found->second[0], found->second[1], found->second[2]};
}

}  // namespace

UrdfRobotVisualizer::UrdfRobotVisualizer(std::string urdf_text)
    : robot_(srf::parse_urdf(urdf_text)) {}

std::vector<srf::simulator::RobotVisual> UrdfRobotVisualizer::visuals(
    const msgs::Pose2D& base, const std::map<std::string, double>& joints) const {
  using srf::simulator::RobotVisual;
  using srf::simulator::RobotVisualType;

  std::map<std::string, std::vector<const srf::UrdfJoint*>> children;
  for (const auto& joint : robot_.joints) {
    children[joint.parent].push_back(&joint);
  }

  std::map<std::string, msgs::Transform> links;
  links[robot_.root] = msgs::Transform{{base.x, base.y, 0.0}, tfmath::from_yaw(base.theta)};
  const std::function<void(const std::string&)> visit = [&](const std::string& parent) {
    const auto found = children.find(parent);
    if (found == children.end()) {
      return;
    }
    for (const auto* joint : found->second) {
      double position = 0.0;
      if (const auto value = joints.find(joint->name); value != joints.end()) {
        position = value->second;
      } else if (joint->name == "gripper_right_joint") {
        if (const auto value = joints.find("gripper_left_joint"); value != joints.end()) {
          position = value->second;
        }
      }
      const msgs::Transform transform = tfmath::compose(
          links.at(parent), tfmath::compose(joint_origin(*joint), joint_motion(*joint, position)));
      links[joint->child] = transform;
      visit(joint->child);
    }
  };
  visit(robot_.root);

  std::vector<RobotVisual> result;
  for (const auto& link : robot_.links) {
    const auto found = links.find(link.name);
    if (found == links.end()) {
      continue;
    }
    for (const auto& visual : link.visuals) {
      if (visual.geometry.type == srf::UrdfGeometryType::None ||
          visual.geometry.type == srf::UrdfGeometryType::Mesh) {
        continue;
      }
      const msgs::Transform transform = tfmath::compose(
          found->second,
          msgs::Transform{{visual.xyz[0], visual.xyz[1], visual.xyz[2]},
                          tfmath::from_rpy(visual.rpy[0], visual.rpy[1], visual.rpy[2])});
      RobotVisual part;
      part.transform = transform;
      part.color = material_color(robot_, visual.material);
      if (visual.geometry.type == srf::UrdfGeometryType::Box) {
        part.type = RobotVisualType::Box;
        part.size = visual.geometry.size;
      } else if (visual.geometry.type == srf::UrdfGeometryType::Cylinder) {
        part.type = RobotVisualType::Cylinder;
        part.radius = visual.geometry.radius;
        part.length = visual.geometry.length;
        part.sides = 24;
      } else if (visual.geometry.type == srf::UrdfGeometryType::Sphere) {
        part.type = RobotVisualType::Cylinder;
        part.radius = visual.geometry.radius;
        part.length = visual.geometry.radius * 2.0;
        part.sides = 16;
      } else {
        continue;
      }
      result.push_back(std::move(part));
    }
  }
  return result;
}

}  // namespace srf_recycling_ws::sim
