#pragma once

#include <array>
#include <map>
#include <optional>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

namespace krosc {

class UrdfError : public std::runtime_error {
 public:
  using std::runtime_error::runtime_error;
};

enum class UrdfGeometryType { None, Box, Cylinder, Sphere, Mesh };

struct UrdfGeometry {
  UrdfGeometryType type = UrdfGeometryType::None;
  std::array<double, 3> size{0.0, 0.0, 0.0};  // box
  double radius = 0.0;                        // cylinder/sphere
  double length = 0.0;                        // cylinder
  std::string filename;                       // mesh
  std::array<double, 3> scale{1.0, 1.0, 1.0};
};

struct UrdfVisual {
  std::array<double, 3> xyz{0.0, 0.0, 0.0};
  std::array<double, 3> rpy{0.0, 0.0, 0.0};
  UrdfGeometry geometry;
  std::string material;
};

struct UrdfLink {
  std::string name;
  std::vector<UrdfVisual> visuals;
  std::vector<UrdfVisual> collisions;
};

struct UrdfJoint {
  std::string name;
  std::string type;  // fixed/continuous/revolute/prismatic/floating/planar
  std::string parent;
  std::string child;
  std::array<double, 3> xyz{0.0, 0.0, 0.0};
  std::array<double, 3> rpy{0.0, 0.0, 0.0};
  std::array<double, 3> axis{1.0, 0.0, 0.0};
  bool has_limits = false;
  double lower = 0.0;
  double upper = 0.0;
  double velocity = 0.0;

  [[nodiscard]] bool movable() const {
    return type == "continuous" || type == "revolute" || type == "prismatic";
  }
};

struct UrdfRobot {
  std::string name;
  std::string root;
  std::vector<UrdfLink> links;
  std::vector<UrdfJoint> joints;
  std::map<std::string, std::array<double, 4>> materials;

  [[nodiscard]] const UrdfLink* find_link(const std::string& link_name) const {
    for (const auto& link : links) {
      if (link.name == link_name) {
        return &link;
      }
    }
    return nullptr;
  }
};

// URDF 본문을 읽어 링크/관절 트리를 만든다 (표준 라이브러리만).
// xacro는 인자 치환(${이름})만 지원한다. 구조(중복, 없는 링크, 부모 둘, 뿌리, 연결)를 검증한다.
[[nodiscard]] UrdfRobot parse_urdf(std::string_view text,
                                   const std::map<std::string, std::string>& xacro_args = {{"namespace", ""}});

}  // namespace krosc
