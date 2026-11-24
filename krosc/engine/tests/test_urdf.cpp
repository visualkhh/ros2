#include <string>

#include "krosc/urdf.hpp"

#include "test_framework.hpp"

namespace {
const char* kUrdf = R"(
<robot name="test_bot">
  <material name="blue"><color rgba="0.0 0.0 0.8 1.0"/></material>
  <link name="base_footprint"/>
  <joint name="base_joint" type="fixed">
    <parent link="base_footprint"/>
    <child link="base_link"/>
    <origin xyz="0 0 0.01" rpy="0 0 0"/>
  </joint>
  <link name="base_link">
    <collision>
      <origin xyz="0 0 0.05" rpy="0 0 0"/>
      <geometry><box size="0.2 0.2 0.1"/></geometry>
    </collision>
  </link>
  <joint name="wheel_joint" type="continuous">
    <parent link="base_link"/>
    <child link="wheel_link"/>
    <origin xyz="0 0.1 0" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
  </joint>
  <link name="wheel_link">
    <visual>
      <geometry><cylinder radius="0.03" length="0.02"/></geometry>
      <material name="blue"/>
    </visual>
  </link>
  <joint name="laser_joint" type="fixed">
    <parent link="base_link"/>
    <child link="base_scan"/>
    <origin xyz="-0.05 0 0.12" rpy="0 0 0"/>
  </joint>
  <link name="base_scan"/>
</robot>
)";
}

KROSC_TEST(urdf_parses_tree_and_geometry) {
  const krosc::UrdfRobot robot = krosc::parse_urdf(kUrdf);
  CHECK_EQ(std::string("test_bot"), robot.name);
  CHECK_EQ(std::string("base_footprint"), robot.root);
  CHECK_EQ(4u, robot.links.size());
  CHECK_EQ(3u, robot.joints.size());
  CHECK_EQ(1u, robot.materials.size());
  const auto* base = robot.find_link("base_link");
  CHECK(base != nullptr);
  if (base != nullptr) {
    CHECK_EQ(1u, base->collisions.size());
    CHECK(base->collisions[0].geometry.type == krosc::UrdfGeometryType::Box);
    CHECK_NEAR(0.2, base->collisions[0].geometry.size[0], 1e-12);
  }
  const auto* wheel = robot.find_link("wheel_link");
  CHECK(wheel != nullptr);
  if (wheel != nullptr) {
    CHECK_EQ(1u, wheel->visuals.size());
    CHECK(wheel->visuals[0].geometry.type == krosc::UrdfGeometryType::Cylinder);
    CHECK_NEAR(0.03, wheel->visuals[0].geometry.radius, 1e-12);
  }
  bool found_movable = false;
  for (const auto& joint : robot.joints) {
    if (joint.name == "wheel_joint") {
      found_movable = joint.movable();
    }
  }
  CHECK(found_movable);
}

KROSC_TEST(urdf_xacro_namespace_substitution) {
  const std::string text = R"(<robot name="x"><link name="${robot_name}base"/></robot>)";
  const auto robot = krosc::parse_urdf(text, {{"robot_name", "tb3_"}});
  CHECK_EQ(std::string("tb3_base"), robot.links[0].name);
  CHECK_THROWS_AS(krosc::parse_urdf(text), krosc::UrdfError);
}

KROSC_TEST(urdf_validation_errors) {
  CHECK_THROWS_AS(krosc::parse_urdf("<not_robot/>"), krosc::UrdfError);
  CHECK_THROWS_AS(krosc::parse_urdf("<robot name='x'></robot>"), krosc::UrdfError);
  CHECK_THROWS_AS(krosc::parse_urdf(R"(
<robot name="x">
  <link name="a"/>
  <link name="a"/>
</robot>)"),
                  krosc::UrdfError);
  CHECK_THROWS_AS(krosc::parse_urdf(R"(
<robot name="x">
  <link name="a"/>
  <link name="b"/>
  <joint name="j" type="fixed"><parent link="a"/><child link="b"/></joint>
  <joint name="j" type="fixed"><parent link="a"/><child link="b"/></joint>
</robot>)"),
                  krosc::UrdfError);
  CHECK_THROWS_AS(krosc::parse_urdf(R"(
<robot name="x">
  <link name="a"/>
  <joint name="j" type="fixed"><parent link="a"/><child link="missing"/></joint>
</robot>)"),
                  krosc::UrdfError);
}
