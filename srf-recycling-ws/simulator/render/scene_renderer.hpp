#pragma once

// 시뮬레이터 카메라용 3D 소프트웨어 렌더러 (CPU, GPU 없음 → Pi·CI·헤드리스에서도 같은 결과).
//   - 삼각형 메시(STL, 상자·원기둥·구 등)를 핀홀 카메라로 투영하고 z-buffer로 그린다.
//   - 결과: RGB8 + 깊이(16UC1, mm). RealSense 실기와 같은 형식.
//   - URDF 로봇(RobotMesh)은 관절값으로 FK를 풀어 링크별 메시를 놓는다 (카메라에 팔·그리퍼가 보인다).
#include <array>
#include <cstdint>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "srf/msgs.hpp"
#include "srf/urdf.hpp"

namespace srf_recycling_ws::sim::render {

struct Vec3 {
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
};

// 강체 변환 (회전 3x3 + 이동).
struct Pose {
  std::array<std::array<double, 3>, 3> r{{{1, 0, 0}, {0, 1, 0}, {0, 0, 1}}};
  Vec3 t;

  [[nodiscard]] Vec3 apply(const Vec3& v) const;
  [[nodiscard]] Vec3 rotate(const Vec3& v) const;
  [[nodiscard]] Pose operator*(const Pose& other) const;
  [[nodiscard]] Pose inverse() const;

  static Pose translation(double x, double y, double z);
  static Pose rpy(const std::array<double, 3>& xyz, const std::array<double, 3>& rpy);  // URDF 규약
  static Pose axis_angle(const std::array<double, 3>& axis, double angle);
  static Pose yaw(double angle);
};

struct Mesh {
  std::vector<std::array<Vec3, 3>> triangles;
  Vec3 center;          // 경계 구 (컬링)
  double radius = 0.0;
  void finish();        // center/radius 계산
};

using Color = std::array<float, 3>;  // 0..1

struct Instance {
  std::shared_ptr<const Mesh> mesh;
  Pose pose;  // world_from_mesh
  Color color{0.6F, 0.6F, 0.6F};
};

// 기본 도형 (원점 중심; 원기둥·원뿔대는 z축 방향).
std::shared_ptr<Mesh> make_box(double size_x, double size_y, double size_z);
std::shared_ptr<Mesh> make_cylinder(double radius, double length, int segments = 24);
std::shared_ptr<Mesh> make_frustum(double bottom_radius, double top_radius, double length, int segments = 24);
// 구겨진 종이처럼 반지름이 흔들리는 구 (roughness 0 = 매끈한 구).
std::shared_ptr<Mesh> make_lumpy_sphere(double radius, double roughness, std::uint32_t seed, int rings = 10,
                                        int segments = 16);
std::shared_ptr<Mesh> load_stl(const std::string& path, const std::array<double, 3>& scale);

struct Camera {
  Pose world_from_optical;  // 광학 프레임: +z 앞, +x 오른쪽, +y 아래 (ROS optical 규약)
  int width = 424;
  int height = 240;
  double fx = 0.0;
  double fy = 0.0;
  double cx = 0.0;
  double cy = 0.0;
  double near = 0.03;
  double far = 10.0;

  // 수평 화각(라디안)으로 내부 파라미터를 채운다 (D435 color ≈ 69°).
  void set_fov(double horizontal_fov);
};

struct Frame {
  int width = 0;
  int height = 0;
  std::vector<std::uint8_t> rgb;       // width*height*3
  std::vector<std::uint16_t> depth_mm; // width*height, 0 = 없음
};

struct Lighting {
  Vec3 direction{-0.4, -0.3, -1.0};  // 월드 기준 빛이 향하는 방향 (위에서 아래로)
  float ambient = 0.45F;
  float diffuse = 0.6F;
  Color background{0.80F, 0.82F, 0.85F};
};

// instances를 camera로 그린다. 같은 Frame을 다시 쓰면 할당을 재사용한다.
void render(const Camera& camera, const std::vector<Instance>& instances, const Lighting& lighting, Frame& frame);

// URDF 로봇의 시각 메시. 메시 파일은 한 번만 읽고, 관절값마다 링크 자세만 다시 계산한다.
class RobotMesh {
 public:
  explicit RobotMesh(const std::string& urdf_path);

  // base_footprint를 base(2D 자세)에 두고 joints로 FK를 풀어 시각 메시를 out에 더한다.
  void add_instances(const srf::msgs::Pose2D& base, const std::map<std::string, double>& joints,
                     std::vector<Instance>& out) const;
  // 링크의 월드 자세 (예: camera_color_optical_frame).
  [[nodiscard]] Pose link_pose(const std::string& link, const srf::msgs::Pose2D& base,
                               const std::map<std::string, double>& joints) const;
  [[nodiscard]] std::size_t triangle_count() const;
  // 실물에 없는 표시용 링크(예: end_effector_link의 빨간 상자)는 카메라 영상에서 뺀다.
  void hide_link(const std::string& link);

 private:
  struct Visual {
    std::string link;
    Pose offset;
    std::shared_ptr<const Mesh> mesh;
    Color color;
  };

  [[nodiscard]] std::map<std::string, Pose> forward(const srf::msgs::Pose2D& base,
                                                    const std::map<std::string, double>& joints) const;
  [[nodiscard]] std::string resolve(const std::string& filename) const;

  srf::UrdfRobot robot_;
  std::string urdf_dir_;
  std::vector<Visual> visuals_;
  std::vector<std::string> hidden_;
};

}  // namespace srf_recycling_ws::sim::render
