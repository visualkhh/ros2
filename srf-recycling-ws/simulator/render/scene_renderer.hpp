#pragma once

// 시뮬레이터 카메라용 3D 소프트웨어 렌더러 (CPU, GPU 없음 → Pi·CI·헤드리스에서도 같은 결과).
//   - 삼각형 메시(STL, 상자·원기둥·구 등) + 재질(색·텍스처·광택·투명·발광)을 핀홀 카메라로 투영한다.
//   - 픽셀 단위 음영(부드러운 법선, Blinn 광택), 원근 보정 텍스처, 투명(뒤→앞 섞기)·그림자 데칼(곱하기),
//     슈퍼샘플링(계단 제거), 행 단위 멀티스레드.
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

struct F3 {
  float x = 0.0F;
  float y = 0.0F;
  float z = 0.0F;
};

struct UV {
  float u = 0.0F;
  float v = 0.0F;
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
  static Pose scale_free_look(const Vec3& eye, const Vec3& target);  // 광학 프레임(+z 앞, +y 아래)
};

using Color = std::array<float, 3>;  // 0..1

// RGBA 텍스처 (8비트 저장, 샘플은 0..1). 좌표는 반복(wrap)된다. v=0이 이미지 위쪽.
struct Texture {
  int width = 0;
  int height = 0;
  std::vector<std::uint8_t> rgba;
  [[nodiscard]] std::array<float, 4> sample(float u, float v) const;  // 쌍선형
};

enum class Blend { Opaque, Alpha, Multiply };

struct Material {
  Color color{1.0F, 1.0F, 1.0F};             // 텍스처에 곱한다
  std::shared_ptr<const Texture> texture;    // 없으면 color만
  float opacity = 1.0F;                      // Alpha: 텍스처 알파와 곱한다
  Blend blend = Blend::Opaque;               // Multiply: 아래를 (1 - 알파×opacity)만큼 어둡게 (그림자 데칼)
  float specular = 0.04F;                    // 0..1 (금속·플라스틱 광택)
  float shininess = 24.0F;
  float emissive = 0.0F;                     // 1이면 조명과 무관하게 color
  std::array<float, 2> uv_scale{1.0F, 1.0F};   // 텍스처 반복 수
  std::array<float, 2> uv_offset{0.0F, 0.0F};  // 움직이는 벨트 등
};

// 단색 재질.
inline Material solid(Color color, float specular = 0.04F, float shininess = 24.0F) {
  Material m;
  m.color = color;
  m.specular = specular;
  m.shininess = shininess;
  return m;
}

// 메시: 위치(필수), 법선·UV(없으면 면 법선·UV 0). 메모리를 아끼려고 float로 둔다.
struct Mesh {
  std::vector<std::array<F3, 3>> positions;
  std::vector<std::array<F3, 3>> normals;
  std::vector<std::array<UV, 3>> uvs;
  Vec3 center;  // 경계 구 (컬링)
  double radius = 0.0;
  void finish();
  void add(const std::array<F3, 3>& p, const std::array<F3, 3>& n, const std::array<UV, 3>& uv);
  [[nodiscard]] std::size_t triangle_count() const { return positions.size(); }
};

struct Instance {
  std::shared_ptr<const Mesh> mesh;
  Pose pose;  // world_from_mesh
  Material material;
};

// 기본 도형 (원점 중심; 원기둥·원뿔대는 z축 방향). UV: 상자 면마다 0..1, 원기둥 옆면 u=각도·v=위→아래.
std::shared_ptr<Mesh> make_box(double size_x, double size_y, double size_z);
std::shared_ptr<Mesh> make_cylinder(double radius, double length, int segments = 32);
std::shared_ptr<Mesh> make_frustum(double bottom_radius, double top_radius, double length, int segments = 32,
                                   bool caps = true);
// 구겨진 종이처럼 반지름이 흔들리는 구 (roughness 0 = 매끈한 구).
std::shared_ptr<Mesh> make_lumpy_sphere(double radius, double roughness, std::uint32_t seed, int rings = 12,
                                        int segments = 20);
// xy 평면의 사각형 (+z를 본다). 데칼·간판.
std::shared_ptr<Mesh> make_quad(double size_x, double size_y);
// 옆면이 찌그러진 원기둥 (찌그러진 캔): 높이 중간을 눌러 반지름을 줄인다.
std::shared_ptr<Mesh> make_crushed_cylinder(double radius, double length, double crush, std::uint32_t seed,
                                            int segments = 32);
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
  double far = 40.0;

  // 수평 화각(라디안)으로 내부 파라미터를 채운다 (D435 color ≈ 69°).
  void set_fov(double horizontal_fov);
};

struct Frame {
  int width = 0;
  int height = 0;
  std::vector<std::uint8_t> rgb;        // width*height*3
  std::vector<std::uint16_t> depth_mm;  // width*height, 0 = 없음
};

struct Lighting {
  Vec3 key_direction{-0.35, -0.25, -1.0};  // 월드 기준 빛이 향하는 방향 (천장 조명)
  float key = 0.62F;
  Vec3 fill_direction{0.6, 0.4, -0.5};
  float fill = 0.18F;
  Color sky{0.42F, 0.43F, 0.45F};     // 위를 보는 면의 주변광
  Color ground{0.24F, 0.23F, 0.22F};  // 아래를 보는 면의 주변광
  Color background{0.55F, 0.57F, 0.60F};
};

struct RenderOptions {
  int supersample = 2;   // 2 = 2x2 슈퍼샘플링
  int threads = 4;
  float noise = 0.0F;    // 센서 잡음 (0..1 표준편차)
  float vignette = 0.12F;
  std::uint32_t seed = 1;
};

// instances를 camera로 그린다. 같은 Frame을 다시 쓰면 할당을 재사용한다.
void render(const Camera& camera, const std::vector<Instance>& instances, const Lighting& lighting, Frame& frame,
            const RenderOptions& options = {});

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
    Material material;
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
