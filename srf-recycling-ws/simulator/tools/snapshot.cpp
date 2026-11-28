// 헤드리스 장면 스냅샷: 시드·장면으로 월드를 만들고 로봇 카메라 영상(RGB·깊이), 위에서 본 영상, 진실 JSON을 저장한다.
//   srf_recycling_ws_snapshot --seed 7 --scene normal --target can --out /tmp/snap
//   (로봇 자세·팔 관절을 바꿔 여러 장을 만들면 VLM/VLA 데이터로도 쓴다)
#include <QImage>

#include <cmath>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>

#include "render/scene_renderer.hpp"
#include "world/recycling_world.hpp"

using namespace srf_recycling_ws::sim;

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kHorizontalFov = 69.0 * kPi / 180.0;  // D435 color

std::vector<double> numbers(const std::string& text) {
  std::vector<double> out;
  std::stringstream stream(text);
  for (std::string part; std::getline(stream, part, ',');) {
    out.push_back(std::stod(part));
  }
  return out;
}

// 눈 위치에서 목표를 보는 광학 프레임 (+z 앞, +x 오른쪽, +y 아래).
render::Pose look_at(render::Vec3 eye, render::Vec3 target) {
  auto norm = [](render::Vec3 v) {
    const double n = std::sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
    return render::Vec3{v.x / n, v.y / n, v.z / n};
  };
  auto cross = [](render::Vec3 a, render::Vec3 b) {
    return render::Vec3{a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x};
  };
  const render::Vec3 z = norm({target.x - eye.x, target.y - eye.y, target.z - eye.z});
  const render::Vec3 x = norm(cross(z, {0, 0, 1}));  // 오른쪽 (위쪽 벡터 = 월드 +z)
  const render::Vec3 y = cross(z, x);                // 아래
  render::Pose pose;
  pose.r = {{{x.x, y.x, z.x}, {x.y, y.y, z.y}, {x.z, y.z, z.z}}};
  pose.t = eye;
  return pose;
}

void save_rgb(const render::Frame& frame, const std::string& path) {
  QImage image(frame.rgb.data(), frame.width, frame.height, frame.width * 3, QImage::Format_RGB888);
  image.save(QString::fromStdString(path));
}

void save_depth(const render::Frame& frame, const std::string& path) {
  QImage image(reinterpret_cast<const uchar*>(frame.depth_mm.data()), frame.width, frame.height, frame.width * 2,
               QImage::Format_Grayscale16);
  image.save(QString::fromStdString(path));
}

}  // namespace

int main(int argc, char** argv) {
  SceneConfig scene;
  std::string out_dir = "snapshot";
  std::string urdf = SRF_RECYCLING_WS_URDF_PATH;
  int width = 640, height = 480;
  srf::msgs::Pose2D robot{-1.0, 0.0, 0.0};  // 기본: 작업 영역 앞
  std::map<std::string, double> joints{{"joint1", 0.0}, {"joint2", -1.0}, {"joint3", 0.3}, {"joint4", 0.7},
                                       {"gripper_left_joint", 0.015}, {"gripper_right_joint", 0.015}};
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    const auto value = [&]() -> std::string { return i + 1 < argc ? argv[++i] : ""; };
    if (arg == "--seed") scene.seed = static_cast<std::uint32_t>(std::stoul(value()));
    else if (arg == "--scene") scene.scene = parse_scene(value()).value_or(Scene::Normal);
    else if (arg == "--target") scene.target = parse_category(value());
    else if (arg == "--items") scene.items = std::stoi(value());
    else if (arg == "--out") out_dir = value();
    else if (arg == "--size") { const auto v = numbers(value()); width = static_cast<int>(v.at(0)); height = static_cast<int>(v.at(1)); }
    else if (arg == "--robot") { const auto v = numbers(value()); robot = {v.at(0), v.at(1), v.at(2)}; }
    else if (arg == "--arm") {
      const auto v = numbers(value());
      joints["joint1"] = v.at(0); joints["joint2"] = v.at(1); joints["joint3"] = v.at(2); joints["joint4"] = v.at(3);
      if (v.size() > 4) joints["gripper_left_joint"] = joints["gripper_right_joint"] = v[4];
    } else {
      std::printf("usage: %s [--seed N] [--scene normal|multi|ambiguous|absent] [--target can|pet|paper]\n"
                  "          [--items N] [--out DIR] [--size W,H] [--robot X,Y,YAW] [--arm J1,J2,J3,J4[,GRIP]]\n"
                  "  --robot 기본: 작업 영역 중심에서 0.55 m 앞에서 작업 영역을 본다\n", argv[0]);
      return arg == "--help" || arg == "-h" ? 0 : 2;
    }
  }
  RecyclingWorld world;
  world.randomize(scene);
  if (robot.x < 0) {  // 기본: 작업 영역 중심을 0.55 m 앞에서 본다
    const auto& area = world.layout().work_area;
    robot = {area.min_x - 0.20, area.center_y(), 0.0};
  }

  std::filesystem::create_directories(out_dir);
  render::RobotMesh robot_mesh(urdf);
  robot_mesh.hide_link("end_effector_link");  // 실물 카메라와 같게
  render::Lighting lighting;

  // 1) 로봇 카메라 (camera_color_optical_frame)
  std::vector<render::Instance> instances;
  world.add_render_instances(instances);
  robot_mesh.add_instances(robot, joints, instances);
  render::Camera camera;
  camera.width = width;
  camera.height = height;
  camera.set_fov(kHorizontalFov);
  camera.world_from_optical = robot_mesh.link_pose("camera_color_optical_frame", robot, joints);
  render::Frame frame;
  render::render(camera, instances, lighting, frame);
  save_rgb(frame, out_dir + "/color.png");
  save_depth(frame, out_dir + "/depth.png");

  // 2) 위에서 본 전경 (디버그·보고용)
  render::Camera overview = camera;
  overview.width = 960;
  overview.height = 640;
  overview.set_fov(60.0 * kPi / 180.0);
  const auto& layout = world.layout();
  overview.world_from_optical = look_at({layout.room_x / 2, -1.2, 2.6}, {layout.room_x / 2, layout.room_y / 2, 0.0});
  render::Frame top;
  render::render(overview, instances, lighting, top);
  save_rgb(top, out_dir + "/overview.png");

  std::ofstream(out_dir + "/truth.json") << world.truth_json(0.0) << "\n";
  std::printf("saved %s/{color.png,depth.png,overview.png,truth.json}  robot=%.2f,%.2f,%.2f  triangles=%zu\n",
              out_dir.c_str(), robot.x, robot.y, robot.theta, robot_mesh.triangle_count());
  return 0;
}
