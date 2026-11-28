// 헤드리스 장면 스냅샷: 시드·장면으로 월드를 만들고 로봇 카메라 영상(RGB·깊이), 위에서 본 영상, 진실 JSON을 저장한다.
//   srf_recycling_ws_snapshot --seed 7 --scene normal --target can --out /tmp/snap
//   (로봇 자세·팔 관절을 바꿔 여러 장을 만들면 VLM/VLA 데이터로도 쓴다)
#include <QGuiApplication>
#include <QImage>

#include <cmath>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>

#include "render/scene_renderer.hpp"
#include "srf/nodes/map/map_io.hpp"
#include "srf/nodes/map/map_marker.hpp"
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
  if (qEnvironmentVariableIsEmpty("QT_QPA_PLATFORM")) {
    qputenv("QT_QPA_PLATFORM", "offscreen");  // 화면 없이 (라벨 글씨용 폰트만 쓴다)
  }
  QGuiApplication application(argc, argv);
  SceneConfig scene;
  double run_seconds = 0.0;   // 월드를 이만큼 돌린 뒤 찍는다 (작업자·지게차가 움직인다)
  std::string request;        // supply | empty:can|pet|paper — 시작할 때 셀 담당 작업자에게 맡긴다
  std::string export_map;     // 진실 지도 + 기본 마커를 이 폴더에 (map.yaml, map.pgm, markers.yaml)
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
    else if (arg == "--distractors") scene.distractors = std::stoi(value());
    else if (arg == "--people") scene.people = std::stoi(value());
    else if (arg == "--forklifts") scene.forklifts = std::stoi(value());
    else if (arg == "--out") out_dir = value();
    else if (arg == "--run") run_seconds = std::stod(value());
    else if (arg == "--request") request = value();
    else if (arg == "--export-map") export_map = value();
    else if (arg == "--size") { const auto v = numbers(value()); width = static_cast<int>(v.at(0)); height = static_cast<int>(v.at(1)); }
    else if (arg == "--robot") { const auto v = numbers(value()); robot = {v.at(0), v.at(1), v.at(2)}; }
    else if (arg == "--arm") {
      const auto v = numbers(value());
      joints["joint1"] = v.at(0); joints["joint2"] = v.at(1); joints["joint3"] = v.at(2); joints["joint4"] = v.at(3);
      if (v.size() > 4) joints["gripper_left_joint"] = joints["gripper_right_joint"] = v[4];
    } else {
      std::printf("usage: %s [--seed N] [--scene normal|multi|ambiguous|absent] [--target can|pet|paper]\n"
                  "          [--items N] [--distractors N] [--people 0-4] [--forklifts 0-1] [--out DIR]\n"
                  "          [--run SEC] [--request supply|empty:can|pet|paper] [--export-map DIR] [--size W,H] [--robot X,Y,YAW] [--arm J1,J2,J3,J4[,GRIP]]\n"
                  "  --robot 기본: 작업 영역 중심에서 0.55 m 앞에서 작업 영역을 본다\n", argv[0]);
      return arg == "--help" || arg == "-h" ? 0 : 2;
    }
  }
  RecyclingWorld world;
  world.randomize(scene);
  if (!export_map.empty()) {
    srf::nodes::save_map(world.truth_map(), (std::filesystem::path(export_map) / "map").string());
    srf::nodes::save_markers(world.default_markers(), (std::filesystem::path(export_map) / "markers.yaml").string());
    std::printf("exported %s/{map.yaml,map.pgm,markers.yaml}\n", export_map.c_str());
    return 0;
  }
  if (request == "supply") {
    world.request_supply();
  } else if (request.rfind("empty:", 0) == 0) {
    if (const auto bin = parse_category(request.substr(6))) world.request_empty(*bin);
  }
  for (int i = 0; i < static_cast<int>(run_seconds / 0.01); ++i) {
    world.step(0.01);
    world.update();
  }
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
  render::RenderOptions options;
  options.threads = 8;
  render::render(camera, instances, lighting, frame, options);
  save_rgb(frame, out_dir + "/color.png");
  save_depth(frame, out_dir + "/depth.png");

  // 2) 홀 전경, 3) 사람 눈높이에서 본 로봇 셀 (디버그·보고용)
  const auto& layout = world.layout();
  const auto save_view = [&](const std::string& file, render::Vec3 eye, render::Vec3 target, double fov) {
    render::Camera view;
    view.width = 1280;
    view.height = 720;
    view.set_fov(fov * kPi / 180.0);
    view.world_from_optical = render::Pose::scale_free_look(eye, target);
    render::Frame image;
    render::render(view, instances, lighting, image, options);
    save_rgb(image, out_dir + "/" + file);
  };
  save_view("overview.png", {layout.hall_x / 2, -0.8, 17.0}, {layout.hall_x / 2, layout.hall_y / 2 - 0.3, 0.0}, 72);
  save_view("cell.png", {robot.x - 1.6, robot.y - 1.8, 1.6}, {layout.bins[1].x - 1.5, layout.bins[1].y, 0.2}, 70);

  std::ofstream(out_dir + "/truth.json") << world.truth_json(0.0) << "\n";
  std::printf("saved %s/{color.png,depth.png,overview.png,cell.png,truth.json}  robot=%.2f,%.2f,%.2f  triangles=%zu\n",
              out_dir.c_str(), robot.x, robot.y, robot.theta, robot_mesh.triangle_count());
  return 0;
}
