// 헤드리스 학습 데이터 생성기 (VLM 미세조정용): 장면·로봇 자세·카메라 각도·팔·조명·방해 요소를 무작위로 바꿔
// 로봇 카메라 RGB를 찍고, 정답을 자동으로 붙인다.
//   - 물품: 시뮬레이터 진실값 + 물체 번호 버퍼 → 보이는 상자, 가림 비율, 카메라 기준 3D 위치, 자세(누움·방향)
//   - 지도 마커(map_marker: 바구니·매트·접근 자세 …): 3D 상자 투영 + 깊이 비교 → 보이는 상자·비율
//   - 지도 구역(통로·감속·셀 …): 깊이 영상을 바닥으로 되돌려 구역에 속한 픽셀 → 영역 상자·비율
//   - 카메라 내부 파라미터·자세, 장면 설정, 진실 JSON
// 출력: <out>/images/000001.png (+ depth/000001.png), <out>/labels.jsonl (한 줄에 한 장)
//   srf_recycling_ws_dataset --out /tmp/ds --count 200 --seed 1
// 질문·답(VQA) 만들기와 학습은 ml/vlm이 labels.jsonl을 읽어서 한다.
#include <QGuiApplication>
#include <QImage>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <map>
#include <random>
#include <sstream>
#include <string>
#include <vector>

#include "render/scene_renderer.hpp"
#include "srf/nodes/map/map_marker.hpp"
#include "world/recycling_world.hpp"

using namespace srf_recycling_ws::sim;

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kHorizontalFov = 69.0 * kPi / 180.0;  // D435 color

std::string esc(const std::string& s) {
  std::string out;
  for (const char c : s) {
    if (c == '"' || c == '\\') out += '\\';
    if (c == '\n') {
      out += "\\n";
      continue;
    }
    out += c;
  }
  return out;
}

std::string num(double v, int digits = 4) {
  char text[48];
  std::snprintf(text, sizeof(text), "%.*f", digits, v);
  return text;
}

struct Box2 {
  int x0 = 1 << 30, y0 = 1 << 30, x1 = -1, y1 = -1;
  int pixels = 0;
  void add(int x, int y) {
    x0 = std::min(x0, x);
    y0 = std::min(y0, y);
    x1 = std::max(x1, x);
    y1 = std::max(y1, y);
    ++pixels;
  }
  [[nodiscard]] bool valid() const { return pixels > 0; }
  [[nodiscard]] std::string json() const {
    return valid() ? "[" + std::to_string(x0) + "," + std::to_string(y0) + "," + std::to_string(x1 + 1) + "," +
                         std::to_string(y1 + 1) + "]"
                   : "null";
  }
};

std::string kv_json(const std::vector<srf::msgs::KeyValue>& properties) {
  std::string out = "{";
  for (std::size_t i = 0; i < properties.size(); ++i) {
    out += (i ? "," : "") + std::string("\"") + esc(properties[i].key) + "\":\"" + esc(properties[i].value) + "\"";
  }
  return out + "}";
}

}  // namespace

int main(int argc, char** argv) {
  if (qEnvironmentVariableIsEmpty("QT_QPA_PLATFORM")) {
    qputenv("QT_QPA_PLATFORM", "offscreen");  // 항상 헤드리스 (라벨 글씨용 폰트만 쓴다)
  }
  QGuiApplication application(argc, argv);

  std::string out_dir = "dataset";
  int count = 100;
  std::uint32_t seed = 1;
  int width = 640, height = 480;
  bool save_depth = false;
  std::string markers_path = SRF_RECYCLING_WS_MARKERS_PATH;
  int min_pixels = 40;  // 이보다 적게 보이면 visible=false
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    const auto value = [&]() -> std::string { return i + 1 < argc ? argv[++i] : ""; };
    if (arg == "--out") out_dir = value();
    else if (arg == "--count") count = std::stoi(value());
    else if (arg == "--seed") seed = static_cast<std::uint32_t>(std::stoul(value()));
    else if (arg == "--size") {
      const std::string v = value();
      width = std::stoi(v.substr(0, v.find(',')));
      height = std::stoi(v.substr(v.find(',') + 1));
    } else if (arg == "--depth") save_depth = true;
    else if (arg == "--markers") markers_path = value();
    else if (arg == "--min-pixels") min_pixels = std::stoi(value());
    else if (arg == "--headless") {  // 항상 헤드리스다 (옵션은 호환용)
    } else {
      std::printf(
          "usage: %s [--out DIR] [--count N] [--seed N] [--size W,H] [--depth] [--markers markers.yaml]\n"
          "          [--min-pixels N]   (항상 헤드리스)\n"
          "  출력: DIR/images/*.png, DIR/labels.jsonl (물품·지도 마커·구역 정답, 카메라)\n",
          argv[0]);
      return arg == "--help" || arg == "-h" ? 0 : 2;
    }
  }
  std::filesystem::create_directories(std::filesystem::path(out_dir) / "images");
  if (save_depth) std::filesystem::create_directories(std::filesystem::path(out_dir) / "depth");

  RecyclingWorld world;
  srf::msgs::MapMarkers markers =
      std::filesystem::exists(markers_path) ? srf::nodes::load_markers(markers_path) : world.default_markers();
  render::RobotMesh robot_mesh(SRF_RECYCLING_WS_URDF_PATH);
  robot_mesh.hide_link("end_effector_link");  // 실물 카메라와 같게

  std::mt19937 rng(seed);
  std::uniform_real_distribution<double> u(0.0, 1.0);
  const auto pick = [&](double p) { return u(rng) < p; };
  // 팔 자세: 접음·위로·앞으로 내림(그리퍼가 보인다)·옆 — 조금씩 흔든다
  const std::vector<std::array<double, 5>> arm_presets{
      {0.0, -1.75, 1.34, 0.90, 0.0},   // 접음
      {0.0, -1.0, 0.3, 0.7, 0.015},    // 위로
      {0.0, 1.3, -1.0, 0.7, 0.019},    // 앞으로 내림 (그리퍼가 화면에)
      {0.0, 0.9, -0.6, 1.0, 0.005},    // 집는 중
      {0.6, 0.4, -0.2, 0.9, 0.012},    // 옆
  };

  std::ofstream labels(std::filesystem::path(out_dir) / "labels.jsonl");
  const auto started = std::chrono::steady_clock::now();
  int written = 0;
  for (int sample = 0; sample < count; ++sample) {
    // 1) 장면
    SceneConfig scene;
    const double r = u(rng);
    // 모호(hold) 사례가 드물어 30%로 올린다. 부재는 vqa.py 의 무작위 대상 질문에서도 충분히 나온다.
    scene.scene = r < 0.35 ? Scene::Normal : r < 0.55 ? Scene::Multi : r < 0.85 ? Scene::Ambiguous : Scene::Absent;
    const std::array<Category, 3> kinds{Category::Can, Category::Pet, Category::Paper};
    if (pick(0.7)) scene.target = kinds[static_cast<std::size_t>(rng() % 3)];
    scene.items = 2 + static_cast<int>(rng() % 5);
    scene.distractors = static_cast<int>(rng() % 3);
    scene.people = static_cast<int>(rng() % 5);
    scene.forklifts = pick(0.6) ? 1 : 0;
    scene.lying_ratio = 0.3 + u(rng) * 0.4;
    scene.seed = seed * 100003U + static_cast<std::uint32_t>(sample);
    scene.auto_supply = false;
    world.randomize(scene);
    const int warm = static_cast<int>(u(rng) * 15.0 / 0.05);  // 사람·지게차가 움직인 뒤
    for (int i = 0; i < warm; ++i) {
      world.step(0.05);
      world.update();
    }
    const auto items = world.items();

    // 2) 로봇 자세: 매트를 보는 곳(70%) · 바구니 앞(20%) · 셀 안 아무 데(10%). 물품·바구니 위에 서지 않게.
    const auto& L = world.layout();
    srf::msgs::Pose2D robot;
    bool placed = false;
    for (int attempt = 0; attempt < 200 && !placed; ++attempt) {
      const double v = u(rng);
      if (v < 0.7) {
        const double azimuth = u(rng) * 2 * kPi;
        const double distance = 0.45 + u(rng) * 0.9;
        const double cx = L.work_area.center_x(), cy = L.work_area.center_y();
        robot.x = cx - distance * std::cos(azimuth);
        robot.y = cy - distance * std::sin(azimuth);
        robot.theta = azimuth + (u(rng) - 0.5) * 0.9;  // ±25°
      } else if (v < 0.9) {
        const auto& bin = L.bins[static_cast<std::size_t>(rng() % L.bins.size())];
        robot.x = bin.x - 0.45 - u(rng) * 0.9;
        robot.y = bin.y + (u(rng) - 0.5) * 1.6;
        robot.theta = std::atan2(bin.y - robot.y, bin.x - robot.x) + (u(rng) - 0.5) * 0.9;
      } else {
        robot.x = L.robot_cell.min_x + 0.4 + u(rng) * (L.robot_cell.max_x - L.robot_cell.min_x - 0.8);
        robot.y = L.robot_cell.min_y + 0.4 + u(rng) * (L.robot_cell.max_y - L.robot_cell.min_y - 0.8);
        robot.theta = u(rng) * 2 * kPi;
      }
      placed = L.robot_cell.contains(robot.x, robot.y, 0.25);
      for (const auto* item : items) {
        placed = placed && std::hypot(item->pose.x - robot.x, item->pose.y - robot.y) > item->radius + 0.2;
      }
      for (const auto& bin : L.bins) {
        placed = placed && std::hypot(bin.x - robot.x, bin.y - robot.y) > 0.35;
      }
    }
    if (!placed) continue;
    auto arm = arm_presets[static_cast<std::size_t>(rng() % arm_presets.size())];
    for (int j = 0; j < 4; ++j) arm[static_cast<std::size_t>(j)] += (u(rng) - 0.5) * 0.25;
    const std::map<std::string, double> joints{{"joint1", arm[0]}, {"joint2", arm[1]}, {"joint3", arm[2]},
                                               {"joint4", arm[3]}, {"gripper_left_joint", arm[4]},
                                               {"gripper_right_joint", arm[4]}};

    // 3) 조명·잡음 무작위화
    render::Lighting lighting;
    lighting.key = static_cast<float>(0.45 + u(rng) * 0.35);
    lighting.key_direction = {-0.6 + u(rng) * 1.2, -0.6 + u(rng) * 1.2, -1.0};
    const float ambient = static_cast<float>(0.8 + u(rng) * 0.4);
    for (auto& c : lighting.sky) c *= ambient;
    render::RenderOptions options;
    options.threads = 8;
    options.ids = true;
    options.noise = static_cast<float>(u(rng) * 0.015);
    options.seed = static_cast<std::uint32_t>(sample + 1);

    // 4) 그린다: 전체(보이는 것) + 물품만(가리지 않았을 때 넓이)
    render::Camera camera;
    camera.width = width;
    camera.height = height;
    camera.set_fov(kHorizontalFov);
    camera.world_from_optical = robot_mesh.link_pose("camera_color_optical_frame", robot, joints);
    std::vector<render::Instance> scene_instances;
    world.add_render_instances(scene_instances);
    robot_mesh.add_instances(robot, joints, scene_instances);
    render::Frame frame;
    render::render(camera, scene_instances, lighting, frame, options);
    std::vector<render::Instance> item_instances;
    for (const auto* item : items) item->add_render(item_instances, world.time());
    render::RenderOptions alone = options;  // 같은 슈퍼샘플이어야 픽셀 수를 비교할 수 있다
    alone.noise = 0.0F;
    render::Frame isolated;
    render::render(camera, item_instances, lighting, isolated, alone);

    const std::string stem = [&] {
      char name[32];
      std::snprintf(name, sizeof(name), "%06d", written + 1);
      return std::string(name);
    }();
    QImage(frame.rgb.data(), frame.width, frame.height, frame.width * 3, QImage::Format_RGB888)
        .save(QString::fromStdString((std::filesystem::path(out_dir) / "images" / (stem + ".png")).string()));
    if (save_depth) {
      QImage(reinterpret_cast<const uchar*>(frame.depth_mm.data()), frame.width, frame.height, frame.width * 2,
             QImage::Format_Grayscale16)
          .save(QString::fromStdString((std::filesystem::path(out_dir) / "depth" / (stem + ".png")).string()));
    }

    // 5) 물품 정답
    std::map<int, Box2> visible, full;
    for (int y = 0; y < frame.height; ++y) {
      for (int x = 0; x < frame.width; ++x) {
        const std::size_t i = static_cast<std::size_t>(y) * static_cast<std::size_t>(frame.width) + static_cast<std::size_t>(x);
        if (frame.ids[i] >= 0) visible[frame.ids[i]].add(x, y);
        if (isolated.ids[i] >= 0) full[isolated.ids[i]].add(x, y);
      }
    }
    const render::Pose optical_from_world = camera.world_from_optical.inverse();
    std::ostringstream line;
    line << "{\"image\":\"images/" << stem << ".png\"" << (save_depth ? ",\"depth\":\"depth/" + stem + ".png\"" : "")
         << ",\"width\":" << frame.width << ",\"height\":" << frame.height << ",\"camera\":{\"fx\":" << num(camera.fx, 3)
         << ",\"fy\":" << num(camera.fy, 3) << ",\"cx\":" << num(camera.cx, 3) << ",\"cy\":" << num(camera.cy, 3)
         << ",\"world_from_optical\":{\"t\":[" << num(camera.world_from_optical.t.x) << ","
         << num(camera.world_from_optical.t.y) << "," << num(camera.world_from_optical.t.z) << "],\"r\":[";
    for (int a = 0; a < 3; ++a)
      for (int b = 0; b < 3; ++b) line << (a + b ? "," : "") << num(camera.world_from_optical.r[a][b], 6);
    line << "]}},\"robot\":{\"x\":" << num(robot.x) << ",\"y\":" << num(robot.y) << ",\"yaw\":" << num(robot.theta)
         << ",\"arm\":[" << num(arm[0]) << "," << num(arm[1]) << "," << num(arm[2]) << "," << num(arm[3]) << ","
         << num(arm[4]) << "]},\"scene\":{\"type\":\"" << scene_name(scene.scene) << "\",\"target\":"
         << (scene.target ? "\"" + std::string(category_name(*scene.target)) + "\"" : "null")
         << ",\"seed\":" << scene.seed << ",\"people\":" << scene.people << ",\"forklifts\":" << scene.forklifts
         << ",\"light\":" << num(lighting.key, 3) << "},\"objects\":[";
    bool first = true;
    for (const auto* item : items) {
      const render::Vec3 c = optical_from_world.apply({item->pose.x, item->pose.y, item->z + item->height / 2});
      const Box2 vis = visible.count(item->id) ? visible[item->id] : Box2{};
      const Box2 all = full.count(item->id) ? full[item->id] : Box2{};
      const double ratio = all.pixels > 0 ? std::min(1.0, static_cast<double>(vis.pixels) / all.pixels) : 0.0;
      const bool truncated = all.valid() && (all.x0 == 0 || all.y0 == 0 || all.x1 == frame.width - 1 || all.y1 == frame.height - 1);
      line << (first ? "" : ",") << "{\"id\":" << item->id << ",\"category\":\"" << category_name(item->category)
           << "\",\"variant\":\"" << esc(item->variant) << "\",\"lying\":" << (item->lying ? "true" : "false")
           << ",\"world\":[" << num(item->pose.x) << "," << num(item->pose.y) << "," << num(item->z) << "],\"yaw\":"
           << num(item->pose.theta) << ",\"camera_xyz\":[" << num(c.x) << "," << num(c.y) << "," << num(c.z)
           << "],\"distance\":" << num(std::sqrt(c.x * c.x + c.y * c.y + c.z * c.z)) << ",\"bbox\":" << vis.json()
           << ",\"bbox_full\":" << all.json() << ",\"pixels\":" << vis.pixels << ",\"visible_ratio\":" << num(ratio, 3)
           << ",\"visible\":" << (vis.pixels >= min_pixels ? "true" : "false") << ",\"truncated\":"
           << (truncated ? "true" : "false") << "}";
      first = false;
    }

    // 6) 지도 마커: 3D 상자 투영 + 깊이 비교 (꼭짓점 8 + 중심)
    line << "],\"markers\":[";
    first = true;
    for (const auto& m : markers.markers) {
      const double sx = std::max(m.scale.x, 0.06), sy = std::max(m.scale.y, 0.06), sz = std::max(m.scale.z, 0.06);
      const auto& q = m.pose.orientation;
      const double yaw = std::atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z));
      Box2 box;
      int seen = 0, inside = 0, samples = 0;
      for (int a = -1; a <= 1; ++a) {
        for (int b = -1; b <= 1; ++b) {
          for (int c3 = -1; c3 <= 1; c3 += 2) {
            if ((a == 0) != (b == 0)) continue;  // 꼭짓점 8 + 중심 2
            const double lx = a * sx / 2, ly = b * sy / 2;
            const render::Vec3 w{m.pose.position.x + lx * std::cos(yaw) - ly * std::sin(yaw),
                                 m.pose.position.y + lx * std::sin(yaw) + ly * std::cos(yaw),
                                 std::max(0.0, m.pose.position.z + c3 * sz / 2)};
            const render::Vec3 p = optical_from_world.apply(w);
            ++samples;
            if (p.z < 0.05) continue;
            const int px = static_cast<int>(camera.fx * p.x / p.z + camera.cx);
            const int py = static_cast<int>(camera.fy * p.y / p.z + camera.cy);
            if (px < 0 || py < 0 || px >= frame.width || py >= frame.height) continue;
            ++inside;
            box.add(px, py);
            const auto depth = frame.depth_mm[static_cast<std::size_t>(py) * static_cast<std::size_t>(frame.width) + static_cast<std::size_t>(px)];
            if (depth == 0 || depth / 1000.0 >= p.z - 0.06) ++seen;  // 앞에 가리는 것이 없다
          }
        }
      }
      line << (first ? "" : ",") << "{\"name\":\"" << esc(m.name) << "\",\"kind\":\"" << esc(m.kind)
           << "\",\"properties\":" << kv_json(m.properties) << ",\"bbox\":" << (inside > 0 ? box.json() : "null")
           << ",\"visible_ratio\":" << num(samples ? static_cast<double>(seen) / samples : 0.0, 3)
           << ",\"in_view\":" << (inside > 0 ? "true" : "false") << "}";
      first = false;
    }

    // 7) 지도 구역: 바닥 픽셀(깊이 → 월드 z≈0)이 어느 구역인가
    line << "],\"zones\":[";
    std::vector<Box2> zone_boxes(markers.zones.size());
    int floor_pixels = 0;
    for (int y = 0; y < frame.height; y += 2) {
      for (int x = 0; x < frame.width; x += 2) {
        const auto depth = frame.depth_mm[static_cast<std::size_t>(y) * static_cast<std::size_t>(frame.width) + static_cast<std::size_t>(x)];
        if (depth == 0) continue;
        const double z = depth / 1000.0;
        const render::Vec3 w = camera.world_from_optical.apply({(x - camera.cx) * z / camera.fx, (y - camera.cy) * z / camera.fy, z});
        if (std::abs(w.z) > 0.03) continue;  // 바닥만
        ++floor_pixels;
        for (std::size_t k = 0; k < markers.zones.size(); ++k) {
          if (srf::nodes::zone_contains(markers.zones[k], w.x, w.y)) zone_boxes[k].add(x, y);
        }
      }
    }
    for (std::size_t k = 0; k < markers.zones.size(); ++k) {
      const auto& z = markers.zones[k];
      line << (k ? "," : "") << "{\"name\":\"" << esc(z.name) << "\",\"kind\":\"" << esc(z.kind) << "\",\"shape\":\""
           << z.shape << "\",\"properties\":" << kv_json(z.properties) << ",\"bbox\":" << zone_boxes[k].json()
           << ",\"floor_ratio\":" << num(floor_pixels ? static_cast<double>(zone_boxes[k].pixels) / floor_pixels : 0.0, 3)
           << "}";
    }
    line << "],\"truth\":" << world.truth_json(world.time()) << "}";
    labels << line.str() << "\n";
    ++written;
    if (written % 20 == 0 || written == count) {
      const double elapsed = std::chrono::duration<double>(std::chrono::steady_clock::now() - started).count();
      std::printf("[dataset] %d/%d (%.2f s/장)\n", written, count, elapsed / written);
      std::fflush(stdout);
    }
  }
  std::printf("[dataset] %d장 → %s/labels.jsonl\n", written, out_dir.c_str());
  return 0;
}
