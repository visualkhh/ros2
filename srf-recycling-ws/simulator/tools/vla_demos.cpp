// VLA 시범 데이터 생성기 (헤드리스): 시뮬레이터 "조교"가 진실값으로 바퀴·팔·그리퍼·부저를 움직이고,
// 4 Hz 마다 로봇이 실제로 받는 관측(RGB·깊이·모터 상태·odom·라이다)과 조교의 행동을 기록한다.
//
//   srf_recycling_ws_vla_demos --out DIR --episodes 200 --seed 1
//
// - 명령은 실기와 같은 OpenCrDevice(SimulatorOpenCr)로 내리고, 영상은 SimulatorRealSense 로 받는다
//   → 엔진 vla_node 가 실기에서 받는 값과 같은 형식.
// - 회차: pick (작업 매트에서 대상에 다가가 집어 듦) → 성공하면 이어서 place (해당 바구니 앞으로 옮겨진 뒤
//   다가가 바구니 위에서 놓음). 바구니 앞까지의 이동은 Nav 몫이라 순간이동으로 대신한다.
// - DART: 일부 회차는 실행 행동에 잡음을 넣고 라벨은 조교의 깨끗한 행동 → 틀어졌을 때 되돌아오는 법을 배운다.
// - 출력: DIR/episodes/NNNNNN_pick/{rgb/KKK.jpg, depth/KKK.png, steps.jsonl}, DIR/episodes.jsonl
#include <QImage>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <memory>
#include <optional>
#include <random>
#include <sstream>
#include <string>
#include <vector>

#include "hardwares/opencr/simulator_opencr.hpp"
#include "hardwares/realsense/simulator_realsense.hpp"
#include "robot/waffle_pi_arm.hpp"
#include "srf/manipulation/arm_chain.hpp"
#include "srf/urdf.hpp"
#include "world/agents.hpp"
#include "world/recycling_world.hpp"

namespace {

using srf_recycling_ws::sim::Category;
using srf_recycling_ws::sim::RecyclableItem;
using srf_recycling_ws::sim::RecyclingWorld;
using srf_recycling_ws::sim::WafflePiArm;

constexpr double kPi = 3.14159265358979323846;
constexpr double kDt = 0.01;          // 시뮬레이션 스텝
constexpr int kTicksPerAction = 25;   // 4 Hz (vla_node 주기)
constexpr double kActionPeriod = kDt * kTicksPerAction;
constexpr double kJointStep = 0.20;   // 행동 한 번에 관절 최대 변화 (0.8 rad/s)
constexpr double kGripStep = 0.006;   // 그리퍼 (m, 24 mm/s)
constexpr double kGripOpen = 0.019;
constexpr double kGripClose = -0.005;
constexpr double kBaseLinkZ = 0.010;  // base_footprint → base_link
const std::array<double, 4> kReady{0.0, -1.0, 0.3, 0.7};  // 들고 다니는 자세 (카메라 시야를 가리지 않는다)

double clampd(double v, double lo, double hi) { return std::max(lo, std::min(hi, v)); }
double wrap(double a) { return std::atan2(std::sin(a), std::cos(a)); }

struct Base {  // 월드 → base_link 2D
  double x, y, c, s;
  explicit Base(const srf::msgs::Pose2D& p) : x(p.x), y(p.y), c(std::cos(p.theta)), s(std::sin(p.theta)) {}
  [[nodiscard]] std::array<double, 2> local(double wx, double wy) const {
    const double dx = wx - x, dy = wy - y;
    return {c * dx + s * dy, -s * dx + c * dy};
  }
};

std::string json_array(const double* v, std::size_t n, int precision = 4) {
  std::ostringstream o;
  o.setf(std::ios::fixed);
  o.precision(precision);
  o << '[';
  for (std::size_t i = 0; i < n; ++i) o << (i ? "," : "") << v[i];
  o << ']';
  return o.str();
}

std::string escape(const std::string& s) {
  std::string out;
  for (const char ch : s) {
    if (ch == '"' || ch == '\\') out += '\\';
    out += ch;
  }
  return out;
}

struct Action {
  double v = 0.0, w = 0.0;
  std::array<double, 4> joints{};
  double gripper = 0.0;
  int buzzer = 0;
};

struct Expert {
  const srf::manipulation::ArmChain& chain;

  // TCP 목표(base_link) → 관절. 실패하면 여러 하향 각도로 다시.
  [[nodiscard]] std::optional<std::array<double, 4>> ik(double x, double y, double z, double pitch_deg,
                                                        const std::array<double, 4>& seed) const {
    for (const double p : {pitch_deg, pitch_deg + 10.0, pitch_deg - 10.0, pitch_deg + 20.0, pitch_deg - 20.0}) {
      std::array<double, 4> q{};
      std::string error;
      if (chain.ik({x, y, z}, p * kPi / 180.0, seed, q, error)) return q;
    }
    return std::nullopt;
  }
};

void step_toward(std::array<double, 4>& from, const std::array<double, 4>& goal) {
  for (std::size_t i = 0; i < 4; ++i) from[i] += clampd(goal[i] - from[i], -kJointStep, kJointStep);
}

bool reached(const std::array<double, 5>& q, const std::array<double, 4>& goal, double grip, double tol = 0.03) {
  for (std::size_t i = 0; i < 4; ++i)
    if (std::abs(q[i] - goal[i]) > tol) return false;
  return std::abs(q[4] - grip) < 0.002;
}

const char* category_word(Category c, std::mt19937& rng) {
  static const std::vector<const char*> can{"can", "aluminum can", "drink can"};
  static const std::vector<const char*> pet{"PET bottle", "plastic bottle"};
  static const std::vector<const char*> paper{"paper", "paper box", "carton"};
  const auto& words = c == Category::Can ? can : c == Category::Pet ? pet : paper;
  return words[std::uniform_int_distribution<std::size_t>(0, words.size() - 1)(rng)];
}

}  // namespace

int main(int argc, char** argv) {
  qputenv("QT_QPA_PLATFORM", "offscreen");
  std::string out_dir = "vla_demos";
  int episodes = 20;
  std::uint32_t seed = 1;
  std::uint32_t width = 424, height = 240;
  double dart_ratio = 0.5;
  int max_steps = 140;  // 35초
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    if (arg == "--out" && i + 1 < argc) out_dir = argv[++i];
    else if (arg == "--episodes" && i + 1 < argc) episodes = std::stoi(argv[++i]);
    else if (arg == "--seed" && i + 1 < argc) seed = static_cast<std::uint32_t>(std::stoul(argv[++i]));
    else if (arg == "--camera" && i + 1 < argc) {
      const std::string size = argv[++i];
      width = static_cast<std::uint32_t>(std::stoul(size.substr(0, size.find('x'))));
      height = static_cast<std::uint32_t>(std::stoul(size.substr(size.find('x') + 1)));
    } else if (arg == "--dart" && i + 1 < argc) dart_ratio = std::stod(argv[++i]);
    else if (arg == "--max-steps" && i + 1 < argc) max_steps = std::stoi(argv[++i]);
    else if (arg == "-h" || arg == "--help") {
      std::cout << "usage: srf_recycling_ws_vla_demos --out DIR [--episodes 20] [--seed 1] [--camera 424x240]\n"
                   "       [--dart 0.5 (잡음 넣는 회차 비율)] [--max-steps 140]\n";
      return 0;
    }
  }

  std::ifstream urdf_file(SRF_RECYCLING_WS_URDF_PATH);
  std::stringstream urdf_text;
  urdf_text << urdf_file.rdbuf();
  std::string chain_error;
  const auto chain = srf::manipulation::ArmChain::from_urdf(srf::parse_urdf(urdf_text.str()), "base_link",
                                                            "end_effector_link",
                                                            {"joint1", "joint2", "joint3", "joint4"}, chain_error);
  if (!chain) {
    std::cerr << "팔 체인 실패: " << chain_error << "\n";
    return 1;
  }
  const Expert expert{*chain};

  RecyclingWorld world;
  auto* robot = static_cast<WafflePiArm*>(world.add(
      std::make_unique<WafflePiArm>(true, seed, SRF_RECYCLING_WS_URDF_PATH, SRF_RECYCLING_WS_SRDF_PATH),
      world.layout().robot_start));
  robot->set_manual_arm(false);
  srf_recycling_ws::sim::SimulatorOpenCr opencr(*robot, true, seed);
  srf_recycling_ws::sim::SimulatorRealSense camera(*robot, SRF_RECYCLING_WS_URDF_PATH, true, seed, width, height);
  camera.start();

  std::filesystem::create_directories(std::filesystem::path(out_dir) / "episodes");
  std::ofstream index(std::filesystem::path(out_dir) / "episodes.jsonl");
  std::mt19937 rng(seed);
  std::uniform_real_distribution<double> u(0.0, 1.0);
  std::normal_distribution<double> gauss(0.0, 1.0);
  const auto started = std::chrono::steady_clock::now();
  int written = 0, successes = 0;
  std::array<double, 5> commanded{};  // 마지막으로 보낸 팔 목표 (컨트롤러 보간 시작점)

  // 행동 하나를 0.25초 동안 실행한다: 바퀴 twist 는 그대로, 팔은 궤적 컨트롤러처럼 목표까지 선형 보간.
  const auto execute = [&](const Action& a) {
    srf::msgs::Twist twist;
    twist.linear.x = a.v;
    twist.angular.z = a.w;
    opencr.write(twist);
    const auto from = commanded;
    const std::array<double, 5> to{a.joints[0], a.joints[1], a.joints[2], a.joints[3], a.gripper};
    for (int t = 1; t <= kTicksPerAction; ++t) {
      std::array<double, 5> q{};
      for (std::size_t i = 0; i < 5; ++i) q[i] = from[i] + (to[i] - from[i]) * t / kTicksPerAction;
      opencr.write_arm(q);
      world.step(kDt);
      world.update();
    }
    commanded = to;
  };

  const auto set_arm_now = [&](const std::array<double, 5>& q) {
    commanded = q;
    opencr.write_arm(q);
    world.step(kDt);
  };

  const auto nearest_person = [&]() {
    double best = 1e9;
    for (const auto& object : world.objects()) {
      if (dynamic_cast<const srf_recycling_ws::sim::Worker*>(object.get()) != nullptr) {
        best = std::min(best, std::hypot(object->pose.x - robot->pose.x, object->pose.y - robot->pose.y));
      }
    }
    return best;
  };

  // 한 회차 실행 + 기록. kind: "pick" | "place". 반환: 성공 여부
  const auto run_episode = [&](const std::string& kind, const RecyclableItem& item, const std::string& instruction,
                               bool dart) -> bool {
    char name[64];
    std::snprintf(name, sizeof(name), "%06d_%s", written + 1, kind.c_str());
    const auto dir = std::filesystem::path(out_dir) / "episodes" / name;
    std::filesystem::create_directories(dir / "rgb");
    std::filesystem::create_directories(dir / "depth");
    std::ofstream steps(dir / "steps.jsonl");
    const auto* bin = [&]() -> const srf_recycling_ws::sim::BinSpec* {
      for (const auto& b : world.layout().bins)
        if (b.category == item.category) return &b;
      return nullptr;
    }();

    enum class Phase { Approach, Pregrasp, Descend, Close, Lift, Carry, OverBin, Open, Retreat, Done };
    Phase phase = Phase::Approach;
    const char* phase_names[] = {"approach", "pregrasp", "descend", "close", "lift", "carry", "over_bin", "open", "retreat", "done"};
    std::array<double, 4> goal = kReady;
    double grip_goal = kind == "pick" ? kGripOpen : kGripClose;
    std::string reason = "timeout";
    bool success = false;
    int stuck = 0, settle = 0;
    double last_progress = 1e9;
    Phase last_phase = phase;
    std::array<double, 3> grasp{};  // 닫기 시작할 때의 물체 위치 (들어 올리면 물체가 손과 같이 움직인다)
    double person_was = nearest_person();
    // 대상 위치 추정 잡음 (VLM bbox + 깊이 + TF 오차를 흉내, 회차마다 고정 편향 + 스텝 잡음)
    const double bias_x = gauss(rng) * 0.008, bias_y = gauss(rng) * 0.008;

    for (int k = 0; k < max_steps && phase != Phase::Done; ++k) {
      // ---- 관측 (행동 전) ----
      const auto status = opencr.arm_status();
      const Base base(robot->pose);
      const double wheel_l = opencr.left_velocity(), wheel_r = opencr.right_velocity();
      const double odom_v = WafflePiArm::kWheelRadius * (wheel_l + wheel_r) / 2;
      const double odom_w = WafflePiArm::kWheelRadius * (wheel_r - wheel_l) / WafflePiArm::kWheelSeparation;
      const auto scan = robot->state().scan_ranges;
      double front = 0.0;
      if (!scan.empty()) {
        front = 1e9;
        const int n = static_cast<int>(scan.size());
        for (int b = -n / 18; b <= n / 18; ++b) {  // 앞 ±20°
          const float r = scan[static_cast<std::size_t>((b + n) % n)];
          if (std::isfinite(r) && r > 0.0F) front = std::min(front, static_cast<double>(r));
        }
      }
      const std::array<double, 4> q4{status.position[0], status.position[1], status.position[2], status.position[3]};
      const auto tcp = chain->tcp(q4);
      const auto item_local = base.local(item.pose.x, item.pose.y);
      const double item_center = std::max(0.02, item.z + item.height / 2) - kBaseLinkZ;
      const auto bin_local = bin ? base.local(bin->x, bin->y) : std::array<double, 2>{0, 0};
      std::array<double, 3> target_truth{}, target_seen{};
      if (kind == "pick") {
        target_truth = {item_local[0], item_local[1], item_center};
      } else {
        target_truth = {bin_local[0], bin_local[1], (bin ? bin->height : 0.1) - kBaseLinkZ};
      }
      target_seen = {target_truth[0] + bias_x + gauss(rng) * 0.004, target_truth[1] + bias_y + gauss(rng) * 0.004,
                     target_truth[2] + gauss(rng) * 0.004};
      const double person = nearest_person();

      char rgb_name[32], depth_name[32];
      std::snprintf(rgb_name, sizeof(rgb_name), "rgb/%03d.jpg", k);
      std::snprintf(depth_name, sizeof(depth_name), "depth/%03d.png", k);
      if (const auto color = camera.snapshot_color()) {
        QImage(color->data.data(), static_cast<int>(color->width), static_cast<int>(color->height),
               static_cast<int>(color->step), QImage::Format_RGB888)
            .save(QString::fromStdString((dir / rgb_name).string()), "JPG", 90);
      }
      if (const auto depth = camera.snapshot_depth()) {
        QImage(depth->data.data(), static_cast<int>(depth->width), static_cast<int>(depth->height),
               static_cast<int>(depth->step), QImage::Format_Grayscale16)
            .save(QString::fromStdString((dir / depth_name).string()), "PNG");
      }

      // ---- 조교 행동 (진실값, 닫힌 루프) ----
      Action a;
      a.gripper = grip_goal;
      a.buzzer = (k == 0 || (person < 1.5 && person_was >= 1.5)) ? 1 : 0;  // 시작 알림 · 사람이 다가오면
      person_was = person;
      const double standoff = kind == "pick" ? 0.20 : 0.30;  // 대상(바구니 중심)을 이 거리 앞에 둔다
      const double tx = kind == "pick" ? item_local[0] : bin_local[0];
      const double ty = kind == "pick" ? item_local[1] : bin_local[1];
      switch (phase) {
        case Phase::Approach: {
          goal = kReady;
          const double bearing = std::atan2(ty, tx + 0.08);  // 팔 어깨 기준
          a.w = clampd(2.0 * bearing, -0.5, 0.5);
          a.v = std::abs(bearing) < 0.3 ? clampd(1.0 * (tx - standoff), -0.06, 0.08) : 0.0;
          if (std::abs(tx - standoff) < 0.025 && std::abs(ty) < 0.02 && reached(status.position, kReady, grip_goal, 0.05)) {
            a.v = a.w = 0.0;
            phase = kind == "pick" ? Phase::Pregrasp : Phase::OverBin;
          }
          break;
        }
        case Phase::Pregrasp:
        case Phase::Descend:
        case Phase::Lift: {
          const double dz = phase == Phase::Pregrasp ? 0.07 : phase == Phase::Descend ? 0.015 : 0.12;
          const double pitch = phase == Phase::Pregrasp ? 35.0 : phase == Phase::Descend ? 45.0 : 20.0;
          if (phase != Phase::Lift) grasp = {item_local[0], item_local[1], item_center};
          const auto q = expert.ik(grasp[0], grasp[1], grasp[2] + dz, pitch, q4);
          if (!q) {
            reason = "ik";
            phase = Phase::Done;
            break;
          }
          goal = *q;
          if (reached(status.position, goal, grip_goal)) {
            phase = phase == Phase::Pregrasp ? Phase::Descend : phase == Phase::Descend ? Phase::Close : Phase::Carry;
          }
          break;
        }
        case Phase::Close:
          grip_goal = kGripClose;
          a.gripper = grip_goal;
          if (robot->holding() && std::abs(status.position[4] - kGripClose) < 0.002) phase = Phase::Lift;
          else if (std::abs(status.position[4] - kGripClose) < 0.002) {
            reason = "missed";
            phase = Phase::Done;
          }
          break;
        case Phase::Carry:
          goal = kReady;
          if (reached(status.position, kReady, grip_goal)) {
            success = robot->holding();
            reason = success ? "ok" : "lost";
            phase = Phase::Done;
          }
          break;
        case Phase::OverBin: {
          const double drop_x = bin_local[0] - 0.06, drop_y = bin_local[1];  // 바구니 안쪽 가까운 절반
          const auto q = expert.ik(drop_x, drop_y, (bin ? bin->height : 0.1) + 0.08 - kBaseLinkZ, 45.0, q4);
          if (!q) {
            reason = "ik";
            phase = Phase::Done;
            break;
          }
          goal = *q;
          if (reached(status.position, goal, grip_goal)) phase = Phase::Open;
          break;
        }
        case Phase::Open:
          grip_goal = kGripOpen;
          a.gripper = grip_goal;
          if (!robot->holding() && std::abs(status.position[4] - kGripOpen) < 0.002) phase = Phase::Retreat;
          break;
        case Phase::Retreat:
          goal = kReady;
          if (reached(status.position, kReady, grip_goal) && ++settle > 2) {
            const auto* in = world.bin_containing(item);
            success = in != nullptr && in->category == item.category && !item.dropped;
            reason = success ? "ok" : in == nullptr ? "missed_bin" : "wrong_bin";
            phase = Phase::Done;
          }
          break;
        case Phase::Done: break;
      }
      // 진행이 없으면 (충돌로 막힘 등) 실패. 단계가 바뀌면 다시 잰다.
      if (phase != last_phase) {
        last_phase = phase;
        last_progress = 1e9;
        stuck = 0;
      }
      double progress = phase == Phase::Approach ? std::abs(tx - standoff) + std::abs(ty) : 0.0;
      for (std::size_t i = 0; i < 4; ++i) progress += std::abs(status.position[i] - goal[i]);
      progress += std::abs(status.position[4] - grip_goal) * 10;
      stuck = progress < last_progress - 0.005 ? 0 : stuck + 1;
      last_progress = std::min(last_progress, progress);
      if (stuck > 16 && phase != Phase::Done) {
        reason = "stuck";
        phase = Phase::Done;
      }
      if (phase == Phase::Done) a.v = a.w = 0.0;
      a.joints = q4;
      step_toward(a.joints, goal);  // 한 번에 0.2 rad 까지

      // ---- 기록 ----
      std::ostringstream line;
      line.setf(std::ios::fixed);
      line.precision(4);
      const double ee[5] = {tcp.x, tcp.y, tcp.z, chain->pitch_down(q4), chain->tool_yaw(q4)};
      const double action_joints[5] = {a.joints[0], a.joints[1], a.joints[2], a.joints[3], a.gripper};
      const double robot_pose[3] = {robot->pose.x, robot->pose.y, robot->pose.theta};
      const double bin_xyz[3] = {bin_local[0], bin_local[1], (bin ? bin->height : 0.1) - kBaseLinkZ};
      line << "{\"step\":" << k << ",\"time\":" << world.time() << ",\"phase\":\"" << phase_names[static_cast<int>(phase)]
           << "\",\"rgb\":\"" << rgb_name << "\",\"depth\":\"" << depth_name << "\",\"state\":{"
           << "\"odom\":{\"v\":" << odom_v << ",\"w\":" << odom_w << "},"
           << "\"wheels\":{\"left\":" << wheel_l << ",\"right\":" << wheel_r << "},"
           << "\"arm\":{\"position\":" << json_array(status.position.data(), 5)
           << ",\"velocity\":" << json_array(status.velocity.data(), 5)
           << ",\"current\":" << json_array(status.current.data(), 5) << "},"
           << "\"ee\":" << json_array(ee, 5) << ",\"scan_front_min\":" << front
           << ",\"target_xyz\":" << json_array(target_seen.data(), 3) << ",\"bin_xyz\":" << json_array(bin_xyz, 3)
           << "},\"truth\":{\"target_xyz\":" << json_array(target_truth.data(), 3)
           << ",\"holding\":" << (robot->holding() ? "true" : "false") << ",\"blocked\":\""
           << escape(robot->arm_blocked_by()) << "\",\"person_min\":" << std::min(person, 99.0)
           << ",\"robot\":" << json_array(robot_pose, 3) << "},"
           << "\"action\":{\"v\":" << a.v << ",\"w\":" << a.w << ",\"joints\":" << json_array(action_joints, 4)
           << ",\"gripper\":" << a.gripper << ",\"buzzer\":" << a.buzzer << "}}\n";
      steps << line.str();

      // ---- 실행 (DART: 잡음 섞은 행동을 실행, 라벨은 위의 깨끗한 행동) ----
      Action run = a;
      if (dart && u(rng) < 0.3) {  // 가끔 틀어지게 (매 스텝이면 목표에 끝내 못 닿는다)
        run.v += gauss(rng) * 0.01;
        run.w += gauss(rng) * 0.06;
        for (auto& j : run.joints) j += gauss(rng) * 0.03;
      }
      execute(run);
    }
    opencr.write(srf::msgs::Twist{});
    ++written;
    successes += success ? 1 : 0;
    index << "{\"episode\":\"" << name << "\",\"kind\":\"" << kind << "\",\"instruction\":\"" << escape(instruction)
          << "\",\"category\":\"" << srf_recycling_ws::sim::category_name(item.category) << "\",\"variant\":\""
          << item.variant << "\",\"lying\":" << (item.lying ? "true" : "false") << ",\"success\":"
          << (success ? "true" : "false") << ",\"reason\":\"" << reason << "\",\"dart\":" << (dart ? "true" : "false")
          << "}\n";
    index.flush();
    return success;
  };

  for (int e = 0; written < episodes; ++e) {
    // ---- 장면 ----
    robot->release_all();
    srf_recycling_ws::sim::SceneConfig scene;
    scene.scene = u(rng) < 0.6 ? srf_recycling_ws::sim::Scene::Normal : srf_recycling_ws::sim::Scene::Multi;
    const std::array<Category, 3> kinds{Category::Can, Category::Pet, Category::Paper};
    scene.target = kinds[std::uniform_int_distribution<std::size_t>(0, 2)(rng)];
    scene.items = 2 + static_cast<int>(u(rng) * 4);
    scene.distractors = static_cast<int>(u(rng) * 3);
    scene.people = static_cast<int>(u(rng) * 4);
    scene.forklifts = u(rng) < 0.5 ? 1 : 0;
    scene.auto_supply = scene.auto_empty = false;
    scene.seed = seed * 7919U + static_cast<std::uint32_t>(e);
    world.randomize(scene);

    const RecyclableItem* target = nullptr;
    for (const auto* item : world.items()) {
      if (item->category == *scene.target && (!target || u(rng) < 0.5)) target = item;
    }
    if (target == nullptr) continue;

    // ---- 로봇: 대상에서 0.45~0.85 m, 방향 ±35° 틀어진 곳 (Nav 가 데려다 놓은 자리 흉내) ----
    bool placed = false;
    for (int attempt = 0; attempt < 100 && !placed; ++attempt) {
      const double az = u(rng) * 2 * kPi, d = 0.45 + u(rng) * 0.4;
      srf::msgs::Pose2D p{target->pose.x - d * std::cos(az), target->pose.y - d * std::sin(az),
                          wrap(az + (u(rng) - 0.5) * 2 * 0.6)};
      placed = true;
      for (const auto* item : world.items())
        if (std::hypot(item->pose.x - p.x, item->pose.y - p.y) < 0.2) placed = false;
      if (placed) robot->pose = p;
    }
    if (!placed) continue;
    const auto& r = u(rng) < 0.5 ? WafflePiArm::kStow : std::array<double, 5>{kReady[0], kReady[1], kReady[2], kReady[3], kGripOpen};
    set_arm_now(r);
    for (int t = 0; t < 30; ++t) world.step(kDt);  // 카메라 첫 프레임

    const std::string word = category_word(target->category, rng);
    if (!run_episode("pick", *target, "pick up the " + word, u(rng) < dart_ratio) || written >= episodes) continue;

    // ---- place: 해당 바구니 앞으로 (Nav 대신 순간이동) ----
    const auto* bin = [&]() -> const srf_recycling_ws::sim::BinSpec* {
      for (const auto& b : world.layout().bins)
        if (b.category == target->category) return &b;
      return nullptr;
    }();
    const double d = 0.5 + u(rng) * 0.3, heading = (u(rng) - 0.5) * 2 * 0.5;
    robot->pose = {bin->x - d * std::cos(heading) - 0.0, bin->y - d * std::sin(heading) + (u(rng) - 0.5) * 0.2,
                   wrap(heading + (u(rng) - 0.5) * 0.6)};
    for (int t = 0; t < 30; ++t) world.step(kDt);
    run_episode("place", *target, std::string("put the ") + word + " in the " +
                                      srf_recycling_ws::sim::category_name(target->category) + " bin",
                u(rng) < dart_ratio);
    if (written % 10 == 0) {
      const double secs = std::chrono::duration<double>(std::chrono::steady_clock::now() - started).count();
      std::printf("[vla_demos] %d회 (성공 %d) %.1f s/회\n", written, successes, secs / written);
      std::fflush(stdout);
    }
  }
  const double secs = std::chrono::duration<double>(std::chrono::steady_clock::now() - started).count();
  std::printf("[vla_demos] %d회 (성공 %d) → %s  %.1f s/회\n", written, successes, out_dir.c_str(), secs / std::max(1, written));
  return 0;
}
