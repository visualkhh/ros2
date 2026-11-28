#pragma once

// 진실 화면: 로봇 카메라(D435 스텁)와 같은 3D 렌더러·같은 장면 데이터로 그린다.
// (srf 코어 SimulatorView는 로봇을 URDF 상자·원기둥으로, 물품을 원기둥으로 그려 카메라와 달라 보인다.)
//   드래그: 회전 · 오른쪽 드래그: 이동 · 휠: 줌 · W/A/S/D: 주행(cmd_vel_teleop) · Q/Z: 속도
//   F: 로봇 따라가기 · L: 라이다 · C: 전경
#include <QElapsedTimer>
#include <QImage>
#include <QMainWindow>
#include <QSet>
#include <QWidget>

#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "render/scene_renderer.hpp"
#include "srf/simulator/snapshot.hpp"

class QTimer;

namespace srf {
class Node;
}

namespace srf_recycling_ws::sim {

// 시뮬레이션 스레드가 채우고(30Hz) 화면 스레드가 복사해 그린다. 메시는 불변이라 복사가 싸다.
struct SharedScene {
  std::mutex mutex;
  std::vector<render::Instance> instances;
  double next_time = 0.0;
};

class RecyclingView final : public QWidget {
 public:
  RecyclingView(SharedScene& scene, srf::simulator::SnapshotStore& snapshots, srf::Node& node,
                std::string teleop_topic, QWidget* parent = nullptr);
  [[nodiscard]] static QString help_text();
  void reset_camera();

 protected:
  void paintEvent(QPaintEvent* event) override;
  void mousePressEvent(QMouseEvent* event) override;
  void mouseMoveEvent(QMouseEvent* event) override;
  void mouseReleaseEvent(QMouseEvent* event) override;
  void wheelEvent(QWheelEvent* event) override;
  void keyPressEvent(QKeyEvent* event) override;
  void keyReleaseEvent(QKeyEvent* event) override;
  void focusOutEvent(QFocusEvent* event) override;

 private:
  void publish_teleop();
  [[nodiscard]] render::Camera make_camera(int width, int height) const;

  SharedScene& scene_;
  srf::simulator::SnapshotStore& snapshots_;
  srf::Node& node_;
  std::string teleop_topic_;
  std::vector<render::Instance> instances_;
  render::Frame frame_;
  render::Lighting lighting_;

  double yaw_ = -2.2;     // 카메라가 대상을 보는 방위 (라디안)
  double pitch_ = 0.55;   // 내려다보는 각
  double distance_ = 2.2;
  render::Vec3 target_{0, 0, 0.15};
  bool follow_ = true;
  bool show_scan_ = true;
  QPoint last_mouse_;
  bool orbiting_ = false;
  bool panning_ = false;
  QSet<int> keys_;
  double teleop_scale_ = 1.0;
  double last_linear_ = 0.0;
  double last_angular_ = 0.0;
  QTimer* teleop_timer_ = nullptr;
  QElapsedTimer fps_clock_;
  int frames_ = 0;
  double fps_ = 0.0;
  double render_ms_ = 0.0;
};

// 진실 화면 + 오른쪽 패널(카메라·팔).
class RecyclingWindow final : public QMainWindow {
 public:
  RecyclingWindow(SharedScene& scene, srf::simulator::SnapshotStore& snapshots, srf::Node& node,
                  std::string teleop_topic, std::vector<QWidget*> side_panels);
  [[nodiscard]] RecyclingView& view() { return *view_; }

 private:
  RecyclingView* view_ = nullptr;
};

}  // namespace srf_recycling_ws::sim
