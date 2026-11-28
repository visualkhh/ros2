#include "plugins/moveit_plugin.hpp"

#include <QCheckBox>
#include <QDockWidget>
#include <QDoubleSpinBox>
#include <QGridLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QMainWindow>
#include <QPushButton>
#include <QSignalBlocker>
#include <QSlider>
#include <QTimer>
#include <QVBoxLayout>
#include <QWidget>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <map>
#include <sstream>
#include <string>

#include "srf/tf/math.hpp"
#include "srf/tf/math.hpp"
#include "srf/msgs.hpp"
#include "srf/qos.hpp"
#include "srf/viz/scene_view.hpp"
#include "srf/viz/viz_model.hpp"

namespace srf_default_ws::viz {

namespace {
double steady_seconds() {
  using namespace std::chrono;
  return duration<double>(steady_clock::now().time_since_epoch()).count();
}

std::map<std::string, std::string> parse_state_fields(const std::string& text) {
  std::map<std::string, std::string> fields;
  std::stringstream stream(text);
  std::string item;
  while (std::getline(stream, item, ';')) {
    const std::size_t equals = item.find('=');
    if (equals == std::string::npos) {
      continue;
    }
    fields[item.substr(0, equals)] = item.substr(equals + 1);
  }
  return fields;
}

bool parse_field_double(const std::map<std::string, std::string>& fields, const std::string& key, double& value) {
  const auto found = fields.find(key);
  if (found == fields.end()) {
    return false;
  }
  try {
    const double parsed = std::stod(found->second);
    if (!std::isfinite(parsed)) {
      return false;
    }
    value = parsed;
    return true;
  } catch (...) {
    return false;
  }
}

bool parse_field_int(const std::map<std::string, std::string>& fields, const std::string& key, int& value) {
  const auto found = fields.find(key);
  if (found == fields.end()) {
    return false;
  }
  try {
    value = std::stoi(found->second);
    return true;
  } catch (...) {
    return false;
  }
}

}  // namespace

void MoveItPlugin::initialize(srf::viz::VizHost& host) {
  host_ = &host;
  view_ = new srf::viz::SceneView(host.model(), &host.window());
  view_->setObjectName("moveitView");
  view_->set_moveit_interactive(true);
  view_->set_teleop_enabled(false);
  view_->set_camera_distance(1.25);
  for (const char* layer : {"map", "global_costmap", "local_costmap", "scan", "imu", "odom", "filtered",
                            "plan", "local_plan", "trail", "logs", "tf"}) {
    view_->set_enabled(layer, false);
  }
  host.add_tab(view_, "MoveIt");
  QObject::connect(view_, &srf::viz::SceneView::moveit_target_dragged, view_,
                   [this](double x, double y, double z) { on_target_dragged(x, y, z); });
  QObject::connect(view_, &srf::viz::SceneView::moveit_pitch_dragged, view_,
                   [this](double degrees) { on_pitch_dragged(degrees); });
  QObject::connect(view_, &srf::viz::SceneView::moveit_drag_finished, view_,
                   [this] { on_drag_finished(); });

  build_dock();
  client_ = std::make_unique<MoveClient>(host.model().node().context(),
                                         srf::msgs::moveit::MOVE_ACTION);
  preview_timer_ = new QTimer(&host.window());
  preview_timer_->setSingleShot(true);
  preview_timer_->setInterval(180);
  QObject::connect(preview_timer_, &QTimer::timeout, preview_timer_, [this] { send_goal(true); });
  scrub_timer_ = new QTimer(&host.window());
  scrub_timer_->setSingleShot(true);
  scrub_timer_->setInterval(60);
  QObject::connect(scrub_timer_, &QTimer::timeout, scrub_timer_,
                   [this] { send_scrub(scrub_index_); });
  refresh_timer_ = new QTimer(&host.window());
  QObject::connect(refresh_timer_, &QTimer::timeout, refresh_timer_, [this] { refresh(); });
  refresh_timer_->start(100);
  refresh();
}

void MoveItPlugin::shutdown() {
  if (refresh_timer_ != nullptr) {
    refresh_timer_->stop();
  }
  if (preview_timer_ != nullptr) {
    preview_timer_->stop();
  }
  if (scrub_timer_ != nullptr) {
    scrub_timer_->stop();
  }
  handle_.reset();
  if (client_) {
    client_->shutdown();
    client_.reset();
  }
}

void MoveItPlugin::build_dock() {
  dock_ = new QDockWidget("MoveIt", &host_->window());
  dock_->setObjectName("moveitDock");
  dock_->setAllowedAreas(Qt::LeftDockWidgetArea | Qt::RightDockWidgetArea | Qt::BottomDockWidgetArea);
  dock_->setFeatures(QDockWidget::DockWidgetMovable | QDockWidget::DockWidgetFloatable |
                            QDockWidget::DockWidgetClosable);
  auto* panel = new QWidget(dock_);
  auto* layout = new QVBoxLayout(panel);
  layout->setContentsMargins(8, 8, 8, 8);
  layout->setSpacing(6);

  auto* title = new QLabel("TCP target (base_link)", panel);
  title->setObjectName("Section");
  layout->addWidget(title);

  const auto make_spin = [&](double minimum, double maximum, double step, int decimals,
                             const QString& suffix) {
    auto* spin = new QDoubleSpinBox(panel);
    spin->setRange(minimum, maximum);
    spin->setSingleStep(step);
    spin->setDecimals(decimals);
    spin->setSuffix(suffix);
    spin->setKeyboardTracking(false);
    return spin;
  };
  x_ = make_spin(-0.8, 0.8, 0.005, 3, " m");
  y_ = make_spin(-0.8, 0.8, 0.005, 3, " m");
  z_ = make_spin(-0.2, 1.0, 0.005, 3, " m");
  pitch_ = make_spin(-90.0, 90.0, 1.0, 1, QString::fromUtf8("°"));
  gripper_ = make_spin(0.0, 27.0, 0.5, 1, " mm");
  auto* grid = new QGridLayout();
  grid->addWidget(new QLabel("X", panel), 0, 0);
  grid->addWidget(x_, 0, 1);
  grid->addWidget(new QLabel("Y", panel), 0, 2);
  grid->addWidget(y_, 0, 3);
  grid->addWidget(new QLabel("Z", panel), 1, 0);
  grid->addWidget(z_, 1, 1);
  grid->addWidget(new QLabel("Pitch", panel), 1, 2);
  grid->addWidget(pitch_, 1, 3);
  grid->addWidget(new QLabel("Grip", panel), 2, 0);
  grid->addWidget(gripper_, 2, 1);
  cartesian_ = new QCheckBox("Cartesian", panel);
  cartesian_->setChecked(true);
  grid->addWidget(cartesian_, 2, 2, 1, 2);
  layout->addLayout(grid);
  auto* hint = new QLabel("MoveIt 탭: 드래그 X/Y · Shift+드래그 Z · Ctrl+드래그 Pitch", panel);
  hint->setWordWrap(true);
  hint->setStyleSheet("color:#7f899b;");
  layout->addWidget(hint);

  auto* buttons = new QHBoxLayout();
  current_button_ = new QPushButton("Current", panel);
  plan_button_ = new QPushButton("Plan", panel);
  execute_button_ = new QPushButton("Execute", panel);
  cancel_button_ = new QPushButton("Cancel", panel);
  for (auto* button : {current_button_, plan_button_, execute_button_,
                       cancel_button_}) {
    buttons->addWidget(button);
  }
  layout->addLayout(buttons);
  execute_button_->setEnabled(false);
  cancel_button_->setEnabled(false);

  layout->addWidget(new QLabel("trajectory", panel));
  scrub_ = new QSlider(Qt::Horizontal, panel);
  scrub_->setRange(0, 0);
  layout->addWidget(scrub_);
  current_label_ = new QLabel("now: waiting for joint_states", panel);
  current_label_->setWordWrap(true);
  status_label_ = new QLabel("MoveIt: waiting for robot_description", panel);
  status_label_->setWordWrap(true);
  status_label_->setMinimumHeight(44);
  layout->addWidget(current_label_);
  layout->addWidget(status_label_);
  layout->addStretch(1);

  const auto target_changed = [this] {
    if (updating_) {
      return;
    }
    publish_target(true);
  };
  for (auto* spin : {x_, y_, z_, pitch_, gripper_}) {
    QObject::connect(spin, qOverload<double>(&QDoubleSpinBox::valueChanged), dock_,
            [target_changed](double) { target_changed(); });
  }
  QObject::connect(cartesian_, &QCheckBox::toggled, dock_, [target_changed](bool) { target_changed(); });
  QObject::connect(current_button_, &QPushButton::clicked, dock_, [this] { set_target_from_current(); });
  QObject::connect(plan_button_, &QPushButton::clicked, dock_, [this] { plan(); });
  QObject::connect(execute_button_, &QPushButton::clicked, dock_, [this] { execute(); });
  QObject::connect(cancel_button_, &QPushButton::clicked, dock_, [this] { stop(); });
  QObject::connect(scrub_, &QSlider::valueChanged, dock_, [this](int index) {
    if (executing_ || updating_) {
      return;
    }
    scrub_index_ = index;
    if (scrub_timer_ != nullptr) {
      scrub_timer_->start();
    } else {
      send_scrub(index);
    }
  });

  dock_->setWidget(panel);
  host_->add_dock(dock_, Qt::RightDockWidgetArea);
  
}

void MoveItPlugin::publish_target(bool preview) {
  (void)preview;
  if (updating_ || x_ == nullptr || client_ == nullptr) {
    return;
  }
  if (handle_ && handle_->status() == srf::GoalStatus::Accepted) {
    return;  // 이전 plan/execute가 끝난 뒤 다음 preview를 보낸다
  }
  if (preview_timer_ != nullptr) {
    preview_timer_->start();
  } else {
    send_goal(true);
  }
}

void MoveItPlugin::send_goal(bool plan_only, const std::string& preset) {
  if (host_ == nullptr || client_ == nullptr || x_ == nullptr) {
    return;
  }
  if (handle_ && handle_->status() == srf::GoalStatus::Accepted) {
    if (status_label_ != nullptr) {
      status_label_->setText("MoveIt: 이전 goal이 아직 실행 중이다");
    }
    return;
  }
  srf::msgs::moveit::MoveGroupGoal goal;
  goal.plan_only = plan_only;
  goal.cartesian = cartesian_ != nullptr ? cartesian_->isChecked() : true;
  goal.grip = gripper_ != nullptr ? gripper_->value() / 1000.0 : -1.0;
  goal.preset = preset;
  if (preset.empty()) {
    const double pitch = pitch_->value() * 3.14159265358979323846 / 180.0;
    const double yaw = std::atan2(y_->value() + 0.000230, x_->value() - 0.059985);
    goal.target_pose.header = srf::msgs::Header{host_->model().node().now(), "base_link"};
    goal.target_pose.pose.position = {x_->value(), y_->value(), z_->value()};
    goal.target_pose.pose.orientation = srf::tfmath::from_rpy(0.0, -pitch, yaw);
  }
  handle_ = client_->send_goal(goal, [this](const srf::msgs::moveit::MoveGroupFeedback& feedback) {
    if (status_label_ != nullptr) {
      status_label_->setText(QString("MoveIt: %1 %2%")
                                 .arg(QString::fromStdString(feedback.state))
                                 .arg(static_cast<int>(feedback.progress * 100.0)));
    }
  });
  if (status_label_ != nullptr) {
    status_label_->setText(plan_only ? "Plan 요청..." : "Execute 요청...");
  }
}

void MoveItPlugin::send_scrub(int index) {
  if (host_ == nullptr) {
    return;
  }
  try {
    const auto response = host_->model().node().call_service<srf::msgs::String, srf::msgs::Bool>(
        "move_group/scrub", srf::msgs::String{std::to_string(index)});
    if (!response.data && status_label_ != nullptr) {
      status_label_->setText("Preview scrub 실패");
    }
  } catch (const std::exception& error) {
    if (status_label_ != nullptr) {
      status_label_->setText(QString("Preview scrub 실패: %1").arg(error.what()));
    }
  }
}

void MoveItPlugin::update_preview() {
  if (host_ == nullptr) {
    return;
  }
  const auto arm = host_->model().moveit_arm();
  const auto path = host_->model().moveit_path();
  const auto target = host_->model().moveit_target_pose();
  if (!arm && !target) {
    host_->scene().clear_arm_preview();
    if (view_ != nullptr) {
      view_->clear_arm_preview();
    }
    return;
  }
  srf::viz::ArmPreview preview;
  preview.active = true;
  if (arm && !arm->poses.empty()) {
    preview.links.reserve(arm->poses.size());
    for (const auto& pose : arm->poses) {
      preview.links.push_back(srf::msgs::Transform{{pose.pose.position.x, pose.pose.position.y, pose.pose.position.z},
                                              pose.pose.orientation});
    }
    const auto& last = arm->poses.back().pose.position;
    preview.tcp = {last.x, last.y, last.z};
  }
  if (path && !path->poses.empty()) {
    preview.tcp_path.reserve(path->poses.size());
    for (const auto& pose : path->poses) {
      preview.tcp_path.push_back({pose.pose.position.x, pose.pose.position.y, pose.pose.position.z});
    }
  }
  if (target) {
    preview.target = srf::msgs::Transform{{target->pose.position.x, target->pose.position.y, target->pose.position.z},
                                     target->pose.orientation};
  }
  if (dragging_ && x_ != nullptr) {
    const double pitch = pitch_->value() * 3.141592653589793 / 180.0;
    const double yaw = std::atan2(y_->value() + 0.000230, x_->value() - 0.059985);
    preview.target = srf::msgs::Transform{{x_->value(), y_->value(), z_->value()},
                                     srf::tfmath::from_rpy(0.0, -pitch, yaw)};
  }
  host_->scene().set_arm_preview(preview);
  if (view_ != nullptr) {
    view_->set_arm_preview(std::move(preview));
  }
}

void MoveItPlugin::set_target_from_current() {
  const auto state = host_->model().moveit_state();
  if (!state) {
    if (status_label_ != nullptr) {
      status_label_->setText("moveit_state 대기 중 (노드 연결 확인)");
    }
    return;
  }
  if (executing_) {
    return;
  }
  const auto fields = parse_state_fields(state->data);
  const auto tcp = fields.find("tcp");
  if (tcp == fields.end()) {
    return;
  }
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
  double pitch = 0.0;
  double grip = 0.0;
  {
    std::stringstream stream(tcp->second);
    std::string item;
    if (!std::getline(stream, item, ',') || !parse_field_double({{"v", item}}, "v", x)) {
      return;
    }
    if (!std::getline(stream, item, ',') || !parse_field_double({{"v", item}}, "v", y)) {
      return;
    }
    if (!std::getline(stream, item, ',') || !parse_field_double({{"v", item}}, "v", z)) {
      return;
    }
  }
  if (!parse_field_double(fields, "pitch", pitch)) {
    return;
  }
  (void)parse_field_double(fields, "grip", grip);
  updating_ = true;
  x_->setValue(x);
  y_->setValue(y);
  z_->setValue(z);
  pitch_->setValue(pitch);
  if (std::isfinite(grip)) {
    gripper_->setValue(grip);
  }
  updating_ = false;
  target_initialized_ = true;
  if (host_ != nullptr) {
    host_->show_tab(view_);
  }
  publish_target(true);
}

void MoveItPlugin::plan() {
  if (executing_) {
    return;
  }
  if (host_ != nullptr) {
    host_->show_tab(view_);
  }
  send_goal(true);
}

void MoveItPlugin::execute() {
  if (executing_) {
    return;
  }
  if (host_ != nullptr) {
    host_->show_tab(view_);
  }
  send_goal(false);
}

void MoveItPlugin::stop() {
  if (handle_) {
    (void)handle_->cancel();
  }
  if (status_label_ != nullptr) {
    status_label_->setText("Cancel 요청...");
  }
}

void MoveItPlugin::on_target_dragged(double x, double y, double z) {
  if (updating_ || executing_ || x_ == nullptr) {
    return;
  }
  updating_ = true;
  x_->setValue(x);
  y_->setValue(y);
  z_->setValue(z);
  updating_ = false;
  dragging_ = true;
  const double current = steady_seconds();
  if (current - last_drag_publish_ >= 0.12) {
    publish_target(true);
    last_drag_publish_ = current;
  }
}

void MoveItPlugin::on_pitch_dragged(double degrees) {
  if (updating_ || executing_ || pitch_ == nullptr) {
    return;
  }
  updating_ = true;
  pitch_->setValue(std::clamp(degrees, -90.0, 90.0));
  updating_ = false;
  dragging_ = true;
  const double current = steady_seconds();
  if (current - last_drag_publish_ >= 0.12) {
    publish_target(true);
    last_drag_publish_ = current;
  }
}

void MoveItPlugin::on_drag_finished() {
  dragging_ = false;
  if (!updating_) {
    publish_target(true);
  }
}


void MoveItPlugin::refresh() {
  if (dock_ == nullptr || current_label_ == nullptr) {
    return;
  }
  if (handle_ && handle_->status() != srf::GoalStatus::Accepted) {
    const auto result = handle_->result(0.0);
    if (status_label_ != nullptr) {
      if (result.status == srf::GoalStatus::Rejected) {
        status_label_->setText("MoveIt: goal rejected");
      } else if (result.result.has_value()) {
        try {
          const auto& move = std::any_cast<const srf::msgs::moveit::MoveGroupResult&>(result.result);
          if (move.error_code != srf::msgs::moveit::MoveGroupResult::NONE) {
            status_label_->setText(QString("MoveIt: %1 (%2)")
                                       .arg(QString::fromStdString(move.error_msg))
                                       .arg(move.error_code));
          }
        } catch (const std::bad_any_cast&) {
        }
      }
    }
    handle_.reset();
  }
  const auto state_message = host_->model().moveit_state();
  if (!state_message) {
    current_label_->setText("now: move_group 노드 대기 중");
    return;
  }
  const auto fields = parse_state_fields(state_message->data);
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
  double pitch = 0.0;
  double grip = 0.0;
  bool have_tcp = false;
  const auto tcp = fields.find("tcp");
  if (tcp != fields.end()) {
    std::stringstream stream(tcp->second);
    std::string item;
    if (std::getline(stream, item, ',') && parse_field_double({{"v", item}}, "v", x) &&
        std::getline(stream, item, ',') && parse_field_double({{"v", item}}, "v", y) &&
        std::getline(stream, item, ',') && parse_field_double({{"v", item}}, "v", z)) {
      have_tcp = true;
    }
  }
  if (have_tcp && parse_field_double(fields, "pitch", pitch)) {
    (void)parse_field_double(fields, "grip", grip);
    current_label_->setText(QString("now TCP  x %1  y %2  z %3  pitch %4°  grip %5 mm")
                                       .arg(x, 0, 'f', 3)
                                       .arg(y, 0, 'f', 3)
                                       .arg(z, 0, 'f', 3)
                                       .arg(pitch, 0, 'f', 1)
                                       .arg(grip, 0, 'f', 1));
    if (!target_initialized_ && !executing_) {
      updating_ = true;
      x_->setValue(x);
      y_->setValue(y);
      z_->setValue(z);
      pitch_->setValue(pitch);
      if (std::isfinite(grip)) {
        gripper_->setValue(grip);
      }
      updating_ = false;
      target_initialized_ = true;
      publish_target(true);
    }
  } else {
    current_label_->setText("now: move_group TCP 계산 대기");
  }

  int points = 0;
  int index = 0;
  int executing = 0;
  int has_plan = 0;
  int error = 0;
  (void)parse_field_int(fields, "points", points);
  (void)parse_field_int(fields, "index", index);
  (void)parse_field_int(fields, "executing", executing);
  (void)parse_field_int(fields, "has_plan", has_plan);
  (void)parse_field_int(fields, "err", error);
  executing_ = executing != 0;
  has_plan_ = has_plan != 0 && points > 0;
  points_ = points;
  if (view_ != nullptr) {
    view_->set_moveit_interactive(!executing_);
  }

  if (scrub_ != nullptr) {
    const QSignalBlocker blocker(scrub_);
    const int last = std::max(0, points - 1);
    if (scrub_->maximum() != last) {
      scrub_->setRange(0, last);
    }
    if (!scrub_->isSliderDown()) {
      scrub_->setValue(std::clamp(index, 0, last));
    }
    scrub_->setEnabled(!executing_ && has_plan_);
  }
  if (plan_button_ != nullptr) {
    plan_button_->setEnabled(!executing_);
    current_button_->setEnabled(!executing_);
    execute_button_->setEnabled(!executing_ && has_plan_);
    cancel_button_->setEnabled(executing_);
    for (auto* spin : {x_, y_, z_, pitch_, gripper_}) {
      spin->setEnabled(!executing_);
    }
    cartesian_->setEnabled(!executing_);
  }
  if (status_label_ != nullptr) {
    if (const auto status = host_->model().moveit_status(); status && !status->data.empty()) {
      status_label_->setText(QString::fromStdString(status->data));
    }
    status_label_->setStyleSheet(error != 0 ? "color:#ff7070;" : "");
  }
  update_preview();
}

}  // namespace srf_default_ws::viz
