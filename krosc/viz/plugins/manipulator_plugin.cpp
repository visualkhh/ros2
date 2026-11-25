#include "krosc/viz/plugins/manipulator_plugin.hpp"

#include <QComboBox>
#include <QDockWidget>
#include <QHBoxLayout>
#include <QLabel>
#include <QPushButton>
#include <QStringList>
#include <QSlider>
#include <QTimer>
#include <QVBoxLayout>
#include <QWidget>

#include <cmath>
#include <limits>
#include <map>
#include <string>

#include "krosc/msgs.hpp"
#include "krosc/qos.hpp"
#include "krosc/viz/viz_model.hpp"
#include "krosc/viz/viz_window.hpp"

namespace krosc::viz {

namespace {
constexpr double kPi = 3.14159265358979323846;
}

ManipulatorPlugin::ManipulatorPlugin()
    : VizPlugin({"manipulator", "Manipulator", "1.0", {"joint_states", "arm_status"}}) {}
ManipulatorPlugin::~ManipulatorPlugin() { shutdown(); }

void ManipulatorPlugin::initialize(VizHost& host) {
  if (active_) {
    return;
  }
  host_ = &host;
  dock_ = new QDockWidget("Manipulator", &host.window());
  dock_->setObjectName("manipulatorDock");
  dock_->setAllowedAreas(Qt::LeftDockWidgetArea | Qt::RightDockWidgetArea | Qt::BottomDockWidgetArea);
  dock_->setFeatures(QDockWidget::DockWidgetMovable | QDockWidget::DockWidgetFloatable |
                     QDockWidget::DockWidgetClosable);
  auto* panel = new QWidget(dock_);
  auto* layout = new QVBoxLayout(panel);
  layout->setContentsMargins(8, 8, 8, 8);
  layout->setSpacing(5);

  auto* buttons = new QHBoxLayout();
  torque_ = new QPushButton("Torque ON", panel);
  auto* stow = new QPushButton("STOW", panel);
  auto* home = new QPushButton("HOME", panel);
  auto* estop = new QPushButton("E-STOP", panel);
  for (auto* button : {torque_, stow, home, estop}) {
    buttons->addWidget(button);
  }
  layout->addLayout(buttons);

  auto* reset_row = new QHBoxLayout();
  auto* reset_name = new QLabel("reset", panel);
  reset_name->setMinimumWidth(46);
  reset_target_ = new QComboBox(panel);
  reset_target_->addItems({"all", "joint1", "joint2", "joint3", "joint4", "gripper"});
  reset_button_ = new QPushButton("Reset (AR)", panel);
  reset_row->addWidget(reset_name);
  reset_row->addWidget(reset_target_, 1);
  reset_row->addWidget(reset_button_);
  layout->addLayout(reset_row);

  const auto set_torque_button = [this] {
    torque_->setText(torque_on_ ? "Torque ON" : "Torque OFF");
    torque_->setStyleSheet(torque_on_ ? "background:#1f6b43;" : "background:#7a2a2a;");
  };
  const auto apply_torque = [this, set_torque_button](bool on) {
    torque_on_ = on;
    if (!torque_on_) {
      moving_ = false;
    }
    host_->model().node().publish("arm_torque", msgs::Bool{torque_on_}, QOS_DEFAULT);
    set_torque_button();
  };
  connect(torque_, &QPushButton::clicked, this, [this, apply_torque] {
    apply_torque(!torque_on_);
    set_status(torque_on_ ? "torque ON" : "torque OFF (arm relaxed)");
  });
  set_torque_button();

  const std::array<std::pair<int, int>, 4> ranges{{{ -180, 180 }, { -126, 126 }, { -114, 114 }, { -114, 126 }}};
  for (int i = 0; i < 4; ++i) {
    auto* row = new QHBoxLayout();
    auto* name = new QLabel(QString("joint%1").arg(i + 1), panel);
    name->setMinimumWidth(46);
    auto* slider = new QSlider(Qt::Horizontal, panel);
    slider->setRange(ranges[static_cast<std::size_t>(i)].first, ranges[static_cast<std::size_t>(i)].second);
    slider->setValue(0);
    auto* value = new QLabel("0.0°", panel);
    value->setMinimumWidth(52);
    value->setAlignment(Qt::AlignRight | Qt::AlignVCenter);
    row->addWidget(name);
    row->addWidget(slider, 1);
    row->addWidget(value);
    layout->addLayout(row);
    sliders_[static_cast<std::size_t>(i)] = slider;
    value_labels_[static_cast<std::size_t>(i)] = value;
    connect(slider, &QSlider::valueChanged, this, [this, i, value, apply_torque](int degrees) {
      value->setText(QString::number(degrees, 'f', 1) + "°");
      if (updating_) {
        return;
      }
      cancel_moveit_if_executing();
      if (!torque_on_) {
        apply_torque(true);
      }
      set_manual_mode(true);
      target_[static_cast<std::size_t>(i)] = static_cast<double>(degrees) * kPi / 180.0;
      moving_ = true;
    });
  }
  auto* gripper_row = new QHBoxLayout();
  auto* gripper_name = new QLabel("gripper", panel);
  gripper_name->setMinimumWidth(46);
  gripper_ = new QSlider(Qt::Horizontal, panel);
  gripper_->setRange(0, 27);
  gripper_->setValue(0);
  gripper_label_ = new QLabel("0.0 mm", panel);
  gripper_label_->setMinimumWidth(52);
  gripper_label_->setAlignment(Qt::AlignRight | Qt::AlignVCenter);
  gripper_row->addWidget(gripper_name);
  gripper_row->addWidget(gripper_, 1);
  gripper_row->addWidget(gripper_label_);
  layout->addLayout(gripper_row);
  connect(gripper_, &QSlider::valueChanged, this, [this, apply_torque](int mm) {
    gripper_label_->setText(QString::number(mm, 'f', 1) + " mm");
    if (updating_) {
      return;
    }
    cancel_moveit_if_executing();
    if (!torque_on_) {
      apply_torque(true);
    }
    set_manual_mode(true);
    target_[4] = static_cast<double>(mm) / 1000.0;
    moving_ = true;
  });

  auto* speed_row = new QHBoxLayout();
  auto* speed_name = new QLabel("speed", panel);
  speed_name->setMinimumWidth(46);
  speed_ = new QSlider(Qt::Horizontal, panel);
  speed_->setRange(10, 80);
  speed_->setValue(35);
  speed_label_ = new QLabel("0.35 rad/s", panel);
  speed_label_->setMinimumWidth(72);
  speed_label_->setAlignment(Qt::AlignRight | Qt::AlignVCenter);
  speed_row->addWidget(speed_name);
  speed_row->addWidget(speed_, 1);
  speed_row->addWidget(speed_label_);
  layout->addLayout(speed_row);
  connect(speed_, &QSlider::valueChanged, this, [this](int value) {
    speed_value_ = static_cast<double>(value) / 100.0;
    speed_label_->setText(QString("%1 rad/s").arg(speed_value_, 0, 'f', 2));
  });

  status_ = new QLabel("arm: waiting for joint_states", panel);
  // 상태 줄이 길어져도 줄바꿈으로 높이가 변해 아래 컨트롤이 밀리지 않게 한 줄 고정.
  status_->setWordWrap(false);
  status_->setFixedHeight(20);
  status_->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Fixed);
  layout->addWidget(status_);

  const auto set_preset = [this](const std::array<double, 5>& positions) {
    updating_ = true;
    for (int i = 0; i < 4; ++i) {
      sliders_[static_cast<std::size_t>(i)]->setValue(
          static_cast<int>(std::lround(positions[static_cast<std::size_t>(i)] * 180.0 / kPi)));
    }
    if (std::isfinite(positions[4])) {
      gripper_->setValue(static_cast<int>(std::lround(positions[4] * 1000.0)));
    }
    updating_ = false;
    target_ = positions;
    moving_ = false;  // 실제 이동은 MoveIt 프리셋이 담당한다.
  };
  connect(stow, &QPushButton::clicked, this, [this, set_preset, apply_torque] {
    cancel_moveit_if_executing();
    apply_torque(true);
    set_preset({0.0, -1.9024, 1.3440, 0.9023, 0.0});
    set_manual_mode(false);
    send_move_preset("stow");
    set_status("STOW → MoveIt 실행");
  });
  connect(home, &QPushButton::clicked, this, [this, set_preset, apply_torque] {
    cancel_moveit_if_executing();
    apply_torque(true);
    set_preset({0.5000, -1.8000, 1.4000, 0.8000, 0.0});
    set_manual_mode(false);
    send_move_preset("home");
    set_status("HOME (safe) → MoveIt 실행");
  });
  connect(estop, &QPushButton::clicked, this, [this] {
    cancel_moveit_if_executing();
    host_->model().node().publish("arm_stop", msgs::Empty{}, QOS_DEFAULT);
    moving_ = false;
    set_status("E-STOP: hold current pose");
  });
  connect(reset_button_, &QPushButton::clicked, this, [this] { reset_arm(); });

  dock_->setWidget(panel);
  host.add_dock(dock_, Qt::RightDockWidgetArea);
  move_client_ = std::make_unique<MoveClient>(host.model().node().context(),
                                              krosc::msgs::moveit::MOVE_ACTION);
  send_timer_ = new QTimer(this);
  connect(send_timer_, &QTimer::timeout, this, &ManipulatorPlugin::send_target);
  send_timer_->start(50);
  refresh_timer_ = new QTimer(this);
  connect(refresh_timer_, &QTimer::timeout, this, &ManipulatorPlugin::refresh);
  refresh_timer_->start(100);
  active_ = true;
}

void ManipulatorPlugin::shutdown() {
  if (!active_) {
    return;
  }
  active_ = false;
  if (send_timer_ != nullptr) {
    send_timer_->stop();
  }
  if (refresh_timer_ != nullptr) {
    refresh_timer_->stop();
  }
  move_handle_.reset();
  if (move_client_) {
    move_client_->shutdown();
    move_client_.reset();
  }
  if (dock_ != nullptr) {
    dock_->deleteLater();
    dock_ = nullptr;
  }
}

void ManipulatorPlugin::cancel_moveit_if_executing() {
  if (host_ == nullptr) {
    return;
  }
  if (move_handle_ && move_handle_->status() == krosc::GoalStatus::Accepted) {
    (void)move_handle_->cancel();
  }
}

void ManipulatorPlugin::set_manual_mode(bool manual) {
  if (host_ == nullptr || manual_mode_ == manual) {
    return;
  }
  std::string error;
  const bool ok =
      manual ? host_->model().switch_controller({"arm_command_controller"},
                                                {"joint_trajectory_controller"}, 1, error)
             : host_->model().switch_controller({"joint_trajectory_controller"},
                                                {"arm_command_controller"}, 1, error);
  if (!ok) {
    set_status(QString("controller switch 실패: %1").arg(QString::fromStdString(error)));
    return;
  }
  manual_mode_ = manual;
}

void ManipulatorPlugin::send_move_preset(const std::string& preset) {
  if (host_ == nullptr || move_client_ == nullptr) {
    return;
  }
  if (move_handle_ && move_handle_->status() == krosc::GoalStatus::Accepted) {
    return;
  }
  krosc::msgs::moveit::MoveGroupGoal goal;
  goal.preset = preset;
  goal.cartesian = false;
  goal.plan_only = false;
  move_handle_ = move_client_->send_goal(
      goal, [this](const krosc::msgs::moveit::MoveGroupFeedback& feedback) {
        set_status(QString("MoveIt %1 %2%")
                       .arg(QString::fromStdString(feedback.state))
                       .arg(static_cast<int>(feedback.progress * 100.0)));
      });
}

void ManipulatorPlugin::send_target() {
  if (host_ == nullptr || !initialized_ || !moving_) {
    return;
  }
  constexpr double dt = 0.05;
  bool moved = false;
  for (int i = 0; i < 4; ++i) {
    double delta = target_[static_cast<std::size_t>(i)] - output_[static_cast<std::size_t>(i)];
    const double step = speed_value_ * dt;
    if (std::abs(delta) > step) {
      delta = std::copysign(step, delta);
    }
    if (std::abs(delta) > 1e-6) {
      output_[static_cast<std::size_t>(i)] += delta;
      moved = true;
    }
  }
  double gripper_delta = target_[4] - output_[4];
  const double gripper_step = 0.016 * dt;
  if (std::abs(gripper_delta) > gripper_step) {
    gripper_delta = std::copysign(gripper_step, gripper_delta);
  }
  if (std::abs(gripper_delta) > 1e-6) {
    output_[4] += gripper_delta;
    moved = true;
  }
  if (!moved) {
    output_ = target_;
    moving_ = false;
  }
  msgs::JointState command;
  for (int i = 0; i < 4; ++i) {
    command.name.push_back("joint" + std::to_string(i + 1));
    command.position.push_back(output_[static_cast<std::size_t>(i)]);
  }
  command.name.push_back("gripper_left_joint");
  command.position.push_back(output_[4]);
  host_->model().node().publish("arm_joint_command", command, QOS_DEFAULT);
}

void ManipulatorPlugin::set_slider_values(const std::array<double, 5>& positions) {
  if (sliders_[0] == nullptr || moving_) {
    return;
  }
  for (int i = 0; i < 4; ++i) {
    if (sliders_[static_cast<std::size_t>(i)]->isSliderDown()) {
      return;
    }
  }
  if (gripper_ != nullptr && gripper_->isSliderDown()) {
    return;
  }
  target_ = positions;
  output_ = positions;
  updating_ = true;
  for (int i = 0; i < 4; ++i) {
    if (!std::isfinite(positions[static_cast<std::size_t>(i)])) {
      continue;
    }
    const int degrees =
        static_cast<int>(std::lround(positions[static_cast<std::size_t>(i)] * 180.0 / kPi));
    sliders_[static_cast<std::size_t>(i)]->setValue(degrees);
    value_labels_[static_cast<std::size_t>(i)]->setText(QString::number(degrees, 'f', 1) + "°");
  }
  if (gripper_ != nullptr && std::isfinite(positions[4])) {
    const int mm = static_cast<int>(std::lround(positions[4] * 1000.0));
    gripper_->setValue(mm);
    gripper_label_->setText(QString::number(mm, 'f', 1) + " mm");
  }
  updating_ = false;
}

void ManipulatorPlugin::set_status(const QString& text) {
  if (status_ == nullptr) {
    return;
  }
  status_->setText(text);
  status_->setToolTip(text);  // 잘린 전체 내용은 툴팁으로
}

void ManipulatorPlugin::refresh() {
  if (host_ == nullptr || status_ == nullptr) {
    return;
  }
  const auto states = host_->model().joint_states();
  if (!states) {
    set_status("arm: no joint_states");
    return;
  }
  std::map<std::string, double> values;
  for (std::size_t i = 0; i < states->name.size() && i < states->position.size(); ++i) {
    values[states->name[i]] = states->position[i];
  }
  std::array<double, 5> positions{};
  positions.fill(std::numeric_limits<double>::quiet_NaN());
  for (int i = 0; i < 4; ++i) {
    const auto it = values.find("joint" + std::to_string(i + 1));
    if (it != values.end()) {
      positions[static_cast<std::size_t>(i)] = it->second;
    }
  }
  const auto gripper = values.find("gripper_left_joint");
  if (gripper != values.end()) {
    positions[4] = gripper->second;
  }
  if (!initialized_ && std::isfinite(positions[0])) {
    output_ = positions;
    target_ = positions;
    initialized_ = true;
  }
  set_slider_values(positions);
  QString text = QString("j1 %1°  j2 %2°  j3 %3°  j4 %4°  grip %5 mm")
                     .arg(std::isfinite(positions[0]) ? positions[0] * 180.0 / kPi : 0.0, 0, 'f', 0)
                     .arg(std::isfinite(positions[1]) ? positions[1] * 180.0 / kPi : 0.0, 0, 'f', 0)
                     .arg(std::isfinite(positions[2]) ? positions[2] * 180.0 / kPi : 0.0, 0, 'f', 0)
                     .arg(std::isfinite(positions[3]) ? positions[3] * 180.0 / kPi : 0.0, 0, 'f', 0)
                     .arg(std::isfinite(positions[4]) ? positions[4] * 1000.0 : 0.0, 0, 'f', 1);
  if (const auto collision = host_->model().collision_status()) {
    text += "  | " + QString::fromStdString(collision->data);
  }
  const QString errors = error_summary();
  if (!errors.isEmpty()) {
    text = "ARM ERR " + errors + "  |  " + text;
    status_->setStyleSheet("color:#ff7070;");
  } else {
    status_->setStyleSheet("");
  }
  set_status(text);
}

void ManipulatorPlugin::reset_arm() {
  if (host_ == nullptr || reset_target_ == nullptr || reset_button_ == nullptr) {
    return;
  }
  const QString target = reset_target_->currentText();
  cancel_moveit_if_executing();
  moving_ = false;
  host_->model().node().publish("arm_reset", msgs::String{target.toStdString()}, QOS_DEFAULT);
  set_status(QString("reset(AR) 요청: %1 — 서보 재부팅 후 토크 복구").arg(target));
  status_->setStyleSheet("color:#ffd166;");
  reset_button_->setEnabled(false);
  QTimer::singleShot(2000, this, [this] {
    if (reset_button_ != nullptr) {
      reset_button_->setEnabled(true);
    }
    if (host_ != nullptr) {
      torque_on_ = true;
      host_->model().node().publish("arm_torque", msgs::Bool{true}, QOS_DEFAULT);
      if (torque_ != nullptr) {
        torque_->setText("Torque ON");
        torque_->setStyleSheet("background:#1f6b43;");
      }
    }
  });
}

QString ManipulatorPlugin::error_summary() const {
  if (host_ == nullptr) {
    return {};
  }
  const auto status = host_->model().arm_status();
  if (!status) {
    return {};
  }
  const QString text = QString::fromStdString(status->data);
  const int start = text.indexOf("err=");
  if (start < 0) {
    return {};
  }
  const int end = text.indexOf(';', start);
  const QString list = text.mid(start + 4, end < 0 ? -1 : end - start - 4);
  const QStringList values = list.split(',');
  QStringList errors;
  for (int i = 0; i < values.size() && i < 5; ++i) {
    if (values[i] == "0" || values[i].isEmpty() || values[i] == "x") {
      continue;
    }
    const QString name = i == 4 ? QString("grip") : QString("j%1").arg(i + 1);
    errors << name + "=" + values[i];
  }
  return errors.join(" ");
}

}  // namespace krosc::viz
