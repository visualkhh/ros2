#include "krosc/viz/plugins/navigation_plugin.hpp"

#include <QAbstractSpinBox>
#include <QApplication>
#include <QDoubleSpinBox>
#include <QDockWidget>
#include <QGridLayout>
#include <QKeyEvent>
#include <QLabel>
#include <QLineEdit>
#include <QPushButton>
#include <QTimer>
#include <QStringList>
#include <QWidget>

#include <cmath>
#include <map>
#include <sstream>

#include "krosc/tf/math.hpp"
#include "krosc/msgs.hpp"
#include "krosc/qos.hpp"
#include "krosc/viz/key_bindings.hpp"
#include "krosc/viz/map_view.hpp"
#include "krosc/viz/scene_view.hpp"
#include "krosc/viz/viz_model.hpp"
#include "krosc/viz/viz_window.hpp"

namespace krosc::viz {

NavigationPlugin::NavigationPlugin()
    : VizPlugin({"navigation", "Navigation", "1.0", {"slam_status", "map_status", "tf"}}) {}
NavigationPlugin::~NavigationPlugin() { shutdown(); }

void NavigationPlugin::initialize(VizHost& host) {
  if (active_) {
    return;
  }
  host_ = &host;
  dock_ = new QDockWidget("Navigation", &host.window());
  dock_->setObjectName("navigationDock");
  dock_->setAllowedAreas(Qt::LeftDockWidgetArea | Qt::RightDockWidgetArea);
  dock_->setFeatures(QDockWidget::DockWidgetMovable | QDockWidget::DockWidgetFloatable |
                     QDockWidget::DockWidgetClosable);
  auto* panel = new QWidget(dock_);
  auto* grid = new QGridLayout(panel);
  grid->setContentsMargins(8, 8, 8, 8);
  grid->setHorizontalSpacing(6);
  grid->setVerticalSpacing(5);

  const auto section = [&](int row, const QString& text) {
    auto* label = new QLabel(text, panel);
    label->setStyleSheet("color:#8f9bb0;");
    grid->addWidget(label, row, 0, 1, 4);
  };

  // ---- 위치 ----
  section(0, "위치 (Localization)");
  pose_button_ = new QPushButton("2D Pose Estimate", panel);
  auto* cancel = new QPushButton("취소 / 정지", panel);
  grid->addWidget(pose_button_, 1, 0, 1, 2);
  grid->addWidget(cancel, 1, 2, 1, 2);
  connect(pose_button_, &QPushButton::clicked, this,
          [this] { set_tool(active_tool_ == "initialpose" ? "" : "initialpose"); });
  connect(cancel, &QPushButton::clicked, this, [this] {
    multi_goals_.clear();
    sync_waypoints();
    update_multi_label();
    host_->model().node().publish("cancel_goal", msgs::Bool{true}, QOS_DEFAULT);
    status_->setText("cancel requested");
    update_buttons();
  });

  // ---- 단일 목표 ----
  section(2, "단일 목표 (맵 클릭)");
  preview_button_ = new QPushButton("경로 미리보기", panel);
  move_button_ = new QPushButton("이동", panel);
  grid->addWidget(preview_button_, 3, 0, 1, 2);
  grid->addWidget(move_button_, 3, 2, 1, 2);
  connect(preview_button_, &QPushButton::clicked, this, [this] {
    single_preview_ = true;
    set_tool(active_tool_ == "goal" ? "" : "goal");
  });
  connect(move_button_, &QPushButton::clicked, this, [this] {
    single_preview_ = false;
    set_tool(active_tool_ == "goal" ? "" : "goal");
  });

  // ---- 멀티 목표 ----
  section(4, "멀티 목표 (웨이포인트)");
  multi_button_ = new QPushButton("웨이포인트 추가", panel);
  clear_button_ = new QPushButton("비우기", panel);
  multi_preview_button_ = new QPushButton("경로 미리보기", panel);
  multi_go_button_ = new QPushButton("이동 시작", panel);
  grid->addWidget(multi_button_, 5, 0, 1, 2);
  grid->addWidget(clear_button_, 5, 2, 1, 2);
  grid->addWidget(multi_preview_button_, 6, 0, 1, 2);
  grid->addWidget(multi_go_button_, 6, 2, 1, 2);
  connect(multi_button_, &QPushButton::clicked, this,
          [this] { set_tool(active_tool_ == "multi" ? "" : "multi"); });
  connect(multi_preview_button_, &QPushButton::clicked, this, [this] { publish_multi(true); });
  connect(multi_go_button_, &QPushButton::clicked, this, [this] { publish_multi(false); });
  connect(clear_button_, &QPushButton::clicked, this, [this] {
    multi_goals_.clear();
    sync_waypoints();
    update_multi_label();
    host_->model().node().publish("cancel_goal", msgs::Bool{true}, QOS_DEFAULT);
    status_->setText("waypoints cleared");
    update_buttons();
  });

  multi_label_ = new QLabel("웨이포인트: 0개", panel);
  multi_label_->setWordWrap(true);
  grid->addWidget(multi_label_, 7, 0, 1, 4);

  status_ = new QLabel("", panel);
  status_->setWordWrap(true);
  status_->setMinimumHeight(28);
  grid->addWidget(status_, 8, 0, 1, 4);

  // ---- costmap ----
  section(9, "COSTMAP (inflation / scaling)");
  const auto add_spin = [&](int row, int column, const QString& text, double value,
                            double minimum, double maximum, double step) {
    grid->addWidget(new QLabel(text, panel), row, column);
    auto* spin = new QDoubleSpinBox(panel);
    spin->setDecimals(2);
    spin->setRange(minimum, maximum);
    spin->setSingleStep(step);
    spin->setValue(value);
    grid->addWidget(spin, row, column + 1);
    return spin;
  };
  global_inflation_ = add_spin(10, 0, "g infl (m)", 0.55, 0.05, 2.0, 0.05);
  global_scaling_ = add_spin(10, 2, "g scale", 3.0, 0.5, 10.0, 0.5);
  local_inflation_ = add_spin(11, 0, "l infl (m)", 0.55, 0.05, 2.0, 0.05);
  local_scaling_ = add_spin(11, 2, "l scale", 3.0, 0.5, 10.0, 0.5);
  local_width_ = add_spin(12, 0, "l width (m)", 4.0, 1.0, 20.0, 0.5);
  local_height_ = add_spin(12, 2, "l height (m)", 4.0, 1.0, 20.0, 0.5);
  costmap_apply_ = new QPushButton("Apply costmap", panel);
  grid->addWidget(costmap_apply_, 13, 0, 1, 4);
  connect(costmap_apply_, &QPushButton::clicked, this, [this] { apply_costmap_params(); });
  costmap_label_ = new QLabel("-", panel);
  costmap_label_->setWordWrap(true);
  grid->addWidget(costmap_label_, 14, 0, 1, 4);

  dock_->setWidget(panel);
  host.add_dock(dock_, Qt::RightDockWidgetArea);

  connect(&host.scene(), &SceneView::tool_pose, this,
          [this](double x, double y, double yaw) { handle_tool_pose(x, y, yaw, false); });
  connect(&host.map_view(), &MapView2D::tool_pose, this,
          [this](double x, double y, double yaw) { handle_tool_pose(x, y, yaw, true); });
  connect(&host.scene(), &SceneView::tool_changed, this, [this](const QString&) { update_buttons(); });
  connect(&host.map_view(), &MapView2D::tool_changed, this, [this](const QString&) { update_buttons(); });
  qApp->installEventFilter(this);
  active_ = true;
  update_buttons();
  load_costmap_params();
}

void NavigationPlugin::load_costmap_params() {
  if (host_ == nullptr) {
    return;
  }
  // peer discovery가 끝나기 전에 동기 RPC로 UI를 막지 않는다.
  const auto* peer = host_->model().node().context().peer_network();
  if (peer != nullptr) {
    const auto services = peer->remote_services();
    if (services.empty() ||
        std::find(services.begin(), services.end(), "global_costmap/get_parameters") ==
            services.end()) {
      return;
    }
  }
  const auto apply = [this](QDoubleSpinBox* spin, const std::string& node,
                            const std::string& name) {
    if (spin == nullptr) {
      return;
    }
    const auto value = host_->model().get_parameter(node, name);
    if (value && std::holds_alternative<double>(*value)) {
      spin->setValue(std::get<double>(*value));
    }
  };
  apply(global_inflation_, "global_costmap", "inflation_radius");
  apply(global_scaling_, "global_costmap", "cost_scaling_factor");
  apply(local_inflation_, "local_costmap", "inflation_radius");
  apply(local_scaling_, "local_costmap", "cost_scaling_factor");
  apply(local_width_, "local_costmap", "width");
  apply(local_height_, "local_costmap", "height");
}

void NavigationPlugin::shutdown() {
  if (!active_) {
    return;
  }
  active_ = false;
  qApp->removeEventFilter(this);
  if (costmap_timer_ != nullptr) {
    costmap_timer_->stop();
  }
  costmap_subscription_.cancel();
  if (host_ != nullptr) {
    set_tool("");
    multi_goals_.clear();
    sync_waypoints();
  }
  if (dock_ != nullptr) {
    dock_->deleteLater();
    dock_ = nullptr;
  }
}

void NavigationPlugin::set_tool(const std::string& tool) {
  active_tool_ = (tool == "initialpose" || tool == "goal" || tool == "multi") ? tool : std::string{};
  if (host_ != nullptr) {
    host_->scene().set_tool(active_tool_);
    host_->map_view().set_tool(active_tool_);
  }
  update_buttons();
}

void NavigationPlugin::update_buttons() {
  if (pose_button_ == nullptr) {
    return;
  }
  const auto style = [](QPushButton* button, const QString& text, bool active) {
    button->setText(text + (active ? " *" : ""));
    button->setStyleSheet(active ? "background:#8a5a1a;" : "");
  };
  style(pose_button_, "2D Pose Estimate", active_tool_ == "initialpose");
  const bool goal_tool = active_tool_ == "goal";
  style(preview_button_, "경로 미리보기", goal_tool && single_preview_);
  style(move_button_, "이동", goal_tool && !single_preview_);
  style(multi_button_, "웨이포인트 추가", active_tool_ == "multi");
  if (multi_preview_button_ != nullptr) {
    const bool has = !multi_goals_.empty();
    multi_preview_button_->setEnabled(has);
    multi_go_button_->setEnabled(has);
    clear_button_->setEnabled(has);
  }
}

void NavigationPlugin::sync_waypoints() {
  if (host_ == nullptr) {
    return;
  }
  host_->scene().set_waypoints(multi_goals_);
  host_->map_view().set_waypoints(multi_goals_);
}

void NavigationPlugin::update_multi_label() {
  if (multi_label_ == nullptr) {
    return;
  }
  if (multi_goals_.empty()) {
    multi_label_->setText("웨이포인트: 0개");
    return;
  }
  QStringList parts;
  for (std::size_t index = 0; index < multi_goals_.size(); ++index) {
    parts << QString("#%1 (%2, %3)")
                 .arg(index + 1)
                 .arg(std::get<0>(multi_goals_[index]), 0, 'f', 2)
                 .arg(std::get<1>(multi_goals_[index]), 0, 'f', 2);
  }
  multi_label_->setText(QString("웨이포인트: %1개  %2").arg(multi_goals_.size()).arg(parts.join("  ")));
}

void NavigationPlugin::remove_last_multi_goal() {
  if (multi_goals_.empty()) {
    return;
  }
  multi_goals_.pop_back();
  sync_waypoints();
  update_multi_label();
  update_buttons();
}

bool NavigationPlugin::eventFilter(QObject* watched, QEvent* event) {
  (void)watched;
  if (!active_ || event->type() != QEvent::KeyPress) {
    return false;
  }
  auto* key = static_cast<QKeyEvent*>(event);
  if (qwerty_key(key) != Qt::Key_Backspace) {
    return false;
  }
  QWidget* focus = QApplication::focusWidget();
  const bool text_input = qobject_cast<QLineEdit*>(focus) != nullptr ||
                          qobject_cast<QAbstractSpinBox*>(focus) != nullptr;
  if (text_input || multi_goals_.empty()) {
    return false;
  }
  remove_last_multi_goal();
  return true;
}

std::string NavigationPlugin::slam_status_text() const {
  if (host_ == nullptr) {
    return "idle";
  }
  if (const auto status = host_->model().slam_status()) {
    return status->data;
  }
  return "idle";
}

void NavigationPlugin::handle_tool_pose(double x, double y, double yaw, bool from_map) {
  if (!from_map && host_->model().fixed_frame() != "map") {
    try {
      const msgs::TransformStamped transform =
          host_->model().tf_buffer().lookup_transform("map", host_->model().fixed_frame());
      const double offset = tfmath::yaw_of(transform.transform.rotation);
      const double c = std::cos(offset);
      const double s = std::sin(offset);
      const double px = x;
      const double py = y;
      x = transform.transform.translation.x + c * px - s * py;
      y = transform.transform.translation.y + s * px + c * py;
      yaw = std::atan2(std::sin(offset + yaw), std::cos(offset + yaw));
      from_map = true;
    } catch (const TransformException&) {
      status_->setText("no map TF (2D Pose Estimate first)");
      return;
    }
  }
  if (active_tool_ == "multi") {
    multi_goals_.emplace_back(x, y, yaw);
    sync_waypoints();
    update_multi_label();
    update_buttons();
    return;
  }
  if (active_tool_ == "initialpose") {
    if (slam_status_text() == "idle") {
      status_->setText("지도를 먼저 load 하세요 (localization) — idle에서는 2D Pose 무효");
      set_tool("");
      return;
    }
    publish_pose("initialpose", x, y, yaw, from_map);
  } else if (active_tool_ == "goal") {
    publish_pose(single_preview_ ? "preview_goal" : "goal_pose", x, y, yaw, from_map);
  }
  set_tool("");
}

void NavigationPlugin::publish_pose(const std::string& topic, double x, double y, double yaw,
                                    bool from_map) {
  if (!from_map && host_->model().fixed_frame() != "map") {
    try {
      const msgs::TransformStamped transform =
          host_->model().tf_buffer().lookup_transform("map", host_->model().fixed_frame());
      const double offset = tfmath::yaw_of(transform.transform.rotation);
      const double c = std::cos(offset);
      const double s = std::sin(offset);
      const double px = x;
      const double py = y;
      x = transform.transform.translation.x + c * px - s * py;
      y = transform.transform.translation.y + s * px + c * py;
      yaw = std::atan2(std::sin(offset + yaw), std::cos(offset + yaw));
    } catch (const TransformException&) {
      status_->setText("no map TF (2D Pose Estimate first)");
      return;
    }
  }
  if (topic != "initialpose" && slam_status_text() != "localization") {
    status_->setText("load a map first (localization)");
    return;
  }
  msgs::Pose pose;
  pose.position = {x, y, 0.0};
  pose.orientation = tfmath::from_yaw(yaw);
  if (topic == "initialpose") {
    msgs::PoseWithCovarianceStamped message;
    message.header = msgs::Header{host_->model().node().now(), "map"};
    message.pose.pose = pose;
    message.pose.covariance[0] = 0.25;
    message.pose.covariance[7] = 0.25;
    message.pose.covariance[35] = 0.0685;
    host_->model().node().publish(topic, message, QOS_DEFAULT);
    status_->setText(QString("initialpose (%.2f, %.2f, %.0fdeg)")
                         .arg(x)
                         .arg(y)
                         .arg(yaw * 180.0 / 3.141592653589793));
  } else {
    msgs::PoseStamped message;
    message.header = msgs::Header{host_->model().node().now(), "map"};
    message.pose = pose;
    host_->model().node().publish(topic, message, QOS_DEFAULT);
    status_->setText(QString("%1 (%.2f, %.2f)").arg(QString::fromStdString(topic)).arg(x).arg(y));
  }
}

void NavigationPlugin::publish_multi(bool preview) {
  if (multi_goals_.empty()) {
    if (preview) {
      msgs::Path empty;
      empty.header = msgs::Header{host_->model().node().now(), "map"};
      host_->model().node().publish("preview_goals", empty, QOS_DEFAULT);
    }
    status_->setText("웨이포인트가 없다");
    return;
  }
  if (!preview && slam_status_text() != "localization") {
    status_->setText("load a map first (localization)");
    return;
  }
  if (host_->model().fixed_frame() != "map") {
    status_->setText("switch fixed frame to map");
    return;
  }
  msgs::Path path;
  path.header = msgs::Header{host_->model().node().now(), "map"};
  for (const auto& [x, y, yaw] : multi_goals_) {
    msgs::PoseStamped pose;
    pose.header = path.header;
    pose.pose.position = {x, y, 0.0};
    pose.pose.orientation = tfmath::from_yaw(yaw);
    path.poses.push_back(std::move(pose));
  }
  host_->model().node().publish(preview ? "preview_goals" : "goal_poses", path, QOS_DEFAULT);
  status_->setText(preview ? QString("미리보기 %1개 (주행 안 함)").arg(multi_goals_.size())
                           : QString("이동 시작: %1개").arg(multi_goals_.size()));
}

void NavigationPlugin::apply_costmap_params() {
  if (host_ == nullptr || global_inflation_ == nullptr) {
    return;
  }
  std::string error;
  const bool global_ok = host_->model().set_parameters(
      "global_costmap",
      {{"inflation_radius", global_inflation_->value()},
       {"cost_scaling_factor", global_scaling_->value()}},
      error);
  const bool local_ok = host_->model().set_parameters(
      "local_costmap",
      {{"inflation_radius", local_inflation_->value()},
       {"cost_scaling_factor", local_scaling_->value()},
       {"width", local_width_->value()},
       {"height", local_height_->value()}},
      error);
  if (!global_ok || !local_ok) {
    status_->setText(QString("costmap 실패: %1").arg(QString::fromStdString(error)));
    return;
  }
  {
    std::lock_guard<std::mutex> lock(costmap_mutex_);
    costmap_state_ =
        "global.inflation_radius=" + std::to_string(global_inflation_->value()) +
        ";global.cost_scaling_factor=" + std::to_string(global_scaling_->value()) +
        ";local.inflation_radius=" + std::to_string(local_inflation_->value()) +
        ";local.cost_scaling_factor=" + std::to_string(local_scaling_->value()) +
        ";local.width=" + std::to_string(local_width_->value()) +
        ";local.height=" + std::to_string(local_height_->value());
    costmap_dirty_ = true;
  }
  refresh_costmap_state();
  status_->setText("costmap applied");
}

void NavigationPlugin::refresh_costmap_state() {
  if (costmap_label_ == nullptr) {
    return;
  }
  std::string text;
  {
    std::lock_guard<std::mutex> lock(costmap_mutex_);
    if (!costmap_dirty_) {
      return;
    }
    costmap_dirty_ = false;
    text = costmap_state_;
  }
  std::map<std::string, std::string> values;
  std::stringstream stream(text);
  std::string item;
  while (std::getline(stream, item, ';')) {
    const auto equal = item.find('=');
    if (equal != std::string::npos) {
      values[item.substr(0, equal)] = item.substr(equal + 1);
    }
  }
  const auto set_spin = [&](QDoubleSpinBox* spin, const std::string& key) {
    const auto found = values.find(key);
    if (found != values.end() && spin != nullptr && !spin->hasFocus()) {
      try {
        spin->setValue(std::stod(found->second));
      } catch (const std::exception&) {
      }
    }
  };
  set_spin(global_inflation_, "global.inflation_radius");
  set_spin(global_scaling_, "global.cost_scaling_factor");
  set_spin(local_inflation_, "local.inflation_radius");
  set_spin(local_scaling_, "local.cost_scaling_factor");
  set_spin(local_width_, "local.width");
  set_spin(local_height_, "local.height");
  costmap_label_->setText(QString("g %1 m / %2 · l %3 m / %4")
                              .arg(QString::fromStdString(values["global.inflation_radius"]))
                              .arg(QString::fromStdString(values["global.cost_scaling_factor"]))
                              .arg(QString::fromStdString(values["local.inflation_radius"]))
                              .arg(QString::fromStdString(values["local.cost_scaling_factor"])) +
                          QString(" / %1x%2 m")
                              .arg(QString::fromStdString(values["local.width"]))
                              .arg(QString::fromStdString(values["local.height"])));
}

}  // namespace krosc::viz
