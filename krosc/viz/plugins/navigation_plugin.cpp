#include "krosc/viz/plugins/navigation_plugin.hpp"

#include <QAbstractSpinBox>
#include <QApplication>
#include <QDockWidget>
#include <QGridLayout>
#include <QKeyEvent>
#include <QLabel>
#include <QLineEdit>
#include <QPushButton>
#include <QStringList>
#include <QWidget>

#include <cmath>

#include "krosc/math.hpp"
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

  pose_button_ = new QPushButton("2D Pose Estimate", panel);
  goal_button_ = new QPushButton("2D Goal Pose", panel);
  grid->addWidget(pose_button_, 0, 0, 1, 2);
  grid->addWidget(goal_button_, 0, 2, 1, 2);
  connect(pose_button_, &QPushButton::clicked, this,
          [this] { set_tool(active_tool_ == "initialpose" ? "" : "initialpose"); });
  connect(goal_button_, &QPushButton::clicked, this,
          [this] { set_tool(active_tool_ == "goal" ? "" : "goal"); });

  multi_button_ = new QPushButton("Multi Goal", panel);
  auto* send = new QPushButton("Send", panel);
  auto* clear = new QPushButton("Clear", panel);
  auto* cancel = new QPushButton("Cancel", panel);
  grid->addWidget(multi_button_, 1, 0);
  grid->addWidget(send, 1, 1);
  grid->addWidget(clear, 1, 2);
  grid->addWidget(cancel, 1, 3);
  connect(multi_button_, &QPushButton::clicked, this,
          [this] { set_tool(active_tool_ == "multi" ? "" : "multi"); });
  connect(send, &QPushButton::clicked, this, [this] { publish_multi(preview_mode_); });
  connect(clear, &QPushButton::clicked, this, [this] {
    multi_goals_.clear();
    sync_waypoints();
    update_multi_label();
    host_->model().node().publish("cancel_goal", msgs::Bool{true}, QOS_DEFAULT);
    status_->setText("goals cleared");
  });
  connect(cancel, &QPushButton::clicked, this, [this] {
    multi_goals_.clear();
    sync_waypoints();
    update_multi_label();
    host_->model().node().publish("cancel_goal", msgs::Bool{true}, QOS_DEFAULT);
    status_->setText("cancel requested");
  });

  multi_label_ = new QLabel("multi goals: 0", panel);
  multi_label_->setWordWrap(true);
  grid->addWidget(multi_label_, 2, 0, 1, 4);

  preview_button_ = new QPushButton("Preview OFF", panel);
  preview_button_->setCheckable(true);
  auto* go = new QPushButton("Go", panel);
  grid->addWidget(preview_button_, 3, 0, 1, 2);
  grid->addWidget(go, 3, 2, 1, 2);
  connect(preview_button_, &QPushButton::toggled, this, [this](bool checked) {
    preview_mode_ = checked;
    preview_button_->setText(checked ? "Preview ON" : "Preview OFF");
  });
  connect(go, &QPushButton::clicked, this, [this] { send_pending(); });

  status_ = new QLabel("", panel);
  status_->setWordWrap(true);
  status_->setMinimumHeight(28);
  grid->addWidget(status_, 4, 0, 1, 4);

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
}

void NavigationPlugin::shutdown() {
  if (!active_) {
    return;
  }
  active_ = false;
  qApp->removeEventFilter(this);
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
  style(goal_button_, "2D Goal Pose", active_tool_ == "goal");
  style(multi_button_, "Multi Goal", active_tool_ == "multi");
  if (preview_button_ != nullptr) {
    preview_button_->setText(preview_mode_ ? "Preview ON" : "Preview OFF");
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
    multi_label_->setText("multi goals: 0");
    return;
  }
  QStringList parts;
  for (std::size_t index = 0; index < multi_goals_.size(); ++index) {
    parts << QString("#%1 (%2, %3)")
                 .arg(index + 1)
                 .arg(std::get<0>(multi_goals_[index]), 0, 'f', 2)
                 .arg(std::get<1>(multi_goals_[index]), 0, 'f', 2);
  }
  multi_label_->setText(QString("multi goals: %1  %2").arg(multi_goals_.size()).arg(parts.join("  ")));
}

void NavigationPlugin::remove_last_multi_goal() {
  if (multi_goals_.empty()) {
    return;
  }
  multi_goals_.pop_back();
  sync_waypoints();
  update_multi_label();
  if (preview_mode_) {
    publish_multi(true);
  }
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
    publish_pose("initialpose", x, y, yaw, from_map);
  } else if (active_tool_ == "goal") {
    last_single_goal_ = std::make_tuple(x, y, yaw);
    publish_pose(preview_mode_ ? "preview_goal" : "goal_pose", x, y, yaw, from_map);
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
    status_->setText("multi goals: 0");
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
  status_->setText(QString("sent %1 goals").arg(multi_goals_.size()));
}

void NavigationPlugin::send_pending() {
  if (last_single_goal_) {
    publish_pose("goal_pose", std::get<0>(*last_single_goal_), std::get<1>(*last_single_goal_),
                 std::get<2>(*last_single_goal_), true);
  }
  if (!multi_goals_.empty()) {
    publish_multi(preview_mode_);
  }
}

}  // namespace krosc::viz
