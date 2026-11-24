#include "krosc/viz/viz_window.hpp"

#include <QAbstractItemView>
#include <QCheckBox>
#include <QComboBox>
#include <QGridLayout>
#include <QGroupBox>
#include <QLineEdit>
#include <QHBoxLayout>
#include <QHeaderView>
#include <QLabel>
#include <QPushButton>
#include <QSignalBlocker>
#include <QSlider>
#include <QSplitter>
#include <QTabWidget>
#include <QStringList>
#include <QStatusBar>
#include <QTableWidget>
#include <QTimer>
#include <QVBoxLayout>
#include <QWidget>

#include <cmath>
#include <sstream>
#include <utility>

#include "krosc/math.hpp"
#include "krosc/viz/map_view.hpp"
#include "krosc/msgs.hpp"
#include "krosc/qos.hpp"

namespace krosc::viz {

namespace {

const char* kStyle = R"(
QMainWindow, QWidget { background: #0d1016; color: #d7dde8; }
QLabel { color: #d7dde8; }
QLabel#Title { font-size: 19px; font-weight: 600; color: #f2f5fa; }
QLabel#Subtitle { color: #7f899b; }
QLabel#Section { color: #9aa6b9; font-weight: 600; margin-top: 8px; }
QComboBox, QPushButton {
  background: #171c26; border: 1px solid #2a3242; border-radius: 4px; padding: 4px 8px; color: #d7dde8;
}
QComboBox::drop-down { border: none; }
QPushButton:hover { background: #202838; }
QCheckBox { color: #c4ccda; spacing: 6px; }
QCheckBox::indicator { width: 14px; height: 14px; border-radius: 3px; border: 1px solid #3a4457; background: #141924; }
QCheckBox::indicator:checked { background: #3f8cff; border-color: #3f8cff; }
QTableWidget {
  background: #10151e; alternate-background-color: #131923; border: 1px solid #222a38; border-radius: 4px;
  gridline-color: #1d2532; color: #c5cddb; selection-background-color: #26334a;
}
QHeaderView::section { background: #151b26; color: #8f9bae; border: none; padding: 4px; }
QStatusBar { color: #7f899b; }
QGroupBox { border: 1px solid #2a3242; border-radius: 4px; margin-top: 10px; padding-top: 8px; color: #9aa6b9; font-weight: 600; }
QGroupBox::title { subcontrol-origin: margin; left: 8px; padding: 0 4px; }
QLineEdit { background: #171c26; border: 1px solid #2a3242; border-radius: 4px; padding: 4px 6px; color: #d7dde8; }
QSlider::groove:horizontal { height: 4px; background: #222a38; border-radius: 2px; }
QSlider::handle:horizontal { background: #3f8cff; width: 12px; margin: -5px 0; border-radius: 6px; }
)";

struct ToggleSpec {
  const char* key;
  const char* label;
};

constexpr ToggleSpec kToggles[] = {
    {"grid", "Grid"},          {"tf", "TF axes"},        {"model", "Robot model"}, {"scan", "Laser scan"},
    {"imu", "IMU"},            {"odom", "Wheel odom"},   {"filtered", "EKF filtered"}, {"map", "Map"},
    {"global_costmap", "Global costmap"}, {"local_costmap", "Local costmap"}, {"plan", "Plan"},
    {"local_plan", "Local plan"}, {"trail", "Trail"},    {"logs", "Logs"},
};

}  // namespace

VizWindow::VizWindow(Context& context, QWidget* parent) : QMainWindow(parent), model_(context) {
  setWindowTitle("krosc viz");
  resize(1440, 860);
  setStyleSheet(kStyle);

  auto* central = new QWidget(this);
  auto* layout = new QHBoxLayout(central);
  layout->setContentsMargins(0, 0, 0, 0);
  layout->setSpacing(0);

  // 패널이 scene_에 연결하므로 scene을 먼저 만든다. 폭은 QSplitter로 조절한다.
  auto* splitter = new QSplitter(Qt::Horizontal, central);
  splitter->setChildrenCollapsible(false);
  splitter->setHandleWidth(5);
  tabs_ = new QTabWidget(splitter);
  auto* tabs = tabs_;
  scene_ = new SceneView(model_, tabs);
  scene_->setObjectName("scene");
  map_view_ = new MapView2D(model_, tabs);
  tabs->addTab(scene_, "3D");
  tabs->addTab(map_view_, "Map");

  auto* panel = new QWidget(splitter);
  panel->setMinimumWidth(300);
  panel->setMaximumWidth(760);
  build_panel(panel);
  splitter->addWidget(panel);
  splitter->addWidget(tabs);
  splitter->setStretchFactor(0, 0);
  splitter->setStretchFactor(1, 1);
  splitter->setSizes({390, 1050});
  layout->addWidget(splitter, 1);
  setCentralWidget(central);

  statusBar()->showMessage(scene_->help_text());

  connect(scene_, &SceneView::follow_changed, this, [this](bool follow) {
    if (auto* box = findChild<QCheckBox*>("followBox"); box != nullptr && box->isChecked() != follow) {
      const QSignalBlocker blocker(box);
      box->setChecked(follow);
    }
  });

  paint_timer_ = new QTimer(this);
  connect(paint_timer_, &QTimer::timeout, this, [this] {
    if (scene_ != nullptr) {
      scene_->update();
    }
    if (map_view_ != nullptr) {
      map_view_->update();  // 지도가 늦게 와도 탭 전환 없이 바로 그린다
    }
  });
  paint_timer_->start(33);

  refresh_timer_ = new QTimer(this);
  connect(refresh_timer_, &QTimer::timeout, this, &VizWindow::refresh_topics);
  refresh_timer_->start(500);
  refresh_topics();

  slam_timer_ = new QTimer(this);
  connect(slam_timer_, &QTimer::timeout, this, &VizWindow::refresh_slam);
  slam_timer_->start(300);
  connect(scene_, &SceneView::tool_changed, this, [this](const QString&) { update_tool_buttons(); });
  connect(scene_, &SceneView::tool_pose, this,
          [this](double x, double y, double yaw) { handle_tool_pose(scene_->tool(), x, y, yaw, false); });
  connect(map_view_, &MapView2D::tool_changed, this, [this](const QString&) { update_tool_buttons(); });
  connect(map_view_, &MapView2D::tool_pose, this,
          [this](double x, double y, double yaw) { handle_tool_pose(map_view_->tool(), x, y, yaw, true); });
  connect(scene_, &SceneView::teleop_scale_changed, this, [this](double scale) {
    if (teleop_slider_ != nullptr) {
      const QSignalBlocker blocker(teleop_slider_);
      teleop_slider_->setValue(static_cast<int>(std::lround(scale * 100.0)));
    }
    if (teleop_label_ != nullptr) {
      teleop_label_->setText(QString("teleop %1 m/s").arg(0.2 * scale, 0, 'f', 2));
    }
  });
  refresh_slam();
  set_fixed_frame(model_.fixed_frame());
}

VizWindow::~VizWindow() {
  model_.node().shutdown();
  if (scene_ != nullptr) {
    scene_->setParent(nullptr);
    delete scene_;
    scene_ = nullptr;
  }
}

void VizWindow::build_panel(QWidget* panel) {
  auto* layout = new QVBoxLayout(panel);
  layout->setContentsMargins(16, 14, 16, 12);
  layout->setSpacing(8);

  auto* title = new QLabel("krosc viz", panel);
  title->setObjectName("Title");
  layout->addWidget(title);
  auto* subtitle = new QLabel("topic-driven viewer · Qt6 / QPainter", panel);
  subtitle->setObjectName("Subtitle");
  layout->addWidget(subtitle);
  connection_label_ = new QLabel("local topics", panel);
  connection_label_->setObjectName("Subtitle");
  connection_label_->setTextInteractionFlags(Qt::TextSelectableByMouse);
  layout->addWidget(connection_label_);

  auto* frame_row = new QHBoxLayout();
  frame_row->addWidget(new QLabel("Fixed frame", panel));
  fixed_frame_ = new QComboBox(panel);
  fixed_frame_->addItems({"map", "odom", "base_footprint", "base_link"});
  frame_row->addWidget(fixed_frame_, 1);
  layout->addLayout(frame_row);

  auto* action_row = new QHBoxLayout();
  auto* follow = new QCheckBox("Follow robot", panel);
  follow->setObjectName("followBox");
  follow->setChecked(true);
  connect(follow, &QCheckBox::toggled, scene_, &SceneView::set_follow);
  action_row->addWidget(follow);
  auto* reset = new QPushButton("Reset view", panel);
  connect(reset, &QPushButton::clicked, scene_, &SceneView::reset_camera);
  action_row->addWidget(reset);
  layout->addLayout(action_row);

  build_slam_panel(panel, layout);

  auto* displays = new QLabel("Displays", panel);
  displays->setObjectName("Section");
  layout->addWidget(displays);

  auto* grid = new QGridLayout();
  grid->setHorizontalSpacing(10);
  grid->setVerticalSpacing(4);
  int row = 0;
  int column = 0;
  for (const auto& spec : kToggles) {
    auto* box = new QCheckBox(spec.label, panel);
    box->setChecked(scene_->enabled(spec.key));
    connect(box, &QCheckBox::toggled, this, [this, key = std::string(spec.key)](bool checked) {
      scene_->set_enabled(key, checked);
    });
    toggles_[spec.key] = box;
    grid->addWidget(box, row, column);
    column = (column + 1) % 2;
    if (column == 0) {
      ++row;
    }
  }
  layout->addLayout(grid);

  auto* topics = new QLabel("Topics", panel);
  topics->setObjectName("Section");
  layout->addWidget(topics);

  table_ = new QTableWidget(0, 3, panel);
  table_->setHorizontalHeaderLabels({"topic", "type", "Hz"});
  table_->horizontalHeader()->setSectionResizeMode(0, QHeaderView::Stretch);
  table_->horizontalHeader()->setSectionResizeMode(1, QHeaderView::Fixed);
  table_->horizontalHeader()->setSectionResizeMode(2, QHeaderView::Fixed);
  table_->setColumnWidth(1, 150);
  table_->setColumnWidth(2, 46);
  table_->setTextElideMode(Qt::ElideRight);
  table_->verticalHeader()->setVisible(false);
  table_->setEditTriggers(QAbstractItemView::NoEditTriggers);
  table_->setSelectionMode(QAbstractItemView::NoSelection);
  table_->setAlternatingRowColors(true);
  table_->setShowGrid(true);
  table_->setWordWrap(false);
  table_->verticalHeader()->setDefaultSectionSize(24);
  table_->setMinimumHeight(120);
  layout->addWidget(table_, 1);

  stats_label_ = new QLabel(panel);
  stats_label_->setObjectName("Subtitle");
  layout->addWidget(stats_label_);

  auto* help = new QLabel(
      "W/A/S/D teleop · Q/Z speed · F follow · C overview\n"
      "left drag orbit · right drag pan · wheel zoom · esc close",
      panel);
  help->setObjectName("Subtitle");
  help->setWordWrap(true);
  layout->addWidget(help);

  connect(fixed_frame_, &QComboBox::currentTextChanged, this,
          [this](const QString& text) { set_fixed_frame(text.toStdString()); });
}


std::string VizWindow::slam_status_text() const {
  if (const auto status = model_.slam_status()) {
    return status->data;
  }
  return "idle";
}

std::string VizWindow::map_status_text() const {
  if (const auto status = model_.map_status()) {
    return status->data;
  }
  return {};
}

void VizWindow::build_slam_panel(QWidget* panel, QVBoxLayout* layout) {
  auto* group = new QGroupBox("SLAM / Map", panel);
  auto* grid = new QGridLayout(group);
  grid->setContentsMargins(8, 10, 8, 8);
  grid->setHorizontalSpacing(6);
  grid->setVerticalSpacing(5);

  slam_mode_label_ = new QLabel("SLAM: -", group);
  slam_button_ = new QPushButton("start SLAM", group);
  grid->addWidget(slam_mode_label_, 0, 0, 1, 2);
  grid->addWidget(slam_button_, 0, 2, 1, 2);
  connect(slam_button_, &QPushButton::clicked, this, [this] {
    const std::string target = slam_status_text() == "mapping" ? "idle" : "mapping";
    model_.node().publish("slam_mode", msgs::String{target}, QOS_DEFAULT);
    slam_status_label_->setText(QString("slam_mode: %1").arg(QString::fromStdString(target)));
  });

  map_combo_ = new QComboBox(group);
  map_combo_->addItem("(select map)");
  map_combo_->setMinimumWidth(200);
  grid->addWidget(new QLabel("map", group), 1, 0);
  grid->addWidget(map_combo_, 1, 1, 1, 2);
  auto* load = new QPushButton("load", group);
  grid->addWidget(load, 1, 3);
  connect(load, &QPushButton::clicked, this, [this] {
    if (map_combo_->currentIndex() <= 0) {
      model_.node().publish("map_unload", msgs::Empty{}, QOS_DEFAULT);
      slam_status_label_->setText("unloading map (none) ...");
      return;
    }
    const QString name = map_combo_->currentText();
    if (name.isEmpty()) {
      slam_status_label_->setText("select a map first");
      return;
    }
    model_.node().publish("map_change", msgs::String{name.toStdString()}, QOS_DEFAULT);
    slam_status_label_->setText(QString("loading %1 ...").arg(name));
  });

  map_name_ = new QLineEdit(group);
  map_name_->setPlaceholderText("map name (auto if empty)");
  grid->addWidget(map_name_, 2, 0, 1, 3);
  auto* save = new QPushButton("save", group);
  grid->addWidget(save, 2, 3);
  connect(save, &QPushButton::clicked, this, [this] {
    const std::string name = map_name_->text().trimmed().toStdString();
    model_.node().publish("map_save", msgs::String{name}, QOS_DEFAULT);
    slam_status_label_->setText(name.empty() ? QString("saving (auto name) ...")
                                             : QString("saving %1 ...").arg(QString::fromStdString(name)));
  });

  auto* unload = new QPushButton("unload", group);
  auto* remove = new QPushButton("delete", group);
  grid->addWidget(unload, 3, 0);
  grid->addWidget(remove, 3, 1);
  connect(unload, &QPushButton::clicked, this, [this] {
    model_.node().publish("map_unload", msgs::Empty{}, QOS_DEFAULT);
    slam_status_label_->setText("unloading ...");
  });
  connect(remove, &QPushButton::clicked, this, [this] {
    model_.node().publish("map_change", msgs::String{""}, QOS_DEFAULT);
    slam_status_label_->setText("deleting loaded map ...");
  });
  slam_status_label_ = new QLabel("", group);
  slam_status_label_->setWordWrap(true);
  slam_status_label_->setMinimumHeight(30);
  grid->addWidget(slam_status_label_, 4, 0, 1, 4);

  pose_button_ = new QPushButton("2D Pose Estimate", group);
  goal_button_ = new QPushButton("2D Goal Pose", group);
  grid->addWidget(pose_button_, 5, 0, 1, 2);
  grid->addWidget(goal_button_, 5, 2, 1, 2);
  connect(pose_button_, &QPushButton::clicked, this,
          [this] { set_tool(active_tool_ == "initialpose" ? "" : "initialpose"); });
  connect(goal_button_, &QPushButton::clicked, this,
          [this] { set_tool(active_tool_ == "goal" ? "" : "goal"); });

  multi_button_ = new QPushButton("Multi Goal", group);
  auto* send = new QPushButton("Send", group);
  auto* clear = new QPushButton("Clear", group);
  auto* cancel = new QPushButton("Cancel", group);
  grid->addWidget(multi_button_, 6, 0);
  grid->addWidget(send, 6, 1);
  grid->addWidget(clear, 6, 2);
  grid->addWidget(cancel, 6, 3);
  connect(multi_button_, &QPushButton::clicked, this,
          [this] { set_tool(active_tool_ == "multi" ? "" : "multi"); });
  connect(send, &QPushButton::clicked, this, [this] { publish_multi(preview_mode_); });
  multi_label_ = new QLabel("multi goals: 0", group);
  multi_label_->setWordWrap(true);
  grid->addWidget(multi_label_, 7, 0, 1, 4);
  connect(clear, &QPushButton::clicked, this, [this] {
    multi_goals_.clear();
    sync_waypoints();
    if (multi_label_ != nullptr) multi_label_->setText("multi goals: 0");
    model_.node().publish("cancel_goal", msgs::Empty{}, QOS_DEFAULT);
    slam_status_label_->setText("goals cleared");
  });
  connect(cancel, &QPushButton::clicked, this, [this] {
    multi_goals_.clear();
    sync_waypoints();
    if (multi_label_ != nullptr) multi_label_->setText("multi goals: 0");
    model_.node().publish("cancel_goal", msgs::Empty{}, QOS_DEFAULT);
    slam_status_label_->setText("cancel requested");
  });

  preview_button_ = new QPushButton("Preview OFF", group);
  preview_button_->setCheckable(true);
  auto* go = new QPushButton("Go", group);
  grid->addWidget(preview_button_, 8, 0, 1, 2);
  grid->addWidget(go, 8, 2);
  connect(preview_button_, &QPushButton::toggled, this, [this](bool checked) {
    preview_mode_ = checked;
    preview_button_->setText(checked ? "Preview ON" : "Preview OFF");
  });
  connect(go, &QPushButton::clicked, this, [this] { send_pending(); });

  teleop_label_ = new QLabel("teleop 0.20 m/s", group);
  teleop_slider_ = new QSlider(Qt::Horizontal, group);
  teleop_slider_->setRange(25, 150);
  teleop_slider_->setValue(100);
  grid->addWidget(teleop_label_, 9, 0, 1, 2);
  grid->addWidget(teleop_slider_, 9, 2, 1, 2);
  connect(teleop_slider_, &QSlider::valueChanged, this, [this](int value) {
    scene_->set_teleop_scale(static_cast<double>(value) / 100.0);
    teleop_label_->setText(QString("teleop %1 m/s").arg(0.2 * static_cast<double>(value) / 100.0, 0, 'f', 2));
  });

  layout->addWidget(group);
  update_tool_buttons();
}

void VizWindow::refresh_slam() {
  if (slam_mode_label_ == nullptr) {
    return;
  }
  const std::string mode = slam_status_text();
  slam_mode_label_->setText(QString("SLAM: %1").arg(QString::fromStdString(mode)));
  slam_button_->setText(mode == "mapping" ? "stop SLAM" : "start SLAM");

  std::vector<std::string> names;
  if (const auto list = model_.map_list()) {
    std::istringstream stream(list->data);
    std::string line;
    while (std::getline(stream, line)) {
      if (!line.empty()) {
        names.push_back(line);
      }
    }
  }
  QStringList items;
  items << "(select map)";
  for (const auto& name : names) {
    items << QString::fromStdString(name);
  }
  bool same = map_combo_->count() == items.size();
  if (same) {
    for (int index = 0; index < items.size(); ++index) {
      if (map_combo_->itemText(index) != items[index]) {
        same = false;
        break;
      }
    }
  }
  if (!same) {
    const QString chosen = map_combo_->currentText();
    const QSignalBlocker blocker(map_combo_);
    map_combo_->clear();
    map_combo_->addItems(items);
    const int index = map_combo_->findText(chosen);
    map_combo_->setCurrentIndex(index >= 0 ? index : 0);
  }
  QString status;
  if (const auto map_status = model_.map_status()) {
    const std::string text = map_status->data;
    status = QString::fromStdString(text);
    if (text != last_map_status_) {
      last_map_status_ = text;
      if (text == "map unloaded" || text == "no map loaded" || text.rfind("deleted ", 0) == 0) {
        const QSignalBlocker blocker(map_combo_);
        map_combo_->setCurrentIndex(0);
      } else if (text.rfind("loaded ", 0) == 0 || text.rfind("saved ", 0) == 0) {
        QString name = QString::fromStdString(text.substr(text.find(' ') + 1));
        const int paren = name.indexOf(" (");
        if (paren > 0) {
          name = name.left(paren);
        }
        const int index = map_combo_->findText(name);
        if (index >= 0) {
          const QSignalBlocker blocker(map_combo_);
          map_combo_->setCurrentIndex(index);
        }
      }
    }
  }
  if (const auto map = model_.map()) {
    status += QString(" | map %1x%2").arg(map->info.width).arg(map->info.height);
  } else {
    status += " | map -";
  }
  if (const auto goal = model_.goal_status()) {
    status += QString(" | goal: %1").arg(QString::fromStdString(goal->data));
  }
  slam_status_label_->setText(status);
  teleop_label_->setText(QString("teleop %1 m/s").arg(0.2 * scene_->teleop_scale(), 0, 'f', 2));
  update_tool_buttons();
}

void VizWindow::update_tool_buttons() {
  if (pose_button_ == nullptr) {
    return;
  }
  const std::string tool = active_tool_;
  const auto style = [](QPushButton* button, const QString& text, bool active) {
    button->setText(text + (active ? " *" : ""));
    button->setStyleSheet(active ? "background:#8a5a1a;" : "");
  };
  style(pose_button_, "2D Pose Estimate", tool == "initialpose");
  style(goal_button_, "2D Goal Pose", tool == "goal");
  style(multi_button_, "Multi Goal", tool == "multi");
  preview_button_->setText(preview_mode_ ? "Preview ON" : "Preview OFF");
}

void VizWindow::set_tool(const std::string& tool) {
  active_tool_ = (tool == "initialpose" || tool == "goal" || tool == "multi") ? tool : std::string{};
  if (scene_ != nullptr) {
    scene_->set_tool(active_tool_);
  }
  if (map_view_ != nullptr) {
    map_view_->set_tool(active_tool_);
  }
  update_tool_buttons();
}

void VizWindow::sync_waypoints() {
  if (scene_ != nullptr) {
    scene_->set_waypoints(multi_goals_);
  }
  if (map_view_ != nullptr) {
    map_view_->set_waypoints(multi_goals_);
  }
}

void VizWindow::handle_tool_pose(const std::string& tool, double x, double y, double yaw, bool from_map) {
  if (!from_map && model_.fixed_frame() != "map") {
    try {
      const msgs::TransformStamped transform = model_.tf_buffer().lookup_transform("map", model_.fixed_frame());
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
      slam_status_label_->setText("no map TF (2D Pose Estimate first)");
      return;
    }
  }
  if (tool == "multi") {
    multi_goals_.emplace_back(x, y, yaw);
    if (slam_status_label_ != nullptr) {
      slam_status_label_->setText(QString("multi goals: %1 (Send로 전송)").arg(multi_goals_.size()));
    }
    update_tool_buttons();
    return;
  }
  if (tool == "initialpose") {
    publish_pose("initialpose", x, y, yaw, from_map);
  } else if (tool == "goal") {
    last_single_goal_ = std::make_tuple(x, y, yaw);
    publish_pose(preview_mode_ ? "preview_goal" : "goal_pose", x, y, yaw, from_map);
  }
  set_tool("");
}

void VizWindow::publish_pose(const std::string& topic, double x, double y, double yaw, bool from_map) {
  if (!from_map && model_.fixed_frame() != "map") {
    try {
      const msgs::TransformStamped transform = model_.tf_buffer().lookup_transform("map", model_.fixed_frame());
      const double offset = tfmath::yaw_of(transform.transform.rotation);
      const double c = std::cos(offset);
      const double s = std::sin(offset);
      const double px = x;
      const double py = y;
      x = transform.transform.translation.x + c * px - s * py;
      y = transform.transform.translation.y + s * px + c * py;
      yaw = std::atan2(std::sin(offset + yaw), std::cos(offset + yaw));
    } catch (const TransformException&) {
      slam_status_label_->setText("no map TF (2D Pose Estimate first)");
      return;
    }
  }
  if (topic != "initialpose" && slam_status_text() != "localization") {
    slam_status_label_->setText("load a map first (localization)");
    return;
  }
  msgs::Pose pose;
  pose.position = {x, y, 0.0};
  pose.orientation = tfmath::from_yaw(yaw);
  if (topic == "initialpose") {
    msgs::PoseWithCovarianceStamped message;
    message.header = msgs::Header{model_.node().now(), "map"};
    message.pose.pose = pose;
    message.pose.covariance[0] = 0.25;
    message.pose.covariance[7] = 0.25;
    message.pose.covariance[35] = 0.0685;
    model_.node().publish(topic, message, QOS_DEFAULT);
    slam_status_label_->setText(QString("initialpose (%.2f, %.2f, %.0fdeg)").arg(x).arg(y).arg(yaw * 180.0 / 3.141592653589793));
  } else {
    msgs::PoseStamped message;
    message.header = msgs::Header{model_.node().now(), "map"};
    message.pose = pose;
    model_.node().publish(topic, message, QOS_DEFAULT);
    slam_status_label_->setText(QString("%1 (%.2f, %.2f)").arg(QString::fromStdString(topic)).arg(x).arg(y));
  }
}

void VizWindow::publish_multi(bool preview) {
  if (multi_goals_.empty()) {
    slam_status_label_->setText("multi goals: 0");
    return;
  }
  if (!preview && slam_status_text() != "localization") {
    slam_status_label_->setText("load a map first (localization)");
    return;
  }
  if (model_.fixed_frame() != "map") {
    slam_status_label_->setText("switch fixed frame to map");
    return;
  }
  msgs::Path path;
  path.header = msgs::Header{model_.node().now(), "map"};
  for (const auto& [x, y, yaw] : multi_goals_) {
    msgs::PoseStamped pose;
    pose.header = path.header;
    pose.pose.position = {x, y, 0.0};
    pose.pose.orientation = tfmath::from_yaw(yaw);
    path.poses.push_back(std::move(pose));
  }
  model_.node().publish(preview ? "preview_goals" : "goal_poses", path, QOS_DEFAULT);
  slam_status_label_->setText(QString("sent %1 goals").arg(multi_goals_.size()));
}

void VizWindow::send_pending() {
  if (last_single_goal_) {
    publish_pose("goal_pose", std::get<0>(*last_single_goal_), std::get<1>(*last_single_goal_),
                 std::get<2>(*last_single_goal_), true);
  }
  if (!multi_goals_.empty()) {
    publish_multi(false);
  }
  if (!last_single_goal_ && multi_goals_.empty()) {
    slam_status_label_->setText("no pending goal");
  }
}

void VizWindow::refresh_topics() {
  const auto rows = model_.topic_rows();
  table_->setRowCount(static_cast<int>(rows.size()));
  for (int i = 0; i < static_cast<int>(rows.size()); ++i) {
    const auto& row = rows[static_cast<std::size_t>(i)];
    auto* topic_item = new QTableWidgetItem(QString::fromStdString(row.topic));
    auto* type_item = new QTableWidgetItem(QString::fromStdString(row.type));
    const bool latched = row.topic == "map" || row.topic == "robot_description" || row.topic == "tf_static" ||
                         row.topic.find("costmap") != std::string::npos || row.topic == "plan";
    auto* hz_item = new QTableWidgetItem(row.count == 0 || latched ? "-" : QString::number(row.hz, 'f', 1));
    topic_item->setToolTip(QString::fromStdString(row.topic));
    type_item->setToolTip(QString::fromStdString(row.type));
    hz_item->setToolTip(QString("%1 messages").arg(row.count));
    const QColor color = row.recent ? QColor(110, 230, 165) : QColor(125, 135, 150);
    topic_item->setForeground(color);
    type_item->setForeground(QColor(130, 140, 158));
    hz_item->setForeground(color);
    table_->setItem(i, 0, topic_item);
    table_->setItem(i, 1, type_item);
    table_->setItem(i, 2, hz_item);
  }
  std::size_t live = 0;
  for (const auto& row : rows) {
    live += row.recent ? 1 : 0;
  }
  stats_label_->setText(QString("%1 topics · %2 live · %3 nodes")
                            .arg(rows.size())
                            .arg(live)
                            .arg(model_.node().context().node_names().size()));
}

void VizWindow::set_tab(int index) {
  if (tabs_ != nullptr) {
    tabs_->setCurrentIndex(index == 1 ? 1 : 0);
  }
}

void VizWindow::set_connection_status(bool connected, const std::string& endpoint) {
  if (connection_label_ == nullptr) {
    return;
  }
  if (endpoint.empty()) {
    connection_label_->setText("local topics");
    connection_label_->setStyleSheet("");
    return;
  }
  connection_label_->setText(QString("remote: %1 · %2")
                                 .arg(QString::fromStdString(endpoint))
                                 .arg(connected ? "connected" : "reconnecting…"));
  connection_label_->setStyleSheet(connected ? "color: #6ee6a5;" : "color: #e6a15a;");
}

void VizWindow::set_fixed_frame(const std::string& frame) {
  model_.set_fixed_frame(frame);
  if (fixed_frame_ != nullptr) {
    const QSignalBlocker blocker(fixed_frame_);
    fixed_frame_->setCurrentText(QString::fromStdString(frame));
  }
  if (scene_ != nullptr) {
    scene_->fixed_frame_changed();
  }
}

bool VizWindow::save_screenshot(const QString& path) {
  scene_->update();
  const QPixmap pixmap = grab();
  return pixmap.save(path);
}

}  // namespace krosc::viz
