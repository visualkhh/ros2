#include "krosc/viz/viz_window.hpp"

#include <QAbstractItemView>
#include <QAbstractSlider>
#include <QApplication>
#include <QCheckBox>
#include <QComboBox>
#include <QDockWidget>
#include <QDoubleSpinBox>
#include <QGridLayout>
#include <QGroupBox>
#include <QLineEdit>
#include <QHBoxLayout>
#include <QHeaderView>
#include <QKeyEvent>
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

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <limits>
#include <sstream>
#include <utility>
#include <vector>

#include "krosc/math.hpp"
#include "krosc/urdf.hpp"
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

double steady_seconds() {
  using namespace std::chrono;
  return duration<double>(steady_clock::now().time_since_epoch()).count();
}

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
  build_manual_docks();
  qApp->installEventFilter(this);
  connect(qApp, &QApplication::focusChanged, this, [this](QWidget* old_focus, QWidget* new_focus) {
    if (teleop_dock_ == nullptr) {
      return;
    }
    const bool old_in_teleop =
        old_focus != nullptr && (old_focus == teleop_dock_ || teleop_dock_->isAncestorOf(old_focus));
    const bool new_in_teleop =
        new_focus != nullptr && (new_focus == teleop_dock_ || teleop_dock_->isAncestorOf(new_focus));
    if (old_in_teleop && !new_in_teleop) {
      if (scene_ != nullptr) {
        scene_->clear_teleop_keys();
      }
      teleop_direction_ = 0;
      publish_teleop_pad();
    }
  });

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
          [this](double x, double y, double yaw) { handle_tool_pose(active_tool_, x, y, yaw, false); });
  connect(map_view_, &MapView2D::tool_changed, this, [this](const QString&) { update_tool_buttons(); });
  connect(map_view_, &MapView2D::tool_pose, this,
          [this](double x, double y, double yaw) { handle_tool_pose(active_tool_, x, y, yaw, true); });
  connect(map_view_, &MapView2D::teleop_key, this,
          [this](int key, bool pressed) {
            if (scene_ != nullptr) {
              scene_->set_teleop_key(key, pressed);
            }
          });
  connect(map_view_, &MapView2D::teleop_focus_lost, this, [this] {
    if (scene_ != nullptr) {
      scene_->clear_teleop_keys();
    }
  });
  connect(tabs_, &QTabWidget::currentChanged, this, [this](int index) {
    if (scene_ != nullptr) {
      scene_->clear_teleop_keys();
    }
    if (index == 1 && map_view_ != nullptr) {
      map_view_->setFocus(Qt::OtherFocusReason);
    } else if (index == 0 && scene_ != nullptr) {
      scene_->setFocus(Qt::OtherFocusReason);
    }
  });
  connect(scene_, &SceneView::teleop_scale_changed, this, [this](double scale) {
    if (teleop_slider_ != nullptr) {
      const QSignalBlocker blocker(teleop_slider_);
      teleop_slider_->setValue(static_cast<int>(std::lround(scale * 100.0)));
    }
    if (teleop_label_ != nullptr) {
      teleop_label_->setText(QString("teleop %1 m/s").arg(0.26 * scale, 0, 'f', 2));
    }
  });
  refresh_slam();
  set_fixed_frame(model_.fixed_frame());
}

VizWindow::~VizWindow() {
  qApp->removeEventFilter(this);
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
    if (map_view_ != nullptr) {
      const std::string key = spec.key;
      if (key == "tf") {
        map_view_->set_show_tf(box->isChecked());
      } else if (key == "odom") {
        map_view_->set_show_odom(box->isChecked());
      } else if (key == "filtered") {
        map_view_->set_show_filtered(box->isChecked());
      }
    }
    connect(box, &QCheckBox::toggled, this, [this, key = std::string(spec.key)](bool checked) {
      scene_->set_enabled(key, checked);
      if (map_view_ != nullptr) {
        if (key == "tf") {
          map_view_->set_show_tf(checked);
        } else if (key == "odom") {
          map_view_->set_show_odom(checked);
        } else if (key == "filtered") {
          map_view_->set_show_filtered(checked);
        }
      }
    });
    toggles_[spec.key] = box;
    grid->addWidget(box, row, column);
    column = (column + 1) % 2;
    if (column == 0) {
      ++row;
    }
  }
  layout->addLayout(grid);

  tf_label_ = new QLabel("TF: -", panel);
  tf_label_->setObjectName("Subtitle");
  tf_label_->setWordWrap(true);
  tf_label_->setTextInteractionFlags(Qt::TextSelectableByMouse);
  layout->addWidget(tf_label_);

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
  collision_button_ = new QPushButton("collision ON", group);
  grid->addWidget(collision_button_, 3, 2, 1, 2);
  connect(collision_button_, &QPushButton::clicked, this, [this] {
    collision_enabled_ = !collision_enabled_;
    const msgs::Bool message{collision_enabled_};
    model_.node().publish("collision_set_enabled", message, QOS_DEFAULT);
    update_collision_button();
  });
  update_collision_button();
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
  if (const auto collision = model_.collision_status()) {
    const std::string text = collision->data;
    const bool enabled = text.rfind("disabled", 0) != 0;
    if (enabled != collision_enabled_) {
      collision_enabled_ = enabled;
      update_collision_button();
    }
    if (collision_button_ != nullptr) {
      collision_button_->setToolTip(QString::fromStdString(text));
    }
  }
  slam_status_label_->setText(status);
  if (teleop_label_ != nullptr) {
    teleop_label_->setText(QString("teleop %1 m/s").arg(0.26 * scene_->teleop_scale(), 0, 'f', 2));
  }
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

void VizWindow::update_collision_button() {
  if (collision_button_ == nullptr) {
    return;
  }
  collision_button_->setText(collision_enabled_ ? "collision ON" : "collision OFF");
  collision_button_->setStyleSheet(collision_enabled_ ? "background:#1f6b43;" : "background:#7a2a2a;");
}


void VizWindow::build_manual_docks() {
  build_teleop_dock();
  build_manipulator_dock();
  build_moveit_dock();
  teleop_dock_->hide();
  manipulator_dock_->hide();
  moveit_dock_->hide();
}

void VizWindow::build_teleop_dock() {
  teleop_dock_ = new QDockWidget("Teleop", this);
  teleop_dock_->setObjectName("teleopDock");
  teleop_dock_->setAllowedAreas(Qt::LeftDockWidgetArea | Qt::RightDockWidgetArea | Qt::BottomDockWidgetArea);
  teleop_dock_->setFeatures(QDockWidget::DockWidgetMovable | QDockWidget::DockWidgetFloatable |
                            QDockWidget::DockWidgetClosable);
  auto* panel = new QWidget(teleop_dock_);
  auto* layout = new QVBoxLayout(panel);
  layout->setContentsMargins(8, 8, 8, 8);
  layout->setSpacing(6);

  auto* pad = new QGridLayout();
  auto* forward = new QPushButton("↑", panel);
  auto* backward = new QPushButton("↓", panel);
  auto* left = new QPushButton("←", panel);
  auto* right = new QPushButton("→", panel);
  auto* stop = new QPushButton("STOP", panel);
  const auto bind = [this](QPushButton* button, int bit) {
    button->setMinimumSize(46, 34);
    connect(button, &QPushButton::pressed, this, [this, bit] { teleop_direction_ |= bit; });
    connect(button, &QPushButton::released, this, [this, bit] { teleop_direction_ &= ~bit; });
  };
  bind(forward, 1);
  bind(backward, 2);
  bind(left, 4);
  bind(right, 8);
  pad->addWidget(forward, 0, 1);
  pad->addWidget(left, 1, 0);
  pad->addWidget(stop, 1, 1);
  pad->addWidget(right, 1, 2);
  pad->addWidget(backward, 2, 1);
  layout->addLayout(pad);
  connect(stop, &QPushButton::clicked, this, [this] {
    teleop_direction_ = 0;
    publish_teleop_pad();
  });

  auto* speed_row = new QHBoxLayout();
  speed_row->addWidget(new QLabel("speed", panel));
  teleop_slider_ = new QSlider(Qt::Horizontal, panel);
  teleop_slider_->setRange(25, 150);
  teleop_slider_->setValue(100);
  teleop_label_ = new QLabel("0.26 m/s", panel);
  speed_row->addWidget(teleop_slider_, 1);
  speed_row->addWidget(teleop_label_);
  layout->addLayout(speed_row);
  connect(teleop_slider_, &QSlider::valueChanged, this, [this](int value) {
    const double scale = static_cast<double>(value) / 100.0;
    scene_->set_teleop_scale(scale);
    teleop_label_->setText(QString("%1 m/s").arg(0.26 * scale, 0, 'f', 2));
  });
  teleop_dock_->setWidget(panel);
  addDockWidget(Qt::RightDockWidgetArea, teleop_dock_);
  connect(teleop_dock_, &QDockWidget::visibilityChanged, this, [this](bool visible) {
    if (!visible) {
      teleop_direction_ = 0;
      publish_teleop_pad();
    }
  });
  teleop_pad_timer_ = new QTimer(this);
  connect(teleop_pad_timer_, &QTimer::timeout, this, &VizWindow::publish_teleop_pad);
  teleop_pad_timer_->start(50);
}

void VizWindow::build_manipulator_dock() {
  manipulator_dock_ = new QDockWidget("Manipulator", this);
  manipulator_dock_->setObjectName("manipulatorDock");
  manipulator_dock_->setAllowedAreas(Qt::LeftDockWidgetArea | Qt::RightDockWidgetArea | Qt::BottomDockWidgetArea);
  manipulator_dock_->setFeatures(QDockWidget::DockWidgetMovable | QDockWidget::DockWidgetFloatable |
                                QDockWidget::DockWidgetClosable);
  auto* panel = new QWidget(manipulator_dock_);
  auto* layout = new QVBoxLayout(panel);
  layout->setContentsMargins(8, 8, 8, 8);
  layout->setSpacing(5);

  auto* buttons = new QHBoxLayout();
  arm_torque_button_ = new QPushButton("Torque ON", panel);
  auto* stow = new QPushButton("STOW", panel);
  auto* home = new QPushButton("HOME", panel);
  auto* estop = new QPushButton("E-STOP", panel);
  for (auto* button : {arm_torque_button_, stow, home, estop}) {
    buttons->addWidget(button);
  }
  layout->addLayout(buttons);
  auto* reset_row = new QHBoxLayout();
  auto* reset_name = new QLabel("reset", panel);
  reset_name->setMinimumWidth(46);
  arm_reset_target_ = new QComboBox(panel);
  arm_reset_target_->addItems({"all", "joint1", "joint2", "joint3", "joint4", "gripper"});
  arm_reset_button_ = new QPushButton("Reset (AR)", panel);
  reset_row->addWidget(reset_name);
  reset_row->addWidget(arm_reset_target_, 1);
  reset_row->addWidget(arm_reset_button_);
  layout->addLayout(reset_row);
  auto set_torque_button = [this] {
    arm_torque_button_->setText(arm_torque_on_ ? "Torque ON" : "Torque OFF");
    arm_torque_button_->setStyleSheet(arm_torque_on_ ? "background:#1f6b43;" : "background:#7a2a2a;");
  };
  connect(arm_torque_button_, &QPushButton::clicked, this, [this, set_torque_button] {
    arm_torque_on_ = !arm_torque_on_;
    if (!arm_torque_on_) {
      arm_moving_ = false;
    }
    model_.node().publish("arm_torque", msgs::Bool{arm_torque_on_}, QOS_DEFAULT);
    set_torque_button();
    arm_status_label_->setText(arm_torque_on_ ? "torque ON" : "torque OFF (arm relaxed)");
  });
  set_torque_button();

  const std::array<std::pair<int, int>, 4> ranges{{{ -180, 180 }, { -110, 110 }, { -90, 80 }, { -97, 113 }}};
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
    arm_sliders_[static_cast<std::size_t>(i)] = slider;
    arm_value_labels_[static_cast<std::size_t>(i)] = value;
    connect(slider, &QSlider::valueChanged, this, [this, i, value](int degrees) {
      value->setText(QString::number(degrees, 'f', 1) + "°");
      if (!arm_updating_) {
        arm_target_[static_cast<std::size_t>(i)] = static_cast<double>(degrees) * 3.141592653589793 / 180.0;
        arm_moving_ = true;
      }
    });
  }
  auto* gripper_row = new QHBoxLayout();
  auto* gripper_name = new QLabel("gripper", panel);
  gripper_name->setMinimumWidth(46);
  gripper_slider_ = new QSlider(Qt::Horizontal, panel);
  gripper_slider_->setRange(0, 27);
  gripper_slider_->setValue(0);
  gripper_value_label_ = new QLabel("0.0 mm", panel);
  gripper_value_label_->setMinimumWidth(52);
  gripper_value_label_->setAlignment(Qt::AlignRight | Qt::AlignVCenter);
  gripper_row->addWidget(gripper_name);
  gripper_row->addWidget(gripper_slider_, 1);
  gripper_row->addWidget(gripper_value_label_);
  layout->addLayout(gripper_row);
  connect(gripper_slider_, &QSlider::valueChanged, this, [this](int mm) {
    gripper_value_label_->setText(QString::number(mm, 'f', 1) + " mm");
    if (!arm_updating_) {
      arm_target_[4] = static_cast<double>(mm) / 1000.0;
      arm_moving_ = true;
    }
  });
  auto* arm_speed_row = new QHBoxLayout();
  auto* arm_speed_name = new QLabel("speed", panel);
  arm_speed_name->setMinimumWidth(46);
  arm_speed_slider_ = new QSlider(Qt::Horizontal, panel);
  arm_speed_slider_->setRange(10, 80);
  arm_speed_slider_->setValue(35);
  arm_speed_label_ = new QLabel("0.35 rad/s", panel);
  arm_speed_label_->setMinimumWidth(72);
  arm_speed_label_->setAlignment(Qt::AlignRight | Qt::AlignVCenter);
  arm_speed_row->addWidget(arm_speed_name);
  arm_speed_row->addWidget(arm_speed_slider_, 1);
  arm_speed_row->addWidget(arm_speed_label_);
  layout->addLayout(arm_speed_row);
  connect(arm_speed_slider_, &QSlider::valueChanged, this, [this](int value) {
    arm_speed_ = static_cast<double>(value) / 100.0;
    arm_speed_label_->setText(QString("%1 rad/s").arg(arm_speed_, 0, 'f', 2));
  });

  arm_status_label_ = new QLabel("arm: waiting for joint_states", panel);
  arm_status_label_->setWordWrap(true);
  arm_status_label_->setMinimumHeight(36);
  layout->addWidget(arm_status_label_);

  const auto set_preset = [this](const std::array<double, 5>& positions) {
    arm_updating_ = true;
    for (int i = 0; i < 4; ++i) {
      arm_sliders_[static_cast<std::size_t>(i)]->setValue(
          static_cast<int>(std::lround(positions[static_cast<std::size_t>(i)] * 180.0 / 3.141592653589793)));
    }
    if (std::isfinite(positions[4])) {
      gripper_slider_->setValue(static_cast<int>(std::lround(positions[4] * 1000.0)));
    }
    arm_updating_ = false;
    arm_target_ = positions;
    arm_moving_ = true;
  };
  connect(stow, &QPushButton::clicked, this, [this, set_preset] {
    set_preset({0.0, -1.9024, 1.3440, 0.9023, 0.0});
    arm_status_label_->setText("STOW target");
  });
  connect(home, &QPushButton::clicked, this, [this, set_preset] {
    set_preset({0.0, 0.0, 0.0, 0.0, 0.0});
    arm_status_label_->setText("HOME target");
  });
  connect(estop, &QPushButton::clicked, this, [this] {
    model_.node().publish("arm_stop", msgs::Empty{}, QOS_DEFAULT);
    arm_moving_ = false;
    arm_status_label_->setText("E-STOP: hold current pose");
  });
  connect(arm_reset_button_, &QPushButton::clicked, this, &VizWindow::reset_arm);

  manipulator_dock_->setWidget(panel);
  addDockWidget(Qt::RightDockWidgetArea, manipulator_dock_);
  splitDockWidget(teleop_dock_, manipulator_dock_, Qt::Vertical);
  arm_send_timer_ = new QTimer(this);
  connect(arm_send_timer_, &QTimer::timeout, this, &VizWindow::send_arm_target);
  arm_send_timer_->start(50);
  manual_refresh_timer_ = new QTimer(this);
  connect(manual_refresh_timer_, &QTimer::timeout, this, &VizWindow::refresh_manual);
  manual_refresh_timer_->start(100);
}

void VizWindow::build_moveit_dock() {
  moveit_dock_ = new QDockWidget("MoveIt", this);
  moveit_dock_->setObjectName("moveitDock");
  moveit_dock_->setAllowedAreas(Qt::LeftDockWidgetArea | Qt::RightDockWidgetArea | Qt::BottomDockWidgetArea);
  moveit_dock_->setFeatures(QDockWidget::DockWidgetMovable | QDockWidget::DockWidgetFloatable |
                            QDockWidget::DockWidgetClosable);
  auto* panel = new QWidget(moveit_dock_);
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
  moveit_x_ = make_spin(-0.8, 0.8, 0.005, 3, " m");
  moveit_y_ = make_spin(-0.8, 0.8, 0.005, 3, " m");
  moveit_z_ = make_spin(-0.2, 1.0, 0.005, 3, " m");
  moveit_pitch_ = make_spin(-90.0, 90.0, 1.0, 1, QString::fromUtf8("°"));
  moveit_gripper_ = make_spin(0.0, 27.0, 0.5, 1, " mm");
  auto* grid = new QGridLayout();
  grid->addWidget(new QLabel("X", panel), 0, 0);
  grid->addWidget(moveit_x_, 0, 1);
  grid->addWidget(new QLabel("Y", panel), 0, 2);
  grid->addWidget(moveit_y_, 0, 3);
  grid->addWidget(new QLabel("Z", panel), 1, 0);
  grid->addWidget(moveit_z_, 1, 1);
  grid->addWidget(new QLabel("Pitch", panel), 1, 2);
  grid->addWidget(moveit_pitch_, 1, 3);
  grid->addWidget(new QLabel("Grip", panel), 2, 0);
  grid->addWidget(moveit_gripper_, 2, 1);
  moveit_cartesian_ = new QCheckBox("Cartesian", panel);
  moveit_cartesian_->setChecked(true);
  grid->addWidget(moveit_cartesian_, 2, 2, 1, 2);
  layout->addLayout(grid);

  auto* buttons = new QHBoxLayout();
  moveit_current_button_ = new QPushButton("Current", panel);
  moveit_plan_button_ = new QPushButton("Plan", panel);
  moveit_execute_button_ = new QPushButton("Execute", panel);
  moveit_cancel_button_ = new QPushButton("Cancel", panel);
  for (auto* button : {moveit_current_button_, moveit_plan_button_, moveit_execute_button_,
                       moveit_cancel_button_}) {
    buttons->addWidget(button);
  }
  layout->addLayout(buttons);
  moveit_execute_button_->setEnabled(false);
  moveit_cancel_button_->setEnabled(false);

  layout->addWidget(new QLabel("trajectory", panel));
  moveit_scrub_ = new QSlider(Qt::Horizontal, panel);
  moveit_scrub_->setRange(0, 0);
  layout->addWidget(moveit_scrub_);
  moveit_current_label_ = new QLabel("now: waiting for joint_states", panel);
  moveit_current_label_->setWordWrap(true);
  moveit_status_label_ = new QLabel("MoveIt: waiting for robot_description", panel);
  moveit_status_label_->setWordWrap(true);
  moveit_status_label_->setMinimumHeight(44);
  layout->addWidget(moveit_current_label_);
  layout->addWidget(moveit_status_label_);
  layout->addStretch(1);

  const auto target_changed = [this] {
    if (moveit_updating_) {
      return;
    }
    moveit_invalidate_plan();
    moveit_preview_target();
  };
  for (auto* spin : {moveit_x_, moveit_y_, moveit_z_, moveit_pitch_, moveit_gripper_}) {
    connect(spin, qOverload<double>(&QDoubleSpinBox::valueChanged), this,
            [target_changed](double) { target_changed(); });
  }
  connect(moveit_cartesian_, &QCheckBox::toggled, this, [target_changed](bool) { target_changed(); });
  connect(moveit_current_button_, &QPushButton::clicked, this, &VizWindow::moveit_set_target_from_current);
  connect(moveit_plan_button_, &QPushButton::clicked, this, &VizWindow::moveit_plan);
  connect(moveit_execute_button_, &QPushButton::clicked, this, &VizWindow::moveit_execute);
  connect(moveit_cancel_button_, &QPushButton::clicked, this, [this] { moveit_stop(true); });
  connect(moveit_scrub_, &QSlider::valueChanged, this, [this](int index) {
    if (!moveit_executing_) {
      moveit_update_preview(index);
    }
  });

  moveit_dock_->setWidget(panel);
  addDockWidget(Qt::RightDockWidgetArea, moveit_dock_);
  splitDockWidget(manipulator_dock_, moveit_dock_, Qt::Vertical);
  moveit_exec_timer_ = new QTimer(this);
  connect(moveit_exec_timer_, &QTimer::timeout, this, &VizWindow::moveit_tick);
}

void VizWindow::publish_teleop_pad() {
  const double scale = scene_ == nullptr ? 1.0 : scene_->teleop_scale();
  const double linear = 0.26 * scale * (((teleop_direction_ & 1) != 0 ? 1.0 : 0.0) -
                                         ((teleop_direction_ & 2) != 0 ? 1.0 : 0.0));
  const double angular = 1.5 * scale * (((teleop_direction_ & 4) != 0 ? 1.0 : 0.0) -
                                        ((teleop_direction_ & 8) != 0 ? 1.0 : 0.0));
  if (linear != teleop_last_linear_ || angular != teleop_last_angular_) {
    model_.publish_cmd_vel(linear, angular);
    teleop_last_linear_ = linear;
    teleop_last_angular_ = angular;
  }
}

void VizWindow::send_arm_target() {
  if (!arm_initialized_ || !arm_moving_) {
    return;
  }
  constexpr double dt = 0.05;
  bool moved = false;
  for (int i = 0; i < 4; ++i) {
    double delta = arm_target_[static_cast<std::size_t>(i)] - arm_output_[static_cast<std::size_t>(i)];
    const double step = arm_speed_ * dt;
    if (std::abs(delta) > step) {
      delta = std::copysign(step, delta);
    }
    if (std::abs(delta) > 1e-6) {
      arm_output_[static_cast<std::size_t>(i)] += delta;
      moved = true;
    }
  }
  double gripper_delta = arm_target_[4] - arm_output_[4];
  const double gripper_step = 0.016 * dt;  // 16 mm/s
  if (std::abs(gripper_delta) > gripper_step) {
    gripper_delta = std::copysign(gripper_step, gripper_delta);
  }
  if (std::abs(gripper_delta) > 1e-6) {
    arm_output_[4] += gripper_delta;
    moved = true;
  }
  if (!moved) {
    arm_output_ = arm_target_;
    arm_moving_ = false;
  }
  msgs::JointState command;
  for (int i = 0; i < 4; ++i) {
    command.name.push_back("joint" + std::to_string(i + 1));
    command.position.push_back(arm_output_[static_cast<std::size_t>(i)]);
  }
  command.name.push_back("gripper_left_joint");
  command.position.push_back(arm_output_[4]);
  model_.node().publish("arm_joint_command", command, QOS_DEFAULT);
}

void VizWindow::set_arm_slider_values(const std::array<double, 5>& positions) {
  if (arm_sliders_[0] == nullptr || arm_moving_) {
    return;
  }
  for (int i = 0; i < 4; ++i) {
    if (arm_sliders_[static_cast<std::size_t>(i)]->isSliderDown()) {
      return;
    }
  }
  if (gripper_slider_ != nullptr && gripper_slider_->isSliderDown()) {
    return;
  }
  arm_target_ = positions;
  arm_output_ = positions;
  arm_updating_ = true;
  for (int i = 0; i < 4; ++i) {
    if (!std::isfinite(positions[static_cast<std::size_t>(i)])) {
      continue;
    }
    const int degrees =
        static_cast<int>(std::lround(positions[static_cast<std::size_t>(i)] * 180.0 / 3.141592653589793));
    arm_sliders_[static_cast<std::size_t>(i)]->setValue(degrees);
    arm_value_labels_[static_cast<std::size_t>(i)]->setText(QString::number(degrees, 'f', 1) + "°");
  }
  if (gripper_slider_ != nullptr && std::isfinite(positions[4])) {
    const int mm = static_cast<int>(std::lround(positions[4] * 1000.0));
    gripper_slider_->setValue(mm);
    gripper_value_label_->setText(QString::number(mm, 'f', 1) + " mm");
  }
  arm_updating_ = false;
}

void VizWindow::refresh_manual() {
  if (arm_status_label_ == nullptr) {
    return;
  }
  const auto states = model_.joint_states();
  if (!states) {
    arm_status_label_->setText("arm: no joint_states");
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
  if (!arm_initialized_ && std::isfinite(positions[0])) {
    arm_output_ = positions;
    arm_target_ = positions;
    arm_initialized_ = true;
  }
  set_arm_slider_values(positions);
  QString status = QString("j1 %1°  j2 %2°  j3 %3°  j4 %4°  grip %5 mm")
                       .arg(std::isfinite(positions[0]) ? positions[0] * 180.0 / 3.141592653589793 : 0.0, 0, 'f', 0)
                       .arg(std::isfinite(positions[1]) ? positions[1] * 180.0 / 3.141592653589793 : 0.0, 0, 'f', 0)
                       .arg(std::isfinite(positions[2]) ? positions[2] * 180.0 / 3.141592653589793 : 0.0, 0, 'f', 0)
                       .arg(std::isfinite(positions[3]) ? positions[3] * 180.0 / 3.141592653589793 : 0.0, 0, 'f', 0)
                       .arg(std::isfinite(positions[4]) ? positions[4] * 1000.0 : 0.0, 0, 'f', 1);
  if (const auto collision = model_.collision_status()) {
    status += "  | " + QString::fromStdString(collision->data);
  }
  const QString arm_errors = arm_error_summary();
  if (!arm_errors.isEmpty()) {
    status = "ARM ERR " + arm_errors + "  |  " + status;
    arm_status_label_->setStyleSheet("color:#ff7070;");
  } else {
    arm_status_label_->setStyleSheet("");
  }
  arm_status_label_->setText(status);
  refresh_moveit();
}

std::optional<std::array<double, 5>> VizWindow::current_arm_state() const {
  const auto states = model_.joint_states();
  if (!states) {
    return std::nullopt;
  }
  std::map<std::string, double> values;
  for (std::size_t i = 0; i < states->name.size() && i < states->position.size(); ++i) {
    values[states->name[i]] = states->position[i];
  }
  std::array<double, 5> positions{};
  positions.fill(std::numeric_limits<double>::quiet_NaN());
  for (int i = 0; i < 4; ++i) {
    const auto found = values.find("joint" + std::to_string(i + 1));
    if (found == values.end()) {
      return std::nullopt;
    }
    positions[static_cast<std::size_t>(i)] = found->second;
  }
  const auto gripper = values.find("gripper_left_joint");
  if (gripper != values.end()) {
    positions[4] = gripper->second;
  }
  return positions;
}

bool VizWindow::ensure_arm_chain(std::string& error) {
  const auto description = model_.robot_description();
  if (!description || description->data.empty()) {
    error = "robot_description 대기 중";
    return false;
  }
  if (arm_chain_ && arm_chain_source_ == description->data && arm_chain_->valid()) {
    return true;
  }
  try {
    const UrdfRobot robot = parse_urdf(description->data);
    std::string build_error;
    auto chain = manipulation::ArmChain::from_urdf(
        robot, "base_link", "end_effector_link", {"joint1", "joint2", "joint3", "joint4"}, build_error);
    if (!chain) {
      error = build_error;
      return false;
    }
    // URDF의 joint2/3 한계(-1.5)는 실제 영점과 달라 현재 자세(-2.05, +1.50 rad)를
    // 포함하지 못한다. 실기에서 확인한 동작 범위로 넓혀 계획이 시작 자세에서 실패하지 않게 한다.
    chain->lower = {-3.141592653589793, -2.20, -2.00, -2.00};
    chain->upper = {3.141592653589793, 2.20, 2.00, 2.20};
    arm_chain_ = std::move(*chain);
    arm_chain_source_ = description->data;
    error.clear();
    return true;
  } catch (const std::exception& exception) {
    error = std::string("URDF: ") + exception.what();
    return false;
  }
}

msgs::Transform VizWindow::target_transform() const {
  const double x = moveit_x_ == nullptr ? 0.0 : moveit_x_->value();
  const double y = moveit_y_ == nullptr ? 0.0 : moveit_y_->value();
  const double z = moveit_z_ == nullptr ? 0.0 : moveit_z_->value();
  const double pitch =
      moveit_pitch_ == nullptr ? 0.0 : moveit_pitch_->value() * 3.141592653589793 / 180.0;
  // link1 원점(base x=0.047985) + joint1 원점(0.012)에서 목표를 바라보는 yaw.
  const double yaw = std::atan2(y + 0.000230, x - 0.059985);
  return {{x, y, z}, tfmath::from_rpy(0.0, pitch, yaw)};
}

void VizWindow::moveit_preview_target() {
  if (scene_ == nullptr) {
    return;
  }
  ArmPreview preview;
  preview.active = true;
  preview.target = target_transform();
  scene_->set_arm_preview(std::move(preview));
}

void VizWindow::moveit_invalidate_plan() {
  if (moveit_executing_) {
    moveit_stop(true);
  }
  moveit_trajectory_.reset();
  moveit_preview_index_ = 0;
  if (moveit_scrub_ != nullptr) {
    const QSignalBlocker blocker(moveit_scrub_);
    moveit_scrub_->setRange(0, 0);
    moveit_scrub_->setValue(0);
  }
  if (moveit_execute_button_ != nullptr) {
    moveit_execute_button_->setEnabled(false);
  }
  if (moveit_status_label_ != nullptr && !moveit_executing_) {
    moveit_status_label_->setText("target changed — Plan을 누르세요");
  }
}

void VizWindow::moveit_set_target_from_current() {
  std::string error;
  if (!ensure_arm_chain(error)) {
    if (moveit_status_label_ != nullptr) {
      moveit_status_label_->setText(QString::fromStdString(error));
    }
    return;
  }
  const auto state = current_arm_state();
  if (!state) {
    if (moveit_status_label_ != nullptr) {
      moveit_status_label_->setText("joint_states 대기 중");
    }
    return;
  }
  if (moveit_executing_) {
    moveit_stop(true);
  }
  const std::array<double, 4> q{(*state)[0], (*state)[1], (*state)[2], (*state)[3]};
  const msgs::Vector3 tcp = arm_chain_->tcp(q);
  const double pitch = arm_chain_->pitch_down(q);
  moveit_updating_ = true;
  moveit_x_->setValue(tcp.x);
  moveit_y_->setValue(tcp.y);
  moveit_z_->setValue(tcp.z);
  moveit_pitch_->setValue(pitch * 180.0 / 3.141592653589793);
  if (std::isfinite((*state)[4])) {
    moveit_gripper_->setValue((*state)[4] * 1000.0);
  }
  moveit_updating_ = false;
  moveit_target_initialized_ = true;
  moveit_invalidate_plan();
  moveit_preview_target();
  if (moveit_status_label_ != nullptr) {
    moveit_status_label_->setText(QString("target = current TCP  (x %1, y %2, z %3, pitch %4°)")
                                      .arg(tcp.x, 0, 'f', 3)
                                      .arg(tcp.y, 0, 'f', 3)
                                      .arg(tcp.z, 0, 'f', 3)
                                      .arg(pitch * 180.0 / 3.141592653589793, 0, 'f', 1));
  }
}

void VizWindow::moveit_plan() {
  std::string error;
  if (!ensure_arm_chain(error)) {
    if (moveit_status_label_ != nullptr) {
      moveit_status_label_->setText(QString::fromStdString("Plan 실패: " + error));
    }
    return;
  }
  const auto state = current_arm_state();
  if (!state) {
    if (moveit_status_label_ != nullptr) {
      moveit_status_label_->setText("Plan 실패: joint_states가 없다");
    }
    return;
  }
  const QString arm_errors = arm_error_summary();
  if (!arm_errors.isEmpty()) {
    if (moveit_status_label_ != nullptr) {
      moveit_status_label_->setText("Plan 실패: ARM ERR " + arm_errors + " — 먼저 Reset");
    }
    return;
  }
  if (moveit_executing_) {
    moveit_stop(true);
  }
  const std::array<double, 4> start{(*state)[0], (*state)[1], (*state)[2], (*state)[3]};
  const msgs::Vector3 target{moveit_x_->value(), moveit_y_->value(), moveit_z_->value()};
  const double pitch = moveit_pitch_->value() * 3.141592653589793 / 180.0;
  std::array<double, 4> goal{};
  if (!arm_chain_->ik(target, pitch, start, goal, error)) {
    moveit_trajectory_.reset();
    moveit_execute_button_->setEnabled(false);
    moveit_preview_target();
    if (moveit_status_label_ != nullptr) {
      moveit_status_label_->setText(QString::fromStdString("Plan 실패: " + error));
    }
    return;
  }
  manipulation::PlanOptions options;
  options.max_joint_speed = std::max(0.10, arm_speed_);
  options.max_joint_accel = std::max(0.5, arm_speed_ * 3.0);
  options.sample_time = 0.05;
  options.cartesian = moveit_cartesian_->isChecked();
  auto trajectory = manipulation::plan_trajectory(*arm_chain_, start, goal, options, error);
  if (!trajectory) {
    moveit_trajectory_.reset();
    moveit_execute_button_->setEnabled(false);
    moveit_preview_target();
    if (moveit_status_label_ != nullptr) {
      moveit_status_label_->setText(QString::fromStdString("Plan 실패: " + error));
    }
    return;
  }
  moveit_trajectory_ = std::move(*trajectory);
  const int last = static_cast<int>(moveit_trajectory_->points.size()) - 1;
  {
    const QSignalBlocker blocker(moveit_scrub_);
    moveit_scrub_->setRange(0, last);
    moveit_scrub_->setValue(last);
  }
  moveit_update_preview(last);
  moveit_execute_button_->setEnabled(true);
  moveit_cancel_button_->setEnabled(false);
  if (moveit_status_label_ != nullptr) {
    QString text = QString("Plan OK: %1 pts, %2 s, %3")
                       .arg(moveit_trajectory_->points.size())
                       .arg(moveit_trajectory_->duration, 0, 'f', 2)
                       .arg(moveit_trajectory_->cartesian ? "Cartesian" : "joint-space");
    if (moveit_trajectory_->fallback) {
      text += "\nCartesian 실패 → joint-space 대체";
    }
    if (!moveit_trajectory_->note.empty() && !moveit_trajectory_->fallback) {
      text += "\n" + QString::fromStdString(moveit_trajectory_->note);
    }
    moveit_status_label_->setText(text);
  }
}

void VizWindow::moveit_execute() {
  if (!moveit_trajectory_ || moveit_trajectory_->points.empty() || !arm_chain_) {
    if (moveit_status_label_ != nullptr) {
      moveit_status_label_->setText("Execute: 계획이 없다. Plan 먼저");
    }
    return;
  }
  const auto state = current_arm_state();
  if (!state) {
    if (moveit_status_label_ != nullptr) {
      moveit_status_label_->setText("Execute: joint_states가 없다");
    }
    return;
  }
  const QString arm_errors = arm_error_summary();
  if (!arm_errors.isEmpty()) {
    if (moveit_status_label_ != nullptr) {
      moveit_status_label_->setText("Execute 거부: ARM ERR " + arm_errors + " — 먼저 Reset");
    }
    return;
  }
  const auto& first = moveit_trajectory_->points.front().position;
  double difference = 0.0;
  for (std::size_t i = 0; i < 4; ++i) {
    difference = std::max(difference, std::abs(first[i] - (*state)[i]));
  }
  if (difference > 0.08) {
    if (moveit_status_label_ != nullptr) {
      moveit_status_label_->setText(
          QString("Execute 거부: 현재 자세가 계획 시작과 %1 rad 다르다. 다시 Plan").arg(difference, 0, 'f', 3));
    }
    return;
  }
  moveit_executing_ = true;
  moveit_start_time_ = steady_seconds();
  moveit_plan_button_->setEnabled(false);
  moveit_execute_button_->setEnabled(false);
  moveit_current_button_->setEnabled(false);
  moveit_cancel_button_->setEnabled(true);
  moveit_scrub_->setEnabled(false);
  arm_moving_ = false;
  moveit_tick();
  if (moveit_executing_) {
    moveit_exec_timer_->start(50);
  }
}

void VizWindow::moveit_tick() {
  if (!moveit_executing_ || !moveit_trajectory_ || moveit_trajectory_->points.empty() || !arm_chain_) {
    if (moveit_exec_timer_ != nullptr) {
      moveit_exec_timer_->stop();
    }
    return;
  }
  const auto& points = moveit_trajectory_->points;
  double elapsed = steady_seconds() - moveit_start_time_;
  const double duration = moveit_trajectory_->duration;
  elapsed = std::clamp(elapsed, 0.0, duration);
  std::size_t index = 0;
  while (index + 1 < points.size() && points[index + 1].time <= elapsed) {
    ++index;
  }
  std::array<double, 4> position = points[index].position;
  if (index + 1 < points.size()) {
    const double segment = points[index + 1].time - points[index].time;
    if (segment > 1e-9) {
      const double ratio = std::clamp((elapsed - points[index].time) / segment, 0.0, 1.0);
      for (std::size_t i = 0; i < 4; ++i) {
        position[i] = points[index].position[i] +
                      (points[index + 1].position[i] - points[index].position[i]) * ratio;
      }
    }
  }
  msgs::JointState command;
  for (int i = 0; i < 4; ++i) {
    command.name.push_back("joint" + std::to_string(i + 1));
    command.position.push_back(position[static_cast<std::size_t>(i)]);
  }
  command.name.push_back("gripper_left_joint");
  command.position.push_back(moveit_gripper_->value() / 1000.0);
  model_.node().publish("arm_joint_command", command, QOS_DEFAULT);
  arm_output_ = {position[0], position[1], position[2], position[3], moveit_gripper_->value() / 1000.0};
  moveit_update_preview(static_cast<int>(index));
  if (moveit_scrub_ != nullptr) {
    const QSignalBlocker blocker(moveit_scrub_);
    moveit_scrub_->setValue(static_cast<int>(index));
  }
  if (moveit_status_label_ != nullptr) {
    const double percent = duration > 1e-9 ? 100.0 * elapsed / duration : 100.0;
    moveit_status_label_->setText(
        QString("Execute: %1 / %2 s  (%3%)").arg(elapsed, 0, 'f', 2).arg(duration, 0, 'f', 2).arg(percent, 0, 'f', 0));
  }
  if (elapsed >= duration - 1e-9) {
    moveit_stop(false);
  }
}

void VizWindow::moveit_stop(bool publish_stop) {
  if (moveit_exec_timer_ != nullptr) {
    moveit_exec_timer_->stop();
  }
  const bool was_executing = moveit_executing_;
  moveit_executing_ = false;
  arm_moving_ = false;
  if (moveit_plan_button_ != nullptr) {
    moveit_plan_button_->setEnabled(true);
    moveit_current_button_->setEnabled(true);
    moveit_cancel_button_->setEnabled(false);
    moveit_scrub_->setEnabled(true);
  }
  if (publish_stop) {
    model_.node().publish("arm_stop", msgs::Empty{}, QOS_DEFAULT);
    moveit_trajectory_.reset();
    if (moveit_scrub_ != nullptr) {
      const QSignalBlocker blocker(moveit_scrub_);
      moveit_scrub_->setRange(0, 0);
      moveit_scrub_->setValue(0);
    }
    if (moveit_execute_button_ != nullptr) {
      moveit_execute_button_->setEnabled(false);
    }
    if (moveit_status_label_ != nullptr) {
      moveit_status_label_->setText(was_executing ? "Cancel: 정지, 다시 Plan 필요" : "Plan 취소");
    }
    moveit_preview_target();
  } else {
    if (moveit_execute_button_ != nullptr) {
      moveit_execute_button_->setEnabled(moveit_trajectory_.has_value());
    }
    if (moveit_status_label_ != nullptr) {
      moveit_status_label_->setText("Execute 완료");
    }
  }
}

void VizWindow::moveit_update_preview(int index) {
  if (!arm_chain_ || !moveit_trajectory_ || moveit_trajectory_->points.empty() || scene_ == nullptr) {
    return;
  }
  const int last = static_cast<int>(moveit_trajectory_->points.size()) - 1;
  index = std::clamp(index, 0, last);
  const auto& q = moveit_trajectory_->points[static_cast<std::size_t>(index)].position;
  ArmPreview preview;
  preview.active = true;
  for (const auto& link : arm_chain_->link_transforms(q)) {
    preview.links.push_back(link.second);
  }
  preview.tcp = arm_chain_->tcp(q);
  preview.tcp_path = moveit_trajectory_->tcp_path;
  preview.target = arm_chain_->fk(moveit_trajectory_->points.back().position);
  scene_->set_arm_preview(std::move(preview));
  moveit_preview_index_ = index;
}

void VizWindow::reset_arm() {
  if (arm_reset_target_ == nullptr || arm_reset_button_ == nullptr) {
    return;
  }
  const QString target = arm_reset_target_->currentText();
  if (moveit_executing_) {
    moveit_stop(true);
  }
  arm_moving_ = false;
  model_.node().publish("arm_reset", msgs::String{target.toStdString()}, QOS_DEFAULT);
  if (arm_status_label_ != nullptr) {
    arm_status_label_->setText(QString("reset(AR) 요청: %1 — 서보 재부팅 후 토크 복구").arg(target));
    arm_status_label_->setStyleSheet("color:#ffd166;");
  }
  arm_reset_button_->setEnabled(false);
  QTimer::singleShot(2000, this, [this] {
    if (arm_reset_button_ != nullptr) {
      arm_reset_button_->setEnabled(true);
    }
  });
}

QString VizWindow::arm_error_summary() const {
  const auto status = model_.arm_status();
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
    if (values[i] == "0" || values[i].isEmpty()) {
      continue;
    }
    const QString name = i == 4 ? QString("grip") : QString("j%1").arg(i + 1);
    errors << name + "=" + values[i];
  }
  return errors.join(" ");
}

void VizWindow::refresh_moveit() {
  if (moveit_dock_ == nullptr || moveit_current_label_ == nullptr) {
    return;
  }
  std::string error;
  const bool ready = ensure_arm_chain(error);
  if (!arm_chain_) {
    moveit_current_label_->setText(QString::fromStdString("now: " + error));
    if (moveit_status_label_ != nullptr && moveit_status_label_->text().startsWith("MoveIt:")) {
      moveit_status_label_->setText(QString::fromStdString("MoveIt: " + error));
    }
    return;
  }
  if (!moveit_target_initialized_ && !moveit_executing_) {
    const auto state = current_arm_state();
    if (state) {
      moveit_set_target_from_current();
    }
  }
  const auto state = current_arm_state();
  if (!state) {
    moveit_current_label_->setText("now: joint_states 대기");
    return;
  }
  const std::array<double, 4> q{(*state)[0], (*state)[1], (*state)[2], (*state)[3]};
  const msgs::Vector3 tcp = arm_chain_->tcp(q);
  const double pitch = arm_chain_->pitch_down(q);
  moveit_current_label_->setText(QString("now TCP  x %1  y %2  z %3  pitch %4°")
                                     .arg(tcp.x, 0, 'f', 3)
                                     .arg(tcp.y, 0, 'f', 3)
                                     .arg(tcp.z, 0, 'f', 3)
                                     .arg(pitch * 180.0 / 3.141592653589793, 0, 'f', 1));
  (void)ready;
}

void VizWindow::set_manual_panels(bool teleop, bool manipulator, bool moveit) {
  if (teleop_dock_ != nullptr) {
    teleop_dock_->setVisible(teleop);
  }
  if (manipulator_dock_ != nullptr) {
    manipulator_dock_->setVisible(manipulator);
  }
  if (moveit_dock_ != nullptr) {
    moveit_dock_->setVisible(moveit);
  }
  if (teleop && manipulator && teleop_dock_ != nullptr && manipulator_dock_ != nullptr) {
    splitDockWidget(teleop_dock_, manipulator_dock_, Qt::Vertical);
  }
  if (moveit && manipulator && manipulator_dock_ != nullptr && moveit_dock_ != nullptr) {
    splitDockWidget(manipulator_dock_, moveit_dock_, Qt::Vertical);
  } else if (moveit && teleop && teleop_dock_ != nullptr && moveit_dock_ != nullptr) {
    splitDockWidget(teleop_dock_, moveit_dock_, Qt::Vertical);
  }
  QList<QDockWidget*> docks;
  QList<int> sizes;
  if (teleop && teleop_dock_ != nullptr) {
    docks.push_back(teleop_dock_);
    sizes.push_back(170);
  }
  if (manipulator && manipulator_dock_ != nullptr) {
    docks.push_back(manipulator_dock_);
    sizes.push_back(330);
  }
  if (moveit && moveit_dock_ != nullptr) {
    docks.push_back(moveit_dock_);
    sizes.push_back(430);
  }
  if (!docks.isEmpty()) {
    resizeDocks(docks, sizes, Qt::Vertical);
  }
}


bool VizWindow::eventFilter(QObject* watched, QEvent* event) {
  if ((event->type() == QEvent::KeyPress || event->type() == QEvent::KeyRelease) && teleop_dock_ != nullptr) {
    QWidget* focus = QApplication::focusWidget();
    if (focus != nullptr && (focus == teleop_dock_ || teleop_dock_->isAncestorOf(focus))) {
      auto* key = static_cast<QKeyEvent*>(event);
      const int code = key->key();
      const bool press = event->type() == QEvent::KeyPress;
      const bool teleop_key = code == Qt::Key_W || code == Qt::Key_A || code == Qt::Key_S || code == Qt::Key_D ||
                              code == Qt::Key_Up || code == Qt::Key_Down || code == Qt::Key_Left ||
                              code == Qt::Key_Right;
      if (teleop_key) {
        if (qobject_cast<QAbstractSlider*>(focus) != nullptr &&
            (code == Qt::Key_Up || code == Qt::Key_Down || code == Qt::Key_Left || code == Qt::Key_Right)) {
          return false;  // 속도 슬라이더의 화살표 조정은 슬라이더에 맡긴다
        }
        if (key->isAutoRepeat()) {
          return true;
        }
        if (scene_ != nullptr) {
          scene_->set_teleop_key(code, press);
        }
        return true;
      }
      if (press) {
        if (code == Qt::Key_Q) {
          scene_->set_teleop_scale(scene_->teleop_scale() * 1.25);
          return true;
        }
        if (code == Qt::Key_Z) {
          scene_->set_teleop_scale(scene_->teleop_scale() / 1.25);
          return true;
        }
        if (code == Qt::Key_F) {
          scene_->set_follow(!scene_->follow());
          return true;
        }
        if (code == Qt::Key_C || code == Qt::Key_R) {
          scene_->reset_camera();
          return true;
        }
      }
    }
  }
  return QMainWindow::eventFilter(watched, event);
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
    sync_waypoints();
    if (multi_label_ != nullptr) {
      QStringList parts;
      for (std::size_t index = 0; index < multi_goals_.size(); ++index) {
        parts << QString("#%1 (%2, %3)")
                     .arg(index + 1)
                     .arg(std::get<0>(multi_goals_[index]), 0, 'f', 2)
                     .arg(std::get<1>(multi_goals_[index]), 0, 'f', 2);
      }
      multi_label_->setText(QString("multi goals: %1  %2").arg(multi_goals_.size()).arg(parts.join("  ")));
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

  if (tf_label_ != nullptr) {
    const auto& buffer = model_.tf_buffer();
    const std::vector<std::string> all = buffer.all_frames();
    QString chain_text;
    const auto contains = [&](const std::string& frame) {
      return std::find(all.begin(), all.end(), frame) != all.end();
    };
    std::string frame;
    if (contains("base_link")) {
      frame = "base_link";
    } else if (contains("base_footprint")) {
      frame = "base_footprint";
    }
    if (!frame.empty()) {
      std::vector<std::string> chain;
      for (int depth = 0; depth < 16; ++depth) {
        chain.push_back(frame);
        const auto parent = buffer.parent_of(frame);
        if (!parent || *parent == frame) {
          break;
        }
        frame = *parent;
      }
      QStringList names;
      for (auto it = chain.rbegin(); it != chain.rend(); ++it) {
        names << QString::fromStdString(*it);
      }
      chain_text = QString(" | %1").arg(names.join(" → "));
    }
    tf_label_->setText(QString("TF: %1 frames%2").arg(all.size()).arg(chain_text));
  }
}

void VizWindow::set_tab(int index) {
  if (tabs_ != nullptr) {
    tabs_->setCurrentIndex(index == 1 ? 1 : 0);
  }
}

void VizWindow::set_camera_distance(double distance) {
  if (scene_ != nullptr) {
    scene_->set_camera_distance(distance);
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
