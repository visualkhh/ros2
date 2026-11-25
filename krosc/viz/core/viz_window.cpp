#include "krosc/viz/viz_window.hpp"

#include <QAbstractItemView>
#include <QAbstractSpinBox>
#include <QAbstractSlider>
#include <QAction>
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
#include <QScrollArea>
#include <QSignalBlocker>
#include <QSlider>
#include <QSplitter>
#include <QTabWidget>
#include <QToolBar>
#include <QStringList>
#include <QStatusBar>
#include <QTableWidget>
#include <QTimer>
#include <QVBoxLayout>
#include <QWidget>

#include <algorithm>
#include <cstdio>
#include <array>
#include <chrono>
#include <cmath>
#include <limits>
#include <sstream>
#include <utility>
#include <vector>

#include "krosc/tf/math.hpp"
#include "krosc/viz/key_bindings.hpp"
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
  tabs_->setObjectName("view-panel");
  auto* tabs = tabs_;
  scene_ = new SceneView(model_, tabs);
  scene_->setObjectName("scene");
  map_view_ = new MapView2D(model_, tabs);
  tabs->addTab(scene_, "3D");
  tabs->addTab(map_view_, "Map");
  scene_->set_teleop_enabled(false);  // teleop 플러그인이 켠다
  auto* panel = new QWidget();
  panel->setMinimumWidth(240);
  panel->setMaximumWidth(760);
  build_panel(panel);
  auto* panel_scroll = new QScrollArea(splitter);
  panel_scroll->setWidgetResizable(true);
  panel_scroll->setFrameShape(QFrame::NoFrame);
  panel_scroll->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
  panel_scroll->setMinimumWidth(240);
  panel_scroll->setMaximumWidth(768);
  panel_scroll->setWidget(panel);
  splitter->addWidget(panel_scroll);
  splitter->addWidget(tabs);
  control_tabs_ = new QTabWidget(splitter);
  control_tabs_->setObjectName("control-panel");
  control_tabs_->setMinimumWidth(240);
  control_tabs_->setMaximumWidth(760);
  control_tabs_->setDocumentMode(true);
  control_tabs_->hide();
  splitter->addWidget(control_tabs_);
  splitter->setStretchFactor(0, 0);
  splitter->setStretchFactor(1, 1);
  splitter->setStretchFactor(2, 0);
  splitter->setSizes({360, 780, 360});
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
  set_fixed_frame(model_.fixed_frame());
}

VizWindow::~VizWindow() {
  for (auto& plugin : plugins_) {
    plugin->shutdown();
  }
  plugins_.clear();
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
      } else if (key == "global_costmap") {
        map_view_->set_show_global_costmap(box->isChecked());
      } else if (key == "local_costmap") {
        map_view_->set_show_local_costmap(box->isChecked());
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
        } else if (key == "global_costmap") {
          map_view_->set_show_global_costmap(checked);
        } else if (key == "local_costmap") {
          map_view_->set_show_local_costmap(checked);
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


void VizWindow::add_dock(QDockWidget* dock, Qt::DockWidgetArea area) {
  (void)area;
  if (dock == nullptr) {
    return;
  }
  if (control_tabs_ != nullptr) {
    // 플러그인 dock은 오른쪽 control-panel 탭으로 옮긴다. dock 객체는 플러그인이
    // shutdown에서 deleteLater()할 수 있게 빈 껍데기로 남겨 둔다.
    QWidget* content = dock->widget();
    if (content == nullptr) {
      return;
    }
    auto* scroll = new QScrollArea(control_tabs_);
    scroll->setWidgetResizable(true);
    scroll->setFrameShape(QFrame::NoFrame);
    scroll->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    scroll->setWidget(content);
    const int index = control_tabs_->addTab(scroll, dock->windowTitle());
    control_tabs_->setCurrentIndex(index);
    control_tabs_->show();
    dock->hide();
    dock->setParent(this);
    return;
  }
  addDockWidget(area, dock);
}

void VizWindow::add_tab(QWidget* tab, const QString& title, bool make_current) {
  if (tabs_ == nullptr || tab == nullptr) {
    return;
  }
  const int index = tabs_->addTab(tab, title);
  if (make_current) {
    tabs_->setCurrentIndex(index);
  }
}

void VizWindow::show_tab(QWidget* tab) {
  if (tabs_ != nullptr && tab != nullptr) {
    tabs_->setCurrentWidget(tab);
  }
}

void VizWindow::load_plugins(VizPluginRegistry& registry, const std::vector<std::string>& ids) {
  for (const auto& id : ids) {
    auto plugin = registry.create(id);
    if (!plugin) {
      std::fprintf(stderr, "[viz] 알 수 없는 플러그인: %s\n", id.c_str());
      continue;
    }
    const VizPluginInfo info = plugin->info();
    std::fprintf(stderr, "[viz] plugin: %s (%s) v%s\n", info.id.c_str(), info.name.c_str(),
                 info.version.c_str());
    plugin->initialize(*this);
    plugins_.push_back(std::move(plugin));
  }
  if (control_tabs_ != nullptr && control_tabs_->count() > 0) {
    control_tabs_->setCurrentIndex(0);
  }
}

void VizWindow::set_manual_panels(bool teleop, bool manipulator) {
  // 호환용. 실제 패널은 teleop/manipulator 플러그인이 만든다.
  (void)teleop;
  (void)manipulator;
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
    tabs_->setCurrentIndex(std::clamp(index, 0, tabs_->count() - 1));
  }
}

void VizWindow::set_tab(const std::string& name) {
  if (tabs_ == nullptr) {
    return;
  }
  const QString wanted = QString::fromStdString(name);
  for (int index = 0; index < tabs_->count(); ++index) {
    if (tabs_->tabText(index).compare(wanted, Qt::CaseInsensitive) == 0) {
      tabs_->setCurrentIndex(index);
      return;
    }
  }
  tabs_->setCurrentIndex(0);
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
