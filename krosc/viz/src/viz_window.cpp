#include "krosc/viz/viz_window.hpp"

#include <QAbstractItemView>
#include <QCheckBox>
#include <QComboBox>
#include <QGridLayout>
#include <QHBoxLayout>
#include <QHeaderView>
#include <QLabel>
#include <QPushButton>
#include <QSignalBlocker>
#include <QStatusBar>
#include <QTableWidget>
#include <QTimer>
#include <QVBoxLayout>
#include <QWidget>

#include <utility>

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

  // 패널이 scene_에 연결하므로 scene을 먼저 만든다. 레이아웃에는 패널 다음에 넣는다.
  scene_ = new SceneView(model_, central);
  scene_->setObjectName("scene");

  auto* panel = new QWidget(central);
  panel->setFixedWidth(370);
  build_panel(panel);
  layout->addWidget(panel);
  layout->addWidget(scene_, 1);
  setCentralWidget(central);

  statusBar()->showMessage(scene_->help_text());

  connect(scene_, &SceneView::follow_changed, this, [this](bool follow) {
    if (auto* box = findChild<QCheckBox*>("followBox"); box != nullptr && box->isChecked() != follow) {
      const QSignalBlocker blocker(box);
      box->setChecked(follow);
    }
  });

  paint_timer_ = new QTimer(this);
  connect(paint_timer_, &QTimer::timeout, scene_, QOverload<>::of(&QWidget::update));
  paint_timer_->start(33);

  refresh_timer_ = new QTimer(this);
  connect(refresh_timer_, &QTimer::timeout, this, &VizWindow::refresh_topics);
  refresh_timer_->start(500);
  refresh_topics();

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
  table_->setMinimumHeight(240);
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
