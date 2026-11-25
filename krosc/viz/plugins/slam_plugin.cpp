#include "krosc/viz/plugins/slam_plugin.hpp"

#include <QComboBox>
#include <QDockWidget>
#include <QGridLayout>
#include <QLabel>
#include <QLineEdit>
#include <QMessageBox>
#include <QPushButton>
#include <QSignalBlocker>
#include <QTimer>
#include <QVBoxLayout>
#include <QWidget>

#include <sstream>
#include <vector>

#include "krosc/msgs.hpp"
#include "krosc/qos.hpp"
#include "krosc/viz/viz_model.hpp"
#include "krosc/viz/viz_window.hpp"

namespace krosc::viz {

SlamPlugin::SlamPlugin() : VizPlugin({"slam", "SLAM / Map", "1.0", {"slam_status", "map_list"}}) {}
SlamPlugin::~SlamPlugin() { shutdown(); }

void SlamPlugin::initialize(VizHost& host) {
  if (active_) {
    return;
  }
  host_ = &host;
  dock_ = new QDockWidget("SLAM / Map", &host.window());
  dock_->setObjectName("slamDock");
  dock_->setAllowedAreas(Qt::LeftDockWidgetArea | Qt::RightDockWidgetArea);
  dock_->setFeatures(QDockWidget::DockWidgetMovable | QDockWidget::DockWidgetFloatable |
                     QDockWidget::DockWidgetClosable);
  auto* panel = new QWidget(dock_);
  auto* grid = new QGridLayout(panel);
  grid->setContentsMargins(8, 8, 8, 8);
  grid->setHorizontalSpacing(6);
  grid->setVerticalSpacing(5);

  mode_label_ = new QLabel("SLAM: -", panel);
  slam_button_ = new QPushButton("start SLAM", panel);
  grid->addWidget(mode_label_, 0, 0, 1, 2);
  grid->addWidget(slam_button_, 0, 2, 1, 2);
  connect(slam_button_, &QPushButton::clicked, this, [this] {
    if (host_ == nullptr) {
      return;
    }
    const std::string target = slam_status_text() == "mapping" ? "idle" : "mapping";
    host_->model().node().publish("slam_mode", msgs::String{target}, QOS_DEFAULT);
    status_label_->setText(QString("slam_mode: %1").arg(QString::fromStdString(target)));
  });

  map_combo_ = new QComboBox(panel);
  map_combo_->addItem("(select map)");
  map_combo_->setMinimumWidth(200);
  grid->addWidget(new QLabel("map", panel), 1, 0);
  grid->addWidget(map_combo_, 1, 1, 1, 2);
  auto* load = new QPushButton("load", panel);
  grid->addWidget(load, 1, 3);
  connect(load, &QPushButton::clicked, this, [this] {
    if (host_ == nullptr) {
      return;
    }
    if (map_combo_->currentIndex() <= 0) {
      host_->model().node().publish("map_unload", msgs::Empty{}, QOS_DEFAULT);
      status_label_->setText("unloading map (none) ...");
      return;
    }
    const QString name = map_combo_->currentText();
    if (name.isEmpty()) {
      status_label_->setText("select a map first");
      return;
    }
    host_->model().node().publish("map_change", msgs::String{name.toStdString()}, QOS_DEFAULT);
    status_label_->setText(QString("loading %1 ...").arg(name));
  });

  map_name_ = new QLineEdit(panel);
  map_name_->setPlaceholderText("map name (auto if empty)");
  grid->addWidget(map_name_, 2, 0, 1, 3);
  auto* save = new QPushButton("save", panel);
  grid->addWidget(save, 2, 3);
  connect(save, &QPushButton::clicked, this, [this] {
    if (host_ == nullptr) {
      return;
    }
    const std::string name = map_name_->text().trimmed().toStdString();
    host_->model().node().publish("map_save", msgs::String{name}, QOS_DEFAULT);
    status_label_->setText(name.empty() ? QString("saving (auto name) ...")
                                        : QString("saving %1 ...").arg(QString::fromStdString(name)));
  });

  auto* unload = new QPushButton("unload", panel);
  auto* remove = new QPushButton("delete", panel);
  grid->addWidget(unload, 3, 0);
  grid->addWidget(remove, 3, 1);
  collision_button_ = new QPushButton("collision ON", panel);
  grid->addWidget(collision_button_, 3, 2, 1, 2);
  connect(collision_button_, &QPushButton::clicked, this, [this] {
    if (host_ == nullptr) {
      return;
    }
    collision_enabled_ = !collision_enabled_;
    host_->model().node().publish("collision_set_enabled", msgs::Bool{collision_enabled_},
                                  QOS_DEFAULT);
    update_collision_button();
  });
  update_collision_button();
  connect(unload, &QPushButton::clicked, this, [this] {
    if (host_ != nullptr) {
      host_->model().node().publish("map_unload", msgs::Empty{}, QOS_DEFAULT);
      status_label_->setText("unloading ...");
    }
  });
  connect(remove, &QPushButton::clicked, this, [this, panel] {
    if (host_ == nullptr) {
      return;
    }
    if (map_combo_->currentIndex() <= 0 || map_combo_->currentText().isEmpty()) {
      status_label_->setText("select a map to delete");
      return;
    }
    const QString name = map_combo_->currentText();
    if (QMessageBox::question(panel, "delete map",
                              QString("Delete map \"%1\" ?").arg(name)) != QMessageBox::Yes) {
      return;
    }
    host_->model().node().publish("map_delete", msgs::String{name.toStdString()}, QOS_DEFAULT);
    status_label_->setText(QString("deleting %1 ...").arg(name));
  });
  status_label_ = new QLabel("", panel);
  status_label_->setWordWrap(true);
  status_label_->setMinimumHeight(30);
  grid->addWidget(status_label_, 4, 0, 1, 4);

  dock_->setWidget(panel);
  host.add_dock(dock_, Qt::RightDockWidgetArea);

  timer_ = new QTimer(this);
  connect(timer_, &QTimer::timeout, this, &SlamPlugin::refresh);
  timer_->start(300);
  active_ = true;
  refresh();
}

void SlamPlugin::shutdown() {
  if (!active_) {
    return;
  }
  active_ = false;
  if (timer_ != nullptr) {
    timer_->stop();
  }
  if (dock_ != nullptr) {
    dock_->deleteLater();
    dock_ = nullptr;
  }
}

std::string SlamPlugin::slam_status_text() const {
  if (host_ == nullptr) {
    return "idle";
  }
  if (const auto status = host_->model().slam_status()) {
    return status->data;
  }
  return "idle";
}

std::string SlamPlugin::map_status_text() const {
  if (host_ == nullptr) {
    return {};
  }
  if (const auto status = host_->model().map_status()) {
    return status->data;
  }
  return {};
}

void SlamPlugin::update_collision_button() {
  if (collision_button_ == nullptr) {
    return;
  }
  collision_button_->setText(collision_enabled_ ? "collision ON" : "collision OFF");
  collision_button_->setStyleSheet(collision_enabled_ ? "background:#1f6b43;" : "background:#7a2a2a;");
}

void SlamPlugin::refresh() {
  if (host_ == nullptr || mode_label_ == nullptr) {
    return;
  }
  const std::string mode = slam_status_text();
  mode_label_->setText(QString("SLAM: %1").arg(QString::fromStdString(mode)));
  slam_button_->setText(mode == "mapping" ? "stop SLAM" : "start SLAM");

  std::vector<std::string> names;
  if (const auto list = host_->model().map_list()) {
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

  QString status = QString::fromStdString(map_status_text());
  if (const auto map_status = host_->model().map_status()) {
    const std::string text = map_status->data;
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
  if (const auto map = host_->model().map()) {
    status += QString(" | map %1x%2").arg(map->info.width).arg(map->info.height);
  } else {
    status += " | map -";
  }
  if (const auto goal = host_->model().goal_status()) {
    status += QString(" | goal: %1").arg(QString::fromStdString(goal->data));
  }
  if (const auto collision = host_->model().collision_status()) {
    const std::string text = collision->data;
    const bool enabled = text.rfind("disabled", 0) != 0;
    if (enabled != collision_enabled_) {
      collision_enabled_ = enabled;
      update_collision_button();
    }
    collision_button_->setToolTip(QString::fromStdString(text));
  }
  status_label_->setText(status);
}

}  // namespace krosc::viz
