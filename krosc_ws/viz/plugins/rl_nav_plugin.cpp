#include "plugins/rl_nav_plugin.hpp"

#include <QComboBox>
#include <QDockWidget>
#include <QHBoxLayout>
#include <QLabel>
#include <QMainWindow>
#include <QPushButton>
#include <QTimer>
#include <QVBoxLayout>
#include <QWidget>

#include <cstdlib>
#include <map>
#include <sstream>
#include <string>

#include "krosc/msgs.hpp"
#include "krosc/qos.hpp"
#include "krosc/viz/viz_model.hpp"

namespace krosc_ws::viz {

namespace {
std::map<std::string, std::string> fields(const std::string& text) {
  std::map<std::string, std::string> out;
  std::stringstream stream(text);
  std::string item;
  while (std::getline(stream, item, ';')) {
    const std::size_t equals = item.find('=');
    if (equals == std::string::npos) {
      continue;
    }
    out[item.substr(0, equals)] = item.substr(equals + 1);
  }
  return out;
}

std::string get(const std::map<std::string, std::string>& values, const std::string& key,
                const std::string& fallback = "-") {
  const auto found = values.find(key);
  return found == values.end() ? fallback : found->second;
}

QString reason_text(int reason) {
  switch (reason) {
    case 0:
      return "RL active";
    case 1:
      return "model 없음/로드 실패";
    case 2:
      return "RL OFF";
    case 3:
      return "scan 없음/stale";
    case 4:
      return "추론 실패";
    case 5:
      return "odom 없음/stale";
    case 6:
      return "plan 없음";
    case 7:
      return "TF(map→robot) 없음";
    default:
      return QString("reason %1").arg(reason);
  }
}
}  // namespace

void RlNavPlugin::initialize(krosc::viz::VizHost& host) {
  host_ = &host;
  dock_ = new QDockWidget("RL Nav", &host.window());
  dock_->setObjectName("rlNavDock");
  dock_->setAllowedAreas(Qt::LeftDockWidgetArea | Qt::RightDockWidgetArea | Qt::BottomDockWidgetArea);
  dock_->setFeatures(QDockWidget::DockWidgetMovable | QDockWidget::DockWidgetFloatable |
                     QDockWidget::DockWidgetClosable);
  auto* panel = new QWidget(dock_);
  auto* layout = new QVBoxLayout(panel);
  layout->setContentsMargins(8, 8, 8, 8);
  layout->setSpacing(6);

  auto* title = new QLabel("RL navigation", panel);
  title->setObjectName("Section");
  layout->addWidget(title);

  toggle_ = new QPushButton("RL OFF", panel);
  toggle_->setMinimumHeight(30);
  layout->addWidget(toggle_);
  QObject::connect(toggle_, &QPushButton::clicked, dock_, [this] { set_enabled(!enabled_); });

  auto* fallback_row = new QHBoxLayout();
  fallback_row->addWidget(new QLabel("fallback", panel));
  fallback_ = new QComboBox(panel);
  fallback_->addItems({"pursuit", "stop"});
  fallback_row->addWidget(fallback_, 1);
  reload_ = new QPushButton("Reload", panel);
  fallback_row->addWidget(reload_);
  layout->addLayout(fallback_row);
  QObject::connect(fallback_, qOverload<int>(&QComboBox::currentIndexChanged), dock_, [this](int) {
    if (!updating_) {
      set_fallback(fallback_->currentText());
    }
  });
  QObject::connect(reload_, &QPushButton::clicked, dock_, [this] {
    if (host_ != nullptr) {
      host_->model().node().publish("rl_nav_reload", krosc::msgs::Empty{}, krosc::QOS_DEFAULT);
    }
  });

  status_ = new QLabel("RL: waiting for rl_nav_controller", panel);
  status_->setWordWrap(true);
  status_->setMinimumHeight(38);
  layout->addWidget(status_);
  detail_ = new QLabel("-", panel);
  detail_->setWordWrap(true);
  detail_->setTextInteractionFlags(Qt::TextSelectableByMouse);
  layout->addWidget(detail_);
  layout->addStretch(1);
  dock_->setWidget(panel);
  host.add_dock(dock_, Qt::RightDockWidgetArea);

  timer_ = new QTimer(dock_);
  QObject::connect(timer_, &QTimer::timeout, dock_, [this] { refresh(); });
  timer_->start(100);
  refresh();
}

void RlNavPlugin::shutdown() {
  if (timer_ != nullptr) {
    timer_->stop();
  }
}

void RlNavPlugin::set_enabled(bool enabled) {
  enabled_ = enabled;
  toggle_->setText(enabled ? "RL ON" : "RL OFF");
  toggle_->setStyleSheet(enabled ? "background:#1f6b43;" : "background:#7a2a2a;");
  if (host_ != nullptr) {
    host_->model().node().publish("rl_nav_enable", krosc::msgs::Bool{enabled}, krosc::QOS_DEFAULT);
  }
}

void RlNavPlugin::set_fallback(const QString& fallback) {
  if (host_ == nullptr) {
    return;
  }
  host_->model().node().publish("rl_nav_fallback",
                                krosc::msgs::String{fallback.toStdString()}, krosc::QOS_DEFAULT);
}

void RlNavPlugin::refresh() {
  if (host_ == nullptr || status_ == nullptr) {
    return;
  }
  const auto message = host_->model().rl_nav_status();
  if (!message) {
    status_->setText("RL: waiting for rl_nav_controller");
    detail_->setText("-");
    return;
  }
  const auto values = fields(message->data);
  const bool enabled = get(values, "enable", "0") == "1";
  const bool policy = get(values, "policy", "0") == "1";
  const bool scan = get(values, "scan", "0") == "1";
  const bool active = get(values, "active", "0") == "1";
  const int reason = std::atoi(get(values, "reason", "0").c_str());
  const QString fallback = QString::fromStdString(get(values, "fallback", "pursuit"));

  updating_ = true;
  if (enabled != enabled_) {
    enabled_ = enabled;
    toggle_->setText(enabled ? "RL ON" : "RL OFF");
    toggle_->setStyleSheet(enabled ? "background:#1f6b43;" : "background:#7a2a2a;");
  }
  fallback_->setCurrentText(fallback);
  updating_ = false;

  status_->setText(QString("RL: %1  |  %2")
                       .arg(active ? "ACTIVE" : (enabled ? "READY" : "OFF"))
                       .arg(reason_text(reason)));
  status_->setStyleSheet(reason == 0 ? "color:#8fe3a8;" : "color:#ffb36b;");
  detail_->setText(
      QString("policy %1 · scan %2 (age %3s, min %4m) · odom %5 · plan %6\n"
              "v %7  w %8  |  a0 %9  a1 %10  |  infer %11 ms\n"
              "fallback: %12%13")
          .arg(policy ? "loaded" : "none")
          .arg(scan ? "ok" : "no")
          .arg(QString::fromStdString(get(values, "scan_age")))
          .arg(QString::fromStdString(get(values, "scan_min")))
          .arg(QString::fromStdString(get(values, "odom", "0")))
          .arg(QString::fromStdString(get(values, "plan", "0")))
          .arg(QString::fromStdString(get(values, "v", "0")))
          .arg(QString::fromStdString(get(values, "w", "0")))
          .arg(QString::fromStdString(get(values, "a0", "0")))
          .arg(QString::fromStdString(get(values, "a1", "0")))
          .arg(QString::fromStdString(get(values, "infer_ms", "0")))
          .arg(fallback)
          .arg(values.count("error") != 0 ? "\nerror: " + QString::fromStdString(get(values, "error")) : ""));
}

}  // namespace krosc_ws::viz
