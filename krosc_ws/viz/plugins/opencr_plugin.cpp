#include "plugins/opencr_plugin.hpp"

#include <QDockWidget>
#include <QGridLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QMainWindow>
#include <QPushButton>
#include <QSpinBox>
#include <QTimer>
#include <QVBoxLayout>
#include <QWidget>

#include <algorithm>
#include <cmath>
#include <map>
#include <string>

#include "krosc/msgs.hpp"
#include "krosc/qos.hpp"
#include "krosc/viz/viz_model.hpp"

namespace krosc_ws::viz {

namespace {
using Values = std::map<std::string, std::string>;

Values values_of(const krosc::msgs::DiagnosticStatus& status) {
  Values out;
  for (const auto& entry : status.values) {
    out[entry.key] = entry.value;
  }
  return out;
}

std::string get(const Values& values, const std::string& key, const std::string& fallback = "-") {
  const auto found = values.find(key);
  return found == values.end() ? fallback : found->second;
}

double number(const Values& values, const std::string& key) {
  const auto found = values.find(key);
  if (found == values.end()) {
    return std::nan("");
  }
  try {
    return std::stod(found->second);
  } catch (const std::exception&) {
    return std::nan("");
  }
}

QString fixed(double value, int digits) {
  if (!std::isfinite(value)) {
    return "-";
  }
  return QString::number(value, 'f', digits);
}

// 3S Li-ion 대략: 9.6V(0%) ~ 12.6V(100%). 부하 시 전압이 처지므로 참고값.
int battery_percent(double voltage) {
  if (!std::isfinite(voltage)) {
    return -1;
  }
  const double percent = (voltage - 9.6) / 3.0 * 100.0;
  return static_cast<int>(std::clamp(std::lround(percent), 0L, 100L));
}

QString level_style(const std::string& text, int level) {
  const char* color = level == 0 ? "#54d18c" : level == 1 ? "#ffb36b" : "#ff6b6b";
  return QString("<span style='color:%1'>%2</span>")
      .arg(color, QString::fromStdString(text).toHtmlEscaped());
}
}  // namespace

void OpencrPlugin::initialize(krosc::viz::VizHost& host) {
  host_ = &host;
  dock_ = new QDockWidget("OpenCR", &host.window());
  dock_->setObjectName("opencrDock");
  dock_->setAllowedAreas(Qt::LeftDockWidgetArea | Qt::RightDockWidgetArea | Qt::BottomDockWidgetArea);
  dock_->setFeatures(QDockWidget::DockWidgetMovable | QDockWidget::DockWidgetFloatable |
                     QDockWidget::DockWidgetClosable);
  auto* panel = new QWidget(dock_);
  auto* layout = new QVBoxLayout(panel);
  layout->setContentsMargins(8, 8, 8, 8);
  layout->setSpacing(4);
  auto* grid = new QGridLayout();
  grid->setHorizontalSpacing(8);
  grid->setVerticalSpacing(3);
  const auto add_row = [&](const char* label, QLabel*& target) {
    const int row = grid->rowCount();
    grid->addWidget(new QLabel(label, panel), row, 0);
    target = new QLabel("-", panel);
    target->setTextInteractionFlags(Qt::TextSelectableByMouse);
    grid->addWidget(target, row, 1);
  };
  add_row("status", status_);
  add_row("battery", battery_);
  add_row("wheel current", current_);
  add_row("wheel temp", temperature_);
  add_row("uptime", uptime_);
  add_row("link", link_);
  add_row("LDS-03", lidar_);
  layout->addLayout(grid);

  auto* buzzer_row = new QHBoxLayout();
  buzzer_row->addWidget(new QLabel("beep", panel));
  frequency_ = new QSpinBox(panel);
  frequency_->setRange(100, 8000);
  frequency_->setSingleStep(100);
  frequency_->setValue(2000);
  frequency_->setSuffix(" Hz");
  duration_ = new QSpinBox(panel);
  duration_->setRange(10, 2000);
  duration_->setSingleStep(10);
  duration_->setValue(150);
  duration_->setSuffix(" ms");
  beep_ = new QPushButton("Beep", panel);
  buzzer_row->addWidget(frequency_);
  buzzer_row->addWidget(duration_);
  buzzer_row->addWidget(beep_);
  layout->addLayout(buzzer_row);
  QObject::connect(beep_, &QPushButton::clicked, dock_, [this] { beep(); });

  dock_->setWidget(panel);
  host.add_dock(dock_, Qt::RightDockWidgetArea);

  timer_ = new QTimer(dock_);
  QObject::connect(timer_, &QTimer::timeout, dock_, [this] { refresh(); });
  timer_->start(200);
  refresh();
}

void OpencrPlugin::shutdown() {
  if (timer_ != nullptr) {
    timer_->stop();
  }
}

void OpencrPlugin::beep() {
  if (host_ == nullptr) {
    return;
  }
  const std::string text =
      std::to_string(frequency_->value()) + "," + std::to_string(duration_->value());
  host_->model().node().publish("buzzer", krosc::msgs::String{text}, krosc::QOS_DEFAULT);
}

void OpencrPlugin::refresh() {
  if (host_ == nullptr) {
    return;
  }
  const auto diagnostics = host_->model().diagnostics();
  if (!diagnostics) {
    return;
  }
  for (const auto& item : diagnostics->status) {
    const Values values = values_of(item);
    if (item.name == "OpenCR") {
      status_->setText(level_style(item.message.empty() ? "-" : item.message, item.level));
      const double volts = number(values, "battery");
      const int percent = battery_percent(volts);
      battery_->setText(QString("%1 V%2")
                            .arg(fixed(volts, 2),
                                 percent < 0 ? QString() : QString(" (≈%1%)").arg(percent)));
      battery_->setStyleSheet(percent < 0 || percent < 25 ? "color:#ff6b6b"
                              : percent < 50              ? "color:#ffb36b"
                                                          : "color:#54d18c");
      current_->setText(QString("L %1 A · R %2 A")
                            .arg(fixed(number(values, "left_current"), 2),
                                 fixed(number(values, "right_current"), 2)));
      temperature_->setText(QString("L %1℃ · R %2℃")
                                .arg(fixed(number(values, "left_temperature"), 0),
                                     fixed(number(values, "right_temperature"), 0)));
      const double seconds = number(values, "uptime_ms") / 1000.0;
      if (std::isfinite(seconds) && seconds >= 0.0) {
        const int hours = static_cast<int>(seconds) / 3600;
        const int minutes = (static_cast<int>(seconds) % 3600) / 60;
        uptime_->setText(QString("%1h %2m").arg(hours).arg(minutes, 2, 10, QChar('0')));
      } else {
        uptime_->setText("-");
      }
      const std::string last_error = get(values, "last_error", "-");
      link_->setText(QString("bad %1 · err %2 · age %3s%4")
                         .arg(QString::fromStdString(get(values, "bad_lines", "0")),
                              QString::fromStdString(get(values, "errors", "0")),
                              fixed(number(values, "status_age"), 2),
                              last_error == "-" ? QString()
                                                : QString(" · %1").arg(QString::fromStdString(last_error))));
    } else if (item.name == "LDS-03") {
      lidar_->setText(level_style(
          QString("scans %1 · bad %2 · age %3s")
              .arg(QString::fromStdString(get(values, "scans", "-")),
                   QString::fromStdString(get(values, "bad_packets", "0")),
                   fixed(number(values, "scan_age"), 2))
              .toStdString(),
          item.level));
    }
  }
}

}  // namespace krosc_ws::viz
