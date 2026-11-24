#include "krosc/viz/plugins/teleop_plugin.hpp"

#include <QAbstractSlider>
#include <QApplication>
#include <QDockWidget>
#include <QGridLayout>
#include <QHBoxLayout>
#include <QKeyEvent>
#include <QLabel>
#include <QPushButton>
#include <QSignalBlocker>
#include <QSlider>
#include <QTimer>
#include <QVBoxLayout>
#include <QWidget>

#include <algorithm>
#include <cmath>

#include "krosc/viz/key_bindings.hpp"
#include "krosc/viz/map_view.hpp"
#include "krosc/viz/scene_view.hpp"
#include "krosc/viz/viz_window.hpp"

namespace krosc::viz {

TeleopPlugin::TeleopPlugin() : VizPlugin({"teleop", "Teleop", "1.0", {}}) {}

TeleopPlugin::~TeleopPlugin() { shutdown(); }

void TeleopPlugin::initialize(VizHost& host) {
  if (active_) {
    return;
  }
  host_ = &host;
  host.scene().set_teleop_enabled(true);

  dock_ = new QDockWidget("Teleop", &host.window());
  dock_->setObjectName("teleopDock");
  dock_->setAllowedAreas(Qt::LeftDockWidgetArea | Qt::RightDockWidgetArea | Qt::BottomDockWidgetArea);
  dock_->setFeatures(QDockWidget::DockWidgetMovable | QDockWidget::DockWidgetFloatable |
                     QDockWidget::DockWidgetClosable);
  auto* panel = new QWidget(dock_);
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
    connect(button, &QPushButton::pressed, this, [this, bit] { direction_ |= bit; });
    connect(button, &QPushButton::released, this, [this, bit] { direction_ &= ~bit; });
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
    direction_ = 0;
    publish_pad();
  });

  auto* speed_row = new QHBoxLayout();
  speed_row->addWidget(new QLabel("speed", panel));
  speed_ = new QSlider(Qt::Horizontal, panel);
  speed_->setRange(25, 150);
  speed_->setValue(100);
  speed_label_ = new QLabel("0.26 m/s", panel);
  speed_row->addWidget(speed_, 1);
  speed_row->addWidget(speed_label_);
  layout->addLayout(speed_row);
  connect(speed_, &QSlider::valueChanged, this, [this](int value) {
    const double scale = static_cast<double>(value) / 100.0;
    host_->scene().set_teleop_scale(scale);
    speed_label_->setText(QString("%1 m/s").arg(0.26 * scale, 0, 'f', 2));
  });
  connect(&host.scene(), &SceneView::teleop_scale_changed, this, [this](double scale) {
    if (speed_ == nullptr) {
      return;
    }
    const QSignalBlocker blocker(speed_);
    speed_->setValue(static_cast<int>(std::lround(scale * 100.0)));
    if (speed_label_ != nullptr) {
      speed_label_->setText(QString("%1 m/s").arg(0.26 * scale, 0, 'f', 2));
    }
  });

  dock_->setAttribute(Qt::WA_InputMethodEnabled, false);
  panel->setAttribute(Qt::WA_InputMethodEnabled, false);
  for (QWidget* child : panel->findChildren<QWidget*>()) {
    child->setAttribute(Qt::WA_InputMethodEnabled, false);
  }
  dock_->setWidget(panel);
  host.add_dock(dock_, Qt::RightDockWidgetArea);

  connect(dock_, &QDockWidget::visibilityChanged, this, [this](bool visible) {
    if (!visible) {
      direction_ = 0;
      key_codes_.clear();
      publish_pad();
    }
  });
  connect(&host.map_view(), &MapView2D::teleop_key, this,
          [this](int key, bool pressed) { host_->scene().set_teleop_key(key, pressed); });
  connect(&host.map_view(), &MapView2D::teleop_focus_lost, this, [this] {
    if (host_ != nullptr) {
      host_->scene().clear_teleop_keys();
    }
  });
  connect(qApp, &QApplication::focusChanged, this, [this](QWidget* old_focus, QWidget* new_focus) {
    if (dock_ == nullptr) {
      return;
    }
    const bool old_in_dock = old_focus != nullptr && (old_focus == dock_ || dock_->isAncestorOf(old_focus));
    const bool new_in_dock = new_focus != nullptr && (new_focus == dock_ || dock_->isAncestorOf(new_focus));
    if (old_in_dock && !new_in_dock) {
      if (host_ != nullptr) {
        host_->scene().clear_teleop_keys();
      }
      key_codes_.clear();
      direction_ = 0;
      publish_pad();
    }
  });

  pad_timer_ = new QTimer(this);
  connect(pad_timer_, &QTimer::timeout, this, &TeleopPlugin::publish_pad);
  pad_timer_->start(50);
  qApp->installEventFilter(this);
  active_ = true;
}

void TeleopPlugin::shutdown() {
  if (!active_) {
    return;
  }
  active_ = false;
  qApp->removeEventFilter(this);
  if (pad_timer_ != nullptr) {
    pad_timer_->stop();
  }
  if (host_ != nullptr) {
    host_->scene().clear_teleop_keys();
    host_->scene().set_teleop_enabled(false);
    host_->model().publish_cmd_vel(0.0, 0.0);
  }
  if (dock_ != nullptr) {
    dock_->deleteLater();
    dock_ = nullptr;
  }
}

void TeleopPlugin::publish_pad() {
  if (host_ == nullptr) {
    return;
  }
  const double scale = host_->scene().teleop_scale();
  const double linear = 0.26 * scale * (((direction_ & 1) != 0 ? 1.0 : 0.0) -
                                        ((direction_ & 2) != 0 ? 1.0 : 0.0));
  const double angular = 1.5 * scale * (((direction_ & 4) != 0 ? 1.0 : 0.0) -
                                        ((direction_ & 8) != 0 ? 1.0 : 0.0));
  if (linear != last_linear_ || angular != last_angular_) {
    host_->model().publish_cmd_vel(linear, angular);
    last_linear_ = linear;
    last_angular_ = angular;
  }
}

bool TeleopPlugin::eventFilter(QObject* watched, QEvent* event) {
  (void)watched;
  if (!active_ || dock_ == nullptr || host_ == nullptr) {
    return false;
  }
  if (event->type() != QEvent::KeyPress && event->type() != QEvent::KeyRelease) {
    return false;
  }
  QWidget* focus = QApplication::focusWidget();
  if (focus == nullptr || (focus != dock_ && !dock_->isAncestorOf(focus))) {
    return false;
  }
  auto* key = static_cast<QKeyEvent*>(event);
  const bool press = event->type() == QEvent::KeyPress;
  const int physical = physical_key(key);
  int code = qwerty_key(key);
  if (!press) {
    const auto found = key_codes_.find(physical);
    if (found != key_codes_.end()) {
      code = found.value();
      key_codes_.erase(found);
    }
  }
  if (is_move_key(code)) {
    if (qobject_cast<QAbstractSlider*>(focus) != nullptr &&
        (code == Qt::Key_Up || code == Qt::Key_Down || code == Qt::Key_Left || code == Qt::Key_Right)) {
      return false;
    }
    if (key->isAutoRepeat()) {
      return true;
    }
    if (press) {
      key_codes_[physical] = code;
    }
    host_->scene().set_teleop_key(code, press);
    return true;
  }
  if (press) {
    if (code == Qt::Key_Q) {
      host_->scene().set_teleop_scale(host_->scene().teleop_scale() * 1.25);
      return true;
    }
    if (code == Qt::Key_Z) {
      host_->scene().set_teleop_scale(host_->scene().teleop_scale() / 1.25);
      return true;
    }
    if (code == Qt::Key_F) {
      host_->scene().set_follow(!host_->scene().follow());
      return true;
    }
    if (code == Qt::Key_C || code == Qt::Key_R) {
      host_->scene().reset_camera();
      return true;
    }
  }
  return false;
}

}  // namespace krosc::viz
