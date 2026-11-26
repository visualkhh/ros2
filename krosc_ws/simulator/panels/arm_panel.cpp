#include "panels/arm_panel.hpp"

#include <QCheckBox>
#include <QGridLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QPushButton>
#include <QSignalBlocker>
#include <QSlider>
#include <QTimer>
#include <QVBoxLayout>

#include <algorithm>
#include <cmath>
#include <string>

#include "robot/waffle_pi_arm.hpp"

namespace krosc_ws::sim {

namespace {

constexpr double kPi = 3.14159265358979323846;

struct JointSpec {
  const char* name;
  double min;
  double max;
  bool millimeter;
};

constexpr std::array<JointSpec, 5> kJoints{{
    {"joint1", -kPi, kPi, false},
    {"joint2", -1.5, 1.5, false},
    {"joint3", -1.5, 1.4, false},
    {"joint4", -1.7, 1.97, false},
    {"gripper", -0.011, 0.02, true},
}};

int to_slider(const JointSpec& spec, double value) {
  return spec.millimeter ? static_cast<int>(std::lround(value * 1000.0))
                         : static_cast<int>(std::lround(value * 1000.0));
}

double from_slider(const JointSpec& spec, int value) {
  return spec.millimeter ? static_cast<double>(value) / 1000.0
                         : static_cast<double>(value) / 1000.0;
}

QString value_text(const JointSpec& spec, double value) {
  if (spec.millimeter) {
    return QString::number(value * 1000.0, 'f', 1) + " mm";
  }
  return QString::number(value * 180.0 / kPi, 'f', 1) + QString::fromUtf8("°");
}

}  // namespace

ArmPanel::ArmPanel(WafflePiArm& robot, QWidget* parent) : QWidget(parent), robot_(robot) {
  setObjectName("arm-panel");
  setMinimumWidth(280);
  setMaximumWidth(360);
  setStyleSheet(
      "#arm-panel { background: #11161f; color: #dbe4f0; }"
      "QLabel { color: #b8c4d6; }"
      "QCheckBox { color: #dbe4f0; }"
      "QPushButton { background: #1b2636; color: #dbe4f0; border: 1px solid #31425a;"
      " border-radius: 4px; padding: 5px 8px; }"
      "QPushButton:hover { background: #24344a; }"
      "QSlider::groove:horizontal { height: 4px; background: #2a3648; border-radius: 2px; }"
      "QSlider::handle:horizontal { width: 12px; margin: -5px 0; border-radius: 6px;"
      " background: #4da3ff; }");

  auto* layout = new QVBoxLayout(this);
  layout->setContentsMargins(10, 10, 10, 10);
  layout->setSpacing(8);

  auto* title = new QLabel("MANIPULATOR", this);
  QFont title_font = title->font();
  title_font.setBold(true);
  title->setFont(title_font);
  layout->addWidget(title);

  manual_ = new QCheckBox("Manual control", this);
  manual_->setChecked(robot_.manual_arm());
  layout->addWidget(manual_);

  auto* grid = new QGridLayout();
  grid->setHorizontalSpacing(8);
  grid->setVerticalSpacing(4);
  for (std::size_t index = 0; index < kJoints.size(); ++index) {
    const auto& spec = kJoints[index];
    auto* name = new QLabel(QString::fromLatin1(spec.name), this);
    name->setStyleSheet("color: #c8d4e6;");
    auto* slider = new QSlider(Qt::Horizontal, this);
    slider->setRange(to_slider(spec, spec.min), to_slider(spec, spec.max));
    slider->setSingleStep(1);
    slider->setPageStep(20);
    auto* value = new QLabel(value_text(spec, 0.0), this);
    value->setStyleSheet("color: #eef4ff;");
    value->setMinimumWidth(70);
    value->setAlignment(Qt::AlignRight | Qt::AlignVCenter);
    grid->addWidget(name, static_cast<int>(index), 0);
    grid->addWidget(slider, static_cast<int>(index), 1);
    grid->addWidget(value, static_cast<int>(index), 2);
    sliders_[index] = slider;
    values_[index] = value;
    connect(slider, &QSlider::valueChanged, this, [this](int) {
      if (!updating_) {
        apply_sliders();
      }
    });
  }
  layout->addLayout(grid);

  auto* buttons = new QHBoxLayout();
  auto* home = new QPushButton("HOME", this);
  auto* stow = new QPushButton("STOW", this);
  buttons->addWidget(home);
  buttons->addWidget(stow);
  layout->addLayout(buttons);
  layout->addStretch(1);

  connect(manual_, &QCheckBox::toggled, this, [this](bool checked) {
    robot_.set_manual_arm(checked);
    refresh();
  });
  connect(home, &QPushButton::clicked, this, [this] { set_target(WafflePiArm::kHome); });
  connect(stow, &QPushButton::clicked, this, [this] { set_target(WafflePiArm::kStow); });

  timer_ = new QTimer(this);
  timer_->setInterval(50);
  connect(timer_, &QTimer::timeout, this, [this] { refresh(); });
  timer_->start();
  refresh();
}

void ArmPanel::apply_sliders() {
  std::array<double, 5> target{};
  for (std::size_t index = 0; index < kJoints.size(); ++index) {
    target[index] = from_slider(kJoints[index], sliders_[index]->value());
  }
  set_target(target);
}

void ArmPanel::set_target(const std::array<double, 5>& target) {
  robot_.set_manual_arm(true);
  robot_.set_arm_target(target);
  if (!manual_->isChecked()) {
    const QSignalBlocker blocker(manual_);
    manual_->setChecked(true);
  }
}

void ArmPanel::refresh() {
  for (auto* slider : sliders_) {
    if (slider->isSliderDown()) {
      return;
    }
  }
  const ArmStatus arm = robot_.arm_status();
  updating_ = true;
  for (std::size_t index = 0; index < kJoints.size(); ++index) {
    const QSignalBlocker blocker(sliders_[index]);
    sliders_[index]->setValue(to_slider(kJoints[index], arm.position[index]));
    values_[index]->setText(value_text(kJoints[index], arm.position[index]));
  }
  updating_ = false;
  if (manual_->isChecked() != robot_.manual_arm()) {
    const QSignalBlocker blocker(manual_);
    manual_->setChecked(robot_.manual_arm());
  }
}

}  // namespace krosc_ws::sim
