#include "plugins/project_panel.hpp"

#include <QDateTime>
#include <QMainWindow>
#include <QLabel>
#include <QTimer>
#include <QVBoxLayout>
#include <QWidget>

#include "srf/viz/viz_model.hpp"

namespace srf_default_ws::viz {

void ProjectPanelPlugin::initialize(srf::viz::VizHost& host) {
  auto* page = new QWidget(&host.window());
  auto* layout = new QVBoxLayout(page);
  layout->setContentsMargins(18, 18, 18, 18);
  layout->setSpacing(8);

  auto* title = new QLabel("srf_default_ws project plugin", page);
  title->setObjectName("Title");
  layout->addWidget(title);

  auto* subtitle = new QLabel(
      "이 탭은 프로젝트 플러그인 예제다. srf-default-ws/viz/plugins/ 에서 직접 수정한다.", page);
  subtitle->setWordWrap(true);
  subtitle->setObjectName("Subtitle");
  layout->addWidget(subtitle);

  auto* body = new QLabel("waiting for data...", page);
  body->setWordWrap(true);
  body->setTextInteractionFlags(Qt::TextSelectableByMouse);
  layout->addWidget(body);
  layout->addStretch(1);

  host.add_tab(page, "Project");

  auto* timer = new QTimer(page);
  QObject::connect(timer, &QTimer::timeout, page, [&host, body] {
    QString text;
    if (const auto arm = host.model().arm_status()) {
      text += "arm_status: " + QString::fromStdString(arm->data) + "\n";
    }
    if (const auto moveit = host.model().moveit_state()) {
      text += "moveit_state: " + QString::fromStdString(moveit->data) + "\n";
    }
    if (const auto diagnostics = host.model().diagnostics(); diagnostics && !diagnostics->status.empty()) {
      text += "diagnostics: " + QString::fromStdString(diagnostics->status.front().message) + "\n";
    }
    text += "updated: " + QDateTime::currentDateTime().toString("HH:mm:ss");
    body->setText(text);
  });
  timer->start(500);
}

}  // namespace srf_default_ws::viz
