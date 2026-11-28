#include "plugins/ai_plugin.hpp"

#include <QDockWidget>
#include <QGridLayout>
#include <QHBoxLayout>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonValue>
#include <QLabel>
#include <QMainWindow>
#include <QPushButton>
#include <QStringList>
#include <QTextEdit>
#include <QTimer>
#include <QVBoxLayout>
#include <QWidget>

#include <utility>

#include "srf/msgs.hpp"
#include "srf/node.hpp"
#include "srf/qos.hpp"
#include "srf/service.hpp"
#include "srf/viz/viz_model.hpp"

namespace srf_default_ws::viz {

namespace {

QString stage_text(const QString& stage) {
  if (stage == "thinking") return "의도 파악 중…";
  if (stage == "plan") return "의도 파악 완료";
  if (stage == "executing") return "실행 중";
  if (stage == "dancing") return "춤 추는 중";
  if (stage == "moveit") return "MoveIt 실행 중";
  if (stage == "done") return "완료";
  if (stage == "error") return "오류";
  return stage.isEmpty() ? "-" : stage;
}

QString pick_stage_text(const QString& stage) {
  if (stage == "switching") return "컨트롤러 전환";
  if (stage == "planning") return "계획";
  if (stage == "replanning") return "재계획";
  if (stage == "executing") return "실행";
  if (stage == "approach") return "접근";
  if (stage == "approaching") return "벽/물체 접근";
  if (stage == "navigating") return "이동";
  if (stage == "descend") return "내려가기";
  if (stage == "close") return "그리퍼 닫기";
  if (stage == "lift") return "들기";
  if (stage == "lower") return "내리기";
  if (stage == "open") return "그리퍼 열기";
  if (stage == "retreat") return "후퇴";
  if (stage == "body_approach") return "몸 위 접근";
  if (stage == "body_lower") return "몸 위로 내리기";
  if (stage == "body_open") return "몸 위에 놓기";
  if (stage == "body_retreat") return "몸 위 후퇴";
  if (stage == "done") return "완료";
  if (stage == "cancelled") return "취소";
  if (stage == "error") return "오류";
  return stage.isEmpty() ? "-" : stage;
}

QString route_target(const QString& intent) {
  if (intent == "pick") return "pick/command → D435 → MoveIt";
  if (intent == "place") return "pick/command → MoveIt";
  if (intent == "drive") return "cmd_vel_nav → twist_mux";
  if (intent == "turn") return "cmd_vel_nav → twist_mux";
  if (intent == "arm_preset") return "move_action";
  if (intent == "dance") return "cmd_vel_nav / arm_joint_command";
  if (intent == "buzzer") return "buzzer/melody";
  if (intent == "hide") return "nav/hide";
  if (intent == "approach") return "nav/approach";
  if (intent == "vlm_query") return "vlm/query";
  if (intent == "vla_pick") return "vla/command";
  if (intent == "stop") return "emergency_stop";
  return intent.isEmpty() ? "-" : intent;
}

QString route_text(const QJsonObject& plan, const QString& source) {
  const QString origin = source == "llm" ? "LLM" : source == "rule" ? "rule"
      : source == "llm+rule" || source == "rule+llm" ? "LLM+rule" : "-";
  return origin + " → " + route_target(plan.value("intent").toString());
}

QString server_text(const QJsonValue& value) {
  if (!value.isObject()) return "-";
  const QJsonObject entry = value.toObject();
  QString url = entry.value("url").toString();
  if (url.startsWith("http://")) url.remove(0, 7);
  if (url.startsWith("https://")) url.remove(0, 8);
  const QString model = entry.value("model").toString();
  const QString mode = entry.value("mode").toString();
  const bool enabled = entry.value("enabled").toBool(true);
  const QString suffix = (mode.isEmpty() ? QString() : QString(" [%1]").arg(mode)) +
                         (entry.value("image").toBool(false) ? QString(" [img]") : QString()) +
                         (enabled ? QString() : QString(" (off)"));
  return QString("%1 @ %2%3").arg(model.isEmpty() ? "-" : model,
                                   url.isEmpty() ? "-" : url, suffix);
}

QString plan_text(const QJsonObject& plan) {
  if (plan.isEmpty()) return "-";
  QStringList parts;
  parts << plan.value("intent").toString();
  const auto add = [&parts, &plan](const char* key) {
    const QJsonValue value = plan.value(key);
    if (value.isString() && !value.toString().isEmpty()) {
      parts << value.toString();
    }
  };
  add("direction");
  const auto number = [&plan](const char* key) {
    const QJsonValue value = plan.value(key);
    return value.isDouble() ? value.toDouble() : 0.0;
  };
  const double meters = number("meters");
  const double degrees = number("degrees");
  if (meters != 0.0) parts << QString("%1 m").arg(meters, 0, 'g', 6);
  if (degrees != 0.0) parts << QString("%1°").arg(degrees, 0, 'g', 6);
  add("preset");
  add("target");
  add("melody");
  add("style");
  return parts.join(" · ");
}

}  // namespace

void AiPlugin::initialize(srf::viz::VizHost& host) {
  host_ = &host;
  state_ = std::make_shared<State>();
  dock_ = new QDockWidget("AI", &host.window());
  dock_->setObjectName("aiDock");
  dock_->setAllowedAreas(Qt::LeftDockWidgetArea | Qt::RightDockWidgetArea |
                         Qt::BottomDockWidgetArea);
  dock_->setFeatures(QDockWidget::DockWidgetMovable | QDockWidget::DockWidgetFloatable |
                     QDockWidget::DockWidgetClosable);
  auto* panel = new QWidget(dock_);
  auto* layout = new QVBoxLayout(panel);
  layout->setContentsMargins(8, 8, 8, 8);
  layout->setSpacing(4);
  auto* grid = new QGridLayout();
  grid->setHorizontalSpacing(8);
  grid->setVerticalSpacing(3);
  const auto add_row = [&](const char* label, QLabel*& target, bool wrap = false) {
    const int row = grid->rowCount();
    grid->addWidget(new QLabel(label, panel), row, 0);
    target = new QLabel("-", panel);
    target->setTextInteractionFlags(Qt::TextSelectableByMouse);
    target->setWordWrap(wrap);
    grid->addWidget(target, row, 1);
  };
  add_row("command", command_);
  add_row("plan", plan_, true);
  add_row("action", action_, true);
  add_row("vlm", vlm_, true);
  auto* server_title = new QLabel("SERVER / ROUTING", panel);
  server_title->setStyleSheet("color:#8a9ab3;font-size:11px;margin-top:4px;");
  layout->addLayout(grid);
  layout->addWidget(server_title);
  auto* server_grid = new QGridLayout();
  server_grid->setHorizontalSpacing(8);
  server_grid->setVerticalSpacing(3);
  const auto add_server = [&](const char* label, QLabel*& target) {
    const int row = server_grid->rowCount();
    server_grid->addWidget(new QLabel(label, panel), row, 0);
    target = new QLabel("-", panel);
    target->setTextInteractionFlags(Qt::TextSelectableByMouse);
    target->setWordWrap(true);
    server_grid->addWidget(target, row, 1);
  };
  add_server("source", source_);
  add_server("route", route_);
  add_server("via", via_);
  add_server("llm srv", llm_server_);
  add_server("vlm srv", vlm_server_);
  add_server("vla srv", vla_server_);
  add_server("llm link", llm_link_);
  add_server("vlm link", vlm_link_);
  add_server("vla link", vla_link_);
  layout->addLayout(server_grid);
  auto* context_title = new QLabel("ROBOT CONTEXT", panel);
  context_title->setStyleSheet("color:#8a9ab3;font-size:11px;margin-top:4px;");
  layout->addWidget(context_title);
  auto* context_grid = new QGridLayout();
  context_grid->setHorizontalSpacing(8);
  context_grid->setVerticalSpacing(3);
  const auto add_context = [&](const char* label, QLabel*& target) {
    const int row = context_grid->rowCount();
    context_grid->addWidget(new QLabel(label, panel), row, 0);
    target = new QLabel("-", panel);
    target->setTextInteractionFlags(Qt::TextSelectableByMouse);
    target->setWordWrap(true);
    context_grid->addWidget(target, row, 1);
  };
  add_context("pose", pose_);
  add_context("size", size_);
  add_context("lidar", lidar_);
  add_context("arm", arm_);
  add_context("motor", motor_);
  add_context("wheels", wheels_);
  add_context("cam/depth", camera_);
  layout->addLayout(context_grid);

  input_ = new QTextEdit(panel);
  input_->setPlaceholderText("예: 앞으로 1m 전진해줘");
  input_->setFixedHeight(64);
  layout->addWidget(input_);

  auto* actions = new QHBoxLayout();
  auto* send = new QPushButton("보내기", panel);
  send->setStyleSheet("background:#1f6b43;color:#eafff3;");
  auto* stop = new QPushButton("정지", panel);
  stop->setStyleSheet("background:#7a2a2a;color:#ffecec;");
  actions->addWidget(send);
  actions->addWidget(stop);
  layout->addLayout(actions);
  QObject::connect(send, &QPushButton::clicked, dock_, [this] {
    send_text(input_->toPlainText());
  });
  QObject::connect(stop, &QPushButton::clicked, dock_, [this] {
    send_text("정지");
  });

  auto* fun = new QGridLayout();
  fun->setHorizontalSpacing(4);
  fun->setVerticalSpacing(4);
  int button_index = 0;
  const auto add_fun = [&](const QString& label, const QString& text, bool primary = false) {
    auto* button = new QPushButton(label, panel);
    if (primary) {
      button->setStyleSheet("background:#173d2a;border-color:#2f8a59;color:#eafff3;");
    }
    QObject::connect(button, &QPushButton::clicked, dock_, [this, text] {
      input_->setPlainText(text);
      send_text(text);
    });
    fun->addWidget(button, button_index / 4, button_index % 4);
    ++button_index;
  };
  add_fun("Mario", "마리오 죽을 때 효과음 내줘", true);
  add_fun("성공음", "성공 효과음 내줘");
  add_fun("Pick", "앞에 있는 노란색 퍽을 집어 올려", true);
  add_fun("Place", "내려놔");
  add_fun("On body", "잡은 물건을 내 몸에 올려놔");
  add_fun("Hide", "안 보이는 곳으로 숨어", true);
  add_fun("Wall", "앞에 보이는 벽까지 가까이 붙어");
  add_fun("Dance", "춤춰", true);
  add_fun("Spin", "빙글빙글 돌아");
  add_fun("Show", "노래부르면서 춤춰", true);
  add_fun("Wave", "손 흔들어");
  layout->addLayout(fun);
  layout->addStretch(1);
  dock_->setWidget(panel);
  host.add_dock(dock_, Qt::RightDockWidgetArea);

  info_subscription_ = host.model().node().subscribe<srf::msgs::String>(
      "ai_info", [state = state_](const srf::msgs::String& message) {
        std::lock_guard<std::mutex> lock(state->mutex);
        state->info = message.data;
      }, srf::QOS_LATCHED);
  status_subscription_ = host.model().node().subscribe<srf::msgs::String>(
      "llm/status", [state = state_](const srf::msgs::String& message) {
        std::lock_guard<std::mutex> lock(state->mutex);
        state->llm_status = message.data;
        // done 이벤트에는 plan/source/via가 없으므로 plan 이벤트를 따로 보관한다.
        if (message.data.find("\"stage\":\"thinking\"") != std::string::npos) {
          state->llm_plan.clear();
        }
        if (message.data.find("\"plan\":") != std::string::npos) {
          state->llm_plan = message.data;
        }
      });
  pick_subscription_ = host.model().node().subscribe<srf::msgs::String>(
      "pick/status", [state = state_](const srf::msgs::String& message) {
        std::lock_guard<std::mutex> lock(state->mutex);
        state->pick_status = message.data;
      });
  vlm_subscription_ = host.model().node().subscribe<srf::msgs::String>(
      "vlm/status", [state = state_](const srf::msgs::String& message) {
        std::lock_guard<std::mutex> lock(state->mutex);
        state->vlm_status = message.data;
      });
  context_subscription_ = host.model().node().subscribe<srf::msgs::String>(
      "robot_context", [state = state_](const srf::msgs::String& message) {
        std::lock_guard<std::mutex> lock(state->mutex);
        state->context = message.data;
      });
  timer_ = new QTimer(dock_);
  QObject::connect(timer_, &QTimer::timeout, dock_, [this] { update_ui(); });
  timer_->start(250);
  update_ui();
}

void AiPlugin::shutdown() {
  if (timer_ != nullptr) {
    timer_->stop();
  }
  info_subscription_.cancel();
  status_subscription_.cancel();
  pick_subscription_.cancel();
  vlm_subscription_.cancel();
  context_subscription_.cancel();
}

void AiPlugin::send_text(const QString& text) {
  const QString trimmed = text.trimmed();
  if (trimmed.isEmpty() || host_ == nullptr) {
    return;
  }
  QJsonObject request;
  request["text"] = trimmed;
  const QByteArray payload = QJsonDocument(request).toJson(QJsonDocument::Compact);
  try {
    (void)host_->model().node().context().services().call<srf::msgs::String,
                                                         srf::msgs::String>(
        "llm/command", srf::msgs::String{payload.toStdString()});
    command_->setText("요청 전송…");
    command_->setStyleSheet("color:#ffb36b;");
  } catch (const std::exception& error) {
    command_->setText(QString("전송 실패: %1").arg(QString::fromUtf8(error.what())));
    command_->setStyleSheet("color:#ff6b6b;");
  }
}

void AiPlugin::update_ui() {
  std::string info_text;
  std::string status_text;
  std::string plan_status_text;
  std::string pick_text;
  std::string vlm_text;
  std::string context_text;
  {
    std::lock_guard<std::mutex> lock(state_->mutex);
    info_text = state_->info;
    status_text = state_->llm_status;
    plan_status_text = state_->llm_plan;
    pick_text = state_->pick_status;
    vlm_text = state_->vlm_status;
    context_text = state_->context;
  }
  const auto fmt = [](const QJsonValue& value, int digits = 2) {
    return QString::number(value.toDouble(), 'f', digits);
  };

  const QJsonDocument info = QJsonDocument::fromJson(QByteArray::fromStdString(info_text));
  if (info.isObject()) {
    const QJsonObject object = info.object();
    llm_server_->setText(server_text(object.value("llm")));
    vlm_server_->setText(server_text(object.value("vlm")));
    vla_server_->setText(server_text(object.value("vla")));
  }

  const QJsonDocument status =
      QJsonDocument::fromJson(QByteArray::fromStdString(status_text));
  if (status.isObject()) {
    const QJsonObject event = status.object();
    const QString stage = event.value("stage").toString();
    const QString request = event.value("request_id").toInt() > 0
                                ? QString("#%1 ").arg(event.value("request_id").toInt())
                                : QString();
    command_->setText(request + stage_text(stage));
    command_->setStyleSheet(stage == "error" ? "color:#ff6b6b;"
                              : stage == "done" ? "color:#54d18c;"
                                                : "color:#ffb36b;");
    const QJsonDocument plan_doc =
        QJsonDocument::fromJson(QByteArray::fromStdString(plan_status_text));
    if (plan_doc.isObject()) {
      const QJsonObject plan_event = plan_doc.object();
      const QJsonValue plan_value = plan_event.value("plan");
      if (plan_value.isObject()) {
        const QJsonObject plan = plan_value.toObject();
        plan_->setText(plan_text(plan));
        const QString source = plan_event.value("source").toString();
        source_->setText(source.isEmpty() ? "-" : source);
        source_->setStyleSheet(source == "llm" || source == "llm+rule"
                                   ? "color:#ffb36b;" : "color:#54d18c;");
        route_->setText(route_text(plan, source));
        const QString via = plan_event.value("via").toString();
        via_->setText(via.isEmpty() ? "-" : via);
        via_->setStyleSheet(via.contains("rule") ? "color:#ffb36b;" : "color:#54d18c;");
      }
    }
    const QString error = event.value("error").toString();
    if (!error.isEmpty()) {
      plan_->setText(error);
      plan_->setStyleSheet("color:#ff6b6b;");
    }
  }

  const QJsonDocument vlm =
      QJsonDocument::fromJson(QByteArray::fromStdString(vlm_text));
  if (vlm.isObject()) {
    const QJsonObject event = vlm.object();
    const QString stage = event.value("stage").toString();
    QString detail = event.value("answer").toString(event.value("error").toString());
    if (detail.size() > 90) {
      detail = detail.left(90) + "…";
    }
    const QString label = stage == "querying" ? "질의 중"
                          : stage == "done" ? "완료"
                          : stage == "error" ? "오류" : stage;
    vlm_->setText(detail.isEmpty() ? label : QString("%1 · %2").arg(label, detail));
    vlm_->setStyleSheet(stage == "error" ? "color:#ff6b6b;"
                         : stage == "done" ? "color:#54d18c;"
                                           : "color:#ffb36b;");
  }

  const QJsonDocument context =
      QJsonDocument::fromJson(QByteArray::fromStdString(context_text));
  if (context.isObject()) {
    const QJsonObject c = context.object();
    const QJsonObject pose = c.value("pose").toObject();
    pose_->setText(QString("%1 x %2 y %3 yaw %4°")
                       .arg(pose.value("frame").toString("-"),
                            fmt(pose.value("x")), fmt(pose.value("y")),
                            fmt(pose.value("yaw"), 1)));
    const QJsonObject robot = c.value("robot").toObject();
    const QJsonArray footprint = robot.value("footprint").toArray();
    if (footprint.size() >= 2) {
      size_->setText(QString("foot %1x%2 · h %3 · reach %4 m%5")
                         .arg(fmt(footprint.at(0)), fmt(footprint.at(1)),
                              fmt(robot.value("height")),
                              fmt(robot.value("arm_reach")),
                              robot.value("from_urdf").toBool() ? " · URDF" : ""));
    }
    const QJsonObject lidar = c.value("lidar").toObject();
    lidar_->setText(QString("front %1 · left %2 · right %3 m")
                        .arg(fmt(lidar.value("front")), fmt(lidar.value("left")),
                             fmt(lidar.value("right"))));
    const QJsonObject arm = c.value("arm").toObject();
    QStringList joints;
    for (const QJsonValue& value : arm.value("joints").toArray()) {
      joints << fmt(value, 1);
    }
    arm_->setText(QString("j %1 · grip %2 mm")
                      .arg(joints.join(" / "),
                           fmt(arm.value("gripper").toDouble() * 1000.0, 1)));
    QStringList temperatures;
    for (const QJsonValue& value : arm.value("temperature").toArray()) {
      temperatures << fmt(value, 0);
    }
    QStringList currents;
    for (const QJsonValue& value : arm.value("current").toArray()) {
      currents << fmt(value, 2);
    }
    const QJsonObject wheels = c.value("wheels").toObject();
    if (!temperatures.isEmpty() || !currents.isEmpty()) {
      motor_->setText(QString("arm T %1°C · I %2A · wheel %3/%4°C")
                          .arg(temperatures.join("/"), currents.join("/"),
                               fmt(wheels.value("left_temperature"), 0),
                               fmt(wheels.value("right_temperature"), 0)));
    }
    wheels_->setText(QString("l %1 · r %2 m/s")
                         .arg(fmt(wheels.value("left_vel")),
                              fmt(wheels.value("right_vel"))));
    const QJsonObject camera = c.value("camera").toObject();
    camera_->setText(QString("C %1 %2s · D %3 %4s · z %5~%6 m")
                         .arg(camera.value("color").toString("-"),
                              fmt(camera.value("color_age"), 1),
                              camera.value("depth").toString("-"),
                              fmt(camera.value("depth_age"), 1),
                              fmt(camera.value("depth_min")),
                              fmt(camera.value("depth_median"))));
    const QJsonObject servers = c.value("servers").toObject();
    const auto set_link = [&](QLabel* label, const QJsonObject& health) {
      if (health.isEmpty()) {
        label->setText("-");
        return;
      }
      if (!health.value("enabled").toBool(true)) {
        label->setText("off");
        label->setStyleSheet("color:#8a9ab3;");
        return;
      }
      const bool ok = health.value("ok").toBool();
      label->setText(ok ? QString("연결됨 · %1").arg(health.value("detail").toString("ok"))
                        : QString("오류 · %1").arg(health.value("detail").toString("연결 실패")));
      label->setStyleSheet(ok ? "color:#54d18c;" : "color:#ff6b6b;");
    };
    set_link(llm_link_, servers.value("llm").toObject());
    set_link(vlm_link_, servers.value("vlm").toObject());
    set_link(vla_link_, servers.value("vla").toObject());
  }

  const QJsonDocument pick = QJsonDocument::fromJson(QByteArray::fromStdString(pick_text));
  if (pick.isObject()) {
    const QJsonObject event = pick.object();
    const QString stage = event.value("stage").toString();
    const QString request = event.value("request_id").toInt() > 0
                                ? QString("#%1 ").arg(event.value("request_id").toInt())
                                : QString();
    action_->setText(request + pick_stage_text(stage));
    action_->setStyleSheet(stage == "error" ? "color:#ff6b6b;"
                              : stage == "done" ? "color:#54d18c;"
                                                : "color:#ffb36b;");
  }
}

}  // namespace srf_default_ws::viz
