import { button, buttonRow, row, setLevel } from './util.js';

export function init(host) {
  const body = host.addPanel('Navigation');
  const status = row(body, 'goal');
  const hint = document.createElement('div');
  hint.className = 'muted';
  body.append(hint);

  let slamMode = 'idle';
  let preview = false;
  let multi = false;
  let armed = false;
  let poseArmed = false;

  const buttons = buttonRow(body);
  const poseButton = button(buttons, '2D Pose', () => {
    poseArmed = !poseArmed;
    if (poseArmed) {
      armed = false;
      multi = false;
      host.setGoalArmed(false);
      host.setMultiGoal(false);
    }
    host.setPoseArmed(poseArmed);
    updateUi();
  });
  const goalButton = button(buttons, '2D Goal', () => {
    armed = !armed;
    if (armed) {
      poseArmed = false;
      multi = false;
      host.setPoseArmed(false);
      host.setMultiGoal(false);
    }
    host.setGoalArmed(armed);
    updateUi();
  });
  button(buttons, 'Cancel', () => {
    host.clearWaypoints();
    host.sendBool('cancel_goal', true);
    setLevel(status, 'cancel requested', 'warn');
  }, 'danger');
  const previewButton = button(buttons, 'Preview OFF', () => {
    preview = !preview;
    previewButton.textContent = preview ? 'Preview ON' : 'Preview OFF';
    previewButton.className = preview ? 'primary' : '';
    host.setGoalTopic(preview ? 'preview_goal' : 'goal_pose');
  });

  const multiRow = buttonRow(body);
  const multiButton = button(multiRow, 'Multi Goal', () => {
    multi = !multi;
    if (multi) {
      armed = false;
      poseArmed = false;
      host.setGoalArmed(false);
      host.setPoseArmed(false);
    }
    host.setMultiGoal(multi);
    updateUi();
  });
  button(multiRow, 'Send', () => {
    const count = host.waypointCount();
    if (count === 0) {
      setLevel(status, 'waypoint가 없다', 'warn');
      return;
    }
    if (slamMode !== 'localization') {
      setLevel(status, 'load a map first (localization)', 'warn');
      return;
    }
    host.sendWaypoints();
    setLevel(status, `sent ${count} goals`, 'ok');
  }, 'primary');
  button(multiRow, 'Clear', () => {
    host.clearWaypoints();
    host.sendBool('cancel_goal', true);
    setLevel(status, 'goals cleared', 'warn');
  });

  const countLabel = document.createElement('div');
  countLabel.className = 'muted';
  body.append(countLabel);

  function updateUi() {
    poseButton.className = poseArmed ? 'primary' : '';
    goalButton.className = armed ? 'primary' : '';
    multiButton.className = multi ? 'primary' : '';
    if (multi) {
      hint.textContent =
        '맵을 누른 채 드래그 = waypoint + 방향 · 클릭만 하면 방향 0 · Backspace = 마지막 취소 · Send로 전송';
    } else if (armed) {
      hint.textContent = '맵을 클릭하면 goal을 전송한다 (1회, 자동 해제)';
    } else if (poseArmed) {
      hint.textContent = '맵을 누른 채 드래그 = 위치+방향(initialpose) · 클릭만 하면 방향 0';
    } else {
      hint.textContent = '2D Goal을 켠 뒤 맵을 클릭하면 이동한다. (그냥 클릭해도 안 움직임)';
    }
  }
  updateUi();

  // goal을 한 번 보내면 자동으로 해제 (실수로 계속 보내는 것 방지)
  host.onGoalSent(() => {
    armed = false;
    host.setGoalArmed(false);
    updateUi();
  });
  host.onPoseSent(() => {
    poseArmed = false;
    host.setPoseArmed(false);
    updateUi();
  });

  setInterval(() => {
    countLabel.textContent = `multi goals: ${host.waypointCount()}`;
  }, 300);

  host.onTopic('slam_status', (message) => {
    slamMode = String(message.data || 'idle');
  });
  host.onTopic('goal_status', (message) => {
    const text = message.data || '-';
    const level = /fail|error|abort|cancel/i.test(text) ? 'bad'
      : /succeed|reached|done/i.test(text) ? 'ok' : 'warn';
    setLevel(status, text, level);
  });
}
