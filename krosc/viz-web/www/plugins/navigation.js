import { button, buttonRow, row, setLevel } from './util.js';

export function init(host) {
  const body = host.addPanel('Navigation');
  const status = row(body, 'goal');
  const hint = document.createElement('div');
  hint.className = 'muted';
  hint.textContent = '맵을 클릭하면 2D goal을 보낸다.';
  body.append(hint);

  let slamMode = 'idle';
  let preview = false;
  let multi = false;

  const buttons = buttonRow(body);
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
    host.setMultiGoal(multi);
    multiButton.className = multi ? 'primary' : '';
    hint.textContent = multi
      ? '맵 클릭 = waypoint 추가 · Backspace = 마지막 취소 · Send로 전체 전송'
      : '맵을 클릭하면 2D goal을 보낸다.';
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
  countLabel.textContent = 'multi goals: 0';
  body.append(countLabel);
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
