import { button, buttonRow, row, setLevel } from './util.js';

export function init(host) {
  const body = host.addPanel('Navigation');
  const status = row(body, 'goal');
  const hint = document.createElement('div');
  hint.className = 'muted';
  hint.textContent = '맵을 클릭하면 2D goal을 보낸다.';
  body.append(hint);

  const buttons = buttonRow(body);
  button(buttons, 'Cancel', () => host.sendBool('cancel_goal', true), 'danger');
  let preview = false;
  const previewButton = button(buttons, 'Preview OFF', () => {
    preview = !preview;
    previewButton.textContent = preview ? 'Preview ON' : 'Preview OFF';
    previewButton.className = preview ? 'primary' : '';
    host.setGoalTopic(preview ? 'preview_goal' : 'goal_pose');
  });
  host.onTopic('goal_status', (message) => {
    const text = message.data || '-';
    const level = /fail|error|abort|cancel/i.test(text) ? 'bad'
      : /succeed|reached|done/i.test(text) ? 'ok' : 'warn';
    setLevel(status, text, level);
  });
}
