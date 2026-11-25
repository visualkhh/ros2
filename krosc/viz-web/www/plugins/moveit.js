import { button, buttonRow, parseFields, row, setLevel } from './util.js';

export function init(host) {
  const body = host.addPanel('MoveIt');
  const status = row(body, 'status');
  const state = row(body, 'state');

  const targetRow = document.createElement('div');
  targetRow.className = 'field';
  const targetLabel = document.createElement('label');
  targetLabel.textContent = 'x,y,z,pitch,grip';
  const target = document.createElement('input');
  target.value = '0.334,0,0.318,0,0';
  targetRow.append(targetLabel, target);
  body.append(targetRow);

  const buttons = buttonRow(body);
  button(buttons, 'Plan', () => {
    host.sendString('moveit_target', `${target.value},1,0`);
    host.sendEmpty('moveit_plan');
  });
  button(buttons, 'Execute', () => {
    host.sendString('moveit_target', `${target.value},1,0`);
    host.sendEmpty('moveit_plan');
    host.sendEmpty('moveit_execute');
  }, 'primary');
  button(buttons, 'Cancel', () => host.sendEmpty('moveit_cancel'), 'danger');

  host.onTopic('moveit_status', (message) => {
    const text = message.data || '-';
    const level = /fail|error|reject|abort/i.test(text) ? 'bad'
      : /ok|done|succeed|ready/i.test(text) ? 'ok' : 'warn';
    setLevel(status, text, level);
  });
  host.onTopic('moveit_state', (message) => {
    const values = parseFields(message.data);
    const tcp = values.get('tcp') || '-';
    const pitch = values.get('pitch') || '-';
    const grip = values.get('grip') || '-';
    const points = values.get('points') || '0';
    const executing = values.get('executing') === '1';
    const hasPlan = values.get('has_plan') === '1';
    setLevel(state, `tcp ${tcp} · pitch ${pitch}° · grip ${grip} · points ${points}` +
                    `${executing ? ' · executing' : hasPlan ? ' · plan ready' : ''}`,
             executing ? 'warn' : hasPlan ? 'ok' : '');
  });
}
