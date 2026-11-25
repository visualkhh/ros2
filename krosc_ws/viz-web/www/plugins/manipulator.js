import { button, buttonRow, row, setLevel, select } from './util.js';

const DEG = 180 / Math.PI;
const RAD = Math.PI / 180;
const NAMES = ['joint1', 'joint2', 'joint3', 'joint4', 'gripper_left_joint'];
const RANGES = [[-180, 180], [-126, 126], [-114, 114], [-114, 126]];

export function init(host) {
  const body = host.addPanel('Manipulator');
  const status = row(body, 'arm');
  status.classList.add('nowrap');

  const actions = buttonRow(body);
  let torqueOn = true;
  const torqueButton = button(actions, 'Torque ON', () => {
    torqueOn = !torqueOn;
    host.sendBool('arm_torque', torqueOn);
    renderTorque();
    if (!torqueOn) moving = false;
  }, 'primary');
  function renderTorque() {
    torqueButton.textContent = torqueOn ? 'Torque ON' : 'Torque OFF';
    torqueButton.className = torqueOn ? 'primary' : 'danger';
  }
  renderTorque();

  const preset = (mode) => {
    torqueOn = true;
    host.sendBool('arm_torque', true);
    renderTorque();
    host.sendBool('arm_manual', false);
    host.sendString('moveit_preset', mode);
    host.sendEmpty('moveit_execute');
    moving = false;
    setLevel(status, `${mode.toUpperCase()} → MoveIt 실행`);
  };
  button(actions, 'STOW', () => preset('stow'), 'primary');
  button(actions, 'HOME', () => preset('home'));
  button(actions, 'E-STOP', () => {
    moving = false;
    host.sendEmpty('arm_stop');
    setLevel(status, 'E-STOP: 현재 자세 유지', 'warn');
  }, 'danger');

  const resetRow = document.createElement('div');
  resetRow.className = 'field';
  const resetLabel = document.createElement('label');
  resetLabel.textContent = 'reset';
  const resetTarget = select(resetRow, ['all', 'joint1', 'joint2', 'joint3', 'joint4', 'gripper']);
  const resetButton = document.createElement('button');
  resetButton.type = 'button';
  resetButton.textContent = 'Reset (AR)';
  resetRow.append(resetTarget, resetButton);
  body.append(resetRow);
  resetButton.addEventListener('click', () => {
    moving = false;
    host.sendString('arm_reset', resetTarget.value);
    setLevel(status, `reset(${resetTarget.value}) 요청`, 'warn');
    resetButton.disabled = true;
    setTimeout(() => {
      resetButton.disabled = false;
      torqueOn = true;
      host.sendBool('arm_torque', true);
      renderTorque();
    }, 2000);
  });

  const output = [0, 0, 0, 0, 0];
  const target = [0, 0, 0, 0, 0];
  const sliders = [];
  const labels = [];
  let moving = false;
  let updating = false;
  let speed = 0.35;

  for (let index = 0; index < 4; ++index) {
    const wrapper = document.createElement('div');
    wrapper.className = 'field';
    const label = document.createElement('label');
    label.textContent = `joint${index + 1}`;
    const range = document.createElement('input');
    range.type = 'range';
    range.min = String(RANGES[index][0]);
    range.max = String(RANGES[index][1]);
    range.value = '0';
    const value = document.createElement('div');
    value.className = 'value';
    value.textContent = '0.0°';
    wrapper.append(label, range, value);
    body.append(wrapper);
    sliders.push(range);
    labels.push(value);
    range.addEventListener('input', () => {
      const degrees = Number(range.value);
      value.textContent = `${degrees.toFixed(1)}°`;
      if (updating) return;
      if (!torqueOn) {
        torqueOn = true;
        host.sendBool('arm_torque', true);
        renderTorque();
      }
      host.sendBool('arm_manual', true);
      target[index] = degrees * RAD;
      moving = true;
    });
  }

  const gripperWrapper = document.createElement('div');
  gripperWrapper.className = 'field';
  const gripperLabel = document.createElement('label');
  gripperLabel.textContent = 'gripper';
  const gripper = document.createElement('input');
  gripper.type = 'range';
  gripper.min = '0';
  gripper.max = '27';
  gripper.value = '0';
  const gripperValue = document.createElement('div');
  gripperValue.className = 'value';
  gripperValue.textContent = '0.0 mm';
  gripperWrapper.append(gripperLabel, gripper, gripperValue);
  body.append(gripperWrapper);
  gripper.addEventListener('input', () => {
    const mm = Number(gripper.value);
    gripperValue.textContent = `${mm.toFixed(1)} mm`;
    if (updating) return;
    if (!torqueOn) {
      torqueOn = true;
      host.sendBool('arm_torque', true);
      renderTorque();
    }
    host.sendBool('arm_manual', true);
    target[4] = mm / 1000;
    moving = true;
  });

  const speedWrapper = document.createElement('div');
  speedWrapper.className = 'field';
  const speedLabel = document.createElement('label');
  speedLabel.textContent = 'speed';
  const speedRange = document.createElement('input');
  speedRange.type = 'range';
  speedRange.min = '10';
  speedRange.max = '80';
  speedRange.value = '35';
  const speedValue = document.createElement('div');
  speedValue.className = 'value';
  speedValue.textContent = '0.35 rad/s';
  speedWrapper.append(speedLabel, speedRange, speedValue);
  body.append(speedWrapper);
  speedRange.addEventListener('input', () => {
    speed = Number(speedRange.value) / 100;
    speedValue.textContent = `${speed.toFixed(2)} rad/s`;
  });

  function applyJointStates(message) {
    if (!message || !Array.isArray(message.names) || !Array.isArray(message.positions)) return;
    const values = new Map();
    for (let index = 0; index < message.names.length && index < message.positions.length; ++index) {
      values.set(message.names[index], message.positions[index]);
    }
    if (moving) return;
    for (let index = 0; index < 4; ++index) {
      const value = values.get(NAMES[index]);
      if (Number.isFinite(value)) {
        output[index] = value;
        target[index] = value;
        updating = true;
        sliders[index].value = String(Math.round(value * DEG));
        labels[index].textContent = `${(value * DEG).toFixed(1)}°`;
        updating = false;
      }
    }
    const grip = values.get(NAMES[4]);
    if (Number.isFinite(grip)) {
      output[4] = grip;
      target[4] = grip;
      updating = true;
      gripper.value = String(Math.round(grip * 1000));
      gripperValue.textContent = `${(grip * 1000).toFixed(1)} mm`;
      updating = false;
    }
    const angle = (value) => `${Math.round(value * DEG)}`.padStart(4, ' ');
    const gripText = `${(output[4] * 1000).toFixed(1)}`.padStart(5, ' ');
    setLevel(status, `j1 ${angle(output[0])}°  j2 ${angle(output[1])}°  ` +
                     `j3 ${angle(output[2])}°  j4 ${angle(output[3])}°  grip ${gripText} mm`);
  }

  host.onTopic('joint_states', applyJointStates);
  host.onTopic('arm_status', (message) => {
    const text = String(message?.data || '');
    const error = /err=([^;]+)/.exec(text);
    if (error && !/^0(,0)*$/.test(error[1])) {
      setLevel(status, `ARM ERR ${error[1]}  |  ${text}`, 'bad');
    }
  });

  setInterval(() => {
    if (!moving) return;
    let moved = false;
    for (let index = 0; index < 5; ++index) {
      const delta = target[index] - output[index];
      const step = (index === 4 ? 0.04 : speed) * 0.05;
      if (Math.abs(delta) > step) {
        output[index] += Math.sign(delta) * step;
        moved = true;
      } else if (Math.abs(delta) > 1e-6) {
        output[index] = target[index];
        moved = true;
      }
    }
    if (!moved) {
      moving = false;
      return;
    }
    host.sendJointState('arm_joint_command', NAMES, output);
  }, 50);
}
