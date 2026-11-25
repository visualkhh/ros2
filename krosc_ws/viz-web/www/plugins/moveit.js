import { button, buttonRow, parseFields, row, setLevel } from './util.js';

export function init(host) {
  const view = host.addView({ id: 'moveit', title: 'MoveIt' });
  const canvas = document.createElement('canvas');
  canvas.className = 'moveit-canvas';
  view.panel.append(canvas);
  const context = canvas.getContext('2d');

  function project(point, width, height, scale) {
    const x = Number(point.x) || 0;
    const y = Number(point.y) || 0;
    const z = Number(point.z) || 0;
    return {
      x: width * 0.5 + (x - y) * scale,
      y: height * 0.78 - (z + (x + y) * 0.35) * scale,
    };
  }

  function draw() {
    const ratio = window.devicePixelRatio || 1;
    const width = canvas.clientWidth;
    const height = canvas.clientHeight;
    if (width > 0 && height > 0) {
      if (canvas.width !== Math.floor(width * ratio) || canvas.height !== Math.floor(height * ratio)) {
        canvas.width = Math.floor(width * ratio);
        canvas.height = Math.floor(height * ratio);
      }
      context.setTransform(ratio, 0, 0, ratio, 0, 0);
      context.clearRect(0, 0, width, height);
      context.fillStyle = '#5f7188';
      context.font = '12px system-ui';
      context.fillText('drag X/Y · Shift+Z · Ctrl+Pitch', 12, 20);
      const scale = Math.min(width, height) * 0.9 / 0.9;
      context.strokeStyle = '#141c27';
      context.lineWidth = 1;
      for (let i = -2; i <= 2; ++i) {
        const a = project({ x: i * 0.1, y: -0.2, z: 0 }, width, height, scale);
        const b = project({ x: i * 0.1, y: 0.2, z: 0 }, width, height, scale);
        const c = project({ x: -0.2, y: i * 0.1, z: 0 }, width, height, scale);
        const d = project({ x: 0.4, y: i * 0.1, z: 0 }, width, height, scale);
        context.beginPath(); context.moveTo(a.x, a.y); context.lineTo(b.x, b.y); context.stroke();
        context.beginPath(); context.moveTo(c.x, c.y); context.lineTo(d.x, d.y); context.stroke();
      }

      const target = host.latest('moveit_target_pose');
      if (target) {
        const point = project(target.pose.position, width, height, scale);
        context.strokeStyle = '#ffb36b';
        context.lineWidth = 2;
        context.beginPath(); context.arc(point.x, point.y, 8, 0, Math.PI * 2); context.stroke();
        context.beginPath(); context.moveTo(point.x - 12, point.y); context.lineTo(point.x + 12, point.y);
        context.moveTo(point.x, point.y - 12); context.lineTo(point.x, point.y + 12); context.stroke();
      }

      const arm = host.latest('moveit_arm');
      if (arm && Array.isArray(arm.poses) && arm.poses.length > 0) {
        const points = arm.poses.map((pose) => project(pose.pose.position, width, height, scale));
        context.strokeStyle = '#9aa6b9';
        context.lineWidth = 5;
        context.beginPath();
        context.moveTo(points[0].x, points[0].y);
        for (let i = 1; i < points.length; ++i) context.lineTo(points[i].x, points[i].y);
        context.stroke();
        context.fillStyle = '#d7dde8';
        for (const point of points) {
          context.beginPath(); context.arc(point.x, point.y, 4, 0, Math.PI * 2); context.fill();
        }
        const tcp = points[points.length - 1];
        context.fillStyle = '#ffb36b';
        context.beginPath(); context.arc(tcp.x, tcp.y, 6, 0, Math.PI * 2); context.fill();
      } else {
        context.fillStyle = '#7f899b';
        context.font = '13px system-ui';
        context.fillText('moveit_arm 대기 중', 16, 24);
      }
    }
    requestAnimationFrame(draw);
  }
  requestAnimationFrame(draw);

  const body = host.addPanel('MoveIt');
  const status = row(body, 'status');
  const state = row(body, 'state');

  const grid = document.createElement('div');
  grid.className = 'moveit-grid';
  body.append(grid);
  const numeric = (label, value, step) => {
    const wrapper = document.createElement('div');
    wrapper.className = 'moveit-field';
    const name = document.createElement('label');
    name.textContent = label;
    const field = document.createElement('input');
    field.type = 'number';
    field.step = String(step);
    field.value = String(value);
    wrapper.append(name, field);
    grid.append(wrapper);
    return field;
  };
  const xInput = numeric('X', 0.334, 0.001);
  const yInput = numeric('Y', 0.0, 0.001);
  const zInput = numeric('Z', 0.318, 0.001);
  const pitchInput = numeric('Pitch', 0.0, 0.1);
  const gripInput = numeric('Grip', 0.0, 0.1);
  const cartesianRow = document.createElement('label');
  cartesianRow.className = 'moveit-check';
  const cartesian = document.createElement('input');
  cartesian.type = 'checkbox';
  cartesian.checked = true;
  cartesianRow.append(cartesian, document.createTextNode('Cartesian'));
  grid.append(cartesianRow);

  let updating = false;
  let targetInitialized = false;
  let previewTimer = null;
  const targetText = (preview) => `${xInput.value},${yInput.value},${zInput.value},` +
    `${pitchInput.value},${gripInput.value},${cartesian.checked ? 1 : 0},${preview ? 1 : 0}`;
  const publishTarget = (preview) => {
    if (!updating) host.sendString('moveit_target', targetText(preview));
  };
  const schedulePreview = () => {
    if (updating) return;
    if (previewTimer) clearTimeout(previewTimer);
    previewTimer = setTimeout(() => publishTarget(true), 120);
  };
  for (const input of [xInput, yInput, zInput, pitchInput, gripInput]) {
    input.addEventListener('input', schedulePreview);
  }
  cartesian.addEventListener('change', schedulePreview);

  // MoveIt 뷰에서 TCP target을 직접 드래그한다 (X/Y, Shift=Z, Ctrl=Pitch).
  let dragView = false;
  let dragMode = 'xy';
  let dragLast = { x: 0, y: 0 };
  let lastPublish = 0;
  const clamp = (value, min, max) => Math.max(min, Math.min(max, value));
  canvas.style.touchAction = 'none';
  canvas.addEventListener('pointerdown', (event) => {
    dragView = true;
    dragMode = event.shiftKey ? 'z' : event.ctrlKey || event.metaKey ? 'pitch' : 'xy';
    dragLast = { x: event.clientX, y: event.clientY };
    canvas.setPointerCapture(event.pointerId);
    event.preventDefault();
  });
  canvas.addEventListener('pointermove', (event) => {
    if (!dragView) return;
    const dx = event.clientX - dragLast.x;
    const dy = event.clientY - dragLast.y;
    dragLast = { x: event.clientX, y: event.clientY };
    const step = 0.0015;
    if (dragMode === 'z') {
      zInput.value = clamp(Number(zInput.value) - dy * step, 0.02, 0.7).toFixed(4);
    } else if (dragMode === 'pitch') {
      pitchInput.value = clamp(Number(pitchInput.value) + dx * 0.25, -90, 90).toFixed(2);
    } else {
      xInput.value = clamp(Number(xInput.value) + dx * step, -0.15, 0.55).toFixed(4);
      yInput.value = clamp(Number(yInput.value) - dy * step, -0.35, 0.35).toFixed(4);
    }
    const now = performance.now();
    if (now - lastPublish >= 50) {
      publishTarget(true);
      lastPublish = now;
    }
    event.preventDefault();
  });
  canvas.addEventListener('pointerup', (event) => {
    if (dragView) {
      publishTarget(true);
    }
    dragView = false;
    event.preventDefault();
  });
  canvas.addEventListener('pointercancel', () => { dragView = false; });

  const presetRow = buttonRow(body);
  const preset = (mode) => {
    host.sendBool('arm_torque', true);
    host.sendBool('arm_manual', false);
    host.sendString('moveit_preset', mode);
    host.sendEmpty('moveit_execute');
  };
  button(presetRow, 'HOME', () => preset('home'));
  button(presetRow, 'STOW', () => preset('stow'), 'primary');

  const actionRow = buttonRow(body);
  button(actionRow, 'Current', () => {
    const values = parseFields(host.latest('moveit_state')?.data);
    const tcp = (values.get('tcp') || '0,0,0').split(',').map(Number);
    updating = true;
    xInput.value = String(tcp[0] || 0);
    yInput.value = String(tcp[1] || 0);
    zInput.value = String(tcp[2] || 0);
    pitchInput.value = String(Number(values.get('pitch')) || 0);
    gripInput.value = String(Number(values.get('grip')) || 0);
    updating = false;
    publishTarget(true);
  });
  button(actionRow, 'Plan', () => {
    publishTarget(false);
    host.sendEmpty('moveit_plan');
    setLevel(status, 'Plan 요청...', 'warn');
  });
  button(actionRow, 'Execute', () => {
    host.sendEmpty('moveit_execute');
    setLevel(status, 'Execute 요청...', 'warn');
  }, 'primary');
  button(actionRow, 'Cancel', () => host.sendEmpty('moveit_cancel'), 'danger');

  const scrubRow = document.createElement('div');
  scrubRow.className = 'field';
  const scrubLabel = document.createElement('label');
  scrubLabel.textContent = 'trajectory';
  const scrub = document.createElement('input');
  scrub.type = 'range';
  scrub.min = '0';
  scrub.max = '0';
  scrub.value = '0';
  scrub.disabled = true;
  scrubRow.append(scrubLabel, scrub);
  body.append(scrubRow);
  scrub.addEventListener('input', () => {
    host.sendString('moveit_scrub', String(scrub.value));
  });

  host.onTopic('moveit_status', (message) => {
    const text = message.data || '-';
    const level = /fail|error|reject|abort/i.test(text) ? 'bad'
      : /ok|done|succeed|ready|완료/i.test(text) ? 'ok' : 'warn';
    setLevel(status, text, level);
  });
  host.onTopic('moveit_state', (message) => {
    const values = parseFields(message.data);
    const tcp = values.get('tcp') || '-';
    const pitch = values.get('pitch') || '-';
    const grip = values.get('grip') || '-';
    const points = Number(values.get('points') || 0);
    const index = Number(values.get('index') || 0);
    const executing = values.get('executing') === '1';
    const hasPlan = values.get('has_plan') === '1';
    setLevel(state, `tcp ${tcp} · pitch ${pitch}° · grip ${grip} mm · points ${points}` +
                    `${executing ? ' · executing' : hasPlan ? ' · plan ready' : ''}`,
             executing ? 'warn' : hasPlan ? 'ok' : '');
    scrub.max = String(Math.max(0, points - 1));
    scrub.value = String(Math.min(index, Number(scrub.max)));
    scrub.disabled = executing || !hasPlan || points <= 1;
    if (!targetInitialized && tcp !== '-') {
      const parts = tcp.split(',').map(Number);
      if (parts.length >= 3 && parts.every(Number.isFinite)) {
        updating = true;
        xInput.value = String(parts[0]);
        yInput.value = String(parts[1]);
        zInput.value = String(parts[2]);
        if (Number.isFinite(Number(pitch))) pitchInput.value = String(pitch);
        if (Number.isFinite(Number(grip))) gripInput.value = String(grip);
        updating = false;
        targetInitialized = true;
      }
    }
  });
}
