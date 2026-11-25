import { button, buttonRow, parseFields, row, setLevel } from './util.js';

export function init(host) {
  const view = host.addView({ id: 'moveit', title: 'MoveIt' });
  const canvas = document.createElement('canvas');
  canvas.className = 'moveit-canvas';
  view.panel.append(canvas);
  const context = canvas.getContext('2d');

  const camera = { yaw: -0.6, pitch: 0.45, distance: 1.05, tx: 0.15, ty: 0.0, tz: 0.18 };
  let targetScreenPoint = null;
  let draggingTarget = false;
  let lastLocalEdit = 0;

  function project(point, width, height) {
    const dx = (Number(point.x) || 0) - camera.tx;
    const dy = (Number(point.y) || 0) - camera.ty;
    const dz = (Number(point.z) || 0) - camera.tz;
    const cy = Math.cos(camera.yaw);
    const sy = Math.sin(camera.yaw);
    const rx = cy * dx + sy * dy;
    const ry = -sy * dx + cy * dy;
    const cp = Math.cos(camera.pitch);
    const sp = Math.sin(camera.pitch);
    const depth = ry * sp - dz * cp + camera.distance;
    if (depth <= 0.05) return null;
    const scale = Math.min(width, height) * 0.92 / depth;
    return { x: width * 0.5 + rx * scale, y: height * 0.58 - (ry * cp + dz * sp) * scale, depth };
  }

  function drawBox(box, width, height) {
    const edges = [[0, 1], [0, 2], [0, 4], [1, 3], [1, 5], [2, 3], [2, 6], [3, 7],
                   [4, 5], [4, 6], [5, 7], [6, 7]];
    const points = box.corners.map((corner) => project(
        { x: corner[0], y: corner[1], z: corner[2] }, width, height));
    if (points.some((point) => point === null)) return;
    context.strokeStyle = '#b8c2d2';
    context.lineWidth = 1.5;
    context.beginPath();
    for (const [a, b] of edges) {
      context.moveTo(points[a].x, points[a].y);
      context.lineTo(points[b].x, points[b].y);
    }
    context.stroke();
    context.fillStyle = 'rgba(160, 175, 195, 0.12)';
    context.beginPath();
    context.moveTo(points[0].x, points[0].y);
    for (const point of points.slice(1)) context.lineTo(point.x, point.y);
    context.closePath();
    context.fill();
  }

  function applyMatrix(matrix, point) {
    return [
      matrix[0][0] * point[0] + matrix[0][1] * point[1] + matrix[0][2] * point[2] + matrix[0][3],
      matrix[1][0] * point[0] + matrix[1][1] * point[1] + matrix[1][2] * point[2] + matrix[1][3],
      matrix[2][0] * point[0] + matrix[2][1] * point[1] + matrix[2][2] * point[2] + matrix[2][3],
    ];
  }

  // 쿼터니언 → 3x3 회전행렬 (목표 자세의 평면에 링을 그리기 위해)
  function quatToMatrix(quaternion) {
    if (!quaternion || ![quaternion.x, quaternion.y, quaternion.z, quaternion.w].every(Number.isFinite)) {
      return [[1, 0, 0], [0, 1, 0], [0, 0, 1]];
    }
    const length = Math.hypot(quaternion.x, quaternion.y, quaternion.z, quaternion.w) || 1;
    const x = quaternion.x / length;
    const y = quaternion.y / length;
    const z = quaternion.z / length;
    const w = quaternion.w / length;
    return [
      [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
      [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
      [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ];
  }

  function drawPrimitive(primitive, width, height) {
    if (primitive.type === 'box') {
      drawBox(primitive, width, height);
      return;
    }
    if (primitive.type === 'cylinder') {
      const count = 12;
      const top = [];
      const bottom = [];
      for (let i = 0; i < count; ++i) {
        const angle = (i / count) * Math.PI * 2;
        const x = Math.cos(angle) * primitive.radius;
        const y = Math.sin(angle) * primitive.radius;
        top.push(applyMatrix(primitive.transform, [x, y, primitive.length / 2]));
        bottom.push(applyMatrix(primitive.transform, [x, y, -primitive.length / 2]));
      }
      const projectedTop = top.map((point) => project({ x: point[0], y: point[1], z: point[2] }, width, height));
      const projectedBottom = bottom.map((point) => project({ x: point[0], y: point[1], z: point[2] }, width, height));
      if (projectedTop.some((point) => !point) || projectedBottom.some((point) => !point)) return;
      context.strokeStyle = '#b8c2d2';
      context.lineWidth = 1.2;
      context.beginPath();
      for (let i = 0; i < count; ++i) {
        const next = (i + 1) % count;
        context.moveTo(projectedTop[i].x, projectedTop[i].y);
        context.lineTo(projectedTop[next].x, projectedTop[next].y);
        context.moveTo(projectedBottom[i].x, projectedBottom[i].y);
        context.lineTo(projectedBottom[next].x, projectedBottom[next].y);
        context.moveTo(projectedTop[i].x, projectedTop[i].y);
        context.lineTo(projectedBottom[i].x, projectedBottom[i].y);
      }
      context.stroke();
      return;
    }
    if (primitive.type === 'sphere') {
      const center = project({ x: primitive.center[0], y: primitive.center[1], z: primitive.center[2] }, width, height);
      const edge = project({
        x: primitive.center[0] + primitive.radius,
        y: primitive.center[1],
        z: primitive.center[2],
      }, width, height);
      if (!center || !edge) return;
      context.strokeStyle = '#b8c2d2';
      context.beginPath();
      context.arc(center.x, center.y, Math.max(1, Math.hypot(edge.x - center.x, edge.y - center.y)), 0, Math.PI * 2);
      context.stroke();
    }
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
      context.fillText('TCP 마커를 클릭해 드래그 · Shift+Z · Ctrl+Pitch · Alt+Grip · 빈 곳/우드래그 orbit',
                       12, height - 12);

      context.strokeStyle = '#141c27';
      context.lineWidth = 1;
      for (let i = -1; i <= 4; ++i) {
        const a = project({ x: i * 0.1, y: -0.3, z: 0 }, width, height);
        const b = project({ x: i * 0.1, y: 0.3, z: 0 }, width, height);
        const c = project({ x: -0.1, y: i * 0.1, z: 0 }, width, height);
        const d = project({ x: 0.4, y: i * 0.1, z: 0 }, width, height);
        if (a && b) { context.beginPath(); context.moveTo(a.x, a.y); context.lineTo(b.x, b.y); context.stroke(); }
        if (c && d) { context.beginPath(); context.moveTo(c.x, c.y); context.lineTo(d.x, d.y); context.stroke(); }
      }

      // 계획된 TCP 궤적 (Qt Viz와 동일한 moveit_path).
      const path = host.latest('moveit_path');
      if (path && Array.isArray(path.poses) && path.poses.length > 1) {
        context.strokeStyle = 'rgba(95, 211, 160, 0.9)';
        context.lineWidth = 2;
        context.beginPath();
        let started = false;
        for (const stamped of path.poses) {
          const point = project(stamped.pose.position, width, height);
          if (!point) {
            started = false;
            continue;
          }
          if (started) context.lineTo(point.x, point.y);
          else { context.moveTo(point.x, point.y); started = true; }
        }
        context.stroke();
        const end = project(path.poses[path.poses.length - 1].pose.position, width, height);
        if (end) {
          context.fillStyle = '#5fd3a0';
          context.beginPath(); context.arc(end.x, end.y, 3.5, 0, Math.PI * 2); context.fill();
        }
      }

      // 미리보기/스크럽 자세의 링크 체인 (계획 시점의 moveit_arm).
      const previewArm = host.latest('moveit_arm');
      if (previewArm && Array.isArray(previewArm.poses) && previewArm.poses.length > 1) {
        const points = previewArm.poses.map((stamped) => project(stamped.pose.position, width, height))
            .filter(Boolean);
        if (points.length > 1) {
          context.strokeStyle = '#6aa9ff';
          context.lineWidth = 1.5;
          context.setLineDash([4, 3]);
          context.beginPath();
          context.moveTo(points[0].x, points[0].y);
          for (let i = 1; i < points.length; ++i) context.lineTo(points[i].x, points[i].y);
          context.stroke();
          context.setLineDash([]);
        }
      }

      if (host.settings.urdf_collision) {
        for (const primitive of (host.urdfPrimitives ? host.urdfPrimitives() : [])) {
          drawPrimitive(primitive, width, height);
        }
      }
      if (host.settings.urdf_visual) {
        for (const primitive of (host.urdfPrimitives ? host.urdfPrimitives((name) => !host.isArmLink(name), true) : [])) {
          drawPrimitive(primitive, width, height);
        }
        const arm = host.armPoints ? host.armPoints() : [];
        const points = arm.map((point) => project(point, width, height)).filter(Boolean);
        if (points.length > 1) {
          context.strokeStyle = '#d7dde8';
          context.lineWidth = 2;
          context.beginPath();
          context.moveTo(points[0].x, points[0].y);
          for (let i = 1; i < points.length; ++i) context.lineTo(points[i].x, points[i].y);
          context.stroke();
          context.fillStyle = '#9aa6b9';
          for (const point of points) {
            context.beginPath(); context.arc(point.x, point.y, 3, 0, Math.PI * 2); context.fill();
          }
        }
      }

      if (host.settings.axes) {
        const origin = project({ x: 0, y: 0, z: 0 }, width, height);
        const axes = [
          [project({ x: 0.08, y: 0, z: 0 }, width, height), '#ff6b6b'],
          [project({ x: 0, y: 0.08, z: 0 }, width, height), '#54d18c'],
          [project({ x: 0, y: 0, z: 0.08 }, width, height), '#6aa9ff'],
        ];
        if (origin) {
          context.lineWidth = 1.5;
          for (const [end, color] of axes) {
            if (!end) continue;
            context.strokeStyle = color;
            context.beginPath(); context.moveTo(origin.x, origin.y); context.lineTo(end.x, end.y); context.stroke();
          }
        }
      }
      if (host.settings.joint_axes) {
        for (const point of (host.armPoints ? host.armPoints() : [])) {
          const origin = project(point, width, height);
          if (!origin) continue;
          const axes = [
            [project({ x: point.x + 0.025, y: point.y, z: point.z }, width, height), '#ff6b6b'],
            [project({ x: point.x, y: point.y + 0.025, z: point.z }, width, height), '#54d18c'],
            [project({ x: point.x, y: point.y, z: point.z + 0.025 }, width, height), '#6aa9ff'],
          ];
          context.lineWidth = 1;
          for (const [end, color] of axes) {
            if (!end) continue;
            context.strokeStyle = color;
            context.beginPath(); context.moveTo(origin.x, origin.y); context.lineTo(end.x, end.y); context.stroke();
          }
        }
      }

      const target = host.latest('moveit_target_pose');
      targetScreenPoint = null;
      // 드래그 중에는 네트워크 왕복을 기다리지 않고 입력값으로 즉시 그린다 (Qt와 동일한 반응).
      let pose = target ? target.pose.position : null;
      let orientation = target ? target.pose.orientation : null;
      if (draggingTarget || performance.now() - lastLocalEdit < 250) {
        pose = { x: Number(xInput.value), y: Number(yInput.value), z: Number(zInput.value) };
      }
      if (pose && Number.isFinite(pose.x) && Number.isFinite(pose.y) && Number.isFinite(pose.z)) {
        const point = project(pose, width, height);
        if (point) {
          targetScreenPoint = point;
          // Qt Viz처럼 목표 자세 평면에 놓인 동그란 판(링) + 3축으로 그린다.
          const r = quatToMatrix(orientation);
          const toWorld = (local) => ({
            x: pose.x + r[0][0] * local[0] + r[0][1] * local[1] + r[0][2] * local[2],
            y: pose.y + r[1][0] * local[0] + r[1][1] * local[1] + r[1][2] * local[2],
            z: pose.z + r[2][0] * local[0] + r[2][1] * local[1] + r[2][2] * local[2],
          });
          const ring = [];
          for (let i = 0; i <= 32; ++i) {
            const angle = (i / 32) * Math.PI * 2;
            ring.push(project(toWorld([0.042 * Math.cos(angle), 0.042 * Math.sin(angle), 0]),
                              width, height));
          }
          if (ring.every(Boolean)) {
            context.beginPath();
            context.moveTo(ring[0].x, ring[0].y);
            for (const item of ring.slice(1)) context.lineTo(item.x, item.y);
            context.closePath();
            context.fillStyle = 'rgba(255, 220, 90, 0.16)';
            context.fill();
            context.strokeStyle = 'rgba(255, 220, 90, 0.85)';
            context.lineWidth = 1.8;
            context.stroke();
          }
          const axes = [
            [toWorld([0.055, 0, 0]), '#ff6b6b'],
            [toWorld([0, 0.055, 0]), '#54d18c'],
            [toWorld([0, 0, 0.055]), '#6aa9ff'],
          ];
          context.lineWidth = 1.6;
          for (const [end, color] of axes) {
            const projected = project(end, width, height);
            if (!projected) continue;
            context.strokeStyle = color;
            context.beginPath(); context.moveTo(point.x, point.y); context.lineTo(projected.x, projected.y);
            context.stroke();
          }
          context.strokeStyle = '#ffdc5a';
          context.lineWidth = 1.6;
          context.beginPath(); context.moveTo(point.x - 6, point.y); context.lineTo(point.x + 6, point.y);
          context.moveTo(point.x, point.y - 6); context.lineTo(point.x, point.y + 6); context.stroke();
          context.fillStyle = '#ffe178';
          context.font = '11px system-ui';
          context.fillText('TCP target', point.x + 14, point.y - 10);
        }
      }

      const arm = host.armPoints ? host.armPoints() : [];
      if (arm.length > 0) {
        const tcp = project(arm[arm.length - 1], width, height);
        if (tcp) {
          context.fillStyle = '#ffb36b';
          context.beginPath(); context.arc(tcp.x, tcp.y, 6, 0, Math.PI * 2); context.fill();
        }
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
    if (updating || draggingTarget) return;
    if (previewTimer) clearTimeout(previewTimer);
    previewTimer = setTimeout(() => publishTarget(true), 120);
  };
  for (const input of [xInput, yInput, zInput, pitchInput, gripInput]) {
    input.addEventListener('input', () => {
      lastLocalEdit = performance.now();
      schedulePreview();
    });
  }
  cartesian.addEventListener('change', schedulePreview);

  const gripRow = document.createElement('div');
  gripRow.className = 'field';
  const gripLabel = document.createElement('label');
  gripLabel.textContent = 'grip';
  const gripSlider = document.createElement('input');
  gripSlider.type = 'range';
  gripSlider.min = '0';
  gripSlider.max = '27';
  gripSlider.step = '0.1';
  gripSlider.value = gripInput.value;
  gripRow.append(gripLabel, gripSlider);
  body.append(gripRow);
  gripSlider.addEventListener('input', () => {
    gripInput.value = gripSlider.value;
    schedulePreview();
  });
  gripInput.addEventListener('input', () => { gripSlider.value = gripInput.value; });

  // MoveIt 뷰에서 TCP target을 직접 드래그한다 (X/Y, Shift=Z, Ctrl=Pitch, Alt=Grip).
  let dragView = false;
  let dragMode = 'xy';
  let dragLast = { x: 0, y: 0 };
  const clamp = (value, min, max) => Math.max(min, Math.min(max, value));
  canvas.style.touchAction = 'none';
  canvas.addEventListener('contextmenu', (event) => event.preventDefault());
  canvas.addEventListener('pointerdown', (event) => {
    const rect = canvas.getBoundingClientRect();
    const localX = event.clientX - rect.left;
    const localY = event.clientY - rect.top;
    const hit = targetScreenPoint !== null &&
      Math.hypot(localX - targetScreenPoint.x, localY - targetScreenPoint.y) <= 20;
    dragView = true;
    dragMode = event.button === 2 || !hit ? 'orbit'
      : event.altKey ? 'grip'
      : event.shiftKey ? 'z'
      : event.ctrlKey || event.metaKey ? 'pitch' : 'xy';
    dragLast = { x: event.clientX, y: event.clientY };
    draggingTarget = dragMode !== 'orbit';
    canvas.setPointerCapture(event.pointerId);
    event.preventDefault();
  });
  canvas.addEventListener('pointermove', (event) => {
    if (!dragView) return;
    const dx = event.clientX - dragLast.x;
    const dy = event.clientY - dragLast.y;
    dragLast = { x: event.clientX, y: event.clientY };
    const step = 0.0015;
    if (dragMode === 'orbit') {
      camera.yaw -= dx * 0.008;
      camera.pitch = clamp(camera.pitch - dy * 0.008, 0.12, 1.25);
    } else if (dragMode === 'grip') {
      gripInput.value = clamp(Number(gripInput.value) - dy * 0.1, 0, 27).toFixed(1);
      gripSlider.value = gripInput.value;
      schedulePreview();
    } else if (dragMode === 'z') {
      zInput.value = clamp(Number(zInput.value) - dy * step, 0.02, 0.7).toFixed(4);
    } else if (dragMode === 'pitch') {
      pitchInput.value = clamp(Number(pitchInput.value) + dx * 0.25, -90, 90).toFixed(2);
    } else {
      xInput.value = clamp(Number(xInput.value) + dx * step, -0.15, 0.55).toFixed(4);
      yInput.value = clamp(Number(yInput.value) - dy * step, -0.35, 0.35).toFixed(4);
    }
    // 드래그 중에는 네트워크로 보내지 않는다. 마커는 로컬 입력값으로 즉시 움직이고
    // (draw()가 draggingTarget일 때 입력값을 그린다), 손을 뗄 때 한 번만 보낸다.
    lastLocalEdit = performance.now();
    event.preventDefault();
  });
  canvas.addEventListener('pointerup', (event) => {
    if (dragView && dragMode !== 'orbit') {
      lastLocalEdit = performance.now();
      publishTarget(true);
    }
    dragView = false;
    draggingTarget = false;
    event.preventDefault();
  });
  canvas.addEventListener('pointercancel', () => { dragView = false; draggingTarget = false; });
  canvas.addEventListener('wheel', (event) => {
    event.preventDefault();
    camera.distance = clamp(camera.distance * (event.deltaY < 0 ? 0.9 : 1.1), 0.35, 4.0);
  }, { passive: false });

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
