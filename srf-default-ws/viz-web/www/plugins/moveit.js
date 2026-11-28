import { button, buttonRow, parseFields, row, setLevel } from './util.js';

export function init(host) {
  // MoveIt 뷰가 구독하는 토픽을 Subscriptions 패널에 MoveIt 소유로 표시한다.
  // 이미 gateways가 구독 중이면 재구독하지 않는다.
  host.registerTopic({ name: 'moveit_target_pose', kind: 12, source: 'MoveIt' }).catch(() => {});
  host.registerTopic({ name: 'moveit_arm', kind: 9, source: 'MoveIt' }).catch(() => {});
  host.registerTopic({ name: 'moveit_path', kind: 9, source: 'MoveIt' }).catch(() => {});
  const view = host.addView({ id: 'moveit', title: 'MoveIt' });
  const canvas = document.createElement('canvas');
  canvas.className = 'moveit-canvas';
  view.panel.append(canvas);
  const context = canvas.getContext('2d');

  const camera = { yaw: -0.6, pitch: 0.45, distance: 1.05, tx: 0.15, ty: 0.0, tz: 0.18 };
  let targetScreenPoint = null;
  let draggingTarget = false;
  let targetHold = false;       // 드래그 후 서버 target 갱신 전까지 로컬 목표 유지
  let targetHoldKey = '';
  let lastLocalEdit = 0;

  const targetKey = (target) => {
    if (!target || !target.pose || !target.pose.position || !target.pose.orientation) return '';
    const p = target.pose.position;
    const q = target.pose.orientation;
    return [p.x, p.y, p.z, q.x, q.y, q.z, q.w].map((value) => Number(value).toFixed(5)).join(',');
  };

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

  // 로컬 입력값(위치/pitch)으로 목표 자세 행렬을 만든다.
  // 드래그 중에는 서버 응답을 기다리지 않고 이 값으로 원반을 즉시 회전시킨다.
  function localRotation(x, y, pitchDegrees) {
    const pitch = pitchDegrees * Math.PI / 180.0;
    const yaw = Math.atan2(y + 0.000230, x - 0.059985);
    const cy = Math.cos(yaw);
    const sy = Math.sin(yaw);
    const cp = Math.cos(pitch);
    const sp = Math.sin(pitch);
    // Rz(yaw) * Ry(-pitch): 서버의 pitch_down(+가 아래) 규약과 맞춘다.
    return [
      [cy * cp, -sy, -cy * sp],
      [sy * cp, cy, -sy * sp],
      [sp, 0, cp],
    ];
  }
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
    try {
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
      context.fillText('TCP 드래그 · Shift=Z · Ctrl+가로=Pitch · Ctrl+세로=Grip · Alt=Grip · 빈 곳 orbit',
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
        const armPrimitives = host.urdfPrimitives
          ? host.urdfPrimitives((name) => host.isArmLink(name), true) : [];
        if (armPrimitives.length > 0) {
          for (const primitive of armPrimitives) drawPrimitive(primitive, width, height);
        }
        const arm = host.armPoints ? host.armPoints() : [];
        const points = armPrimitives.length > 0 ? []
          : arm.map((point) => project(point, width, height)).filter(Boolean);
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
      // 드래그 중에는 네트워크 왕복을 기다리지 않고 입력값으로 즉시 그린다.
      // 드래그가 끝난 뒤에도 서버가 새 target을 발행할 때까지 로컬 값을 유지한다.
      // (preview가 늦거나 IK 실패일 때 옛 위치로 순간적으로 튀는 것을 막는다.)
      if (targetHold && targetKey(target) !== targetHoldKey) {
        targetHold = false;
      }
      // 현재 TCP는 하드코딩 체인이 아니라 URDF end_effector_link의 FK를 쓴다.
      // moveit_arm/preview와 같은 링크 기준이라 카메라 회전 중에도 마커가 붙어 있다.
      const linkFrames = host.urdfLinkFrames ? host.urdfLinkFrames() : [];
      const endFrame = linkFrames.find((frame) => frame.name === 'end_effector_link');
      const currentTcp = endFrame ? endFrame.point : null;
      let pose = target ? target.pose.position : null;
      let orientation = target ? target.pose.orientation : null;
      if (!pose && currentTcp) {
        // 아직 moveit_target_pose가 없으면 실제 TCP 위치에 마커를 그린다 (드래그 시작 가능)
        pose = currentTcp;
        orientation = { x: 0, y: 0, z: 0, w: 1 };
      }
      const local = draggingTarget || targetHold || performance.now() - lastLocalEdit < 250;
      if (local) {
        pose = { x: Number(xInput.value), y: Number(yInput.value), z: Number(zInput.value) };
      }
      if (pose && Number.isFinite(pose.x) && Number.isFinite(pose.y) && Number.isFinite(pose.z)) {
        const point = project(pose, width, height);
        if (point) {
          targetScreenPoint = point;
          // Qt Viz처럼 목표 자세 평면에 놓인 동그란 판(링) + 3축으로 그린다.
          // 드래그/입력 중에는 로컬 pitch로 즉시 회전한다 (응답 대기 없음).
          const r = local ? localRotation(pose.x, pose.y, Number(pitchInput.value) || 0)
                          : quatToMatrix(orientation);
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

          // 목표 그리퍼 크기: TCP 링 안의 작은 원이 grip(0~27mm)에 비례해 차오른다.
          // 0mm=작은 점, 27mm=바깥 링을 거의 꽉 채움. 방향/자세와 무관하게 크기만 보여준다.
          const gripMm = Math.max(0, Number(gripInput.value) || 0);
          const gripRatio = Math.min(1, gripMm / 27);
          const innerRadius = 0.004 + (0.042 - 0.004) * gripRatio;
          const inner = [];
          for (let i = 0; i <= 32; ++i) {
            const angle = (i / 32) * Math.PI * 2;
            inner.push(project(toWorld([innerRadius * Math.cos(angle), innerRadius * Math.sin(angle), 0]),
                               width, height));
          }
          if (inner.every(Boolean)) {
            context.beginPath();
            context.moveTo(inner[0].x, inner[0].y);
            for (const item of inner.slice(1)) context.lineTo(item.x, item.y);
            context.closePath();
            context.fillStyle = 'rgba(255, 179, 107, 0.32)';
            context.fill();
            context.strokeStyle = '#ffb36b';
            context.lineWidth = 1.5;
            context.stroke();
          }
          context.fillStyle = '#ffb36b';
          context.font = '10px system-ui';
          context.fillText(`grip ${gripMm.toFixed(1)} mm`, point.x + 14, point.y + 18);

          context.fillStyle = '#ffe178';
          context.font = '11px system-ui';
          context.fillText('TCP target', point.x + 14, point.y - 10);
        }
      }

      if (currentTcp) {
        const tcp = project(currentTcp, width, height);
        if (tcp) {
          context.fillStyle = '#ffb36b';
          context.beginPath(); context.arc(tcp.x, tcp.y, 6, 0, Math.PI * 2); context.fill();
        }
      }
    }
    } catch (error) {
      console.error('moveit draw', error);
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
  // 숫자 입력은 라이브 input 대신 클릭 → 확인 다이얼로그로 받는다 (costmap과 같은 패턴).
  const editDialog = document.createElement('dialog');
  editDialog.className = 'edit-dialog';
  const editForm = document.createElement('form');
  const editTitle = document.createElement('div');
  editTitle.className = 'edit-title';
  const editCurrent = document.createElement('div');
  editCurrent.className = 'edit-current';
  const editLabel = document.createElement('label');
  const editCaption = document.createElement('span');
  editCaption.textContent = '새 값';
  const editInput = document.createElement('input');
  editInput.type = 'number';
  editInput.step = 'any';
  editInput.required = true;
  editLabel.append(editCaption, editInput);
  const editButtons = document.createElement('div');
  editButtons.className = 'btn-row';
  editForm.append(editTitle, editCurrent, editLabel, editButtons);
  editDialog.append(editForm);
  body.append(editDialog);

  let editResolve = null;
  const finishEdit = (value) => {
    if (!editResolve) return;
    const resolve = editResolve;
    editResolve = null;
    if (editDialog.open) editDialog.close();
    resolve(value);
  };
  editForm.addEventListener('submit', (event) => {
    event.preventDefault();
    const value = Number(editInput.value);
    if (!Number.isFinite(value)) {
      editInput.reportValidity();
      return;
    }
    finishEdit(value);
  });
  editDialog.addEventListener('cancel', (event) => {
    event.preventDefault();
    finishEdit(null);
  });
  editDialog.addEventListener('close', () => finishEdit(null));
  const cancelEdit = document.createElement('button');
  cancelEdit.type = 'button';
  cancelEdit.textContent = '취소';
  cancelEdit.addEventListener('click', () => finishEdit(null));
  const applyEdit = document.createElement('button');
  applyEdit.type = 'button';
  applyEdit.className = 'primary';
  applyEdit.textContent = '확인';
  applyEdit.addEventListener('click', () => editForm.requestSubmit());
  editButtons.append(cancelEdit, applyEdit);

  const openNumberDialog = (caption, value, step) => new Promise((resolve) => {
    editResolve = resolve;
    editTitle.textContent = caption;
    editCurrent.textContent = `현재값: ${value}`;
    editCaption.textContent = '새 값';
    editInput.step = step > 0 ? String(step) : 'any';
    editInput.value = String(value);
    editDialog.showModal();
    editInput.focus();
    editInput.select();
  });

  const numeric = (label, value, step) => {
    const wrapper = document.createElement('div');
    wrapper.className = 'moveit-field';
    const name = document.createElement('label');
    name.textContent = label;
    const field = document.createElement('button');
    field.type = 'button';
    field.className = 'value-button';
    field.title = '클릭해서 수정';
    const input = document.createElement('input');
    input.type = 'number';
    input.step = String(step);
    input.hidden = true;
    let current = String(value);
    // 기존 코드는 input.value를 읽고 쓴다. 드래그/Current/상태 갱신도
    // 버튼 텍스트에 바로 반영되도록 value를 own accessor로 가린다.
    Object.defineProperty(input, 'value', {
      configurable: true,
      get() { return current; },
      set(next) {
        current = String(next);
        field.textContent = current;
      },
    });
    field.textContent = current;
    field.addEventListener('click', async () => {
      const next = await openNumberDialog(label, current, step);
      if (next === null) return;
      input.value = String(next);
      input.dispatchEvent(new Event('input', { bubbles: true }));
    });
    wrapper.append(name, field, input);
    grid.append(wrapper);
    return input;
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
  let currentGoal = null;
  const sendGoal = async (planOnly, preset = '') => {
    if (updating) return;
    const params = {
      x: Number(xInput.value),
      y: Number(yInput.value),
      z: Number(zInput.value),
      pitch: Number(pitchInput.value),
      grip: Number(gripInput.value),
      cartesian: cartesian.checked,
      plan_only: planOnly,
    };
    if (preset) params.preset = preset;
    try {
      const response = await host.rpc('moveit/send_goal', params);
      if (!response.accepted) {
        setLevel(status, `MoveIt: ${response.error || 'goal 거부'}`, 'warn');
        return;
      }
      currentGoal = response.goal_id;
      setLevel(status, planOnly ? 'Plan 요청...' : 'Execute 요청...', 'warn');
    } catch (error) {
      setLevel(status, `MoveIt 실패: ${error.message}`, 'bad');
    }
  };
  const schedulePreview = () => {
    if (updating || draggingTarget) return;
    if (previewTimer) clearTimeout(previewTimer);
    previewTimer = setTimeout(() => sendGoal(true), 180);
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
  const gripValue = document.createElement('div');
  gripValue.className = 'moveit-grip-value';
  const gripVisual = document.createElement('div');
  gripVisual.className = 'moveit-grip-visual';
  const jawLeft = document.createElement('span');
  jawLeft.className = 'jaw';
  const jawRight = document.createElement('span');
  jawRight.className = 'jaw';
  gripVisual.append(jawLeft, jawRight);
  gripRow.append(gripLabel, gripSlider, gripVisual, gripValue);
  body.append(gripRow);
  const updateGripUi = () => {
    const value = Number(gripInput.value);
    const valid = Number.isFinite(value) ? value : 0;
    gripValue.textContent = `${valid.toFixed(1)} mm`;
    gripSlider.value = gripInput.value;
    gripVisual.style.setProperty('--grip-gap', `${2 + Math.max(0, valid) * 0.8}px`);
  };
  gripSlider.addEventListener('input', () => {
    gripInput.value = gripSlider.value;
    updateGripUi();
    schedulePreview();
  });
  gripInput.addEventListener('input', () => {
    updateGripUi();
  });
  updateGripUi();

  const modeRow = buttonRow(body);
  const modeHint = document.createElement('div');
  modeHint.className = 'muted';
  modeHint.textContent = '드래그 모드: 원하는 항목을 고른 뒤 TCP 마커를 드래그';
  body.append(modeHint);
  const modeButtons = new Map();
  for (const [label, mode] of [['XY', 'xy'], ['Z', 'z'], ['Pitch', 'pitch'], ['Grip', 'grip']]) {
    const modeButton = button(modeRow, label, () => {
      dragOverride = mode;
      for (const [key, item] of modeButtons) item.className = key === mode ? 'primary' : '';
    });
    if (mode === 'xy') modeButton.className = 'primary';
    modeButtons.set(mode, modeButton);
  }

  // MoveIt 뷰에서 TCP target을 직접 드래그한다.
  // Ctrl+가로=Pitch, Ctrl+세로=Grip, Shift=Z, Alt=Grip.
  let dragView = false;
  let dragMode = 'xy';
  let dragOverride = 'xy';  // 모바일용: XY/Z/Pitch/Grip 모드 버튼
  let dragAxis = null;      // Ctrl 드래그에서 첫 이동 방향으로 축을 고정
  let dragLast = { x: 0, y: 0 };
  const clamp = (value, min, max) => Math.max(min, Math.min(max, value));
  canvas.style.touchAction = 'none';
  canvas.addEventListener('contextmenu', (event) => event.preventDefault());
  canvas.addEventListener('pointerdown', (event) => {
    const rect = canvas.getBoundingClientRect();
    const localX = event.clientX - rect.left;
    const localY = event.clientY - rect.top;
    const hit = targetScreenPoint !== null &&
      Math.hypot(localX - targetScreenPoint.x, localY - targetScreenPoint.y) <= 34;
    dragView = true;
    dragMode = event.button === 2 || !hit ? 'orbit'
      : event.altKey ? 'grip'
      : event.shiftKey ? 'z'
      : event.ctrlKey || event.metaKey ? 'pitch_grip' : dragOverride;
    dragAxis = null;
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
      updateGripUi();
      schedulePreview();
    } else if (dragMode === 'z') {
      zInput.value = clamp(Number(zInput.value) - dy * step, 0.02, 0.7).toFixed(4);
    } else if (dragMode === 'pitch') {
      pitchInput.value = clamp(Number(pitchInput.value) + dx * 0.25, -90, 90).toFixed(2);
    } else if (dragMode === 'pitch_grip') {
      // Ctrl 드래그: 처음 움직인 축으로 고정한다. 가로=Pitch, 세로=Grip.
      if (dragAxis === null && (Math.abs(dx) > 1 || Math.abs(dy) > 1)) {
        dragAxis = Math.abs(dx) >= Math.abs(dy) ? 'pitch' : 'grip';
      }
      if (dragAxis === 'pitch') {
        pitchInput.value = clamp(Number(pitchInput.value) + dx * 0.25, -90, 90).toFixed(2);
      } else if (dragAxis === 'grip') {
        gripInput.value = clamp(Number(gripInput.value) - dy * 0.1, 0, 27).toFixed(1);
        updateGripUi();
      }
      schedulePreview();
    } else {
      // 화면에서 끈 방향 그대로 TCP가 따라오도록 현재 카메라 yaw/pitch 기준으로
      // 월드 XY를 계산한다. 카메라를 돌린 뒤에도 '보고 있는 화면' 기준으로 움직인다.
      const cy = Math.cos(camera.yaw);
      const sy = Math.sin(camera.yaw);
      const cp = Math.cos(camera.pitch);
      const right = dx * step;
      const down = (dy / Math.max(0.2, cp)) * step;
      const moveX = cy * right + sy * down;
      const moveY = sy * right - cy * down;
      xInput.value = clamp(Number(xInput.value) + moveX, -0.15, 0.55).toFixed(4);
      yInput.value = clamp(Number(yInput.value) + moveY, -0.35, 0.35).toFixed(4);
    }
    // 드래그 중에는 네트워크로 보내지 않는다. 마커는 로컬 입력값으로 즉시 움직이고
    // (draw()가 draggingTarget일 때 입력값을 그린다), 손을 뗄 때 한 번만 보낸다.
    // orbit 중에는 local edit로 취급하면 안 된다. 그러면 회전 중 TCP target이
    // 입력 필드 값으로 250ms 동안 튀었다가 제자리로 돌아온다.
    if (dragMode !== 'orbit') {
      lastLocalEdit = performance.now();
    }
    event.preventDefault();
  });
  canvas.addEventListener('pointerup', (event) => {
    if (dragView && dragMode !== 'orbit') {
      lastLocalEdit = performance.now();
      targetHold = true;
      targetHoldKey = targetKey(host.latest('moveit_target_pose'));
      schedulePreview();
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
  const preset = async (mode) => {
    host.setArmTorque(true);
    try {
      await host.rpc('controller/switch', {
        start: ['joint_trajectory_controller'],
        stop: ['arm_command_controller'],
        strictness: 1,
      });
    } catch (error) {
      setLevel(status, `controller 전환 실패: ${error.message}`, 'bad');
      return;
    }
    sendGoal(false, mode);
  };
  button(presetRow, 'HOME', () => preset('home'));
  button(presetRow, 'STOW', () => preset('stow'), 'primary');

  const actionRow = buttonRow(body);
  button(actionRow, 'Current', () => {
    targetHold = false;
    lastLocalEdit = 0;
    const values = parseFields(host.latest('moveit_state')?.data);
    const tcp = (values.get('tcp') || '0,0,0').split(',').map(Number);
    updating = true;
    xInput.value = String(tcp[0] || 0);
    yInput.value = String(tcp[1] || 0);
    zInput.value = String(tcp[2] || 0);
    pitchInput.value = String(Number(values.get('pitch')) || 0);
    gripInput.value = String(Number(values.get('grip')) || 0);
    updating = false;
    schedulePreview();
  });
  button(actionRow, 'Plan', () => sendGoal(true));
  button(actionRow, 'Execute', () => sendGoal(false), 'primary');
  button(actionRow, 'Cancel', async () => {
    try {
      const response = await host.rpc('moveit/cancel', { goal_id: currentGoal || '' });
      setLevel(status, response.ok ? 'Cancel 요청...' : 'Cancel할 goal이 없다', response.ok ? 'warn' : 'bad');
    } catch (error) {
      setLevel(status, `Cancel 실패: ${error.message}`, 'bad');
    }
  }, 'danger');

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
  let scrubTimer = null;
  scrub.addEventListener('input', () => {
    if (scrubTimer) clearTimeout(scrubTimer);
    scrubTimer = setTimeout(() => {
      host.rpc('moveit/scrub', { index: Number(scrub.value) }).catch((error) => {
        setLevel(status, `Preview 실패: ${error.message}`, 'bad');
      });
    }, 60);
  });

  host.onEvent('moveit/feedback', (params) => {
    setLevel(status, `MoveIt: ${params.state || '-'} ${Math.round((params.progress || 0) * 100)}%`, 'warn');
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
        if (Number.isFinite(Number(grip))) {
          gripInput.value = String(grip);
          gripSlider.value = gripInput.value;
          updateGripUi();
        }
        updating = false;
        targetInitialized = true;
      }
    }
  });
}
