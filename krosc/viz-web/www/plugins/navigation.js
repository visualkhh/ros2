import { button, buttonRow, row, setLevel } from './util.js';

export function init(host) {
  const body = host.addPanel('Navigation');
  const status = row(body, 'goal');
  const hint = document.createElement('div');
  hint.className = 'muted';
  body.append(hint);

  let slamMode = 'idle';
  let mode = '';  // '', 'pose', 'preview', 'move', 'waypoint'

  // ---- 위치 ----
  const poseTitle = document.createElement('div');
  poseTitle.className = 'muted';
  poseTitle.textContent = '위치 (Localization)';
  body.append(poseTitle);
  const poseRow = buttonRow(body);
  const poseButton = button(poseRow, '2D Pose Estimate', () => toggle('pose'));
  const cancelButton = button(poseRow, '취소 / 정지', () => {
    host.clearWaypoints();
    host.sendBool('cancel_goal', true);
    setLevel(status, 'cancel requested', 'warn');
  }, 'danger');

  // ---- 단일 목표 ----
  const singleTitle = document.createElement('div');
  singleTitle.className = 'muted';
  singleTitle.textContent = '단일 목표 (맵 클릭)';
  body.append(singleTitle);
  const singleRow = buttonRow(body);
  const previewButton = button(singleRow, '경로 미리보기', () => toggle('preview'));
  const moveButton = button(singleRow, '이동', () => toggle('move'), 'primary');

  // ---- 멀티 목표 ----
  const multiTitle = document.createElement('div');
  multiTitle.className = 'muted';
  multiTitle.textContent = '멀티 목표 (웨이포인트)';
  body.append(multiTitle);
  const multiRow = buttonRow(body);
  const waypointButton = button(multiRow, '웨이포인트 추가', () => toggle('waypoint'));
  const multiPreview = button(multiRow, '경로 미리보기', () => sendMulti(true));
  const multiGo = button(multiRow, '이동 시작', () => sendMulti(false), 'primary');
  const clearButton = button(multiRow, '비우기', () => {
    host.clearWaypoints();
    host.sendBool('cancel_goal', true);
    setLevel(status, 'waypoints cleared', 'warn');
    updateUi();
  });
  const countLabel = document.createElement('div');
  countLabel.className = 'muted';
  body.append(countLabel);

  function disarm() {
    host.setPoseArmed(false);
    host.setGoalArmed(false);
    host.setMultiGoal(false);
  }

  function toggle(next) {
    if (next === 'pose' && slamMode === 'idle') {
      setLevel(status, '지도를 먼저 load 하세요 (localization) — idle에서는 2D Pose가 적용되지 않습니다', 'bad');
      host.toast('SLAM/Map 탭에서 지도를 load 한 뒤 2D Pose를 쓰세요');
      mode = '';
      disarm();
      updateUi();
      return;
    }
    const same = mode === next;
    disarm();
    mode = same ? '' : next;
    if (mode === 'pose') {
      host.setPoseArmed(true);
    } else if (mode === 'preview' || mode === 'move') {
      host.setGoalTopic(mode === 'preview' ? 'preview_goal' : 'goal_pose');
      host.setGoalArmed(true);
    } else if (mode === 'waypoint') {
      host.setMultiGoal(true);
    }
    updateUi();
  }

  function sendMulti(preview) {
    const count = host.waypointCount();
    if (count === 0) {
      setLevel(status, '웨이포인트가 없다 — [웨이포인트 추가] 후 맵을 클릭', 'warn');
      return;
    }
    if (slamMode !== 'localization') {
      setLevel(status, '지도를 먼저 load 하세요 (localization)', 'warn');
      return;
    }
    host.sendWaypoints(preview);
    setLevel(status, preview ? `미리보기 ${count}개 (주행 안 함)` : `이동 시작: ${count}개`,
             preview ? 'warn' : 'ok');
  }

  function updateUi() {
    poseButton.className = mode === 'pose' ? 'primary' : '';
    previewButton.className = mode === 'preview' ? 'primary' : '';
    moveButton.className = mode === 'move' ? 'primary' : '';
    waypointButton.className = mode === 'waypoint' ? 'primary' : '';
    const count = host.waypointCount();
    multiPreview.disabled = count === 0;
    multiGo.disabled = count === 0;
    clearButton.disabled = count === 0;

    if (mode === 'pose') {
      hint.textContent = '맵을 누른 채 드래그 = 로봇 위치 + 방향 (1회)';
    } else if (mode === 'preview') {
      hint.textContent = '맵을 클릭 = 경로만 계산해 표시 (로봇 안 움직임)';
    } else if (mode === 'move') {
      hint.textContent = '맵을 클릭 = 그 목표로 이동 시작';
    } else if (mode === 'waypoint') {
      hint.textContent = '맵 클릭 = 웨이포인트 추가 · 드래그 = 방향 · Backspace = 마지막 취소';
    } else {
      hint.textContent = '모드를 고르세요: 위치 / 단일(미리보기·이동) / 멀티(웨이포인트)';
    }
    countLabel.textContent = `웨이포인트: ${count}개` +
      (slamMode === 'localization' ? '' : ` · SLAM: ${slamMode}`);
  }

  host.onGoalSent(() => {
    mode = '';
    disarm();
    updateUi();
  });
  host.onPoseSent(() => {
    mode = '';
    disarm();
    updateUi();
  });

  setInterval(updateUi, 300);

  host.onTopic('slam_status', (message) => {
    slamMode = String(message.data || 'idle');
    updateUi();
  });
  host.onTopic('goal_status', (message) => {
    const text = message.data || '-';
    const level = /fail|error|abort|cancel/i.test(text) ? 'bad'
      : /succeed|reached|done|preview/i.test(text) ? 'ok' : 'warn';
    setLevel(status, text, level);
  });

  // ---- costmap ----
  const costmapTitle = document.createElement('div');
  costmapTitle.className = 'muted';
  costmapTitle.textContent = 'COSTMAP (inflation / scaling)';
  body.append(costmapTitle);
  const costmapGrid = document.createElement('div');
  costmapGrid.className = 'moveit-grid';
  body.append(costmapGrid);
  const numberField = (label, value, step) => {
    const wrapper = document.createElement('div');
    wrapper.className = 'moveit-field';
    const name = document.createElement('label');
    name.textContent = label;
    const field = document.createElement('input');
    field.type = 'number';
    field.step = String(step);
    field.value = String(value);
    wrapper.append(name, field);
    costmapGrid.append(wrapper);
    return field;
  };
  const globalInflation = numberField('global infl (m)', 0.55, 0.05);
  const globalScaling = numberField('global scale', 3.0, 0.5);
  const localInflation = numberField('local infl (m)', 0.55, 0.05);
  const localScaling = numberField('local scale', 3.0, 0.5);
  const costmapRow = buttonRow(body);
  button(costmapRow, 'Apply costmap', () => {
    host.sendString('costmap_params',
      `global,inflation_radius,${globalInflation.value};` +
      `global,cost_scaling_factor,${globalScaling.value};` +
      `local,inflation_radius,${localInflation.value};` +
      `local,cost_scaling_factor,${localScaling.value}`);
    setLevel(status, 'costmap applied', 'ok');
  }, 'primary');
  const costmapState = document.createElement('div');
  costmapState.className = 'muted';
  body.append(costmapState);

  host.onTopic('costmap_params_state', (message) => {
    const values = new Map();
    for (const item of String(message.data || '').split(';')) {
      const equal = item.indexOf('=');
      if (equal < 0) continue;
      values.set(item.slice(0, equal), item.slice(equal + 1));
    }
    const assign = (field, key) => {
      const value = values.get(key);
      if (value !== undefined && document.activeElement !== field) field.value = value;
    };
    assign(globalInflation, 'global.inflation_radius');
    assign(globalScaling, 'global.cost_scaling_factor');
    assign(localInflation, 'local.inflation_radius');
    assign(localScaling, 'local.cost_scaling_factor');
    costmapState.textContent =
      `g ${values.get('global.inflation_radius') || '-'} m / ${values.get('global.cost_scaling_factor') || '-'} · ` +
      `l ${values.get('local.inflation_radius') || '-'} m / ${values.get('local.cost_scaling_factor') || '-'}`;
  });

  updateUi();
}
