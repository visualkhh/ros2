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
  const cancelButton = button(poseRow, '취소 / 정지', async () => {
    host.clearWaypoints();
    await host.cancelNavigation();
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
  const clearButton = button(multiRow, '비우기', async () => {
    host.clearWaypoints();
    await host.cancelNavigation();
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
      host.setGoalMode(mode);
      host.setGoalArmed(true);
    } else if (mode === 'waypoint') {
      host.setMultiGoal(true);
    }
    updateUi();
  }

  async function sendMulti(preview) {
    const count = host.waypointCount();
    if (count === 0) {
      setLevel(status, '웨이포인트가 없다 — [웨이포인트 추가] 후 맵을 클릭', 'warn');
      return;
    }
    if (slamMode !== 'localization') {
      setLevel(status, '지도를 먼저 load 하세요 (localization)', 'warn');
      return;
    }
    try {
      await host.sendWaypoints(preview);
      setLevel(status, preview ? `미리보기 ${count}개 (주행 안 함)` : `이동 시작: ${count}개`,
               preview ? 'warn' : 'ok');
    } catch (error) {
      setLevel(status, `navigation: ${error.message}`, 'bad');
    }
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
  // 값 입력은 라이브 input 대신 클릭 → 확인 다이얼로그로 받는다.
  // 주행/파라미터 갱신 중에 입력 중인 값이 서버 값으로 덮어써지지 않게 한다.
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
    editInput.step = 'any';
    editInput.value = String(value);
    editDialog.showModal();
    editInput.focus();
    editInput.select();
  });

  const numberField = (label, value, step) => {
    const wrapper = document.createElement('div');
    wrapper.className = 'moveit-field';
    const name = document.createElement('label');
    name.textContent = label;
    const field = document.createElement('button');
    field.type = 'button';
    field.className = 'value-button';
    field.title = '클릭해서 수정';
    let current = Number(value);
    let edited = false;
    const render = () => {
      field.textContent = String(current);
    };
    render();
    field.addEventListener('click', async () => {
      const next = await openNumberDialog(label, current, step);
      if (next === null) return;
      current = next;
      edited = true;
      render();
    });
    wrapper.append(name, field);
    costmapGrid.append(wrapper);
    return {
      element: field,
      get value() { return current; },
      set value(next) {
        if (edited) return;  // 사용자가 수정한 값은 서버 응답으로 덮지 않는다
        const parsed = Number(next);
        if (!Number.isFinite(parsed)) return;
        current = parsed;
        render();
      },
    };
  };
  const globalInflation = numberField('global infl (m)', 0.55, 0.05);
  const globalScaling = numberField('global scale', 3.0, 0.5);
  const localInflation = numberField('local infl (m)', 0.55, 0.05);
  const localScaling = numberField('local scale', 3.0, 0.5);
  const localWidth = numberField('local width (m)', 4.0, 0.5);
  const localHeight = numberField('local height (m)', 4.0, 0.5);
  const costmapState = document.createElement('div');
  costmapState.className = 'muted';
  const costmapRow = buttonRow(body);

  async function applyCostmap() {
    try {
      const global = await host.setParameters('global_costmap', {
        inflation_radius: Number(globalInflation.value),
        cost_scaling_factor: Number(globalScaling.value),
      });
      const local = await host.setParameters('local_costmap', {
        inflation_radius: Number(localInflation.value),
        cost_scaling_factor: Number(localScaling.value),
        width: Number(localWidth.value),
        height: Number(localHeight.value),
      });
      const results = [...(global.results || []), ...(local.results || [])];
      const failed = results.find((item) => !item.successful);
      if (failed) {
        setLevel(status, `costmap 실패: ${failed.reason}`, 'bad');
        return;
      }
      setLevel(status, 'costmap applied', 'ok');
      costmapState.textContent =
        `g ${globalInflation.value} m / ${globalScaling.value} · ` +
        `l ${localInflation.value} m / ${localScaling.value} / ` +
        `${localWidth.value}x${localHeight.value} m`;
    } catch (error) {
      setLevel(status, `costmap 실패: ${error.message}`, 'bad');
    }
  }

  button(costmapRow, 'Apply costmap', applyCostmap, 'primary');
  body.append(costmapState);

  (async () => {
    try {
      const global = await host.getParameters('global_costmap',
        ['inflation_radius', 'cost_scaling_factor']);
      const local = await host.getParameters('local_costmap',
        ['inflation_radius', 'cost_scaling_factor', 'width', 'height']);
      const assign = (field, values, key) => {
        if (values?.[key] !== undefined) field.value = values[key];
      };
      assign(globalInflation, global.values, 'inflation_radius');
      assign(globalScaling, global.values, 'cost_scaling_factor');
      assign(localInflation, local.values, 'inflation_radius');
      assign(localScaling, local.values, 'cost_scaling_factor');
      assign(localWidth, local.values, 'width');
      assign(localHeight, local.values, 'height');
    } catch (error) {
      setLevel(status, `costmap 값 읽기 실패: ${error.message}`, 'warn');
    }
  })();


  updateUi();
}
