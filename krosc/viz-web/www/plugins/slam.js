import { button, buttonRow, row, select, setLevel } from './util.js';

export function init(host) {
  const body = host.addPanel('SLAM / Map');
  const mode = row(body, 'SLAM');
  const mapStatus = row(body, 'map');
  const collision = row(body, 'collision');

  const mapRow = document.createElement('div');
  mapRow.className = 'field';
  const mapLabel = document.createElement('label');
  mapLabel.textContent = 'map';
  const mapSelect = document.createElement('select');
  mapRow.append(mapLabel, mapSelect);
  body.append(mapRow);
  mapSelect.addEventListener('change', () => {
    if (mapSelect.value) {
      host.toast(`map 선택: ${mapSelect.value} — load를 눌러 적용`);
    }
  });

  const buttons = buttonRow(body);
  let slamMode = 'idle';
  const slamButton = button(buttons, 'start SLAM', () => {
    const target = slamMode === 'mapping' ? 'idle' : 'mapping';
    slamMode = target;
    updateSlamButton();
    host.sendString('slam_mode', target);
    setLevel(mode, `slam_mode: ${target}`, 'warn');
  });
  function updateSlamButton() {
    const running = slamMode === 'mapping';
    slamButton.textContent = running ? 'stop SLAM' : 'start SLAM';
    slamButton.className = running ? 'danger' : '';
  }
  button(buttons, 'load', () => {
    if (!mapSelect.value) { host.toast('map을 고른다'); return; }
    host.sendString('map_change', mapSelect.value);
  });
  button(buttons, 'unload', () => host.sendEmpty('map_unload'));
  // 저장/삭제는 로봇 링크가 잠깐 끊겨도 유실되지 않게 몇 번 재시도한다.
  let pending = null;  // {kind:'save'|'delete', name, tries, timer}
  function sendPending(kind, name) {
    if (pending) clearTimeout(pending.timer);
    pending = { kind, name, tries: 1, timer: null };
    host.sendString(kind === 'save' ? 'map_save' : 'map_delete', name);
    setLevel(mapStatus, kind === 'save' ? `saving ${name} ...` : `deleting ${name} ...`, 'warn');
    pending.timer = setTimeout(() => retryPending(), 4000);
  }
  function retryPending() {
    if (!pending) return;
    if (pending.tries >= 3) {
      setLevel(mapStatus, `${pending.kind} 실패: ${pending.name} (로봇 연결 확인)`, 'bad');
      pending = null;
      return;
    }
    pending.tries += 1;
    host.sendString(pending.kind === 'save' ? 'map_save' : 'map_delete', pending.name);
    pending.timer = setTimeout(() => retryPending(), 4000);
  }
  button(buttons, 'save', () => {
    const name = window.prompt('map 이름', `map_${new Date().toISOString().slice(0, 16).replace(/[:T]/g, '')}`);
    if (name) sendPending('save', name.trim());
  });
  button(buttons, 'delete', () => {
    const name = mapSelect.value;
    if (!name) {
      host.toast('삭제할 map을 먼저 고른다');
      return;
    }
    if (!window.confirm(`map "${name}" 을(를) 삭제할까?`)) return;
    sendPending('delete', name);
  }, 'danger');
  let collisionEnabled = true;
  const collisionButton = button(buttons, 'collision ON', () => {
    collisionEnabled = !collisionEnabled;
    host.sendBool('collision_set_enabled', collisionEnabled);
    updateCollisionButton();
  }, 'primary');
  function updateCollisionButton() {
    collisionButton.textContent = collisionEnabled ? 'collision ON' : 'collision OFF';
    collisionButton.className = collisionEnabled ? 'primary' : 'danger';
  }

  host.onTopic('slam_status', (message) => {
    slamMode = message.data || 'idle';
    setLevel(mode, slamMode, slamMode === 'mapping' ? 'ok' : '');
    updateSlamButton();  // 이미 매핑 중이면 버튼이 'stop SLAM'으로 보여야 한다
  });
  host.onTopic('map_status', (message) => {
    const text = message.data || '-';
    setLevel(mapStatus, text);
    if (pending && (text === `saved ${pending.name}` || text === `deleted ${pending.name}` ||
                    text.startsWith('save failed') || text.startsWith('delete failed'))) {
      clearTimeout(pending.timer);
      pending = null;
    }
    // 지도 load 직후에는 지도 전체가 보이게 맞춘다 (새로고침 없이 바로 보이도록)
    if (text.startsWith('loaded ')) {
      if (host.fitMap) host.fitMap();
      host.toast(`${text} — localization 시작 (필요하면 2D Pose로 보정)`);
    }
  });
  host.onTopic('map_list', (message) => {
    const current = mapSelect.value;
    const names = String(message.data || '').split('\n').filter(Boolean);
    mapSelect.replaceChildren();
    const empty = document.createElement('option');
    empty.value = '';
    empty.textContent = '(select map)';
    mapSelect.append(empty);
    for (const name of names) {
      const option = document.createElement('option');
      option.value = name;
      option.textContent = name;
      mapSelect.append(option);
    }
    if (names.includes(current)) mapSelect.value = current;
  });
  host.onTopic('collision_status', (message) => {
    const match = String(message.data || '').match(/enabled=([01])/);
    if (match) {
      collisionEnabled = match[1] === '1';
      updateCollisionButton();
      setLevel(collision, collisionEnabled ? 'ON' : 'OFF', collisionEnabled ? 'ok' : 'warn');
    } else {
      setLevel(collision, message.data || '-');
    }
  });
  updateCollisionButton();
}
