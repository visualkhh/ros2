import { button, buttonRow, row, select, setLevel } from './util.js';

export function init(host) {
  const body = host.addPanel('SLAM / Map');
  let lastLoadedToast = '';
  const mode = row(body, 'SLAM');
  const loadedName = row(body, 'loaded map');
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
  const slamButton = button(buttons, 'start SLAM', async () => {
    const target = slamMode === 'mapping' ? 'idle' : 'mapping';
    try {
      await host.rpc('map/slam_mode', { mode: target });
      setLevel(mode, `slam_mode: ${target}`, 'warn');
    } catch (error) {
      setLevel(mode, `slam_mode 실패: ${error.message}`, 'bad');
    }
  });
  function updateSlamButton() {
    const running = slamMode === 'mapping';
    slamButton.textContent = running ? 'stop SLAM' : 'start SLAM';
    slamButton.className = running ? 'danger' : '';
  }
  button(buttons, 'load', async () => {
    if (!mapSelect.value) { host.toast('map을 고른다'); return; }
    try {
      const response = await host.rpc('map/load', { name: mapSelect.value });
      if (response.result !== 0) {
        setLevel(mapStatus, `load 실패: ${mapSelect.value} (${response.result})`, 'bad');
      }
    } catch (error) {
      setLevel(mapStatus, `load 실패: ${error.message}`, 'bad');
    }
  });
  button(buttons, 'unload', async () => {
    try {
      await host.rpc('map/unload', {});
    } catch (error) {
      setLevel(mapStatus, `unload 실패: ${error.message}`, 'bad');
    }
  });
  // 저장/삭제는 로봇 링크가 잠깐 끊겨도 유실되지 않게 몇 번 재시도한다.
  async function sendPending(kind, name) {
    const method = kind === 'save' ? 'map/save' : 'map/delete';
    setLevel(mapStatus, kind === 'save' ? `saving ${name} ...` : `deleting ${name} ...`, 'warn');
    for (let tries = 1; tries <= 3; tries += 1) {
      try {
        const response = await host.rpc(method, { name });
        if (response.result) {
          return;
        }
      } catch (error) {
        if (tries === 3) {
          setLevel(mapStatus, `${kind} 실패: ${name} (${error.message})`, 'bad');
          return;
        }
      }
      if (tries < 3) await new Promise((resolve) => setTimeout(resolve, 1000));
    }
    setLevel(mapStatus, `${kind} 실패: ${name}`, 'bad');
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
  const collisionButton = button(buttons, 'collision ON', async () => {
    const next = !collisionEnabled;
    try {
      const response = await host.rpc('params/set', {
        node: 'collision_monitor',
        parameters: { enabled: next },
      });
      if (response.results?.every((item) => item.successful)) {
        collisionEnabled = next;
        updateCollisionButton();
      } else {
        setLevel(collision, response.results?.[0]?.reason || 'collision 설정 실패', 'bad');
      }
    } catch (error) {
      setLevel(collision, `collision 실패: ${error.message}`, 'bad');
    }
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
  host.onTopic('map_name', (message) => {
    const name = String(message.data || '');
    setLevel(loadedName, name || '(없음)', name ? 'ok' : '');
    if (name) {
      mapSelect.value = name;
    } else {
      mapSelect.value = '';
    }
  });
  host.onTopic('map_status', (message) => {
    const text = message.data || '-';
    setLevel(mapStatus, text);
    // 지도 load 직후에는 지도 전체가 보이게 맞춘다 (새로고침 없이 바로 보이도록)
    if (text.startsWith('loaded ')) {
      if (host.fitMap) host.fitMap();
      // 래치 메시지가 재전송될 때마다 토스트가 반복되지 않게 한 번만
      if (lastLoadedToast !== text) {
        lastLoadedToast = text;
      }
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
