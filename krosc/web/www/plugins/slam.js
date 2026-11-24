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

  const buttons = buttonRow(body);
  let slamMode = 'idle';
  button(buttons, 'start SLAM', () => {
    const target = slamMode === 'mapping' ? 'idle' : 'mapping';
    host.sendString('slam_mode', target);
    setLevel(mode, `slam_mode: ${target}`, 'warn');
  });
  button(buttons, 'load', () => {
    if (!mapSelect.value) { host.toast('map을 고른다'); return; }
    host.sendString('map_change', mapSelect.value);
  });
  button(buttons, 'unload', () => host.sendEmpty('map_unload'));
  button(buttons, 'save', () => {
    const name = window.prompt('map 이름', `map_${new Date().toISOString().slice(0, 16).replace(/[:T]/g, '')}`);
    if (name) host.sendString('map_save', name.trim());
  });
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
  });
  host.onTopic('map_status', (message) => setLevel(mapStatus, message.data || '-'));
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
