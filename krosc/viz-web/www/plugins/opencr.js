import { button, buttonRow, row, setLevel } from './util.js';

export function init(host) {
  const body = host.addPanel('OpenCR');
  const status = row(body, 'status');
  const battery = row(body, 'battery');
  const current = row(body, 'wheel current');
  const temperature = row(body, 'wheel temp');
  const uptime = row(body, 'uptime');
  const link = row(body, 'link');
  const lidar = row(body, 'LDS-03');

  // 부저: 비프음 (Hz, ms)
  const buzzerRow = buttonRow(body);
  const buzzerGrid = document.createElement('div');
  buzzerGrid.className = 'moveit-grid';
  buzzerRow.append(buzzerGrid);
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
    buzzerGrid.append(wrapper);
    return field;
  };
  const frequency = numberField('beep Hz', 2000, 100);
  const duration = numberField('beep ms', 150, 10);
  button(buzzerRow, 'Beep', () => {
    host.sendString('buzzer', `${frequency.value},${duration.value}`);
  }, 'primary');

  const asNumber = (values, key) => {
    const value = Number(values.get(key));
    return Number.isFinite(value) ? value : null;
  };
  const fixed = (value, digits) => (value === null ? '-' : value.toFixed(digits));

  host.onTopic('diagnostics', (message) => {
    const statuses = Array.isArray(message.status) ? message.status : [];
    for (const item of statuses) {
      const values = new Map((item.values || []).map((entry) => [entry.key, entry.value]));
      if (item.name === 'OpenCR') {
        setLevel(status, item.message || '-', item.level === 0 ? 'ok' : item.level === 1 ? 'warn' : 'bad');
        setLevel(battery, `${fixed(asNumber(values, 'battery'), 2)} V`,
                 (asNumber(values, 'battery') ?? 0) >= 10.5 ? 'ok' : 'warn');
        setLevel(current,
                 `L ${fixed(asNumber(values, 'left_current'), 2)} A · R ${fixed(asNumber(values, 'right_current'), 2)} A`);
        setLevel(temperature,
                 `L ${fixed(asNumber(values, 'left_temperature'), 0)}℃ · R ${fixed(asNumber(values, 'right_temperature'), 0)}℃`);
        const uptimeSeconds = (asNumber(values, 'uptime_ms') ?? 0) / 1000.0;
        const hours = Math.floor(uptimeSeconds / 3600);
        const minutes = Math.floor((uptimeSeconds % 3600) / 60);
        setLevel(uptime, `${hours}h ${String(minutes).padStart(2, '0')}m`);
        const lastError = values.get('last_error') || '-';
        setLevel(link,
                 `bad ${values.get('bad_lines') || '0'} · err ${values.get('errors') || '0'} · ` +
                 `age ${fixed(asNumber(values, 'status_age'), 2)}s` +
                 (lastError !== '-' ? ` · ${lastError}` : ''),
                 lastError !== '-' ? 'warn' : '');
      } else if (item.name === 'LDS-03') {
        setLevel(lidar,
                 `scans ${values.get('scans') || '-'} · bad ${values.get('bad_packets') || '0'} · ` +
                 `age ${fixed(asNumber(values, 'scan_age'), 2)}s`,
                 item.level === 0 ? 'ok' : 'warn');
      }
    }
  });
}
