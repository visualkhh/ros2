import { element } from './util.js';

const LEVELS = ['ok', 'warn', 'bad', 'stale'];

export function init(host) {
  const body = host.addPanel('Diagnostics');
  const list = element('div');
  body.append(list);

  // RobotStatus(OpenCR/LDS)와 RL controller가 각자 DiagnosticArray를 발행한다.
  // 마지막 메시지로 목록을 통째로 교체하면 서로 덮어써서 깜빡이므로,
  // status.name별 최신 값을 합쳐서 보여준다.
  const statuses = new Map();

  function render() {
    list.replaceChildren();
    if (statuses.size === 0) {
      list.append(element('div', 'muted', 'diagnostics 없음'));
      return;
    }
    const ordered = [...statuses.entries()].sort(([a], [b]) => a.localeCompare(b));
    for (const [name, status] of ordered) {
      const item = element('div', 'diag');
      const title = element(
          'div', 'name',
          `${status.name || name} [${LEVELS[status.level] || status.level}]`);
      title.style.color =
          status.level >= 2 ? 'var(--bad)'
          : status.level === 1 ? 'var(--warn)'
          : 'var(--accent)';
      const message = element('div', 'msg', status.message || '-');
      item.append(title, message);
      if (status.values && status.values.length > 0) {
        const values = status.values.map((entry) => `${entry.key}=${entry.value}`).join(' · ');
        item.append(element('div', 'kv', values));
      }
      list.append(item);
    }
  }

  host.onTopic('diagnostics', (message) => {
    for (const status of (message.status || [])) {
      statuses.set(status.name || '-', status);
    }
    render();
  });

  render();
}
