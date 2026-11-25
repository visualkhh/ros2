import { element } from './util.js';

const LEVELS = ['ok', 'warn', 'bad', 'stale'];

export function init(host) {
  const body = host.addPanel('Diagnostics');
  const list = element('div');
  body.append(list);

  host.onTopic('diagnostics', (message) => {
    const statuses = message.status || [];
    list.replaceChildren();
    if (statuses.length === 0) {
      list.append(element('div', 'muted', 'diagnostics 없음'));
      return;
    }
    for (const status of statuses) {
      const item = element('div', 'diag');
      const name = element('div', 'name', `${status.name || '-'} [${LEVELS[status.level] || status.level}]`);
      name.style.color = status.level >= 2 ? 'var(--bad)' : status.level === 1 ? 'var(--warn)' : 'var(--accent)';
      const message = element('div', 'msg', status.message || '-');
      item.append(name, message);
      if (status.values && status.values.length > 0) {
        const values = status.values.map((entry) => `${entry.key}=${entry.value}`).join(' · ');
        item.append(element('div', 'kv', values));
      }
      list.append(item);
    }
  });
}
