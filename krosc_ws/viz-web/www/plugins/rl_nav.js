import { button, buttonRow, parseFields, row, select, setLevel } from './util.js';

export function init(host) {
  const body = host.addPanel('RL Nav');
  const status = row(body, 'state');
  const provider = row(body, 'provider');
  const runtime = row(body, 'onnx');
  const detail = row(body, 'policy');
  const fallbackRow = document.createElement('div');
  fallbackRow.className = 'field';
  const fallbackLabel = document.createElement('label');
  fallbackLabel.textContent = 'fallback';
  fallbackRow.append(fallbackLabel);
  const fallback = select(fallbackRow, ['rpp', 'mppi', 'dwb']);
  body.append(fallbackRow);

  const modelRow = document.createElement('div');
  modelRow.className = 'field';
  const modelLabel = document.createElement('label');
  modelLabel.textContent = 'model';
  const model = document.createElement('select');
  modelRow.append(modelLabel, model);
  body.append(modelRow);

  const buttons = buttonRow(body);
  let enabled = false;
  const toggle = button(buttons, 'RL OFF', () => {
    enabled = !enabled;
    host.sendBool('rl_nav_enable', enabled);
    updateToggle();
  }, 'danger');
  button(buttons, 'Reload', () => host.sendEmpty('rl_nav_reload'));

  function updateToggle() {
    toggle.textContent = enabled ? 'RL ON' : 'RL OFF';
    toggle.className = enabled ? 'primary' : 'danger';
  }

  let requestedFallback = null;
  let requestedModel = null;
  fallback.addEventListener('change', () => {
    requestedFallback = fallback.value;
    host.sendString('rl_nav_fallback', fallback.value);
    host.toast(`fallback → ${fallback.value} 요청`);
  });
  model.addEventListener('change', () => {
    if (!model.value) return;
    requestedModel = model.value;
    host.sendString('rl_nav_model', model.value);
    host.toast(`model → ${model.value} 요청`);
  });

  host.onTopic('rl_nav_model_list', (message) => {
    const current = model.value;
    const names = String(message.data || '').split('\n').filter(Boolean);
    model.replaceChildren();
    for (const name of names) {
      const option = document.createElement('option');
      option.value = name;
      option.textContent = name;
      model.append(option);
    }
    if (names.includes(current)) model.value = current;
  });

  host.onTopic('rl_nav_status', (message) => {
    const values = parseFields(message.data);
    enabled = values.get('enable') === '1';
    updateToggle();
    const active = values.get('active') === '1';
    const reason = values.get('reason') || '?';
    const stateText = active ? 'ACTIVE' : enabled ? 'READY' : 'OFF';
    setLevel(status, `${stateText} · reason ${reason}`, active ? 'ok' : enabled ? 'warn' : '');
    const providerName = values.get('provider') || '-';
    setLevel(provider, providerName, providerName === '-' ? 'warn' : 'ok');
    const ort = values.get('onnxruntime') === '1';
    setLevel(runtime, `${ort ? 'runtime' : 'none'} ${values.get('ort_version') || ''}`.trim(),
             ort ? 'ok' : 'warn');
    setLevel(detail,
             `policy ${values.get('policy') || '0'} · infer ${values.get('infer_ms') || '-'}ms · ` +
             `scan ${values.get('scan') || '0'} · fallback ${values.get('fallback') || '-'}`,
             values.get('error') ? 'bad' : '');
    const currentFallback = values.get('fallback');
    if (currentFallback) fallback.value = currentFallback;
    if (requestedFallback && currentFallback === requestedFallback) {
      host.toast(`fallback 적용됨: ${currentFallback}`);
      requestedFallback = null;
    }
    const currentModel = values.get('model');
    if (currentModel && currentModel !== '-') model.value = currentModel;
    if (requestedModel && currentModel === requestedModel) {
      host.toast(`model 적용됨: ${currentModel}`);
      requestedModel = null;
    }
    if (values.get('error')) detail.title = values.get('error');
  });
}
