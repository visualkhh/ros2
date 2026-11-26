import { button, buttonRow, row, setLevel } from './util.js';

// 평문 명령 -> llm_input 노드(Ollama) -> llm/status.
// 브라우저는 krosc service를 직접 못 부르므로 viz-web RPC('llm/command')를 거친다.
export function init(host) {
  const body = host.addPanel('AI');
  const status = row(body, 'llm');
  const plan = row(body, 'plan');
  plan.classList.add('nowrap');

  const inputRow = document.createElement('div');
  inputRow.style.display = 'block';
  inputRow.style.margin = '6px 0 4px';
  const input = document.createElement('textarea');
  input.rows = 3;
  input.placeholder = '예: 앞으로 1m 전진해줘';
  input.autocomplete = 'off';
  input.style.display = 'block';
  input.style.width = '100%';
  input.style.minHeight = '3.6em';
  input.style.maxHeight = '12em';
  input.style.resize = 'vertical';
  input.style.boxSizing = 'border-box';
  input.style.padding = '6px 8px';
  input.style.font = 'inherit';
  input.style.lineHeight = '1.35';
  input.style.background = '#1a2230';
  input.style.color = 'var(--text, #d7e0ec)';
  input.style.border = '1px solid #2a3850';
  input.style.borderRadius = '5px';
  inputRow.append(input);
  body.append(inputRow);
  const hint = document.createElement('div');
  hint.textContent = 'Enter=줄바꿈 · ⌘/Ctrl+Enter=전송';
  hint.style.fontSize = '11px';
  hint.style.opacity = '0.6';
  hint.style.margin = '0 0 4px';
  body.append(hint);

  const actions = buttonRow(body);
  const send = async () => {
    const text = input.value.trim();
    if (!text) return;
    setLevel(status, '요청 전송…', 'warn');
    try {
      const response = await host.rpc('llm/command', { text });
      const result = response && response.raw ? JSON.parse(response.raw) : response;
      if (!result || result.accepted !== true) {
        setLevel(status, `거부: ${(result && result.error) || '알 수 없음'}`, 'bad');
        return;
      }
      setLevel(status, result.request_id ? `요청 #${result.request_id} 접수` : '접수', 'ok');
    } catch (error) {
      setLevel(status, `RPC 실패: ${error.message}`, 'bad');
    }
  };
  button(actions, '보내기', send, 'primary');
  button(actions, '정지', () => host.rpc('llm/command', { text: 'stop' }).catch(() => {}), 'danger');
  input.addEventListener('keydown', (event) => {
    if (event.key === 'Enter' && (event.metaKey || event.ctrlKey)) {
      event.preventDefault();
      send();
    }
  });

  const history = document.createElement('div');
  history.style.display = 'block';
  history.style.marginTop = '4px';
  const log = document.createElement('div');
  log.style.fontFamily = 'Menlo, monospace';
  log.style.fontSize = '11px';
  log.style.whiteSpace = 'pre-wrap';
  log.style.opacity = '0.85';
  history.append(log);
  body.append(history);

  const stages = {
    thinking: 'Ollama에 의도 파악 요청 중…',
    plan: '의도 파악 완료',
    executing: '실행 중',
    moveit: 'MoveIt 실행 중',
    done: '완료',
    error: '오류',
  };
  host.onTopic('llm/status', (message) => {
    let event;
    try {
      event = JSON.parse(message.data);
    } catch {
      return;
    }
    const stage = stages[event.stage] || event.stage || '-';
    const request = event.request_id ? `#${event.request_id} ` : '';
    setLevel(status, `${request}${stage}`,
             event.stage === 'error' ? 'bad'
               : event.stage === 'done' ? 'ok'
                 : event.stage === 'thinking' || event.stage === 'executing' ? 'warn' : '');
    if (event.plan) {
      const parts = [event.plan.intent];
      if (event.plan.meters !== undefined) parts.push(`${event.plan.meters} m`);
      if (event.plan.preset) parts.push(event.plan.preset);
      if (event.plan.target) parts.push(event.plan.target);
      if (event.plan.question) parts.push(`"${event.plan.question}"`);
      setLevel(plan, parts.join(' · '));
    }
    if (event.error) {
      setLevel(plan, event.error, 'bad');
    }
    if (event.stage === 'done' || event.stage === 'error') {
      log.textContent = `${new Date().toLocaleTimeString()} ${request}${stage}` +
        (event.distance !== undefined ? ` · ${Number(event.distance).toFixed(2)} m` : '') +
        (event.answer ? ` · ${event.answer}` : '') +
        (event.error ? ` · ${event.error}` : '') + '\n' + log.textContent;
      log.textContent = log.textContent.split('\n').slice(0, 6).join('\n');
    }
  });
}
