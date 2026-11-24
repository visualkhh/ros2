export function element(tag, className = '', text = '') {
  const node = document.createElement(tag);
  if (className) node.className = className;
  if (text) node.textContent = text;
  return node;
}

export function row(parent, label) {
  const wrapper = element('div', 'field');
  const left = element('label', '', label);
  const right = element('div', 'value', '-');
  wrapper.append(left, right);
  parent.append(wrapper);
  return right;
}

export function button(parent, label, onClick, className = '') {
  const node = element('button', className, label);
  node.type = 'button';
  node.addEventListener('click', onClick);
  parent.append(node);
  return node;
}

export function buttonRow(parent) {
  const wrapper = element('div', 'btn-row');
  parent.append(wrapper);
  return wrapper;
}

export function select(parent, options = []) {
  const node = element('select');
  for (const option of options) {
    const item = document.createElement('option');
    item.value = option.value ?? option;
    item.textContent = option.label ?? option;
    node.append(item);
  }
  parent.append(node);
  return node;
}

export function input(parent, placeholder = '') {
  const node = element('input');
  node.placeholder = placeholder;
  parent.append(node);
  return node;
}

export function parseFields(text) {
  const values = new Map();
  for (const part of String(text || '').split(';')) {
    const equals = part.indexOf('=');
    if (equals > 0) values.set(part.slice(0, equals), part.slice(equals + 1));
  }
  return values;
}

export function setLevel(node, text, level = '') {
  node.textContent = text;
  node.className = `value ${level}`;
}

export function number(value, digits = 3) {
  const parsed = Number(value);
  return Number.isFinite(parsed) ? parsed.toFixed(digits) : '-';
}
