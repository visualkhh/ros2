import { button, buttonRow, row } from './util.js';

export function init(host) {
  const body = host.addPanel('Teleop');
  const speed = row(body, 'speed');
  const linearInput = document.createElement('input');
  linearInput.type = 'range';
  linearInput.min = '0.05';
  linearInput.max = '0.30';
  linearInput.step = '0.01';
  linearInput.value = '0.18';
  const angularInput = document.createElement('input');
  angularInput.type = 'range';
  angularInput.min = '0.3';
  angularInput.max = '1.8';
  angularInput.step = '0.05';
  angularInput.value = '0.8';
  speed.replaceChildren(linearInput, angularInput);

  const buttons = buttonRow(body);
  let timer = null;
  let linear = 0;
  let angular = 0;

  function send() { host.sendTwist('cmd_vel', linear, angular); }
  function start(nextLinear, nextAngular) {
    linear = nextLinear;
    angular = nextAngular;
    if (timer) clearInterval(timer);
    send();
    timer = setInterval(send, 50);
  }
  function stop() {
    if (timer) clearInterval(timer);
    timer = null;
    linear = 0;
    angular = 0;
    host.sendTwist('cmd_vel', 0, 0);
  }

  const forward = button(buttons, '↑', () => {});
  const left = button(buttons, '←', () => {});
  const stopButton = button(buttons, 'STOP', stop, 'danger');
  const right = button(buttons, '→', () => {});
  const back = button(buttons, '↓', () => {});
  const layout = document.createElement('div');
  layout.className = 'btn-row';
  layout.append(left, stopButton, right);
  forward.remove();
  back.remove();
  body.append(forward, layout, back);

  function bind(node, linearValue, angularValue) {
    node.addEventListener('pointerdown', (event) => {
      event.preventDefault();
      node.setPointerCapture(event.pointerId);
      start(linearValue(), angularValue());
    });
    node.addEventListener('pointerup', stop);
    node.addEventListener('pointercancel', stop);
    node.addEventListener('pointerleave', (event) => {
      if (event.buttons === 0) stop();
    });
  }

  bind(forward, () => Number(linearInput.value), () => 0);
  bind(back, () => -Number(linearInput.value) * 0.65, () => 0);
  bind(left, () => 0, () => Number(angularInput.value));
  bind(right, () => 0, () => -Number(angularInput.value));

  window.addEventListener('blur', stop);
}
