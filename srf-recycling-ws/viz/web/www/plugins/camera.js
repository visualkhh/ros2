import { element } from './util.js';

const COLOR_TOPIC = 'camera/color/image_raw';
const DEPTH_TOPIC = 'camera/depth/image_rect_raw';

const STYLE = `
.camera-view {
  position: absolute;
  inset: 0;
  box-sizing: border-box;
  /* #view-tabs는 top:6px에 떠 있다. Camera 툴바가 탭과 겹치지 않게 띄운다. */
  padding-top: 44px;
  display: flex;
  flex-direction: column;
  background: #0b0f15;
  min-height: 0;
}
.camera-toolbar {
  display: flex;
  align-items: center;
  gap: 16px;
  padding: 8px 12px;
  border-bottom: 1px solid #263041;
  background: #121821;
  color: #cdd6e4;
  font-size: 12px;
  flex: none;
}
.camera-toolbar .title { font-weight: 600; color: #e8eef8; }
.camera-toolbar .stat { color: #8e9bb0; white-space: nowrap; }
.camera-toolbar .stat b { color: #cdd6e4; font-weight: 500; }
.camera-toolbar .ok { color: #54d18c; }
.camera-toolbar .warn { color: #f0c060; }
.camera-toolbar .bad { color: #ff6b6b; }
.camera-grid {
  flex: 1;
  display: grid;
  grid-template-columns: 1fr 1fr;
  gap: 8px;
  padding: 8px;
  min-height: 0;
}
.camera-pane {
  display: flex;
  flex-direction: column;
  min-width: 0;
  min-height: 0;
  background: #121821;
  border: 1px solid #263041;
  border-radius: 6px;
  overflow: hidden;
}
.camera-pane-head {
  display: flex;
  align-items: center;
  justify-content: space-between;
  gap: 8px;
  padding: 6px 10px;
  border-bottom: 1px solid #263041;
  font-size: 12px;
}
.camera-pane-head .name { color: #e8eef8; font-weight: 600; }
.camera-pane-head .info { color: #8e9bb0; font-variant-numeric: tabular-nums; }
.camera-stage {
  flex: 1;
  min-height: 0;
  display: flex;
  align-items: center;
  justify-content: center;
  position: relative;
  background: #000;
}
.camera-canvas {
  width: 100%;
  height: 100%;
  max-height: none;
  object-fit: contain;
  display: block;
}
.camera-empty {
  position: absolute;
  inset: 0;
  display: flex;
  align-items: center;
  justify-content: center;
  color: #667286;
  font-size: 13px;
  text-align: center;
  padding: 12px;
}
.camera-legend {
  padding: 4px 10px;
  border-top: 1px solid #263041;
  color: #7e8ca3;
  font-size: 11px;
  flex: none;
}
@media (max-width: 900px) {
  .camera-grid { grid-template-columns: 1fr; }
}
`;

function installStyle() {
  if (document.getElementById('camera-plugin-style')) return;
  const style = document.createElement('style');
  style.id = 'camera-plugin-style';
  style.textContent = STYLE;
  document.head.append(style);
}

function makePane(parent, name) {
  const pane = element('div', 'camera-pane');
  const head = element('div', 'camera-pane-head');
  const title = element('div', 'name', name);
  const info = element('div', 'info', '대기');
  head.append(title, info);
  const stage = element('div', 'camera-stage');
  const canvas = document.createElement('canvas');
  canvas.className = 'camera-canvas';
  const empty = element('div', 'camera-empty', '영상 없음');
  stage.append(canvas, empty);
  pane.append(head, stage);
  parent.append(pane);
  return { pane, info, canvas, context: canvas.getContext('2d'), empty };
}

// D435 depth는 16UC1(밀리미터) little-endian으로 온다.
// 0.2 m(파랑) ~ 3.0 m(빨강) 범위를 256색 LUT로 칠한다.
const DEPTH_LUT = new Uint8Array(256 * 3);
for (let index = 0; index < 256; ++index) {
  const t = index / 255;
  const clamp = (value) => Math.max(0, Math.min(1, value));
  DEPTH_LUT[index * 3] = Math.round(clamp(1.5 - Math.abs(4 * t - 3)) * 255);
  DEPTH_LUT[index * 3 + 1] = Math.round(clamp(1.5 - Math.abs(4 * t - 2)) * 255);
  DEPTH_LUT[index * 3 + 2] = Math.round(clamp(1.5 - Math.abs(4 * t - 1)) * 255);
}

export function init(host) {
  installStyle();
  // 플러그인이 토픽을 등록하는 경로. 이미 게이트웨이가 구독 중이면 재구독하지 않는다.
  host.registerTopic({ name: COLOR_TOPIC, kind: 17, qos: 'sensor', source: 'Camera' })
      .catch(() => {});
  host.registerTopic({ name: DEPTH_TOPIC, kind: 17, qos: 'sensor', source: 'Camera' })
      .catch(() => {});
  const view = host.addView({ id: 'camera', title: 'Camera' });
  const root = element('div', 'camera-view');
  const toolbar = element('div', 'camera-toolbar');
  const title = element('div', 'title', 'RealSense D435');
  const colorStat = element('div', 'stat', 'color -');
  const depthStat = element('div', 'stat', 'depth -');
  const linkStat = element('div', 'stat', '대기');
  const particleLabel = document.createElement('label');
  particleLabel.className = 'stat';
  const particles = document.createElement('input');
  particles.type = 'checkbox';
  particles.checked = true;
  particleLabel.append(particles, document.createTextNode(' particles'));
  toolbar.append(title, colorStat, depthStat, linkStat, particleLabel);
  const grid = element('div', 'camera-grid');
  root.append(toolbar, grid);
  view.panel.append(root);

  const color = makePane(grid, 'Color');
  const depth = makePane(grid, 'Depth');
  const legend = element('div', 'camera-legend', 'depth: 0.2 m(파랑) → 3.0 m(빨강) · 정렬되지 않은 depth 센서 시점');
  depth.pane.append(legend);

  let colorFrames = 0;
  let depthFrames = 0;
  let colorFps = 0;
  let depthFps = 0;
  let lastColor = 0;
  let lastDepth = 0;
  let colorPending = false;

  const fpsText = (value) => (value > 0 ? `${value} fps` : '-');
  const setLink = (text, level = '') => {
    linkStat.textContent = text;
    linkStat.className = `stat ${level}`;
  };

  // 시뮬레이터는 RGB8 raw, 실기는 JPEG를 보낸다. 둘 다 binary payload 그대로다.
  function drawRgb(message, target) {
    if (message.encoding !== 'rgb8' && message.encoding !== 'bgr8') {
      return false;
    }
    const width = message.width;
    const height = message.height;
    if (target.canvas.width !== width || target.canvas.height !== height) {
      target.canvas.width = width;
      target.canvas.height = height;
    }
    const image = target.context.createImageData(width, height);
    const pixels = image.data;
    const data = message.data;
    const step = message.step || width * 3;
    const bgr = message.encoding === 'bgr8';
    for (let y = 0; y < height; ++y) {
      const sourceRow = y * step;
      const targetRow = y * width * 4;
      for (let x = 0; x < width; ++x) {
        const source = sourceRow + x * 3;
        const targetIndex = targetRow + x * 4;
        pixels[targetIndex] = data[source + (bgr ? 2 : 0)];
        pixels[targetIndex + 1] = data[source + 1];
        pixels[targetIndex + 2] = data[source + (bgr ? 0 : 2)];
        pixels[targetIndex + 3] = 255;
      }
    }
    target.context.putImageData(image, 0, 0);
    target.empty.style.display = 'none';
    target.info.textContent =
        `${width}×${height} · ${(message.data.length / 1024).toFixed(0)} KB`;
    return true;
  }

  function drawColor(message) {
    if (!message || !message.data) {
      color.info.textContent = '대기';
      return;
    }
    if (message.encoding === 'jpeg') {
      drawJpeg(message, color, 'color');
    } else if (!drawRgb(message, color)) {
      color.info.textContent = `지원하지 않는 ${message.encoding}`;
    }
  }

  function drawJpeg(message, target, label) {
    if (!message || !message.data || message.encoding !== 'jpeg') {
      target.info.textContent = message ? `지원하지 않는 ${message.encoding}` : '대기';
      return;
    }
    if (label === 'color') {
      if (colorPending) return;
      colorPending = true;
    }
    const blob = new Blob([message.data], { type: 'image/jpeg' });
    createImageBitmap(blob).then((bitmap) => {
      if (target.canvas.width !== message.width || target.canvas.height !== message.height) {
        target.canvas.width = message.width;
        target.canvas.height = message.height;
      }
      target.context.drawImage(bitmap, 0, 0, message.width, message.height);
      bitmap.close();
      target.empty.style.display = 'none';
      target.info.textContent =
          `${message.width}×${message.height} · ${(message.data.length / 1024).toFixed(0)} KB`;
    }).catch(() => {
      target.info.textContent = 'JPEG 디코드 실패';
    }).finally(() => {
      if (label === 'color') colorPending = false;
    });
  }

  function drawDepth(message) {
    if (!message || !message.data || message.encoding !== '16UC1') {
      if (message && message.encoding === 'jpeg') {
        drawJpeg(message, depth, 'depth');
      } else {
        depth.info.textContent = message ? `지원하지 않는 ${message.encoding}` : '대기';
      }
      return;
    }
    const width = message.width;
    const height = message.height;
    if (depth.canvas.width !== width || depth.canvas.height !== height) {
      depth.canvas.width = width;
      depth.canvas.height = height;
    }
    const image = depth.context.createImageData(width, height);
    const pixels = image.data;
    const data = message.data;
    const count = width * height;
    for (let index = 0; index < count; ++index) {
      const out = index * 4;
      pixels[out + 3] = 255;
      if (particles.checked) {
        const x = index % width;
        const y = (index / width) | 0;
        if ((x & 1) !== 0 || (y & 1) !== 0) continue;  // 2x2 샘플 = particle
      }
      const offset = index * 2;
      const mm = data[offset] | (data[offset + 1] << 8);
      if (mm === 0) continue;  // no return = black
      const t = Math.max(0, Math.min(1, (mm / 1000 - 0.2) / 2.8));
      const colorIndex = Math.round(t * 255) * 3;
      pixels[out] = DEPTH_LUT[colorIndex];
      pixels[out + 1] = DEPTH_LUT[colorIndex + 1];
      pixels[out + 2] = DEPTH_LUT[colorIndex + 2];
    }
    depth.context.putImageData(image, 0, 0);
    depth.empty.style.display = 'none';
    depth.info.textContent =
        `${width}×${height} · ${(message.data.length / 1024).toFixed(0)} KB`;
  }

  host.onTopic(COLOR_TOPIC, (message) => {
    colorFrames += 1;
    lastColor = performance.now();
    setLink('연결', 'ok');
    drawColor(message);
  });
  host.onTopic(DEPTH_TOPIC, (message) => {
    depthFrames += 1;
    lastDepth = performance.now();
    if (!lastColor || performance.now() - lastColor > 2500) setLink('연결', 'ok');
    drawDepth(message);
  });

  setInterval(() => {
    colorFps = colorFrames;
    depthFps = depthFrames;
    colorFrames = 0;
    depthFrames = 0;
    colorStat.innerHTML = `color <b>${fpsText(colorFps)}</b>`;
    depthStat.innerHTML = `depth <b>${fpsText(depthFps)}</b>`;
    const now = performance.now();
    if (!lastColor && !lastDepth) {
      setLink('카메라 메시지 없음', 'warn');
    } else if (now - Math.max(lastColor, lastDepth) > 2500) {
      setLink('끊김', 'bad');
    } else {
      setLink('연결', 'ok');
    }
  }, 1000);
}
