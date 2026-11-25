const $ = (id) => document.getElementById(id);
const textDecoder = new TextDecoder();
const textEncoder = new TextEncoder();

const KIND_NAMES = {
  1: 'Twist', 2: 'String', 3: 'TF', 4: 'Scan', 5: 'Imu', 6: 'Odom',
  7: 'JointState', 8: 'Grid', 9: 'Path', 10: 'Diagnostics', 11: 'Log',
  12: 'Pose', 13: 'PoseCov', 14: 'Empty', 15: 'Bool',
};

const state = {
  ws: null,
  wsOpen: false,
  robotConnected: false,
  latest: new Map(),
  stats: new Map(),
  subscribers: new Map(),
  tf: new Map(),
  goalTopic: 'goal_pose',
  settings: {
    grid: true,
    map: true,
    global_costmap: false,
    local_costmap: true,
    scan: true,
    path: true,
    local_plan: true,
    robot: true,
    tf: false,
  },
};

class Reader {
  constructor(buffer, offset = 0) {
    this.view = new DataView(buffer);
    this.offset = offset;
  }
  u8() { const value = this.view.getUint8(this.offset); this.offset += 1; return value; }
  u16() { const value = this.view.getUint16(this.offset, true); this.offset += 2; return value; }
  u32() { const value = this.view.getUint32(this.offset, true); this.offset += 4; return value; }
  f32() { const value = this.view.getFloat32(this.offset, true); this.offset += 4; return value; }
  f64() { const value = this.view.getFloat64(this.offset, true); this.offset += 8; return value; }
  str() {
    const length = this.u16();
    const bytes = new Uint8Array(this.view.buffer, this.view.byteOffset + this.offset, length);
    this.offset += length;
    return textDecoder.decode(bytes);
  }
  bytes(length) {
    const value = new Uint8Array(this.view.buffer, this.view.byteOffset + this.offset, length);
    this.offset += length;
    return value;
  }
  remaining() { return this.view.byteLength - this.offset; }
}

class Writer {
  constructor() { this.data = []; }
  u8(value) { this.data.push(value & 0xff); }
  u16(value) { this.data.push(value & 0xff, (value >> 8) & 0xff); }
  u32(value) {
    this.data.push(value & 0xff, (value >> 8) & 0xff, (value >> 16) & 0xff, (value >> 24) & 0xff);
  }
  f64(value) {
    const buffer = new ArrayBuffer(8);
    new DataView(buffer).setFloat64(0, value, true);
    this.data.push(...new Uint8Array(buffer));
  }
  str(value) {
    const bytes = textEncoder.encode(value);
    this.u16(bytes.length);
    this.data.push(...bytes);
  }
  bytes(value) { this.data.push(...value); }
  result() { return new Uint8Array(this.data); }
}

function readHeader(reader) {
  return { stamp: reader.f64(), frame: reader.str() };
}
function readPoint(reader) {
  return { x: reader.f64(), y: reader.f64(), z: reader.f64() };
}
function readQuaternion(reader) {
  return { x: reader.f64(), y: reader.f64(), z: reader.f64(), w: reader.f64() };
}
function readPose(reader) {
  return { position: readPoint(reader), orientation: readQuaternion(reader) };
}
function readDoubles(reader, count) {
  const values = new Array(count);
  for (let index = 0; index < count; ++index) values[index] = reader.f64();
  return values;
}
function yawOf(quaternion) {
  const { x, y, z, w } = quaternion;
  return Math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z));
}

function decodeFrame(buffer) {
  const reader = new Reader(buffer);
  const type = reader.u8();
  const topic = reader.str();
  const kind = reader.u8();
  let value = null;
  switch (kind) {
    case 1: {
      const linear = readPoint(reader);
      const angular = readPoint(reader);
      value = { linear, angular };
      break;
    }
    case 2:
      value = { data: reader.str() };
      break;
    case 3: {
      const count = reader.u32();
      const transforms = [];
      for (let index = 0; index < count; ++index) {
        const header = readHeader(reader);
        const child = reader.str();
        const translation = readPoint(reader);
        const rotation = readQuaternion(reader);
        transforms.push({ header, child, translation, rotation });
      }
      value = { transforms };
      break;
    }
    case 4: {
      const header = readHeader(reader);
      const angle_min = reader.f64();
      const angle_max = reader.f64();
      const angle_increment = reader.f64();
      const range_min = reader.f64();
      const range_max = reader.f64();
      const count = reader.u32();
      const ranges = new Array(count);
      for (let index = 0; index < count; ++index) ranges[index] = reader.f32();
      const intensities = reader.u32();
      reader.bytes(intensities * 4);
      value = { header, angle_min, angle_max, angle_increment, range_min, range_max, ranges };
      break;
    }
    case 5: {
      const header = readHeader(reader);
      const orientation = readQuaternion(reader);
      readDoubles(reader, 9);
      const angular = readPoint(reader);
      readDoubles(reader, 9);
      const linear = readPoint(reader);
      readDoubles(reader, 9);
      value = { header, orientation, angular, linear };
      break;
    }
    case 6: {
      const header = readHeader(reader);
      const child = reader.str();
      const pose = readPose(reader);
      readDoubles(reader, 36);
      const linear = readPoint(reader);
      const angular = readPoint(reader);
      readDoubles(reader, 36);
      value = { header, child, pose, linear, angular };
      break;
    }
    case 7: {
      const header = readHeader(reader);
      const count = reader.u32();
      const names = [];
      for (let index = 0; index < count; ++index) names.push(reader.str());
      const positions = readDoubles(reader, reader.u32());
      const velocities = readDoubles(reader, reader.u32());
      const efforts = readDoubles(reader, reader.u32());
      value = { header, names, positions, velocities, efforts };
      break;
    }
    case 8: {
      const header = readHeader(reader);
      const map_load_time = reader.f64();
      const resolution = reader.f64();
      const width = reader.u32();
      const height = reader.u32();
      const origin = readPose(reader);
      const length = reader.u32();
      const data = new Int8Array(reader.bytes(length));
      value = { header, map_load_time, resolution, width, height, origin, data };
      break;
    }
    case 9: {
      const header = readHeader(reader);
      const count = reader.u32();
      const poses = [];
      for (let index = 0; index < count; ++index) {
        const poseHeader = readHeader(reader);
        poses.push({ header: poseHeader, pose: readPose(reader) });
      }
      value = { header, poses };
      break;
    }
    case 10: {
      const header = readHeader(reader);
      const count = reader.u32();
      const status = [];
      for (let index = 0; index < count; ++index) {
        const level = reader.u8();
        const name = reader.str();
        const message = reader.str();
        const hardware_id = reader.str();
        const values = [];
        const valueCount = reader.u32();
        for (let item = 0; item < valueCount; ++item) {
          values.push({ key: reader.str(), value: reader.str() });
        }
        status.push({ level, name, message, hardware_id, values });
      }
      value = { header, status };
      break;
    }
    case 11: {
      const stamp = reader.f64();
      const level = reader.u8();
      const name = reader.str();
      const message = reader.str();
      const file = reader.str();
      const functionName = reader.str();
      const line = reader.u32();
      value = { stamp, level, name, message, file, function: functionName, line };
      break;
    }
    case 12:
      value = { header: readHeader(reader), pose: readPose(reader) };
      break;
    case 13:
      value = { header: readHeader(reader), pose: readPose(reader), covariance: readDoubles(reader, 36) };
      break;
    case 14:
      value = {};
      break;
    case 15:
      value = { data: reader.u8() !== 0 };
      break;
    default:
      value = null;
  }
  return { type, topic, kind, value };
}

function applyTransforms(transforms) {
  for (const transform of transforms) {
    state.tf.set(transform.child, {
      parent: transform.header.frame,
      x: transform.translation.x,
      y: transform.translation.y,
      yaw: yawOf(transform.rotation),
    });
  }
}

function compose(first, second) {
  const cos = Math.cos(first.yaw);
  const sin = Math.sin(first.yaw);
  return {
    x: first.x + cos * second.x - sin * second.y,
    y: first.y + sin * second.x + cos * second.y,
    yaw: first.yaw + second.yaw,
  };
}

function lookup(target, source, depth = 0) {
  if (target === source) return { x: 0, y: 0, yaw: 0 };
  if (depth > 64) return null;
  const link = state.tf.get(source);
  if (!link) return null;
  const parent = lookup(target, link.parent, depth + 1);
  if (!parent) return null;
  return compose(parent, { x: link.x, y: link.y, yaw: link.yaw });
}

function transformPoint(transform, x, y) {
  if (!transform) return { x, y };
  const cos = Math.cos(transform.yaw);
  const sin = Math.sin(transform.yaw);
  return { x: transform.x + cos * x - sin * y, y: transform.y + sin * x + cos * y };
}

function frameTransform(frame) {
  if (!frame || frame === 'map') return { x: 0, y: 0, yaw: 0 };
  return lookup('map', frame);
}

function robotPose() {
  const direct = lookup('map', 'base_footprint');
  if (direct) return direct;
  const odom = state.latest.get('odom');
  if (!odom) return null;
  const pose = {
    x: odom.pose.position.x,
    y: odom.pose.position.y,
    yaw: yawOf(odom.pose.orientation),
  };
  const odomMap = lookup('map', odom.header.frame);
  return odomMap ? compose(odomMap, pose) : pose;
}

function topic(topicName) {
  return state.latest.get(topicName);
}

function body(topicName, kind, payload) {
  const writer = new Writer();
  writer.u8(2);
  writer.str(topicName);
  writer.u8(kind);
  writer.bytes(payload);
  return writer.result();
}

function sendBinary(topicName, kind, payload) {
  if (!state.ws || state.ws.readyState !== WebSocket.OPEN) return false;
  state.ws.send(body(topicName, kind, payload));
  return true;
}

function sendString(topicName, value) {
  const writer = new Writer();
  writer.str(value);
  return sendBinary(topicName, 2, writer.data);
}
function sendBool(topicName, value) {
  return sendBinary(topicName, 15, [value ? 1 : 0]);
}
function sendEmpty(topicName) {
  return sendBinary(topicName, 14, []);
}
function sendTwist(topicName, linear, angular) {
  const writer = new Writer();
  writer.f64(linear); writer.f64(0); writer.f64(0);
  writer.f64(0); writer.f64(0); writer.f64(angular);
  return sendBinary(topicName, 1, writer.data);
}
function sendGoal(x, y, yaw = 0) {
  const writer = new Writer();
  writer.f64(Date.now() / 1000);
  writer.str('map');
  writer.f64(x); writer.f64(y); writer.f64(0);
  writer.f64(0); writer.f64(0); writer.f64(Math.sin(yaw / 2)); writer.f64(Math.cos(yaw / 2));
  return sendBinary(state.goalTopic, 12, writer.data);
}

function onTopic(topicName, callback) {
  if (!state.subscribers.has(topicName)) state.subscribers.set(topicName, new Set());
  state.subscribers.get(topicName).add(callback);
  const latest = state.latest.get(topicName);
  if (latest) {
    try { callback(latest); } catch (error) { console.error(error); }
  }
  return () => state.subscribers.get(topicName)?.delete(callback);
}

function emit(topicName, value) {
  const subscribers = state.subscribers.get(topicName);
  if (!subscribers) return;
  for (const callback of subscribers) {
    try { callback(value); } catch (error) { console.error(error); }
  }
}

function toast(message) {
  const element = $('toast');
  element.textContent = message;
  element.classList.add('show');
  clearTimeout(toast.timer);
  toast.timer = setTimeout(() => element.classList.remove('show'), 4000);
}

const host = {
  onTopic,
  latest: topic,
  sendString,
  sendBool,
  sendEmpty,
  sendTwist,
  sendGoal,
  setGoalTopic(topicName) { state.goalTopic = topicName; },
  toast,
  robotPose,
  lookup,
  addPanel(title) {
    const section = document.createElement('section');
    section.className = 'panel';
    const heading = document.createElement('h2');
    heading.textContent = title;
    const bodyElement = document.createElement('div');
    bodyElement.className = 'body';
    section.append(heading, bodyElement);
    $('right').append(section);
    return bodyElement;
  },
  addRow(parent, left, right) {
    const row = document.createElement('div');
    row.className = 'field';
    const label = document.createElement('label');
    label.textContent = left;
    const value = document.createElement('div');
    value.className = 'value';
    if (right !== undefined) value.textContent = right;
    row.append(label, value);
    parent.append(row);
    return value;
  },
};

const displays = [
  ['grid', 'Grid'],
  ['map', 'Map'],
  ['global_costmap', 'Global costmap'],
  ['local_costmap', 'Local costmap'],
  ['scan', 'Laser scan'],
  ['path', 'Plan'],
  ['local_plan', 'Local plan'],
  ['robot', 'Robot model'],
  ['tf', 'TF axes'],
];

for (const [key, label] of displays) {
  const wrapper = document.createElement('label');
  const checkbox = document.createElement('input');
  checkbox.type = 'checkbox';
  checkbox.checked = state.settings[key];
  checkbox.addEventListener('change', () => { state.settings[key] = checkbox.checked; });
  wrapper.append(checkbox, document.createTextNode(label));
  $('displays').append(wrapper);
}

const topicRows = new Map();
function updateTopicRow(topicName, kind) {
  let row = topicRows.get(topicName);
  if (!row) {
    row = document.createElement('div');
    row.className = 'topic';
    row.innerHTML = `<span class="name"></span><span class="kind"></span><span class="hz"></span>`;
    $('topics').append(row);
    row.querySelector('.name').textContent = topicName;
    row.querySelector('.kind').textContent = KIND_NAMES[kind] ?? kind;
    topicRows.set(topicName, row);
  }
  const stats = state.stats.get(topicName);
  row.querySelector('.hz').textContent = stats.hz ? stats.hz.toFixed(1) : '-';
}

function handleFrame(buffer) {
  let frame;
  try {
    frame = decodeFrame(buffer);
  } catch (error) {
    console.error('frame decode', error);
    return;
  }
  if (frame.type !== 1) return;
  state.robotConnected = true;
  updateRobotStatus();
  state.latest.set(frame.topic, frame.value);
  let stats = state.stats.get(frame.topic);
  if (!stats) {
    stats = { count: 0, last: performance.now(), hz: 0, kind: frame.kind, countAtTick: 0 };
    state.stats.set(frame.topic, stats);
  }
  stats.count += 1;
  stats.kind = frame.kind;
  if (frame.kind === 3 && frame.value) applyTransforms(frame.value.transforms);
  if (frame.kind === 8) frame.value._canvas = null;
  updateTopicRow(frame.topic, frame.kind);
  emit(frame.topic, frame.value);
}

setInterval(() => {
  const now = performance.now();
  for (const [topicName, stats] of state.stats) {
    const elapsed = (now - stats.last) / 1000;
    if (elapsed >= 0.5) {
      stats.hz = (stats.count - stats.countAtTick) / elapsed;
      stats.countAtTick = stats.count;
      stats.last = now;
      updateTopicRow(topicName, stats.kind);
    }
  }
}, 500);

function updateRobotStatus() {
  $('robot-dot').className = `dot ${state.robotConnected ? 'on' : 'off'}`;
  $('robot-text').textContent = state.robotConnected ? 'robot connected' : 'robot offline';
}
function updateWsStatus() {
  $('ws-dot').className = `dot ${state.wsOpen ? 'on' : 'off'}`;
  $('ws-text').textContent = state.wsOpen ? 'websocket' : 'websocket offline';
}

function connect() {
  const scheme = location.protocol === 'https:' ? 'wss' : 'ws';
  const socket = new WebSocket(`${scheme}://${location.host}/ws`);
  socket.binaryType = 'arraybuffer';
  state.ws = socket;
  socket.onopen = () => { state.wsOpen = true; updateWsStatus(); };
  socket.onclose = () => {
    state.wsOpen = false;
    state.robotConnected = false;
    updateWsStatus();
    updateRobotStatus();
    setTimeout(connect, 1000);
  };
  socket.onerror = () => socket.close();
  socket.onmessage = (event) => {
    if (typeof event.data === 'string') {
      try {
        const status = JSON.parse(event.data);
        if (status.type === 'robot') {
          state.robotConnected = Boolean(status.connected);
          updateRobotStatus();
        }
      } catch (error) { console.error(error); }
      return;
    }
    handleFrame(event.data);
  };
}
updateWsStatus();
updateRobotStatus();
connect();

// canvas
const canvas = $('scene');
const context = canvas.getContext('2d');
const view = { cx: 0, cy: 0, scale: 60 };
let dragging = false;
let dragged = false;
let dragStart = { x: 0, y: 0 };
let lastPointer = { x: 0, y: 0 };

function resizeCanvas() {
  const ratio = window.devicePixelRatio || 1;
  canvas.width = Math.max(1, Math.floor(canvas.clientWidth * ratio));
  canvas.height = Math.max(1, Math.floor(canvas.clientHeight * ratio));
  context.setTransform(ratio, 0, 0, ratio, 0, 0);
}
window.addEventListener('resize', resizeCanvas);
resizeCanvas();

function screenToWorld(x, y) {
  return {
    x: view.cx + (x - canvas.clientWidth / 2) / view.scale,
    y: view.cy - (y - canvas.clientHeight / 2) / view.scale,
  };
}
function worldToScreen(x, y) {
  return {
    x: canvas.clientWidth / 2 + (x - view.cx) * view.scale,
    y: canvas.clientHeight / 2 - (y - view.cy) * view.scale,
  };
}

canvas.addEventListener('pointerdown', (event) => {
  dragging = true;
  dragged = false;
  dragStart = { x: event.clientX, y: event.clientY };
  canvas.setPointerCapture(event.pointerId);
});
canvas.addEventListener('pointermove', (event) => {
  const world = screenToWorld(event.clientX - canvas.getBoundingClientRect().left,
                              event.clientY - canvas.getBoundingClientRect().top);
  $('cursor').textContent = `${world.x.toFixed(2)}, ${world.y.toFixed(2)}`;
  if (dragging) {
    const dx = event.clientX - dragStart.x;
    const dy = event.clientY - dragStart.y;
    if (Math.abs(dx) + Math.abs(dy) > 3) dragged = true;
    view.cx -= dx / view.scale;
    view.cy += dy / view.scale;
    dragStart = { x: event.clientX, y: event.clientY };
  }
});
canvas.addEventListener('pointerup', (event) => {
  dragging = false;
  if (!dragged) {
    const world = screenToWorld(event.clientX - canvas.getBoundingClientRect().left,
                                event.clientY - canvas.getBoundingClientRect().top);
    sendGoal(world.x, world.y);
  }
});
canvas.addEventListener('wheel', (event) => {
  event.preventDefault();
  const rect = canvas.getBoundingClientRect();
  const before = screenToWorld(event.clientX - rect.left, event.clientY - rect.top);
  const factor = event.deltaY < 0 ? 1.15 : 1 / 1.15;
  view.scale = Math.min(400, Math.max(5, view.scale * factor));
  const after = screenToWorld(event.clientX - rect.left, event.clientY - rect.top);
  view.cx += before.x - after.x;
  view.cy += before.y - after.y;
}, { passive: false });

$('fit').addEventListener('click', () => {
  const map = topic('map');
  if (map) {
    view.cx = map.origin.position.x + map.width * map.resolution / 2;
    view.cy = map.origin.position.y + map.height * map.resolution / 2;
    const bounds = Math.max(map.width * map.resolution, map.height * map.resolution, 1);
    view.scale = Math.min(canvas.clientWidth, canvas.clientHeight) / bounds * 0.9;
    $('follow').checked = false;
    return;
  }
  view.cx = 0; view.cy = 0; view.scale = 60;
});

function drawGrid() {
  if (!state.settings.grid) return;
  const width = canvas.clientWidth;
  const height = canvas.clientHeight;
  const topLeft = screenToWorld(0, 0);
  const bottomRight = screenToWorld(width, height);
  const minX = Math.min(topLeft.x, bottomRight.x);
  const maxX = Math.max(topLeft.x, bottomRight.x);
  const minY = Math.min(topLeft.y, bottomRight.y);
  const maxY = Math.max(topLeft.y, bottomRight.y);
  let step = 1;
  if (view.scale < 15) step = 5;
  else if (view.scale > 120) step = 0.5;
  context.lineWidth = 1;
  context.strokeStyle = '#141c27';
  context.beginPath();
  for (let x = Math.ceil(minX / step) * step; x <= maxX; x += step) {
    const screen = worldToScreen(x, 0);
    context.moveTo(screen.x, 0);
    context.lineTo(screen.x, height);
  }
  for (let y = Math.ceil(minY / step) * step; y <= maxY; y += step) {
    const screen = worldToScreen(0, y);
    context.moveTo(0, screen.y);
    context.lineTo(width, screen.y);
  }
  context.stroke();
  const origin = worldToScreen(0, 0);
  context.strokeStyle = '#26364a';
  context.beginPath();
  context.moveTo(origin.x - 10, origin.y); context.lineTo(origin.x + 10, origin.y);
  context.moveTo(origin.x, origin.y - 10); context.lineTo(origin.x, origin.y + 10);
  context.stroke();
}

function mapCanvas(grid) {
  if (grid._canvas) return grid._canvas;
  const image = document.createElement('canvas');
  image.width = grid.width;
  image.height = grid.height;
  const imageContext = image.getContext('2d');
  const imageData = imageContext.createImageData(grid.width, grid.height);
  const pixels = imageData.data;
  for (let y = 0; y < grid.height; ++y) {
    for (let x = 0; x < grid.width; ++x) {
      const value = grid.data[y * grid.width + x];
      const index = (y * grid.width + x) * 4;
      if (value < 0) {
        pixels[index] = 12; pixels[index + 1] = 16; pixels[index + 2] = 22; pixels[index + 3] = 0;
      } else if (value >= 65) {
        pixels[index] = 28; pixels[index + 1] = 34; pixels[index + 2] = 44; pixels[index + 3] = 255;
      } else {
        pixels[index] = 150; pixels[index + 1] = 170; pixels[index + 2] = 190; pixels[index + 3] = 255;
      }
    }
  }
  imageContext.putImageData(imageData, 0, 0);
  grid._canvas = image;
  return image;
}

function costmapCanvas(grid) {
  if (grid._costCanvas) return grid._costCanvas;
  const image = document.createElement('canvas');
  image.width = grid.width;
  image.height = grid.height;
  const imageContext = image.getContext('2d');
  const imageData = imageContext.createImageData(grid.width, grid.height);
  const pixels = imageData.data;
  for (let index = 0; index < grid.data.length; ++index) {
    const value = grid.data[index];
    const offset = index * 4;
    if (value <= 0) {
      pixels[offset + 3] = 0;
      continue;
    }
    const intensity = Math.min(1, value / 254);
    pixels[offset] = 230;
    pixels[offset + 1] = 70 + Math.round(100 * (1 - intensity));
    pixels[offset + 2] = 60;
    pixels[offset + 3] = Math.round(30 + 110 * intensity);
  }
  imageContext.putImageData(imageData, 0, 0);
  grid._costCanvas = image;
  return image;
}

function drawGridMessage(name, builder, alpha) {
  const grid = topic(name);
  if (!grid) return;
  const transform = frameTransform(grid.header.frame);
  if (!transform) return;
  const origin = transformPoint(transform, grid.origin.position.x, grid.origin.position.y);
  const originScreen = worldToScreen(origin.x, origin.y);
  context.save();
  context.globalAlpha = alpha;
  context.translate(originScreen.x, originScreen.y);
  context.rotate(-(transform.yaw + yawOf(grid.origin.orientation)));
  context.scale(view.scale, -view.scale);
  context.imageSmoothingEnabled = false;
  const image = builder(grid);
  context.drawImage(image, 0, 0, grid.width * grid.resolution, grid.height * grid.resolution);
  context.restore();
}

function drawPath(message, color, width) {
  if (!message || message.poses.length === 0) return;
  const transform = frameTransform(message.header.frame);
  if (!transform) return;
  context.beginPath();
  let started = false;
  for (const pose of message.poses) {
    const point = transformPoint(transform, pose.pose.position.x, pose.pose.position.y);
    const screen = worldToScreen(point.x, point.y);
    if (!started) { context.moveTo(screen.x, screen.y); started = true; }
    else context.lineTo(screen.x, screen.y);
  }
  context.strokeStyle = color;
  context.lineWidth = width;
  context.stroke();
}

function drawScan() {
  if (!state.settings.scan) return;
  const scan = topic('scan');
  if (!scan) return;
  const transform = frameTransform(scan.header.frame);
  if (!transform) return;
  context.fillStyle = 'rgba(120,190,255,0.8)';
  const count = scan.ranges.length;
  const stride = count > 720 ? 2 : 1;
  const cosYaw = Math.cos(transform.yaw);
  const sinYaw = Math.sin(transform.yaw);
  for (let index = 0; index < count; index += stride) {
    const range = scan.ranges[index];
    if (!Number.isFinite(range) || range < scan.range_min || range > scan.range_max) continue;
    const angle = scan.angle_min + index * scan.angle_increment;
    const localX = Math.cos(angle) * range;
    const localY = Math.sin(angle) * range;
    const x = transform.x + cosYaw * localX - sinYaw * localY;
    const y = transform.y + sinYaw * localX + cosYaw * localY;
    const screen = worldToScreen(x, y);
    context.fillRect(screen.x - 1, screen.y - 1, 2, 2);
  }
}

function drawRobot() {
  if (!state.settings.robot) return;
  const pose = robotPose();
  if (!pose) return;
  const screen = worldToScreen(pose.x, pose.y);
  context.save();
  context.translate(screen.x, screen.y);
  context.rotate(-pose.yaw);
  context.beginPath();
  context.moveTo(16, 0);
  context.lineTo(-10, -9);
  context.lineTo(-6, 0);
  context.lineTo(-10, 9);
  context.closePath();
  context.fillStyle = '#54d18c';
  context.fill();
  context.restore();
  if (state.settings.tf) {
    const size = 28;
    context.strokeStyle = '#ff6b6b';
    context.beginPath(); context.moveTo(screen.x, screen.y);
    context.lineTo(screen.x + Math.cos(pose.yaw) * size, screen.y - Math.sin(pose.yaw) * size);
    context.stroke();
    context.strokeStyle = '#6aa9ff';
    context.beginPath(); context.moveTo(screen.x, screen.y);
    context.lineTo(screen.x - Math.sin(pose.yaw) * size, screen.y - Math.cos(pose.yaw) * size);
    context.stroke();
  }
}

function drawGoal() {
  for (const name of ['goal_pose', 'preview_goal']) {
    const goal = topic(name);
    if (!goal) continue;
    const transform = frameTransform(goal.header.frame);
    if (!transform) continue;
    const point = transformPoint(transform, goal.pose.position.x, goal.pose.position.y);
    const screen = worldToScreen(point.x, point.y);
    context.strokeStyle = name === 'preview_goal' ? '#ffb36b' : '#54d18c';
    context.lineWidth = 2;
    context.beginPath();
    context.moveTo(screen.x - 7, screen.y); context.lineTo(screen.x + 7, screen.y);
    context.moveTo(screen.x, screen.y - 7); context.lineTo(screen.x, screen.y + 7);
    context.stroke();
  }
}

function render() {
  const width = canvas.clientWidth;
  const height = canvas.clientHeight;
  context.clearRect(0, 0, width, height);
  if ($('follow').checked) {
    const pose = robotPose();
    if (pose) {
      view.cx += (pose.x - view.cx) * 0.15;
      view.cy += (pose.y - view.cy) * 0.15;
    }
  }
  drawGrid();
  if (state.settings.global_costmap) drawGridMessage('global_costmap/costmap', costmapCanvas, 0.5);
  if (state.settings.local_costmap) drawGridMessage('local_costmap/costmap', costmapCanvas, 0.55);
  if (state.settings.map) drawGridMessage('map', mapCanvas, 1.0);
  if (state.settings.path) drawPath(topic('plan'), '#6aa9ff', 2);
  if (state.settings.local_plan) drawPath(topic('local_plan'), '#54d18c', 1.5);
  drawScan();
  drawGoal();
  drawRobot();
  requestAnimationFrame(render);
}
requestAnimationFrame(render);

canvas.tabIndex = 0;
canvas.focus();
const keys = new Set();
let teleopTimer = null;
function sendKeyboard() {
  let linear = 0;
  let angular = 0;
  if (keys.has('w') || keys.has('arrowup')) linear += 0.18;
  if (keys.has('s') || keys.has('arrowdown')) linear -= 0.12;
  if (keys.has('a') || keys.has('arrowleft')) angular += 0.8;
  if (keys.has('d') || keys.has('arrowright')) angular -= 0.8;
  sendTwist('cmd_vel', linear, angular);
}
window.addEventListener('keydown', (event) => {
  if (event.target instanceof HTMLInputElement || event.target instanceof HTMLSelectElement ||
      event.target instanceof HTMLTextAreaElement) {
    return;
  }
  const key = event.key.toLowerCase();
  if (['w', 'a', 's', 'd', 'arrowup', 'arrowdown', 'arrowleft', 'arrowright', ' '].includes(key)) {
    event.preventDefault();
    keys.add(key);
    if (!teleopTimer) teleopTimer = setInterval(sendKeyboard, 50);
  }
});
window.addEventListener('keyup', (event) => {
  const key = event.key.toLowerCase();
  keys.delete(key);
  if (keys.size === 0 && teleopTimer) {
    clearInterval(teleopTimer);
    teleopTimer = null;
    sendTwist('cmd_vel', 0, 0);
  }
});
window.addEventListener('blur', () => {
  keys.clear();
  if (teleopTimer) { clearInterval(teleopTimer); teleopTimer = null; }
  sendTwist('cmd_vel', 0, 0);
});

async function loadPlugins() {
  let manifest;
  try {
    manifest = await fetch('/plugins.json', { cache: 'no-store' }).then((response) => response.json());
  } catch (error) {
    toast(`plugins.json: ${error.message}`);
    return;
  }
  for (const entry of manifest) {
    try {
      const module = await import(entry.module);
      if (typeof module.init === 'function') await module.init(host, entry);
    } catch (error) {
      console.error(`plugin ${entry.name}`, error);
      toast(`plugin ${entry.name}: ${error.message}`);
    }
  }
}
loadPlugins();
