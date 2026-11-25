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
    urdf_visual: true,
    urdf_collision: false,
    axes: true,
    joint_axes: false,
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

function rotationY(angle) {
  const c = Math.cos(angle);
  const s = Math.sin(angle);
  return [[c, 0, s], [0, 1, 0], [-s, 0, c]];
}
function rotationZ(angle) {
  const c = Math.cos(angle);
  const s = Math.sin(angle);
  return [[c, -s, 0], [s, c, 0], [0, 0, 1]];
}
function multiply(a, b) {
  const out = [[0, 0, 0], [0, 0, 0], [0, 0, 0]];
  for (let row = 0; row < 3; ++row) {
    for (let column = 0; column < 3; ++column) {
      for (let k = 0; k < 3; ++k) out[row][column] += a[row][k] * b[k][column];
    }
  }
  return out;
}
function applyRotation(matrix, vector) {
  return [
    matrix[0][0] * vector[0] + matrix[0][1] * vector[1] + matrix[0][2] * vector[2],
    matrix[1][0] * vector[0] + matrix[1][1] * vector[1] + matrix[1][2] * vector[2],
    matrix[2][0] * vector[0] + matrix[2][1] * vector[1] + matrix[2][2] * vector[2],
  ];
}

// URDF(turtlebot3_real)의 4축 체인을 joint_states로 순방향 계산한다.
// link1 mount -> joint1(z) -> joint2(y) -> joint3(y) -> joint4(y) -> end.
function armPoints() {
  const message = state.latest.get('joint_states');
  const values = new Map();
  if (message && Array.isArray(message.names) && Array.isArray(message.positions)) {
    for (let i = 0; i < message.names.length && i < message.positions.length; ++i) {
      values.set(message.names[i], message.positions[i]);
    }
  }
  const q = [values.get('joint1') || 0, values.get('joint2') || 0,
             values.get('joint3') || 0, values.get('joint4') || 0];
  const points = [];
  let position = [0.047985, -0.00023, 0.130175];
  let rotation = [[1, 0, 0], [0, 1, 0], [0, 0, 1]];
  const advance = (offset) => {
    const moved = applyRotation(rotation, offset);
    position = [position[0] + moved[0], position[1] + moved[1], position[2] + moved[2]];
    points.push({ x: position[0], y: position[1], z: position[2] });
  };
  advance([0.012, 0, 0]);
  rotation = multiply(rotation, rotationZ(q[0]));
  advance([0, 0, 0.0595]);
  rotation = multiply(rotation, rotationY(q[1]));
  advance([0.024, 0, 0.128]);
  rotation = multiply(rotation, rotationY(q[2]));
  advance([0.124, 0, 0]);
  rotation = multiply(rotation, rotationY(q[3]));
  advance([0.126, 0, 0]);
  return points;
}

function mat4Identity() {
  return [[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]];
}
function mat4Multiply(a, b) {
  const out = [[0, 0, 0, 0], [0, 0, 0, 0], [0, 0, 0, 0], [0, 0, 0, 0]];
  for (let row = 0; row < 4; ++row) {
    for (let column = 0; column < 4; ++column) {
      for (let k = 0; k < 4; ++k) out[row][column] += a[row][k] * b[k][column];
    }
  }
  return out;
}
function mat4Translate(x, y, z) {
  return [[1, 0, 0, x], [0, 1, 0, y], [0, 0, 1, z], [0, 0, 0, 1]];
}
function mat4RotX(angle) {
  const c = Math.cos(angle);
  const s = Math.sin(angle);
  return [[1, 0, 0, 0], [0, c, -s, 0], [0, s, c, 0], [0, 0, 0, 1]];
}
function mat4RotY(angle) {
  const c = Math.cos(angle);
  const s = Math.sin(angle);
  return [[c, 0, s, 0], [0, 1, 0, 0], [-s, 0, c, 0], [0, 0, 0, 1]];
}
function mat4RotZ(angle) {
  const c = Math.cos(angle);
  const s = Math.sin(angle);
  return [[c, -s, 0, 0], [s, c, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]];
}
function mat4Rpy(roll, pitch, yaw) {
  return mat4Multiply(mat4Multiply(mat4RotZ(yaw), mat4RotY(pitch)), mat4RotX(roll));
}
function mat4RotAxis(axis, angle) {
  const length = Math.hypot(axis[0], axis[1], axis[2]) || 1;
  const x = axis[0] / length;
  const y = axis[1] / length;
  const z = axis[2] / length;
  const c = Math.cos(angle);
  const s = Math.sin(angle);
  const t = 1 - c;
  return [
    [t * x * x + c, t * x * y - s * z, t * x * z + s * y, 0],
    [t * x * y + s * z, t * y * y + c, t * y * z - s * x, 0],
    [t * x * z - s * y, t * y * z + s * x, t * z * z + c, 0],
    [0, 0, 0, 1],
  ];
}
function mat4Apply(matrix, point) {
  return [
    matrix[0][0] * point[0] + matrix[0][1] * point[1] + matrix[0][2] * point[2] + matrix[0][3],
    matrix[1][0] * point[0] + matrix[1][1] * point[1] + matrix[1][2] * point[2] + matrix[1][3],
    matrix[2][0] * point[0] + matrix[2][1] * point[1] + matrix[2][2] * point[2] + matrix[2][3],
  ];
}

let urdfCache = null;
let urdfSource = '';

function parseUrdfVector(text, fallback) {
  if (!text) return fallback;
  const values = text.trim().split(/\s+/).map(Number);
  return values.length >= fallback.length && values.every(Number.isFinite) ? values : fallback;
}
function parseUrdfOrigin(element) {
  const origin = element.getElementsByTagName('origin')[0];
  return {
    xyz: parseUrdfVector(origin?.getAttribute('xyz'), [0, 0, 0]),
    rpy: parseUrdfVector(origin?.getAttribute('rpy'), [0, 0, 0]),
  };
}
function parseUrdfGeometry(element) {
  const geometry = element.getElementsByTagName('geometry')[0];
  if (!geometry) return { type: 'none' };
  const box = geometry.getElementsByTagName('box')[0];
  if (box) {
    return { type: 'box', size: parseUrdfVector(box.getAttribute('size'), [0.01, 0.01, 0.01]) };
  }
  const cylinder = geometry.getElementsByTagName('cylinder')[0];
  if (cylinder) {
    return {
      type: 'cylinder',
      radius: Number(cylinder.getAttribute('radius') || 0.01),
      length: Number(cylinder.getAttribute('length') || 0.01),
    };
  }
  const sphere = geometry.getElementsByTagName('sphere')[0];
  if (sphere) {
    return { type: 'sphere', radius: Number(sphere.getAttribute('radius') || 0.01) };
  }
  return { type: 'mesh' };
}

function urdfModel() {
  const description = state.latest.get('robot_description');
  const text = description?.data;
  if (!text) return null;
  if (urdfCache && urdfSource === text) return urdfCache;
  const document = new DOMParser().parseFromString(text, 'text/xml');
  const clean = (value) => String(value || '').replace(/\$\{namespace\}/g, '');
  const links = new Map();
  for (const link of document.getElementsByTagName('link')) {
    const name = clean(link.getAttribute('name'));
    const collect = (tag) => {
      const result = [];
      for (const item of link.getElementsByTagName(tag)) {
        result.push({ origin: parseUrdfOrigin(item), ...parseUrdfGeometry(item) });
      }
      return result;
    };
    links.set(name, { collisions: collect('collision'), visuals: collect('visual') });
  }
  const joints = [];
  for (const joint of document.getElementsByTagName('joint')) {
    const mimicElement = joint.getElementsByTagName('mimic')[0];
    joints.push({
      name: clean(joint.getAttribute('name')),
      type: joint.getAttribute('type') || 'fixed',
      parent: clean(joint.getElementsByTagName('parent')[0]?.getAttribute('link')),
      child: clean(joint.getElementsByTagName('child')[0]?.getAttribute('link')),
      origin: parseUrdfOrigin(joint),
      axis: parseUrdfVector(joint.getElementsByTagName('axis')[0]?.getAttribute('xyz'), [0, 0, 1]),
      mimic: mimicElement ? clean(mimicElement.getAttribute('joint')) : null,
      multiplier: Number(mimicElement?.getAttribute('multiplier') || 1),
    });
  }
  urdfSource = text;
  urdfCache = { links, joints };
  return urdfCache;
}

function urdfTransforms() {
  const model = urdfModel();
  if (!model) return null;
  const message = state.latest.get('joint_states');
  const positions = new Map();
  if (message && Array.isArray(message.names) && Array.isArray(message.positions)) {
    for (let i = 0; i < message.names.length && i < message.positions.length; ++i) {
      positions.set(message.names[i], message.positions[i]);
    }
  }
  const byParent = new Map();
  for (const joint of model.joints) {
    if (!joint.parent || !joint.child) continue;
    if (!byParent.has(joint.parent)) byParent.set(joint.parent, []);
    byParent.get(joint.parent).push(joint);
  }
  const transforms = new Map();
  const visit = (link, transform, depth) => {
    if (depth > 64 || transforms.has(link)) return;
    transforms.set(link, transform);
    for (const joint of byParent.get(link) || []) {
      let position = positions.get(joint.name);
      if (!Number.isFinite(position)) {
        position = joint.mimic ? (positions.get(joint.mimic) || 0) * joint.multiplier : 0;
      }
      let local = mat4Multiply(
          mat4Translate(joint.origin.xyz[0], joint.origin.xyz[1], joint.origin.xyz[2]),
          mat4Rpy(joint.origin.rpy[0], joint.origin.rpy[1], joint.origin.rpy[2]));
      if (joint.type === 'revolute' || joint.type === 'continuous') {
        local = mat4Multiply(local, mat4RotAxis(joint.axis, position));
      } else if (joint.type === 'prismatic') {
        local = mat4Multiply(local, mat4Translate(
            joint.axis[0] * position, joint.axis[1] * position, joint.axis[2] * position));
      }
      visit(joint.child, mat4Multiply(transform, local), depth + 1);
    }
  };
  visit('base_link', mat4Identity(), 0);
  return transforms;
}

function urdfPrimitives(filter, useVisual = false) {
  const model = urdfModel();
  const transforms = urdfTransforms();
  if (!model || !transforms) return [];
  const primitives = [];
  for (const [name, link] of model.links) {
    if (filter && !filter(name)) continue;
    const linkTransform = transforms.get(name);
    if (!linkTransform) continue;
    const visual = link.visuals.some((item) => item.type !== 'mesh' && item.type !== 'none');
    const source = useVisual && visual ? link.visuals : link.collisions;
    for (const item of source) {
      if (item.type === 'none' || item.type === 'mesh') continue;
      const transform = mat4Multiply(
          linkTransform,
          mat4Multiply(
              mat4Translate(item.origin.xyz[0], item.origin.xyz[1], item.origin.xyz[2]),
              mat4Rpy(item.origin.rpy[0], item.origin.rpy[1], item.origin.rpy[2])));
      if (item.type === 'box') {
        const half = item.size.map((value) => Math.max(0.001, value * 0.5));
        const corners = [];
        for (const sx of [-1, 1]) {
          for (const sy of [-1, 1]) {
            for (const sz of [-1, 1]) {
              corners.push(mat4Apply(transform, [sx * half[0], sy * half[1], sz * half[2]]));
            }
          }
        }
        primitives.push({ name, type: 'box', corners });
      } else if (item.type === 'cylinder') {
        primitives.push({
          name, type: 'cylinder', transform,
          radius: Math.max(0.002, item.radius), length: Math.max(0.002, item.length),
        });
      } else if (item.type === 'sphere') {
        primitives.push({
          name, type: 'sphere', center: mat4Apply(transform, [0, 0, 0]),
          radius: Math.max(0.002, item.radius),
        });
      }
    }
  }
  return primitives;
}

function urdfBoxes(filter) {
  return urdfPrimitives(filter, false).filter((item) => item.type === 'box');
}

function isArmLink(name) {
  return name === 'end_effector_link' || /^link[1-5]$/.test(name) || name.startsWith('gripper');
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
function sendJointState(topicName, names, positions) {
  const writer = new Writer();
  writer.f64(Date.now() / 1000);
  writer.str('');
  writer.u32(names.length);
  for (const name of names) writer.str(name);
  writer.u32(positions.length);
  for (const position of positions) writer.f64(position);
  writer.u32(0);
  writer.u32(0);
  return sendBinary(topicName, 7, writer.data);
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

let currentPluginView = null;
const viewRegistry = new Map();
let activeView = null;

function activateView(id) {
  const selected = viewRegistry.get(id);
  if (!selected) {
    return;
  }
  activeView = id;
  for (const item of viewRegistry.values()) {
    item.tab.classList.toggle('active', item.id === id);
    item.panel.classList.toggle('active', item.id === id);
  }
  $('hint').textContent = id === '3d'
    ? 'left drag orbit · right/shift drag pan · wheel zoom · WASD teleop'
    : id === 'moveit'
      ? 'TCP 마커 드래그 X/Y · Shift+Z · Ctrl+Pitch · Plan/Execute는 control-panel'
      : 'drag pan · wheel zoom · click 2D goal · WASD teleop';
  const tools = $('view-tools');
  if (tools) {
    tools.style.display = id === '3d' || id === 'moveit' ? 'flex' : 'none';
  }
  syncViewTools();
  if (typeof resizeCanvas === 'function') {
    resizeCanvas();
  }
}

const viewToolDefs = [
  ['Visual', 'urdf_visual'],
  ['Collision', 'urdf_collision'],
  ['Axes', 'axes'],
  ['Joint Axes', 'joint_axes'],
  ['Grid', 'grid'],
  ['Scan', 'scan'],
];

function syncViewTools() {
  for (const button of document.querySelectorAll('.view-tool')) {
    button.classList.toggle('active', Boolean(state.settings[button.dataset.setting]));
  }
}

function applyViewSetting(key, value) {
  state.settings[key] = value;
  const checkbox = document.querySelector(`#displays input[data-key="${key}"]`);
  if (checkbox) {
    checkbox.checked = value;
  }
  syncViewTools();
}

function buildViewTools() {
  const bar = $('view-tools');
  if (!bar) return;
  for (const [label, key] of viewToolDefs) {
    const button = document.createElement('button');
    button.type = 'button';
    button.className = 'view-tool';
    button.dataset.setting = key;
    button.textContent = label;
    button.addEventListener('click', () => applyViewSetting(key, !state.settings[key]));
    bar.append(button);
  }
  syncViewTools();
}

const host = {
  onTopic,
  latest: topic,
  sendString,
  sendBool,
  sendEmpty,
  sendTwist,
  sendJointState,
  sendGoal,
  setGoalTopic(topicName) { state.goalTopic = topicName; },
  toast,
  robotPose,
  lookup,
  armPoints,
  urdfBoxes,
  urdfPrimitives,
  isArmLink,
  settings: state.settings,
  addView({ id, title }) {
    const viewId = id || title;
    if (!viewId) {
      throw new Error('addView: id가 필요하다');
    }
    if (viewRegistry.has(viewId)) {
      return viewRegistry.get(viewId);
    }
    const tab = document.createElement('button');
    tab.type = 'button';
    tab.className = 'view-tab';
    tab.dataset.view = viewId;
    tab.textContent = title || viewId;
    const panel = document.createElement('div');
    panel.className = 'view-panel';
    $('view-tabs').append(tab);
    $('view-panels').append(panel);
    const view = {
      id: viewId,
      tab,
      panel,
      activate() { activateView(viewId); },
    };
    tab.addEventListener('click', view.activate);
    viewRegistry.set(viewId, view);
    return view;
  },
  addPanel(title) {
    const section = document.createElement('section');
    section.className = 'panel';
    const heading = document.createElement('h2');
    heading.textContent = title;
    const bodyElement = document.createElement('div');
    bodyElement.className = 'body';
    section.append(heading, bodyElement);
    (currentPluginView ?? $('control-panel-panels')).append(section);
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
  ['urdf_visual', 'URDF visual'],
  ['urdf_collision', 'URDF collision'],
  ['tf', 'TF axes'],
];

for (const [key, label] of displays) {
  const wrapper = document.createElement('label');
  const checkbox = document.createElement('input');
  checkbox.type = 'checkbox';
  checkbox.checked = state.settings[key];
  checkbox.dataset.key = key;
  checkbox.addEventListener('change', () => {
    state.settings[key] = checkbox.checked;
    syncViewTools();
  });
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

// canvas/view composition: 플러그인(내장 뷰 포함)이 addView로 탭을 만든다.
const mapView = host.addView({ id: 'map', title: 'Map' });
const canvas = document.createElement('canvas');
canvas.id = 'scene';
mapView.panel.append(canvas);
const context = canvas.getContext('2d');

const view3d = host.addView({ id: '3d', title: '3D' });
const canvas3d = document.createElement('canvas');
canvas3d.id = 'scene3d';
view3d.panel.append(canvas3d);
const context3d = canvas3d.getContext('2d');

const view = { cx: 0, cy: 0, scale: 60 };
const camera3d = { yaw: -0.55, pitch: 0.62, distance: 2.4, tx: 0, ty: 0 };
let dragging = false;
let dragged = false;
let dragStart = { x: 0, y: 0 };
let lastPointer = { x: 0, y: 0 };

function resizeCanvas() {
  const ratio = window.devicePixelRatio || 1;
  for (const [element, elementContext] of [[canvas, context], [canvas3d, context3d]]) {
    element.width = Math.max(1, Math.floor(element.clientWidth * ratio));
    element.height = Math.max(1, Math.floor(element.clientHeight * ratio));
    elementContext.setTransform(ratio, 0, 0, ratio, 0, 0);
  }
}
window.addEventListener('resize', resizeCanvas);
resizeCanvas();
buildViewTools();
activateView('map');

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
        pixels[index] = 150; pixels[index + 1] = 160; pixels[index + 2] = 172; pixels[index + 3] = 255;
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
    pixels[offset] = Math.round(150 * (1 - intensity));
    pixels[offset + 1] = Math.round(220 * intensity);
    pixels[offset + 2] = 255;
    pixels[offset + 3] = Math.round(25 + 125 * intensity);
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
  context.fillStyle = 'rgba(0, 230, 255, 0.85)';
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

function project3d(x, y, z) {
  const width = canvas3d.clientWidth;
  const height = canvas3d.clientHeight;
  if (width < 2 || height < 2) return null;
  const dx = x - camera3d.tx;
  const dy = y - camera3d.ty;
  const cy = Math.cos(camera3d.yaw);
  const sy = Math.sin(camera3d.yaw);
  const rx = cy * dx + sy * dy;
  const ry = -sy * dx + cy * dy;
  const cp = Math.cos(camera3d.pitch);
  const sp = Math.sin(camera3d.pitch);
  const depth = ry * sp - z * cp + camera3d.distance;
  if (depth <= 0.08) return null;
  const scale = Math.min(width, height) * 0.95 / depth;
  return {
    x: width * 0.5 + rx * scale,
    y: height * 0.56 - (ry * cp + z * sp) * scale,
    depth,
  };
}

function draw3dGrid() {
  context3d.lineWidth = 1;
  const extent = 5;
  for (let i = -extent; i <= extent; ++i) {
    const a = project3d(i, -extent, 0);
    const b = project3d(i, extent, 0);
    const c = project3d(-extent, i, 0);
    const d = project3d(extent, i, 0);
    context3d.strokeStyle = i === 0 ? '#2a3850' : '#141c27';
    if (a && b) {
      context3d.beginPath();
      context3d.moveTo(a.x, a.y);
      context3d.lineTo(b.x, b.y);
      context3d.stroke();
    }
    if (c && d) {
      context3d.beginPath();
      context3d.moveTo(c.x, c.y);
      context3d.lineTo(d.x, d.y);
      context3d.stroke();
    }
  }
}

function draw3dScan() {
  if (!state.settings.scan) return;
  const scan = topic('scan');
  if (!scan) return;
  const transform = frameTransform(scan.header.frame);
  if (!transform) return;
  context3d.fillStyle = 'rgba(0, 230, 255, 0.85)';
  const count = scan.ranges.length;
  const stride = count > 720 ? 2 : 1;
  const cy = Math.cos(transform.yaw);
  const sy = Math.sin(transform.yaw);
  for (let index = 0; index < count; index += stride) {
    const range = scan.ranges[index];
    if (!Number.isFinite(range) || range < scan.range_min || range > scan.range_max) continue;
    const angle = scan.angle_min + index * scan.angle_increment;
    const localX = Math.cos(angle) * range;
    const localY = Math.sin(angle) * range;
    const worldX = transform.x + cy * localX - sy * localY;
    const worldY = transform.y + sy * localX + cy * localY;
    const point = project3d(worldX, worldY, 0.05);
    if (point) {
      const size = Math.max(1, Math.min(3, 90 / point.depth));
      context3d.fillRect(point.x - size * 0.5, point.y - size * 0.5, size, size);
    }
  }
}

function draw3dBox(cx, cy, yaw, width, length, height, color) {
  const c = Math.cos(yaw);
  const s = Math.sin(yaw);
  const corners = [
    [-width / 2, -length / 2], [width / 2, -length / 2],
    [width / 2, length / 2], [-width / 2, length / 2],
  ];
  const bottom = corners.map(([x, y]) => project3d(cx + c * x - s * y, cy + s * x + c * y, 0));
  const top = corners.map(([x, y]) => project3d(cx + c * x - s * y, cy + s * x + c * y, height));
  context3d.strokeStyle = color;
  context3d.lineWidth = 1.5;
  for (let i = 0; i < 4; ++i) {
    const a = bottom[i];
    const b = bottom[(i + 1) % 4];
    const t = top[i];
    if (a && b) {
      context3d.beginPath(); context3d.moveTo(a.x, a.y); context3d.lineTo(b.x, b.y); context3d.stroke();
    }
    if (a && t) {
      context3d.beginPath(); context3d.moveTo(a.x, a.y); context3d.lineTo(t.x, t.y); context3d.stroke();
    }
  }
  context3d.fillStyle = 'rgba(160, 180, 200, 0.18)';
  context3d.beginPath();
  let started = false;
  for (const point of top) {
    if (!point) continue;
    if (!started) { context3d.moveTo(point.x, point.y); started = true; }
    else context3d.lineTo(point.x, point.y);
  }
  if (started) {
    context3d.closePath();
    context3d.fill();
  }
}

function projectBasePoint(point, transform, c, s) {
  return project3d(transform.x + c * point[0] - s * point[1],
                   transform.y + s * point[0] + c * point[1], point[2]);
}

function draw3dPrimitive(primitive, transform) {
  const c = Math.cos(transform.yaw);
  const s = Math.sin(transform.yaw);
  if (primitive.type === 'box') {
    const edges = [[0, 1], [0, 2], [0, 4], [1, 3], [1, 5], [2, 3], [2, 6], [3, 7],
                   [4, 5], [4, 6], [5, 7], [6, 7]];
    const projected = primitive.corners.map((corner) => projectBasePoint(corner, transform, c, s));
    if (projected.some((point) => point === null)) return;
    context3d.strokeStyle = '#b8c2d2';
    context3d.lineWidth = 1;
    context3d.beginPath();
    for (const [a, b] of edges) {
      context3d.moveTo(projected[a].x, projected[a].y);
      context3d.lineTo(projected[b].x, projected[b].y);
    }
    context3d.stroke();
    context3d.fillStyle = 'rgba(150, 165, 185, 0.12)';
    context3d.beginPath();
    context3d.moveTo(projected[0].x, projected[0].y);
    for (const point of projected.slice(1)) context3d.lineTo(point.x, point.y);
    context3d.closePath();
    context3d.fill();
  } else if (primitive.type === 'cylinder') {
    const segments = 14;
    const top = [];
    const bottom = [];
    for (let i = 0; i < segments; ++i) {
      const angle = (i / segments) * Math.PI * 2;
      const x = Math.cos(angle) * primitive.radius;
      const y = Math.sin(angle) * primitive.radius;
      top.push(projectBasePoint(mat4Apply(primitive.transform, [x, y, primitive.length / 2]), transform, c, s));
      bottom.push(projectBasePoint(mat4Apply(primitive.transform, [x, y, -primitive.length / 2]), transform, c, s));
    }
    if (top.some((point) => point === null) || bottom.some((point) => point === null)) return;
    context3d.strokeStyle = '#b8c2d2';
    context3d.lineWidth = 1;
    context3d.beginPath();
    for (let i = 0; i < segments; ++i) {
      const next = (i + 1) % segments;
      context3d.moveTo(top[i].x, top[i].y); context3d.lineTo(top[next].x, top[next].y);
      context3d.moveTo(bottom[i].x, bottom[i].y); context3d.lineTo(bottom[next].x, bottom[next].y);
      context3d.moveTo(top[i].x, top[i].y); context3d.lineTo(bottom[i].x, bottom[i].y);
    }
    context3d.stroke();
  } else if (primitive.type === 'sphere') {
    const center = projectBasePoint(primitive.center, transform, c, s);
    if (!center) return;
    const edge = projectBasePoint(
        [primitive.center[0] + primitive.radius, primitive.center[1], primitive.center[2]], transform, c, s);
    const radius = edge ? Math.hypot(edge.x - center.x, edge.y - center.y) : 2;
    context3d.strokeStyle = '#b8c2d2';
    context3d.beginPath();
    context3d.arc(center.x, center.y, Math.max(1, radius), 0, Math.PI * 2);
    context3d.stroke();
  }
}

function draw3dRobot() {
  if (!state.settings.robot) return;
  const pose = robotPose();
  const transform = frameTransform('base_link') || { x: 0, y: 0, yaw: 0 };
  const c = Math.cos(transform.yaw);
  const s = Math.sin(transform.yaw);
  if (state.settings.urdf_visual) {
    const basePrimitives = urdfPrimitives((name) => !isArmLink(name), true);
    if (basePrimitives.length === 0 && pose) {
      draw3dBox(pose.x, pose.y, pose.yaw, 0.28, 0.24, 0.13, '#7f899b');
    } else {
      for (const primitive of basePrimitives) {
        draw3dPrimitive(primitive, transform);
      }
    }
    const points = [];
    for (const point of armPoints()) {
      const projected = project3d(transform.x + c * point.x - s * point.y,
                                 transform.y + s * point.x + c * point.y, point.z);
      if (projected) points.push(projected);
    }
    if (points.length > 1) {
      context3d.strokeStyle = '#d7dde8';
      context3d.lineWidth = 2;
      context3d.beginPath();
      context3d.moveTo(points[0].x, points[0].y);
      for (let i = 1; i < points.length; ++i) context3d.lineTo(points[i].x, points[i].y);
      context3d.stroke();
      context3d.fillStyle = '#9aa6b9';
      for (const point of points) {
        context3d.beginPath();
        context3d.arc(point.x, point.y, 3, 0, Math.PI * 2);
        context3d.fill();
      }
    }
  }
  if (state.settings.urdf_collision) {
    for (const primitive of urdfPrimitives(isArmLink, false)) {
      draw3dPrimitive(primitive, transform);
    }
  }
}

function draw3dArm() {
  const arm = armPoints();
  if (arm.length === 0) return;
  const transform = frameTransform('base_link') || { x: 0, y: 0, yaw: 0 };
  const c = Math.cos(transform.yaw);
  const s = Math.sin(transform.yaw);
  const tcp = arm[arm.length - 1];
  const point = project3d(transform.x + c * tcp.x - s * tcp.y,
                          transform.y + s * tcp.x + c * tcp.y, tcp.z);
  if (!point) return;
  context3d.fillStyle = '#ffb36b';
  context3d.beginPath();
  context3d.arc(point.x, point.y, 5, 0, Math.PI * 2);
  context3d.fill();
}

function draw3dGoal() {
  const goal = topic('goal_pose');
  if (!goal) return;
  const transform = frameTransform(goal.header.frame);
  if (!transform) return;
  const x = transform.x + Math.cos(transform.yaw) * goal.pose.position.x -
            Math.sin(transform.yaw) * goal.pose.position.y;
  const y = transform.y + Math.sin(transform.yaw) * goal.pose.position.x +
            Math.cos(transform.yaw) * goal.pose.position.y;
  const bottom = project3d(x, y, 0);
  const top = project3d(x, y, 0.5);
  if (bottom && top) {
    context3d.strokeStyle = '#54d18c';
    context3d.lineWidth = 2;
    context3d.beginPath();
    context3d.moveTo(bottom.x, bottom.y);
    context3d.lineTo(top.x, top.y);
    context3d.stroke();
    context3d.fillStyle = '#54d18c';
    context3d.beginPath();
    context3d.arc(top.x, top.y, 4, 0, Math.PI * 2);
    context3d.fill();
  }
}

function draw3dAxes() {
  if (!state.settings.axes) return;
  const origin = project3d(0, 0, 0);
  if (!origin) return;
  const axes = [
    [project3d(0.6, 0, 0), '#ff6b6b'],
    [project3d(0, 0.6, 0), '#54d18c'],
    [project3d(0, 0, 0.6), '#6aa9ff'],
  ];
  context3d.lineWidth = 2;
  for (const [point, color] of axes) {
    if (!point) continue;
    context3d.strokeStyle = color;
    context3d.beginPath();
    context3d.moveTo(origin.x, origin.y);
    context3d.lineTo(point.x, point.y);
    context3d.stroke();
  }
}

function draw3dJointAxes() {
  if (!state.settings.joint_axes) return;
  const transform = frameTransform('base_link') || { x: 0, y: 0, yaw: 0 };
  const c = Math.cos(transform.yaw);
  const s = Math.sin(transform.yaw);
  const toWorld = (point) => ({
    x: transform.x + c * point.x - s * point.y,
    y: transform.y + s * point.x + c * point.y,
    z: point.z,
  });
  for (const point of armPoints()) {
    const origin = project3d(toWorld(point).x, toWorld(point).y, point.z);
    if (!origin) continue;
    const axes = [
      [project3d(toWorld({ x: point.x + 0.06, y: point.y, z: point.z }).x,
                 toWorld({ x: point.x + 0.06, y: point.y, z: point.z }).y, point.z), '#ff6b6b'],
      [project3d(toWorld({ x: point.x, y: point.y + 0.06, z: point.z }).x,
                 toWorld({ x: point.x, y: point.y + 0.06, z: point.z }).y, point.z), '#54d18c'],
      [project3d(toWorld(point).x, toWorld(point).y, point.z + 0.06), '#6aa9ff'],
    ];
    context3d.lineWidth = 1.5;
    for (const [end, color] of axes) {
      if (!end) continue;
      context3d.strokeStyle = color;
      context3d.beginPath();
      context3d.moveTo(origin.x, origin.y);
      context3d.lineTo(end.x, end.y);
      context3d.stroke();
    }
  }
}

function draw3d() {
  const width = canvas3d.clientWidth;
  const height = canvas3d.clientHeight;
  if (width < 2 || height < 2) return;
  context3d.clearRect(0, 0, width, height);
  if ($('follow').checked) {
    const pose = robotPose();
    if (pose) {
      camera3d.tx += (pose.x - camera3d.tx) * 0.05;
      camera3d.ty += (pose.y - camera3d.ty) * 0.05;
    }
  }
  draw3dGrid();
  draw3dAxes();
  draw3dScan();
  draw3dRobot();
  draw3dJointAxes();
  draw3dArm();
  draw3dGoal();
}

let drag3d = false;
let drag3dMode = 'orbit';
let drag3dLast = { x: 0, y: 0 };
canvas3d.addEventListener('contextmenu', (event) => event.preventDefault());
canvas3d.addEventListener('pointerdown', (event) => {
  drag3d = true;
  drag3dMode = event.button === 2 || event.shiftKey ? 'pan' : 'orbit';
  drag3dLast = { x: event.clientX, y: event.clientY };
  canvas3d.setPointerCapture(event.pointerId);
});
canvas3d.addEventListener('pointermove', (event) => {
  if (!drag3d) return;
  const dx = event.clientX - drag3dLast.x;
  const dy = event.clientY - drag3dLast.y;
  drag3dLast = { x: event.clientX, y: event.clientY };
  if (drag3dMode === 'pan') {
    const step = camera3d.distance * 0.0016;
    const cy = Math.cos(camera3d.yaw);
    const sy = Math.sin(camera3d.yaw);
    camera3d.tx -= (cy * dx + sy * dy) * step;
    camera3d.ty -= (sy * dx - cy * dy) * step;
    $('follow').checked = false;
  } else {
    camera3d.yaw -= dx * 0.008;
    camera3d.pitch = Math.max(0.12, Math.min(1.25, camera3d.pitch - dy * 0.008));
  }
});
canvas3d.addEventListener('pointerup', () => { drag3d = false; });
canvas3d.addEventListener('dblclick', () => {
  camera3d.tx = 0;
  camera3d.ty = 0;
  camera3d.distance = 2.4;
  camera3d.yaw = -0.55;
  camera3d.pitch = 0.62;
  $('follow').checked = true;
});
canvas3d.addEventListener('wheel', (event) => {
  event.preventDefault();
  camera3d.distance = Math.max(0.6, Math.min(12.0, camera3d.distance * (event.deltaY < 0 ? 0.9 : 1.1)));
}, { passive: false });

function render() {
  if (activeView === '3d') {
    draw3d();
    requestAnimationFrame(render);
    return;
  }
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
  const tabs = $('control-panel-tabs');
  const panels = $('control-panel-panels');
  const views = [];
  const activate = (selected) => {
    for (const item of views) {
      const active = item === selected;
      item.tab.classList.toggle('active', active);
      item.view.classList.toggle('active', active);
      item.tab.setAttribute('aria-selected', active ? 'true' : 'false');
    }
  };

  for (const entry of manifest) {
    const label = entry.title || entry.name;
    const tab = document.createElement('button');
    tab.type = 'button';
    tab.className = 'plugin-tab';
    tab.textContent = label;
    tab.title = label;
    tab.setAttribute('role', 'tab');
    const view = document.createElement('div');
    view.className = 'plugin-view';
    view.setAttribute('role', 'tabpanel');
    tabs.append(tab);
    panels.append(view);

    const previous = currentPluginView;
    currentPluginView = view;
    try {
      const module = await import(entry.module);
      if (typeof module.init === 'function') await module.init(host, entry);
    } catch (error) {
      console.error(`plugin ${entry.name}`, error);
      toast(`plugin ${entry.name}: ${error.message}`);
    } finally {
      currentPluginView = previous;
    }

    if (view.childElementCount === 0) {
      tab.remove();
      view.remove();
      continue;
    }
    const item = { tab, view };
    views.push(item);
    tab.addEventListener('click', () => activate(item));
  }

  if (views.length > 0) {
    activate(views[0]);
  }
}
loadPlugins();
