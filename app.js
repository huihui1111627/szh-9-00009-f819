'use strict';

const TANK_CAPACITY = 100;
const HULL_HALF_LENGTH = 45;
const HULL_HALF_BEAM = 25;
const CRANE_BASE = { x: -30, y: 0 };
const BOOM_RADIUS = 18;
const SWL_LIMIT = 20;
const SAFE_PUMP_RATE = 12;
const ROLL_LIMIT = 6;
const PITCH_LIMIT = 6;
const TENSION_LIMIT = 850;
const WARNING_TENSION = 720;

const tankSeed = { FP: 48, FS: 45, AP: 50, AS: 47 };
const tankDefs = [
  { id: 'FP', name: '左前压载舱', x: 22, y: 14 },
  { id: 'FS', name: '右前压载舱', x: 22, y: -14 },
  { id: 'AP', name: '左后压载舱', x: -22, y: 14 },
  { id: 'AS', name: '右后压载舱', x: -22, y: -14 }
];

const deckLoadsSeed = [
  { id: 'pipe', name: '铺管段', x: 18, y: 0, mass: 16, w: 18, h: 8 },
  { id: 'generator', name: '发电机组', x: -8, y: 14, mass: 14, w: 10, h: 8 },
  { id: 'container', name: '工具集装箱', x: -12, y: -14, mass: 14, w: 12, h: 8 }
];

const restrictedZones = [
  { id: 'helideck', name: '直升机平台禁区', x: 26, y: -14, radius: 8 },
  { id: 'wellhead', name: '井口设备安全区', x: 8, y: -8, radius: 7 },
  { id: 'hot', name: '热介质设备安全区', x: -8, y: 13, radius: 8 }
];

const mooringDefs = [
  { id: 'M1', name: '左艏系泊缆', fairlead: { x: 40, y: 20 }, anchor: { x: 86, y: 56 }, rest: 58 },
  { id: 'M2', name: '右艏系泊缆', fairlead: { x: 40, y: -20 }, anchor: { x: 86, y: -56 }, rest: 58 },
  { id: 'M3', name: '左艉系泊缆', fairlead: { x: -40, y: 20 }, anchor: { x: -86, y: 56 }, rest: 58 },
  { id: 'M4', name: '右艉系泊缆', fairlead: { x: -40, y: -20 }, anchor: { x: -86, y: -56 }, rest: 58 }
];

const state = {
  running: true,
  simTime: 0,
  lastRiskCheck: -99,
  env: {
    windSpeed: 12,
    waveHeight: 2.4,
    waveDir: 135,
    currentSpeed: 0.8
  },
  attitude: {
    roll: 0,
    pitch: 0,
    yaw: 0,
    rollVel: 0,
    pitchVel: 0,
    yawVel: 0,
    heave: 0,
    posX: 0,
    posY: 0
  },
  tanks: tankDefs.map((tank) => ({ ...tank, volume: tankSeed[tank.id] })),
  deckLoads: deckLoadsSeed.map((load) => ({ ...load })),
  crane: {
    angle: 315,
    targetAngle: 270,
    height: 0,
    load: 8,
    suspended: false,
    activeCommandId: null,
    paused: false
  },
  commands: [],
  logs: [],
  commandSeq: 1,
  prediction: null
};

const $ = (id) => document.getElementById(id);
const clamp = (value, min, max) => Math.min(max, Math.max(min, value));
const degToRad = (deg) => deg * Math.PI / 180;
const radToDeg = (rad) => rad * 180 / Math.PI;
const angleDiff = (from, to) => ((to - from + 540) % 360) - 180;
const fmt = (value, digits = 1) => Number(value).toFixed(digits);
const clockText = (seconds) => {
  const mins = Math.floor(seconds / 60).toString().padStart(2, '0');
  const secs = Math.floor(seconds % 60).toString().padStart(2, '0');
  return `T+${mins}:${secs}`;
};

function cloneTanks(tanks = state.tanks) {
  return tanks.map((tank) => ({ ...tank }));
}

function getTank(id, tanks = state.tanks) {
  return tanks.find((tank) => tank.id === id);
}

function hookPosition(angle = state.crane.angle) {
  const rad = degToRad(angle);
  return {
    x: CRANE_BASE.x + BOOM_RADIUS * Math.cos(rad),
    y: CRANE_BASE.y + BOOM_RADIUS * Math.sin(rad)
  };
}

function signedAngleDelta(command, currentAngle = state.crane.angle) {
  if (command.kind !== 'slew') return 0;
  return angleDiff(currentAngle, command.targetAngle);
}

function commandLabel(command) {
  if (command.kind === 'ballast') {
    return `${getTank(command.from).name} → ${getTank(command.to).name}`;
  }
  if (command.kind === 'lift') return `起吊 ${fmt(command.load, 1)} t 货物`;
  if (command.kind === 'slew') return `吊臂回转至 ${Math.round(command.targetAngle)}°`;
  if (command.kind === 'pause') return '操作员暂停吊装';
  return command.kind;
}

function commandProgress(command) {
  if (command.kind === 'ballast') return clamp(command.transferred / command.amount, 0, 1);
  if (command.kind === 'lift') return clamp(command.height / command.targetHeight, 0, 1);
  if (command.kind === 'slew') {
    const total = Math.abs(angleDiff(command.startAngle, command.targetAngle));
    return total === 0 ? 1 : 1 - Math.abs(signedAngleDelta(command)) / total;
  }
  return 1;
}

function activeCommands(kind = null) {
  return state.commands.filter((command) => {
    const active = command.status === 'active' || command.status === 'paused';
    return active && (!kind || command.kind === kind);
  });
}

function frozenCommands() {
  return state.commands.filter((command) => command.status === 'frozen');
}

function environmentalForce(env = state.env, time = state.simTime) {
  const rad = degToRad(env.waveDir);
  const wind = env.windSpeed * env.windSpeed * 0.045;
  const wave = env.waveHeight * 28;
  const current = env.currentSpeed * 48;
  const gust = 1 + 0.08 * Math.sin(time * 1.37) + 0.035 * Math.sin(time * 2.91);
  const magnitude = (wind + wave + current) * gust;
  return {
    magnitude,
    x: magnitude * Math.cos(rad),
    y: magnitude * Math.sin(rad),
    rad
  };
}

function loadMoments(tanks = state.tanks, crane = state.crane) {
  let rollMoment = 0;
  let pitchMoment = 0;
  tanks.forEach((tank) => {
    rollMoment += tank.volume * tank.y * 0.9;
    pitchMoment += tank.volume * tank.x * 0.9;
  });
  state.deckLoads.forEach((load) => {
    rollMoment += load.mass * load.y * 2.2;
    pitchMoment += load.mass * load.x * 2.2;
  });
  if (crane.suspended) {
    const hook = hookPosition(crane.angle);
    rollMoment += crane.load * hook.y * 2.8;
    pitchMoment += crane.load * hook.x * 2.8;
  }
  return { rollMoment, pitchMoment };
}

function equilibriumAngles(sim = state) {
  const env = environmentalForce(sim.env, sim.simTime);
  const load = loadMoments(sim.tanks, sim.crane);
  const envRoll = env.y * 13;
  const envPitch = env.x * 13;
  const wavePhase = sim.simTime * Math.PI * 0.55 + degToRad(sim.env.waveDir);
  return {
    roll: (load.rollMoment + envRoll) / 18000 + sim.env.waveHeight * 0.22 * Math.sin(wavePhase),
    pitch: (load.pitchMoment + envPitch) / 18000 + sim.env.waveHeight * 0.17 * Math.cos(wavePhase * 0.9),
    yaw: 3.1 * Math.sin(degToRad(sim.env.waveDir - 45)) + sim.env.waveHeight * 0.18 * Math.sin(wavePhase * 0.7)
  };
}

function calculateTensions(sim = state) {
  return mooringDefs.map((line) => {
    const fx = line.fairlead.x + sim.attitude.posX;
    const fy = line.fairlead.y + sim.attitude.posY;
    const dx = line.anchor.x - fx;
    const dy = line.anchor.y - fy;
    const length = Math.hypot(dx, dy);
    const tension = Math.round(420 + Math.max(0, length - 58) * 150);
    const share = clamp((Math.max(0, dx * sim.attitude.posX + dy * sim.attitude.posY) / length) * 18, 0, 90);
    return {
      ...line,
      tension: Math.round(tension + share),
      length,
      utilization: clamp((tension + share) / TENSION_LIMIT, 0, 1.1)
    };
  });
}

function advancePhysics(sim = state, dt = 0.1) {
  const target = equilibriumAngles(sim);
  const axes = [
    ['roll', target.roll],
    ['pitch', target.pitch],
    ['yaw', target.yaw]
  ];
  axes.forEach(([axis, targetValue]) => {
    const velocityKey = `${axis}Vel`;
    const acceleration = 0.82 * (targetValue - sim.attitude[axis]) - 0.24 * sim.attitude[velocityKey];
    sim.attitude[velocityKey] += acceleration * dt;
    sim.attitude[axis] += sim.attitude[velocityKey] * dt;
  });

  const env = environmentalForce(sim.env, sim.simTime);
  const targetPosX = env.x / 150 + sim.env.waveHeight * 0.16 * Math.sin(sim.simTime * 0.9);
  const targetPosY = env.y / 150 + sim.env.waveHeight * 0.16 * Math.cos(sim.simTime * 0.8);
  sim.attitude.posX += (targetPosX - sim.attitude.posX) * dt * 0.55;
  sim.attitude.posY += (targetPosY - sim.attitude.posY) * dt * 0.55;
  sim.attitude.heave = sim.env.waveHeight * 0.42 * Math.sin(sim.simTime * Math.PI * 0.55);
}

function segmentHitsCircle(start, end, zone) {
  const vx = end.x - start.x;
  const vy = end.y - start.y;
  const wx = start.x - zone.x;
  const wy = start.y - zone.y;
  const a = vx * vx + vy * vy;
  const b = 2 * (wx * vx + wy * vy);
  const c = wx * wx + wy * wy - zone.radius * zone.radius;
  if (c <= 0) return true;
  const discriminant = b * b - 4 * a * c;
  if (discriminant < 0 || a === 0) return false;
  const root = (-b - Math.sqrt(discriminant)) / (2 * a);
  return root >= 0 && root <= 1;
}

function distancePointToSegment(point, start, end) {
  const vx = end.x - start.x;
  const vy = end.y - start.y;
  const wx = point.x - start.x;
  const wy = point.y - start.y;
  const lengthSq = vx * vx + vy * vy;
  const t = lengthSq === 0 ? 0 : clamp((wx * vx + wy * vy) / lengthSq, 0, 1);
  const projectionX = start.x + t * vx;
  const projectionY = start.y + t * vy;
  return Math.hypot(point.x - projectionX, point.y - projectionY);
}

function zoneEncounters(fromAngle, toAngle) {
  const steps = Math.max(8, Math.ceil(Math.abs(angleDiff(fromAngle, toAngle)) / 3));
  const found = [];
  for (let index = 0; index <= steps; index += 1) {
    const angle = fromAngle + angleDiff(fromAngle, toAngle) * index / steps;
    const hook = hookPosition(angle);
    restrictedZones.forEach((zone) => {
      const distance = Math.hypot(hook.x - zone.x, hook.y - zone.y);
      if (distance <= zone.radius && !found.some((item) => item.zone.id === zone.id)) {
        found.push({ zone, angle: ((angle % 360) + 360) % 360, distance: Math.max(0, distance) });
      }
    });
    if (Math.abs(hook.x) > HULL_HALF_LENGTH - 2 || Math.abs(hook.y) > HULL_HALF_BEAM - 2) {
      if (!found.some((item) => item.zone.id === 'overboard')) {
        found.push({ zone: { id: 'overboard', name: '舷外落水风险区', radius: 0 }, angle: ((angle % 360) + 360) % 360, distance: 0 });
      }
    }
  }
  return found;
}

function currentZoneClearance() {
  const hook = hookPosition();
  let nearest = null;
  restrictedZones.forEach((zone) => {
    const distance = Math.hypot(hook.x - zone.x, hook.y - zone.y) - zone.radius;
    if (!nearest || distance < nearest.distance) nearest = { name: zone.name, distance };
  });
  const edgeX = Math.abs(hook.x) - HULL_HALF_LENGTH;
  const edgeY = Math.abs(hook.y) - HULL_HALF_BEAM;
  const edgeDistance = Math.max(edgeX, edgeY);
  if (edgeDistance > nearest.distance) nearest = { name: '舷侧边界', distance: edgeDistance };
  return nearest;
}

function transferMoment(fromId, toId, amount) {
  const from = tankDefs.find((tank) => tank.id === fromId);
  const to = tankDefs.find((tank) => tank.id === toId);
  return {
    roll: amount * 0.9 * (to.y - from.y),
    pitch: amount * 0.9 * (to.x - from.x)
  };
}

function validateBallast(command) {
  const reasons = [];
  const from = getTank(command.from);
  const to = getTank(command.to);
  if (from.id === to.id) reasons.push('源舱和目标舱不能相同。');
  if (from.volume < command.amount) reasons.push(`${from.name}当前只有 ${fmt(from.volume)} m³，无法转移 ${fmt(command.amount)} m³。`);
  if (TANK_CAPACITY - to.volume < command.amount) reasons.push(`${to.name}可用舱容不足。`);
  if (command.rate > SAFE_PUMP_RATE) reasons.push(`泵流量 ${fmt(command.rate)} m³/min 超过 12 m³/min 阈值，液位转移过快。`);

  const overlaps = activeCommands('ballast').filter((active) => {
    const ids = [active.from, active.to, command.from, command.to];
    return new Set(ids).size < 4;
  });
  if (overlaps.length) reasons.push('已有压载步骤正在操作同一舱室，可能导致控制目标互相覆盖。');

  const target = equilibriumAngles(state);
  const corrective = { roll: -target.roll * 18000, pitch: -target.pitch * 18000 };
  const proposed = transferMoment(command.from, command.to, command.amount);
  const activeMoments = activeCommands('ballast').map((active) => transferMoment(active.from, active.to, active.remaining ?? active.amount));
  const combinedRoll = proposed.roll + activeMoments.reduce((sum, item) => sum + item.roll, 0);
  const combinedPitch = proposed.pitch + activeMoments.reduce((sum, item) => sum + item.pitch, 0);
  const cancellationRoll = corrective.roll !== 0 && combinedRoll * corrective.roll < 0 && Math.abs(combinedRoll) > 0.7 * Math.abs(corrective.roll);
  const cancellationPitch = corrective.pitch !== 0 && combinedPitch * corrective.pitch < 0 && Math.abs(combinedPitch) > 0.7 * Math.abs(corrective.pitch);
  if (cancellationRoll || cancellationPitch) reasons.push('该转移与现有修正方向相反，净力矩会抵消超过 70% 的必要恢复力矩。');

  const projection = projectFuture({ commands: [command], duration: 20 });
  if (projection.maxRoll > ROLL_LIMIT) reasons.push(`预测横摇 ${fmt(projection.maxRoll, 2)}° 超过 ${ROLL_LIMIT}° 安全限值。`);
  if (projection.maxPitch > PITCH_LIMIT) reasons.push(`预测纵摇 ${fmt(projection.maxPitch, 2)}° 超过 ${PITCH_LIMIT}° 安全限值。`);
  if (projection.maxTension > TENSION_LIMIT) reasons.push(`预测系泊张力 ${Math.round(projection.maxTension)} kN 超过 ${TENSION_LIMIT} kN。`);

  return [...new Set(reasons)];
}

function validateSlew(command) {
  const reasons = [];
  if (activeCommands('slew').length || state.crane.activeCommandId) reasons.push('吊机已有正在执行的动作。');
  const encounters = zoneEncounters(command.startAngle ?? state.crane.angle, command.targetAngle);
  if (encounters.length) {
    reasons.push(`吊物路径进入：${encounters.map((item) => `${item.zone.name}@${Math.round(item.angle)}°`).join('、')}。`);
  }
  const projection = projectFuture({ commands: [command], duration: 20 });
  if (projection.maxTension > TENSION_LIMIT) reasons.push(`回转使预测最大张力达到 ${Math.round(projection.maxTension)} kN。`);
  if (projection.maxRoll > ROLL_LIMIT) reasons.push(`回转使预测横摇达到 ${fmt(projection.maxRoll, 2)}°。`);
  return [...new Set(reasons)];
}

function validateLift(command) {
  const reasons = [];
  if (activeCommands('lift').length) reasons.push('已有吊装步骤正在执行。');
  if (command.load > SWL_LIMIT) reasons.push(`吊物 ${fmt(command.load)} t 超过安全工作载荷 ${SWL_LIMIT} t。`);
  if (state.env.waveHeight > 4.5) reasons.push(`有义波高 ${fmt(state.env.waveHeight)} m，超过吊装建议限值 4.5 m。`);
  if (state.env.windSpeed > 22) reasons.push(`风速 ${fmt(state.env.windSpeed)} m/s，超过吊装建议限值 22 m/s。`);
  const encounters = zoneEncounters(state.crane.angle, state.crane.angle);
  if (encounters.length) reasons.push(`起吊点位于 ${encounters.map((item) => item.zone.name).join('、')}。`);
  const projection = projectFuture({ commands: [command], duration: 20 });
  if (projection.maxRoll > ROLL_LIMIT || projection.maxPitch > PITCH_LIMIT) reasons.push('起吊后预测姿态超过吊装安全姿态窗口。');
  if (projection.maxTension > TENSION_LIMIT) reasons.push(`起吊后最大系泊张力预测为 ${Math.round(projection.maxTension)} kN。`);
  return [...new Set(reasons)];
}

function makeSnapshot() {
  const tensions = calculateTensions();
  return {
    time: state.simTime,
    env: { ...state.env },
    roll: state.attitude.roll,
    pitch: state.attitude.pitch,
    yaw: state.attitude.yaw,
    heave: state.attitude.heave,
    tanks: Object.fromEntries(state.tanks.map((tank) => [tank.id, tank.volume])),
    boomAngle: state.crane.angle,
    hookHeight: state.crane.height,
    hookLoad: state.crane.load,
    tensions: Object.fromEntries(tensions.map((line) => [line.id, line.tension]))
  };
}

function createCommand(kind, details, reasons) {
  const command = {
    id: `CMD-${String(state.commandSeq++).padStart(3, '0')}`,
    kind,
    status: reasons.length ? 'frozen' : 'active',
    createdAt: state.simTime,
    frozenAt: reasons.length ? state.simTime : null,
    reasons,
    snapshot: reasons.length ? makeSnapshot() : null,
    ...details
  };
  state.commands.unshift(command);
  if (reasons.length) {
    freezeCommand(command, reasons, '下达前预测');
  } else {
    activateCommand(command);
    addAudit('info', `${command.id} 已激活：${commandLabel(command)}`);
  }
  return command;
}

function activateCommand(command) {
  command.status = 'active';
  command.frozenAt = null;
  command.snapshot = null;
  if (command.kind === 'lift') {
    state.crane.suspended = true;
    state.crane.activeCommandId = command.id;
    state.crane.paused = false;
  }
  if (command.kind === 'slew') {
    state.crane.activeCommandId = command.id;
    state.crane.paused = false;
  }
}

function freezeCommand(command, reasons, source = '运行中监测') {
  command.status = 'frozen';
  command.frozenAt = state.simTime;
  command.snapshot = makeSnapshot();
  addAudit('danger', `${command.id} 被冻结（${source}）：${reasons.join('；')}`);
}

function completeCommand(command) {
  command.status = 'completed';
  command.completedAt = state.simTime;
  if (state.crane.activeCommandId === command.id) state.crane.activeCommandId = null;
  addAudit('success', `${command.id} 完成：${commandLabel(command)}`);
}

function scheduleBallast() {
  const from = $('ballastFrom').value;
  const to = $('ballastTo').value;
  const amount = Number($('transferAmount').value);
  const rate = Number($('pumpRate').value);
  const command = {
    kind: 'ballast',
    from,
    to,
    amount,
    remaining: amount,
    transferred: 0,
    rate,
    startSnapshot: makeSnapshot()
  };
  const reasons = validateBallast(command);
  createCommand('ballast', command, reasons);
}

function scheduleLift() {
  const load = Number($('hookLoad').value);
  const command = {
    kind: 'lift',
    load,
    height: 0,
    targetHeight: 18,
    speed: 1.5,
    startAngle: state.crane.angle
  };
  state.crane.load = load;
  createCommand('lift', command, validateLift(command));
}

function scheduleSlew() {
  const targetAngle = Number($('targetAngle').value);
  const command = {
    kind: 'slew',
    startAngle: state.crane.angle,
    targetAngle,
    speed: 20
  };
  createCommand('slew', command, validateSlew(command));
}

function pauseCrane() {
  const active = state.commands.find((command) =>
    command.status === 'active' && (command.kind === 'lift' || command.kind === 'slew'));
  if (!active) {
    addAudit('warn', '暂停吊装被拒绝：当前没有活动吊装步骤。');
    return;
  }
  active.status = 'paused';
  active.pausedAt = state.simTime;
  active.snapshot = makeSnapshot();
  state.crane.paused = true;
  if (state.crane.activeCommandId === active.id) state.crane.activeCommandId = null;
  addAudit('warn', `${active.id} 已由操作员暂停，冻结现场基线并等待恢复校验。`);
}

function reconcileCommand(commandId) {
  const command = state.commands.find((item) => item.id === commandId);
  if (!command || command.status !== 'frozen') return;
  const baseline = command.snapshot;
  const current = makeSnapshot();
  const failures = [];

  const compare = (label, currentValue, baseValue, tolerance, unit = '') => {
    const delta = Math.abs(currentValue - baseValue);
    if (delta > tolerance) failures.push(`${label}漂移 ${fmt(currentValue, 2)}${unit}，基线 ${fmt(baseValue, 2)}${unit}，偏差 ${fmt(delta, 2)}${unit}（允许 ${fmt(tolerance, 2)}${unit}）`);
  };

  compare('风速', current.env.windSpeed, baseline.env.windSpeed, 4, ' m/s');
  compare('波高', current.env.waveHeight, baseline.env.waveHeight, 0.8, ' m');
  compare('浪向', Math.abs(angleDiff(baseline.env.waveDir, current.env.waveDir)), 0, 12, '°');
  compare('横摇', current.roll, baseline.roll, 0.8, '°');
  compare('纵摇', current.pitch, baseline.pitch, 0.8, '°');

  state.tanks.forEach((tank) => {
    compare(`${tank.name}液位水量`, current.tanks[tank.id], baseline.tanks[tank.id], 2, ' m³');
  });

  if (command.kind !== 'ballast') {
    compare('吊臂方位', Math.abs(angleDiff(baseline.boomAngle, current.boomAngle)), 0, 4, '°');
    compare('吊钩高度', current.hookHeight, baseline.hookHeight, 0.8, ' m');
    compare('吊物重量', current.hookLoad, baseline.hookLoad, 1, ' t');
  }

  mooringDefs.forEach((line) => {
    compare(`${line.name}张力`, current.tensions[line.id], baseline.tensions[line.id], 80, ' kN');
  });

  if (command.kind === 'slew') {
    const encounters = zoneEncounters(state.crane.angle, command.targetAngle);
    if (encounters.length) failures.push(`现场恢复后吊物仍将进入：${encounters.map((item) => item.zone.name).join('、')}`);
  }
  if (command.kind === 'lift') {
    const encounters = zoneEncounters(state.crane.angle, state.crane.angle);
    if (encounters.length) failures.push(`吊钩当前仍处于：${encounters.map((item) => item.zone.name).join('、')}`);
  }
  if (command.kind === 'ballast' && command.rate > SAFE_PUMP_RATE) {
    failures.push(`压载泵流量仍为 ${fmt(command.rate)} m³/min，需先降到 12 m³/min 以下。`);
  }

  const resultEl = document.querySelector(`[data-command-id="${command.id}"] .check-result`);
  if (failures.length) {
    addAudit('danger', `${command.id} 恢复校验失败：${failures.join('；')}`);
    if (resultEl) {
      resultEl.className = 'check-result fail';
      resultEl.textContent = `校验未通过：${failures.join('；')}`;
    }
    return;
  }

  const repeated = command.kind === 'ballast' ? validateBallast(command)
    : command.kind === 'slew' ? validateSlew(command)
    : command.kind === 'lift' ? validateLift(command)
    : [];
  if (repeated.length) {
    addAudit('danger', `${command.id} 二次风险评估未通过：${repeated.join('；')}`);
    if (resultEl) {
      resultEl.className = 'check-result fail';
      resultEl.textContent = `二次风险评估未通过：${repeated.join('；')}`;
    }
    return;
  }

  activateCommand(command);
  addAudit('success', `${command.id} 现场状态与原方案一致，已解除冻结并恢复。`);
}

function discardCommand(commandId) {
  const command = state.commands.find((item) => item.id === commandId);
  if (!command) return;
  command.status = 'completed';
  command.completedAt = state.simTime;
  command.aborted = true;
  addAudit('warn', `${command.id} 已按操作员确认废弃，未继续执行。`);
}

function adoptSafeRate(commandId) {
  const command = state.commands.find((item) => item.id === commandId);
  if (!command) return;
  command.rate = 10;
  command.reasons = command.reasons.filter((reason) => !reason.includes('泵流量'));
  addAudit('info', `${command.id} 已将泵流量下调至 10 m³/min，可重新发起现场校验。`);
}

function updateBallast(command, dt, tanks, freezeReasons) {
  if (command.status !== 'active') return;
  const ratePerSecond = command.rate / 60;
  const quantity = Math.min(command.remaining, ratePerSecond * dt);
  const from = getTank(command.from, tanks);
  const to = getTank(command.to, tanks);
  from.volume -= quantity;
  to.volume += quantity;
  command.remaining -= quantity;
  command.transferred += quantity;
  if (command.rate > SAFE_PUMP_RATE && !freezeReasons.length) {
    freezeReasons.push(`压载流量 ${fmt(command.rate)} m³/min 超过 ${SAFE_PUMP_RATE} m³/min，液位转移过快。`);
  }
  if (command.remaining <= 0.01) command.status = 'completed';
}

function updateCraneCommand(command, dt, crane, sim, freezeReasons, options = {}) {
  if (command.status !== 'active') return;
  if (command.kind === 'lift') {
    command.height = Math.min(command.targetHeight, command.height + command.speed * dt);
    crane.height = command.height;
    crane.suspended = true;
    if (command.height >= command.targetHeight) {
      command.status = 'completed';
      if (options.log) addAudit('success', `${command.id} 吊物已到达计划高度。`);
    }
  }
  if (command.kind === 'slew') {
    const delta = signedAngleDelta(command, crane.angle);
    const step = clamp(delta, -command.speed * dt, command.speed * dt);
    const nextAngle = crane.angle + step;
    const encounters = zoneEncounters(crane.angle, nextAngle);
    if (encounters.length) {
      freezeReasons.push(`吊物进入${encounters.map((item) => `${item.zone.name}(${Math.round(item.angle)}°)`).join('、')}。`);
      if (options.markFrozen) freezeCommand(command, [...new Set(freezeReasons)], '连续监测');
      return;
    }
    crane.angle = (nextAngle + 360) % 360;
    if (Math.abs(signedAngleDelta(command, crane.angle)) < 0.15) {
      crane.angle = command.targetAngle;
      command.status = 'completed';
      if (options.log) addAudit('success', `${command.id} 吊臂已到达目标方位。`);
    }
  }
}

function runRiskChecks() {
  const commandReasons = new Map();
  const addReasons = (command, reasons) => {
    if (!reasons.length) return;
    if (!commandReasons.has(command.id)) commandReasons.set(command.id, []);
    commandReasons.get(command.id).push(...reasons);
  };

  activeCommands('ballast').forEach((command) => {
    const reasons = [];
    if (command.rate > SAFE_PUMP_RATE) reasons.push(`压载流量 ${fmt(command.rate)} m³/min 超过 ${SAFE_PUMP_RATE} m³/min，液位转移过快。`);
    const target = equilibriumAngles(state);
    const proposed = transferMoment(command.from, command.to, command.remaining);
    if (proposed.roll * (-target.roll) < 0 && Math.abs(proposed.roll) > Math.abs(target.roll * 18000) * 0.7) {
      reasons.push('剩余转移量继续加重横摇偏差，控制动作与恢复目标相抵消。');
    }
    if (proposed.pitch * (-target.pitch) < 0 && Math.abs(proposed.pitch) > Math.abs(target.pitch * 18000) * 0.7) {
      reasons.push('剩余转移量继续加重纵摇偏差，控制动作与恢复目标相抵消。');
    }
    addReasons(command, [...new Set(reasons)]);
  });

  activeCommands('slew').forEach((command) => {
    const encounters = zoneEncounters(state.crane.angle, state.crane.angle);
    if (encounters.length) addReasons(command, [`吊物进入${encounters.map((item) => `${item.zone.name}(${Math.round(item.angle)}°)`).join('、')}。`]);
  });

  activeCommands('lift').forEach((command) => {
    const encounters = zoneEncounters(state.crane.angle, state.crane.angle);
    if (encounters.length) addReasons(command, [`吊物进入${encounters.map((item) => item.zone.name).join('、')}。`]);
  });

  const tensions = calculateTensions(state);
  const maxTension = Math.max(...tensions.map((line) => line.tension));
  if (Math.abs(state.attitude.roll) > ROLL_LIMIT) {
    activeCommands().forEach((command) => addReasons(command, [`实时横摇 ${fmt(state.attitude.roll, 2)}° 超过 ${ROLL_LIMIT}°。`]));
  }
  if (Math.abs(state.attitude.pitch) > PITCH_LIMIT) {
    activeCommands().forEach((command) => addReasons(command, [`实时纵摇 ${fmt(state.attitude.pitch, 2)}° 超过 ${PITCH_LIMIT}°。`]));
  }
  if (maxTension > TENSION_LIMIT) {
    activeCommands().forEach((command) => addReasons(command, [`实时最大系泊张力 ${Math.round(maxTension)} kN 超过 ${TENSION_LIMIT} kN。`]));
  }

  commandReasons.forEach((reasons, id) => {
    const command = state.commands.find((item) => item.id === id);
    if (command && command.status === 'active') freezeCommand(command, [...new Set(reasons)], '连续监测');
  });
}

function simulateCommands(sim, candidateCommands, dt) {
  const all = candidateCommands.map((command, index) => ({
    ...command,
    id: command.id || `PRED-${index}`,
    status: 'active',
    remaining: command.remaining ?? command.amount ?? 0,
    transferred: command.transferred ?? 0,
    height: command.height ?? 0
  }));
  const reasons = [];
  all.forEach((command) => {
    if (command.kind === 'ballast') updateBallast(command, dt, sim.tanks, reasons);
    if (command.kind === 'lift' || command.kind === 'slew') updateCraneCommand(command, dt, sim.crane, sim, reasons);
  });
  return reasons;
}

function projectFuture({ commands = null, duration = 20 } = {}) {
  const sim = {
    simTime: state.simTime,
    env: { ...state.env },
    attitude: { ...state.attitude },
    tanks: cloneTanks(),
    crane: { ...state.crane }
  };
  const active = commands || state.commands.filter((command) => command.status === 'active');
  const projected = active.map((command) => ({
    ...command,
    remaining: command.remaining ?? command.amount ?? 0,
    transferred: command.transferred ?? 0,
    height: command.height ?? 0
  }));
  const points = [];
  let maxRoll = Math.abs(sim.attitude.roll);
  let maxPitch = Math.abs(sim.attitude.pitch);
  let maxTension = Math.max(...calculateTensions(sim).map((line) => line.tension));
  let minClearance = currentZoneClearance().distance;
  let equilibriumEta = null;
  const stepCount = 20;

  for (let index = 1; index <= stepCount; index += 1) {
    sim.simTime += duration / stepCount;
    simulateCommands(sim, projected, duration / stepCount);
    advancePhysics(sim, duration / stepCount);
    const roll = sim.attitude.roll;
    const pitch = sim.attitude.pitch;
    const tensions = calculateTensions(sim);
    const tension = Math.max(...tensions.map((line) => line.tension));
    const clearance = Math.min(...restrictedZones.map((zone) => {
      const hook = hookPosition(sim.crane.angle);
      return Math.hypot(hook.x - zone.x, hook.y - zone.y) - zone.radius;
    }));
    maxRoll = Math.max(maxRoll, Math.abs(roll));
    maxPitch = Math.max(maxPitch, Math.abs(pitch));
    maxTension = Math.max(maxTension, tension);
    minClearance = Math.min(minClearance, clearance);
    if (equilibriumEta === null && Math.abs(roll) < 1 && Math.abs(pitch) < 1 && tension < WARNING_TENSION) equilibriumEta = index;
    points.push({ t: index, roll, pitch, tension, clearance });
  }

  return {
    points,
    sim,
    maxRoll,
    maxPitch,
    maxTension,
    minClearance,
    equilibriumEta
  };
}

function updateSimulation(dt) {
  state.simTime += dt;
  state.commands.filter((command) => command.status === 'active').forEach((command) => {
    if (command.kind === 'ballast') {
      updateBallast(command, dt, state.tanks, []);
      if (command.status === 'completed') completeCommand(command);
    }
    if (command.kind === 'lift' || command.kind === 'slew') {
      const before = command.status;
      const runtimeReasons = [];
      updateCraneCommand(command, dt, state.crane, state, runtimeReasons, { markFrozen: true, log: true });
      if (before === 'active' && command.status === 'completed') completeCommand(command);
    }
  });
  advancePhysics(state, dt);
  if (state.simTime - state.lastRiskCheck >= 0.5) {
    runRiskChecks();
    state.lastRiskCheck = state.simTime;
  }
}

function resumePausedCrane() {
  const command = state.commands.find((item) => item.status === 'paused' && (item.kind === 'lift' || item.kind === 'slew'));
  if (!command) {
    addAudit('warn', '恢复吊装被拒绝：没有处于暂停状态的吊装步骤。');
    return;
  }
  const baseline = command.snapshot;
  const current = makeSnapshot();
  const failures = [];
  const check = (label, value, base, tolerance, unit) => {
    if (Math.abs(value - base) > tolerance) failures.push(`${label}偏差 ${fmt(Math.abs(value - base), 2)}${unit}`);
  };
  check('横摇', current.roll, baseline.roll, 0.8, '°');
  check('纵摇', current.pitch, baseline.pitch, 0.8, '°');
  check('吊臂方位', Math.abs(angleDiff(baseline.boomAngle, current.boomAngle)), 0, 4, '°');
  check('吊钩高度', current.hookHeight, baseline.hookHeight, 0.8, ' m');
  mooringDefs.forEach((line) => check(line.name, current.tensions[line.id], baseline.tensions[line.id], 80, ' kN'));
  const encounters = command.kind === 'slew'
    ? zoneEncounters(current.boomAngle, command.targetAngle)
    : zoneEncounters(current.boomAngle, current.boomAngle);
  if (encounters.length) failures.push(`路径仍受限于${encounters.map((item) => item.zone.name).join('、')}`);
  if (state.env.waveHeight > baseline.env.waveHeight + 0.8) failures.push('波高较暂停基线升高超过 0.8 m');
  if (state.env.windSpeed > baseline.env.windSpeed + 4) failures.push('风速较暂停基线升高超过 4 m/s');
  if (failures.length) {
    addAudit('danger', `${command.id} 现场校验失败，继续保持暂停：${failures.join('；')}`);
    return;
  }
  command.status = 'active';
  state.crane.paused = false;
  state.crane.activeCommandId = command.id;
  addAudit('success', `${command.id} 现场状态与暂停基线一致，吊装已恢复。`);
}

function addAudit(level, message) {
  state.logs.unshift({
    time: state.simTime,
    level,
    message
  });
  state.logs = state.logs.slice(0, 200);
}

function csvCell(value) {
  return `"${String(value).replaceAll('"', '""')}"`;
}

function exportAudit() {
  const rows = [['time', 'level', 'message'], ...state.logs.map((log) => [clockText(log.time), log.level, log.message])];
  const csv = `\ufeff${rows.map((row) => row.map(csvCell).join(',')).join('\n')}`;
  const blob = new Blob([csv], { type: 'text/csv;charset=utf-8' });
  const url = URL.createObjectURL(blob);
  const link = document.createElement('a');
  link.href = url;
  link.download = `platform-audit-${Date.now()}.csv`;
  link.click();
  URL.revokeObjectURL(url);
}

function snapshotRows(snapshot) {
  if (!snapshot) return '';
  return `
    <dt>时间</dt><dd>${clockText(snapshot.time)}</dd>
    <dt>风浪</dt><dd>${fmt(snapshot.env.windSpeed)} m/s / ${fmt(snapshot.env.waveHeight)} m</dd>
    <dt>姿态</dt><dd>R ${fmt(snapshot.roll, 2)}° P ${fmt(snapshot.pitch, 2)}°</dd>
    <dt>吊臂/高度</dt><dd>${Math.round(snapshot.boomAngle)}° / ${fmt(snapshot.hookHeight, 1)} m</dd>
  `;
}

function renderTanks() {
  $('tankBody').innerHTML = state.tanks.map((tank) => {
    const level = Math.round(tank.volume);
    return `<tr>
      <td>${tank.name}</td>
      <td>${level}%</td>
      <td>${fmt(tank.volume)} m³</td>
      <td>${activeCommands('ballast').some((cmd) => cmd.from === tank.id || cmd.to === tank.id) ? '<span class="tag tag-warn">转移中</span>' : '<span class="tag">稳定</span>'}</td>
    </tr>`;
  }).join('');
}

function renderMooring() {
  const tensions = calculateTensions();
  $('mooringList').innerHTML = tensions.map((line) => {
    const level = line.tension >= TENSION_LIMIT ? 'danger' : line.tension >= WARNING_TENSION ? 'warn' : '';
    return `<div class="tension-item">
      <div class="tension-head"><strong>${line.name}</strong><strong>${line.tension} kN</strong></div>
      <div class="tension-bar"><i class="${level}" style="width:${clamp(line.utilization * 100, 0, 100)}%"></i></div>
      <div class="tension-meta"><span>限值 ${TENSION_LIMIT} kN</span><span>缆长 ${fmt(line.length, 1)} m</span></div>
    </div>`;
  }).join('');
  const max = Math.max(...tensions.map((line) => line.tension));
  const status = $('mooringStatus');
  status.className = `tag ${max >= TENSION_LIMIT ? 'tag-danger' : max >= WARNING_TENSION ? 'tag-warn' : 'tag-ok'}`;
  status.textContent = max >= TENSION_LIMIT ? '超限' : max >= WARNING_TENSION ? '高载荷' : '正常';
}

function renderDeckLoads() {
  $('deckLoads').innerHTML = state.deckLoads.map((load) => `
    <div class="load-item">
      <div class="load-head"><strong>${load.name}</strong><strong>${load.mass} t</strong></div>
      <div class="load-meta"><span>X ${load.x} m</span><span>Y ${load.y} m</span><span>固绑有效</span></div>
    </div>
  `).join('');
}

function renderCommands() {
  $('commandQueue').innerHTML = state.commands.slice(0, 8).map((command) => {
    const statusMap = { active: ['执行中', 'tag-ok'], paused: ['暂停待校验', 'tag-warn'], frozen: ['已冻结', 'tag-danger'], completed: [command.aborted ? '已废弃' : '已完成', 'tag'] };
    const [statusText, statusClass] = statusMap[command.status];
    const detail = command.kind === 'ballast'
      ? `${fmt(command.transferred)}/${fmt(command.amount)} m³ · ${fmt(command.rate)} m³/min`
      : command.kind === 'slew'
        ? `${Math.round(command.startAngle)}° → ${Math.round(command.targetAngle)}°`
        : `${fmt(command.load)} t · 高度 ${fmt(command.height, 1)}/${fmt(command.targetHeight, 1)} m`;
    return `<div class="command-item ${command.status}" data-command-id="${command.id}">
      <div class="command-head"><strong>${command.id}</strong><span class="tag ${statusClass}">${statusText}</span></div>
      <div class="command-meta">${commandLabel(command)}<br>${detail}</div>
      ${command.status === 'active' || command.status === 'paused' ? `<div class="progress"><i style="width:${clamp(commandProgress(command) * 100, 0, 100)}%"></i></div>` : ''}
    </div>`;
  }).join('') || '<div class="empty-state">暂无控制步骤</div>';

  const frozen = frozenCommands();
  $('freezeCount').textContent = `${frozen.length} 项冻结`;
  $('freezeCount').className = 'tag tag-danger';
  $('freezePanel').innerHTML = frozen.length ? frozen.map((command) => `
    <article class="frozen-item" data-command-id="${command.id}">
      <div class="frozen-title"><strong>${command.id}</strong><span class="tag tag-danger">已冻结</span></div>
      <p class="reason">${command.reasons.join('；')}</p>
      <dl class="snapshot">${snapshotRows(command.snapshot)}</dl>
      <div class="check-result"></div>
      <div class="button-row">
        <button class="btn btn-primary reconcile" type="button">现场校验并恢复</button>
        <button class="btn btn-ghost adopt-rate ${command.kind === 'ballast' && command.rate > SAFE_PUMP_RATE ? '' : 'hidden'}" type="button">采用 10 m³/min</button>
        <button class="btn btn-ghost abort" type="button">废弃步骤</button>
      </div>
    </article>
  `).join('') : '<div class="empty-state">当前没有被冻结的高风险步骤。</div>';
}

function renderAudit() {
  $('auditLog').innerHTML = state.logs.map((log) => `
    <div class="audit-item ${log.level}">
      <span class="audit-time">${clockText(log.time)}</span>${log.message}
    </div>
  `).join('');
}

function renderAttitude() {
  const { roll, pitch, yaw, heave } = state.attitude;
  $('rollValue').textContent = `${fmt(roll, 2)}°`;
  $('pitchValue').textContent = `${fmt(pitch, 2)}°`;
  $('yawValue').textContent = `${fmt(yaw, 1)}°`;
  $('heaveValue').textContent = `${fmt(heave, 1)} m`;
  $('rollBar').style.left = `${clamp(50 + roll * 5, 4, 96)}%`;
  $('pitchBar').style.left = `${clamp(50 + pitch * 5, 4, 96)}%`;
  $('yawBar').style.left = `${clamp(50 + yaw * 5, 4, 96)}%`;
  $('heaveBar').style.left = `${clamp(50 + heave * 16, 4, 96)}%`;
  const moments = loadMoments();
  $('cgY').textContent = `${fmt(moments.rollMoment / 2.2, 0)} t·m(横)`;
  $('cgX').textContent = `${fmt(moments.pitchMoment / 2.2, 0)} t·m(纵)`;
}

function drawPlatform() {
  const canvas = $('platformCanvas');
  const ctx = canvas.getContext('2d');
  const width = canvas.width;
  const height = canvas.height;
  const cx = width / 2;
  const cy = height / 2 + 4;
  const scale = 5.2;
  ctx.clearRect(0, 0, width, height);

  const seaGradient = ctx.createRadialGradient(cx, cy, 20, cx, cy, width * 0.7);
  seaGradient.addColorStop(0, '#0f3448');
  seaGradient.addColorStop(1, '#07131d');
  ctx.fillStyle = seaGradient;
  ctx.fillRect(0, 0, width, height);

  ctx.save();
  ctx.strokeStyle = 'rgba(56,213,244,.13)';
  ctx.lineWidth = 1;
  for (let index = 0; index < 22; index += 1) {
    const y = (index * 31 + state.simTime * 18) % height;
    ctx.beginPath();
    ctx.moveTo(0, y);
    ctx.bezierCurveTo(width * .25, y - 16, width * .55, y + 16, width, y - 6);
    ctx.stroke();
  }
  ctx.restore();

  const env = environmentalForce();
  ctx.save();
  ctx.translate(78, 72);
  ctx.strokeStyle = 'rgba(255,191,71,.9)';
  ctx.fillStyle = 'rgba(255,191,71,.95)';
  ctx.lineWidth = 3;
  ctx.beginPath();
  ctx.moveTo(0, 0);
  ctx.lineTo(34 * Math.cos(env.rad), 34 * Math.sin(env.rad));
  ctx.stroke();
  ctx.beginPath();
  ctx.arc(34 * Math.cos(env.rad), 34 * Math.sin(env.rad), 4, 0, Math.PI * 2);
  ctx.fill();
  ctx.fillText(`风浪流 ${Math.round(state.env.waveDir)}°`, -28, -18);
  ctx.restore();

  const toCanvas = (point) => ({
    x: cx + point.x * scale,
    y: cy - point.y * scale
  });

  calculateTensions(state).forEach((line) => {
    const start = toCanvas({ x: line.fairlead.x + state.attitude.posX, y: line.fairlead.y + state.attitude.posY });
    const end = toCanvas(line.anchor);
    const color = line.tension >= TENSION_LIMIT ? '#ff5c72' : line.tension >= WARNING_TENSION ? '#ffbf47' : '#41d994';
    ctx.strokeStyle = color;
    ctx.lineWidth = 2;
    ctx.beginPath();
    ctx.moveTo(start.x, start.y);
    ctx.lineTo(end.x, end.y);
    ctx.stroke();
    ctx.fillStyle = color;
    ctx.font = '12px sans-serif';
    ctx.fillText(`${line.id} ${line.tension}`, end.x + 6, end.y);
  });

  ctx.save();
  ctx.translate(cx + state.attitude.posX * scale, cy - state.attitude.posY * scale);
  ctx.rotate(-degToRad(state.attitude.yaw));
  ctx.fillStyle = '#1c3448';
  ctx.strokeStyle = '#58e0f4';
  ctx.lineWidth = 2;
  ctx.beginPath();
  ctx.roundRect(-HULL_HALF_LENGTH * scale, -HULL_HALF_BEAM * scale, HULL_HALF_LENGTH * 2 * scale, HULL_HALF_BEAM * 2 * scale, 18);
  ctx.fill();
  ctx.stroke();

  ctx.fillStyle = 'rgba(255,255,255,.08)';
  ctx.font = '13px sans-serif';
  ctx.fillText('艏 FORE', -24, -HULL_HALF_BEAM * scale + 20);
  ctx.fillText('艉 AFT', -24, HULL_HALF_BEAM * scale - 14);

  state.tanks.forEach((tank) => {
    const x = tank.x * scale;
    const y = -tank.y * scale;
    ctx.fillStyle = 'rgba(56,213,244,.18)';
    ctx.strokeStyle = 'rgba(56,213,244,.65)';
    ctx.fillRect(x - 24, y - 17, 48, 34);
    ctx.strokeRect(x - 24, y - 17, 48, 34);
    ctx.fillStyle = '#c7f5ff';
    ctx.font = '11px sans-serif';
    ctx.fillText(`${tank.id} ${Math.round(tank.volume)}%`, x - 18, y + 4);
  });

  state.deckLoads.forEach((load) => {
    ctx.fillStyle = 'rgba(146,169,184,.46)';
    ctx.strokeStyle = 'rgba(231,240,246,.65)';
    const x = load.x * scale;
    const y = -load.y * scale;
    ctx.fillRect(x - load.w * scale / 2, y - load.h * scale / 2, load.w * scale, load.h * scale);
    ctx.strokeRect(x - load.w * scale / 2, y - load.h * scale / 2, load.w * scale, load.h * scale);
    ctx.fillStyle = 'rgba(231,240,246,.82)';
    ctx.font = '11px sans-serif';
    ctx.textAlign = 'center';
    ctx.fillText(load.name, x, y + 4);
    ctx.textAlign = 'left';
  });

  restrictedZones.forEach((zone) => {
    const zx = zone.x * scale;
    const zy = -zone.y * scale;
    ctx.beginPath();
    ctx.fillStyle = 'rgba(255,92,114,.14)';
    ctx.strokeStyle = 'rgba(255,92,114,.75)';
    ctx.setLineDash([5, 5]);
    ctx.arc(zx, zy, zone.radius * scale, 0, Math.PI * 2);
    ctx.fill();
    ctx.stroke();
    ctx.setLineDash([]);
    const shortName = zone.name.replace('安全区', '').replace('禁区', '');
    const labelWidth = shortName.length * 12 + 10;
    ctx.fillStyle = 'rgba(7,19,29,.78)';
    ctx.fillRect(zx - labelWidth / 2, zy - 4, labelWidth, 18);
    ctx.fillStyle = '#ffd4da';
    ctx.font = '11px sans-serif';
    ctx.textAlign = 'center';
    ctx.fillText(shortName, zx, zy + 9);
    ctx.textAlign = 'left';
  });

  const base = toCanvasLocal(CRANE_BASE, scale);
  const hook = toCanvasLocal(hookPosition(), scale);
  ctx.strokeStyle = '#ffbf47';
  ctx.lineWidth = 4;
  ctx.beginPath();
  ctx.moveTo(base.x, base.y);
  ctx.lineTo(hook.x, hook.y);
  ctx.stroke();
  ctx.fillStyle = '#ffbf47';
  ctx.beginPath();
  ctx.arc(base.x, base.y, 7, 0, Math.PI * 2);
  ctx.fill();
  ctx.fillStyle = state.crane.paused ? '#ffbf47' : '#38d5f4';
  ctx.beginPath();
  ctx.arc(hook.x, hook.y, 5 + state.crane.height * .08, 0, Math.PI * 2);
  ctx.fill();
  ctx.restore();

  function toCanvasLocal(point) {
    return { x: point.x * scale, y: -point.y * scale };
  }
}

function drawPrediction(projection) {
  const canvas = $('predictionCanvas');
  const ctx = canvas.getContext('2d');
  const width = canvas.width;
  const height = canvas.height;
  ctx.clearRect(0, 0, width, height);
  ctx.strokeStyle = 'rgba(146,169,184,.18)';
  ctx.lineWidth = 1;
  for (let y = 20; y < height; y += 30) {
    ctx.beginPath();
    ctx.moveTo(34, y);
    ctx.lineTo(width - 16, y);
    ctx.stroke();
  }
  const series = [
    { key: 'roll', color: '#38d5f4', label: '横摇' },
    { key: 'pitch', color: '#ffbf47', label: '纵摇' }
  ];
  series.forEach((item) => {
    ctx.strokeStyle = item.color;
    ctx.lineWidth = 2;
    ctx.beginPath();
    projection.points.forEach((point, index) => {
      const x = 34 + index * ((width - 54) / (projection.points.length - 1));
      const y = height / 2 - point[item.key] * 10;
      if (index === 0) ctx.moveTo(x, y);
      else ctx.lineTo(x, y);
    });
    ctx.stroke();
  });
  ctx.fillStyle = '#92a9b8';
  ctx.font = '12px sans-serif';
  ctx.fillText('±6°安全姿态窗口', 38, 18);
  ctx.fillStyle = '#38d5f4';
  ctx.fillText('横摇', width - 90, 18);
  ctx.fillStyle = '#ffbf47';
  ctx.fillText('纵摇', width - 48, 18);
}

function renderPrediction() {
  state.prediction = projectFuture({ duration: 20 });
  drawPrediction(state.prediction);
  $('predictionBody').innerHTML = state.prediction.points.filter((_, index) => index % 2 === 0 || index === state.prediction.points.length - 1).map((point) => `
    <tr>
      <td>T+${point.t}s</td>
      <td>${fmt(point.roll, 2)}°</td>
      <td>${fmt(point.pitch, 2)}°</td>
      <td>${point.tension} kN</td>
    </tr>
  `).join('');

  const risk = state.prediction.maxTension > TENSION_LIMIT || state.prediction.maxRoll > ROLL_LIMIT || state.prediction.maxPitch > PITCH_LIMIT || state.prediction.minClearance < 0;
  const badge = $('predictionBadge');
  badge.className = `tag ${risk ? 'tag-danger' : state.prediction.maxTension > WARNING_TENSION || state.prediction.minClearance < 3 ? 'tag-warn' : 'tag-ok'}`;
  badge.textContent = risk ? '预测越限' : state.prediction.minClearance < 3 ? '接近禁区' : '可接受';

  const notes = [];
  notes.push({ cls: state.prediction.maxRoll > ROLL_LIMIT ? 'danger' : '', text: `最大横摇 ${fmt(state.prediction.maxRoll, 2)}°，限值 ${ROLL_LIMIT}°` });
  notes.push({ cls: state.prediction.maxTension > TENSION_LIMIT ? 'danger' : '', text: `最大系泊张力 ${Math.round(state.prediction.maxTension)} kN，限值 ${TENSION_LIMIT} kN` });
  notes.push({ cls: state.prediction.minClearance < 0 ? 'danger' : state.prediction.minClearance < 3 ? 'warn' : '', text: `吊物距最近安全区边界 ${fmt(state.prediction.minClearance, 1)} m` });
  notes.push({ cls: '', text: state.prediction.equilibriumEta ? `预计 ${state.prediction.equilibriumEta} 秒后恢复姿态/张力平衡窗口` : '20 秒内未进入稳定平衡窗口' });
  $('predictionNotes').innerHTML = notes.map((note) => `<li class="${note.cls}">${note.text}</li>`).join('');
}

function renderCrane() {
  const clearance = currentZoneClearance();
  $('boomAngle').textContent = `${Math.round(state.crane.angle)}°`;
  $('hookHeight').textContent = `${fmt(state.crane.height, 1)} m`;
  $('hookZone').textContent = `${clearance.name} ${fmt(clearance.distance, 1)} m`;
  $('targetAngleValue').textContent = `${$('targetAngle').value}°`;
  const status = $('craneStatus');
  if (state.crane.paused) {
    status.className = 'tag tag-warn';
    status.textContent = '暂停待校验';
  } else if (activeCommands('lift').length || activeCommands('slew').length) {
    status.className = 'tag tag-ok';
    status.textContent = activeCommands('slew').length ? '回转中' : '起吊中';
  } else {
    status.className = 'tag';
    status.textContent = '待命';
  }
}

function renderSystem() {
  const frozen = frozenCommands().length;
  const tensions = calculateTensions();
  const maxTension = Math.max(...tensions.map((line) => line.tension));
  const attitudeRisk = Math.abs(state.attitude.roll) > ROLL_LIMIT || Math.abs(state.attitude.pitch) > PITCH_LIMIT;
  const pill = $('systemStatus');
  if (frozen || attitudeRisk || maxTension > TENSION_LIMIT) {
    pill.className = 'status-pill status-danger';
    pill.textContent = frozen ? `${frozen} 项高风险冻结` : '实时安全越限';
  } else if (state.crane.paused || maxTension > WARNING_TENSION) {
    pill.className = 'status-pill status-warn';
    pill.textContent = state.crane.paused ? '吊装中断待恢复' : '载荷接近预警值';
  } else {
    pill.className = 'status-pill status-normal';
    pill.textContent = '系统正常';
  }
  const summary = $('hazardSummary');
  summary.classList.toggle('has-risk', Boolean(frozen));
  summary.textContent = frozen ? `${frozen} 个步骤已冻结，监测继续运行` : '无高风险步骤';
  $('simClock').textContent = clockText(state.simTime);
}

function render() {
  renderSystem();
  renderAttitude();
  renderTanks();
  renderMooring();
  renderDeckLoads();
  renderCrane();
  renderCommands();
  renderAudit();
  renderPrediction();
  drawPlatform();
}

function populateTankSelects() {
  const options = state.tanks.map((tank) => `<option value="${tank.id}">${tank.name}</option>`).join('');
  $('ballastFrom').innerHTML = options;
  $('ballastTo').innerHTML = state.tanks.map((tank) => `<option value="${tank.id}">${tank.name}</option>`).reverse().join('');
  $('ballastFrom').value = 'AS';
  $('ballastTo').value = 'AP';
}

function syncEnvironment() {
  state.env.windSpeed = Number($('windSpeed').value);
  state.env.waveHeight = Number($('waveHeight').value);
  state.env.waveDir = Number($('waveDir').value);
  state.env.currentSpeed = Number($('currentSpeed').value);
  $('windSpeedValue').textContent = `${state.env.windSpeed} m/s`;
  $('waveHeightValue').textContent = `${fmt(state.env.waveHeight)} m`;
  $('waveDirValue').textContent = `${state.env.waveDir}°`;
  $('currentSpeedValue').textContent = `${fmt(state.env.currentSpeed)} m/s`;
  $('forceDir').textContent = `${state.env.waveDir}°`;
}

function resetScene() {
  state.simTime = 0;
  state.lastRiskCheck = -99;
  state.commands = [];
  state.crane = {
    angle: 315,
    targetAngle: 270,
    height: 0,
    load: 8,
    suspended: false,
    activeCommandId: null,
    paused: false
  };
  state.attitude = {
    roll: 0,
    pitch: 0,
    yaw: 0,
    rollVel: 0,
    pitchVel: 0,
    yawVel: 0,
    heave: 0,
    posX: 0,
    posY: 0
  };
  state.tanks = tankDefs.map((tank) => ({ ...tank, volume: tankSeed[tank.id] }));
  $('hookLoad').value = 8;
  $('targetAngle').value = 270;
  $('transferAmount').value = 12;
  $('pumpRate').value = 8;
  $('windSpeed').value = 12;
  $('waveHeight').value = 2.4;
  $('waveDir').value = 135;
  $('currentSpeed').value = 0.8;
  addAudit('info', '场景已重置，所有控制步骤清空，现场状态回到初始方案。');
  syncEnvironment();
  render();
}

function bindEvents() {
  ['windSpeed', 'waveHeight', 'waveDir', 'currentSpeed'].forEach((id) => $(id).addEventListener('input', syncEnvironment));
  $('transferAmount').addEventListener('input', () => {
    $('transferAmountValue').textContent = `${$('transferAmount').value} m³`;
  });
  $('pumpRate').addEventListener('input', () => {
    $('pumpRateValue').textContent = `${fmt(Number($('pumpRate').value))} m³/min`;
    $('pumpHealth').className = `tag ${Number($('pumpRate').value) > SAFE_PUMP_RATE ? 'tag-danger' : 'tag-ok'}`;
    $('pumpHealth').textContent = Number($('pumpRate').value) > SAFE_PUMP_RATE ? '超过安全流量' : '流量正常';
  });
  $('hookLoad').addEventListener('input', () => {
    const load = Number($('hookLoad').value);
    $('hookLoadValue').textContent = `${fmt(load)} t`;
    state.crane.load = load;
  });
  $('targetAngle').addEventListener('input', () => {
    $('targetAngleValue').textContent = `${$('targetAngle').value}°`;
  });
  $('startTransfer').addEventListener('click', scheduleBallast);
  $('startLift').addEventListener('click', scheduleLift);
  $('pauseLift').addEventListener('click', pauseCrane);
  $('resumeLift').addEventListener('click', resumePausedCrane);
  $('slewBoom').addEventListener('click', scheduleSlew);
  $('exportAudit').addEventListener('click', exportAudit);
  $('resetScene').addEventListener('click', resetScene);
  $('simToggle').addEventListener('click', () => {
    state.running = !state.running;
    $('simToggle').textContent = state.running ? '暂停演算' : '继续演算';
    addAudit('info', state.running ? '仿真时钟继续。' : '仿真时钟已由操作员暂停；监测页面仍可查看。');
  });

  $('freezePanel').addEventListener('click', (event) => {
    const panel = event.target.closest('.frozen-item');
    if (!panel) return;
    const id = panel.dataset.commandId;
    if (event.target.classList.contains('reconcile')) reconcileCommand(id);
    if (event.target.classList.contains('abort')) discardCommand(id);
    if (event.target.classList.contains('adopt-rate')) adoptSafeRate(id);
  });
}

function loop(timestamp) {
  if (!loop.last) loop.last = timestamp;
  const frameDt = Math.min(0.05, (timestamp - loop.last) / 1000);
  loop.last = timestamp;
  if (state.running) {
    updateSimulation(frameDt);
  }
  render();
  requestAnimationFrame(loop);
}

function init() {
  populateTankSelects();
  syncEnvironment();
  bindEvents();
  addAudit('info', '系统上线：姿态、压载舱、系泊、甲板载荷与起重状态开始连续监测。');
  addAudit('info', '高风险步骤将在执行前预测和执行中监测两个阶段冻结，并保存现场快照。');
  requestAnimationFrame(loop);
}

document.addEventListener('DOMContentLoaded', init);
