// Run with: npm test
import { test } from 'node:test';
import assert from 'node:assert/strict';
import {
  driveLeg, turnInPlace, detectCrash, buildSampleExpr, parseSample, makeBleSampler, makeSimSampler,
  SENSOR_EXPR, BLE_SENSORS, BLE_SENSORS_MIN, CM_PER_DEG,
} from '../js/motion.js';
import { SimRobot } from '../js/robot-sim.js';

const WHEEL_CM = Math.PI * 6.5;

// Scripted 1-D world on a fake clock: each poll costs 90 ms, sleeps advance
// time instantly. stuckAt: the robot cannot pass this point (wheels stall).
function world({ wallAt = 1e9, stuckAt = Infinity, joltAt = Infinity, yawKickAt = Infinity, encoders = true, imu = true, failAfter = Infinity } = {}) {
  const w = { t: 0, x: 0, rpm: 0, cmds: [], polls: 0, jolted: false };
  w.advance = (ms) => {
    const nx = w.x + ((w.rpm / 60) * WHEEL_CM * ms) / 1000;
    if (nx > joltAt && !w.jolted) { w.jolted = true; w.spike = true; }
    w.x = Math.min(nx, stuckAt, w.jolted ? joltAt : Infinity);
    w.t += ms;
  };
  w.clock = { now: () => w.t, sleep: async (ms) => w.advance(ms) };
  w.bus = {
    submit: async (c) => {
      w.cmds.push(c);
      if (c.cmd === 'drive') w.rpm = c.args.left;
      if (c.cmd === 'stop') w.rpm = 0;
      return { ok: true };
    },
  };
  w.sample = async () => {
    if (++w.polls > failAfter) throw new Error('timeout: poll');
    w.advance(90);
    const s = { distanceCm: Math.min(300, wallAt - w.x) };
    if (encoders) { s.encL = w.x / CM_PER_DEG; s.encR = w.x / CM_PER_DEG; }
    if (imu) {
      s.acc = { x: w.spike ? -9 : 0, y: 0, z: -9.8 };
      s.yaw = w.x > yawKickAt ? 20 : 0;
    }
    w.spike = false;
    return s;
  };
  w.leg = (opts) => driveLeg(w.bus, { sample: w.sample, clock: w.clock, makeCommand: (cmd, args, src) => ({ cmd, args, src, gen: 3 }), ...opts });
  w.drives = () => w.cmds.filter((c) => c.cmd === 'drive').map((c) => c.args.left);
  return w;
}

test('driveLeg reaches the target with a ramp and ends with a stop', async () => {
  const w = world();
  const r = await w.leg({ cm: 30, speed: 40 });
  assert.equal(r.reason, 'done');
  assert.equal(r.ok, true);
  assert.ok(Math.abs(r.droveCm - 30) < 1.5, `drove ${r.droveCm}`);
  assert.ok(Math.abs(w.x - r.droveCm) < 1e-6, 'measured from the encoders');
  const d = w.drives();
  assert.equal(d[0], 12, 'starts at minRpm');
  assert.ok(Math.max(...d) <= 40 && Math.max(...d) >= 38);
  assert.equal(w.cmds.at(-1).cmd, 'stop');
  assert.equal(w.rpm, 0);
  assert.ok(w.cmds.every((c) => c.gen === 3 && c.src === 'agent'), 'stamped makeCommand');
  assert.ok(w.cmds.filter((c) => c.cmd === 'drive').every((c) => c.args.leg === true && c.args.left === c.args.right));
});

test('driveLeg: realistic IMU noise at rest and while driving does not trigger a crash', async () => {
  // hardware: get_acc z about -9.6 at rest; small noise on every axis, low shake, yaw jitter
  let seed = 7;
  const rnd = () => { seed = (seed * 16807) % 2147483647; return seed / 2147483647 - 0.5; };
  for (const encoders of [true, false]) {
    const w = world({ encoders, imu: false });
    const inner = w.sample;
    w.sample = async () => ({
      ...(await inner()),
      acc: { x: 0.6 * rnd(), y: 0.6 * rnd(), z: -9.6 + 0.6 * rnd() },
      shake: Math.round(8 * (rnd() + 0.5)),
      yaw: 30 + 2 * rnd(),
    });
    const r = await w.leg({ cm: 60 });
    assert.equal(r.reason, 'done', `${r.reason} ${r.detail} (encoders ${encoders})`);
  }
});

// Hardware-like robot: 150..270 ms per 8-value poll, first-order motor lag and
// coast (tau 150 ms), +-1 degree encoder noise, mirrored EM2 already undone.
function laggyWorld({ stuckAt = Infinity, seed = 3 } = {}) {
  const w = { t: 0, x: 0, v: 0, rpm: 0, cmds: [] };
  const rnd = () => { seed = (seed * 16807) % 2147483647; return seed / 2147483647; };
  w.advance = (ms) => {
    for (let k = 0; k < ms; k += 5) {
      const target = (w.rpm / 60) * WHEEL_CM;
      w.v += (target - w.v) * (5 / 150);
      w.x = Math.min(w.x + (w.v * 5) / 1000, stuckAt);
      if (w.x >= stuckAt) w.v = Math.min(w.v, 0);
    }
    w.t += ms;
  };
  w.clock = { now: () => w.t, sleep: async (ms) => w.advance(ms) };
  w.bus = { submit: async (c) => { w.cmds.push(c); if (c.cmd === 'drive') w.rpm = c.args.left; if (c.cmd === 'stop') w.rpm = 0; return { ok: true }; } };
  w.sample = async () => {
    const lat = 150 + 120 * rnd();
    w.advance(lat / 2);
    const s = { distanceCm: 300, encL: w.x / CM_PER_DEG + (rnd() * 2 - 1), encR: w.x / CM_PER_DEG + (rnd() * 2 - 1),
      acc: { x: 0.3 * rnd(), y: 0.3 * rnd(), z: -9.6 }, yaw: 0.5 * rnd(), shake: 3 };
    w.advance(lat / 2);
    return s;
  };
  w.leg = (opts) => driveLeg(w.bus, { sample: w.sample, clock: w.clock, ...opts });
  return w;
}

test('driveLeg with hardware-like polls (4 to 6 Hz), motor lag, encoder noise and coast', async () => {
  for (const seed of [3, 11, 29, 47]) {
    const w = laggyWorld({ seed });
    const r = await w.leg({ cm: 40, speed: 40 });
    assert.equal(r.reason, 'done', `seed ${seed}: ${r.reason} ${r.detail}`);
    assert.ok(Math.abs(r.droveCm - w.x) < 0.3, `seed ${seed}: reported ${r.droveCm} vs actual ${w.x} (coast counted)`);
    assert.ok(w.x > 39 && w.x < 43, `seed ${seed}: x ${w.x}`);
    const b = laggyWorld({ seed, stuckAt: 18 });
    const rb = await b.leg({ cm: 40, speed: 40 });
    assert.equal(rb.reason, 'stall', `seed ${seed}: ${rb.reason}`);
    assert.ok(rb.contactCm > 17 && b.x < 15, `seed ${seed}: contact ${rb.contactCm} x ${b.x}`);
  }
});

test('driveLeg stops for an obstacle without backing off', async () => {
  const w = world({ wallAt: 45 });
  const r = await w.leg({ cm: 60, stopAtCm: 20 });
  assert.equal(r.reason, 'obstacle');
  assert.ok(w.x >= 22 && w.x <= 27, `x ${w.x}`);
  assert.ok(w.drives().every((v) => v >= 0), 'no back-off');
  assert.equal(w.cmds.at(-1).cmd, 'stop');
});

test('driveLeg detects a stall, backs off and reports the net distance', async () => {
  const w = world({ stuckAt: 15 });
  const r = await w.leg({ cm: 40 });
  assert.equal(r.reason, 'stall');
  assert.equal(r.detail, 'stall');
  assert.ok(Math.abs(r.contactCm - 15) < 0.5);
  assert.ok(r.backedCm >= 4.5 && r.backedCm < 8, `backed ${r.backedCm}`);
  assert.ok(Math.abs(r.droveCm - w.x) < 1e-6 && w.x < 11, `x ${w.x}`);
  assert.ok(w.drives().some((v) => v < 0));
  assert.equal(w.cmds.at(-1).cmd, 'stop');
});

test('driveLeg detects a jolt without encoders and a heading kick', async () => {
  const w = world({ joltAt: 12, encoders: false });
  const r = await w.leg({ cm: 40 });
  assert.equal(r.reason, 'crash');
  assert.equal(r.detail, 'jolt');
  assert.ok(r.backedCm > 0, 'time-based back-off');
  assert.equal(w.cmds.at(-1).cmd, 'stop');

  const h = world({ yawKickAt: 10 });
  const rh = await h.leg({ cm: 40 });
  assert.equal(rh.reason, 'crash');
  assert.equal(rh.detail, 'twist', 'yaw turned on a straight leg');
});

test('driveLeg without encoders uses time x speed', async () => {
  const w = world({ encoders: false, imu: false });
  const r = await w.leg({ cm: 25 });
  assert.equal(r.reason, 'done');
  assert.ok(Math.abs(w.x - 25) < 3, `x ${w.x}`);
});

test('driveLeg aborts, fails on sensor errors and always stops', async () => {
  const ac = new AbortController();
  const w = world();
  const r = await w.leg({ cm: 80, signal: ac.signal, onSample: () => { if (w.polls === 4) ac.abort(); } });
  assert.equal(r.reason, 'aborted');
  assert.equal(r.ok, false);
  assert.equal(w.cmds.at(-1).cmd, 'stop');
  assert.equal(w.rpm, 0);

  const e = world({ failAfter: 3 });
  const re = await e.leg({ cm: 80 });
  assert.equal(re.reason, 'error');
  assert.match(re.note, /sensor poll failed/);
  assert.equal(e.cmds.at(-1).cmd, 'stop');

  // stop generation: the bus refuses the drive
  const c = world();
  c.bus.submit = async (cmd) => { c.cmds.push(cmd); return cmd.cmd === 'drive' ? { ok: false, error: 'cancelled by stop' } : { ok: true }; };
  const rc = await c.leg({ cm: 30 });
  assert.equal(rc.reason, 'aborted');
  assert.equal(rc.cancelled, true);
  assert.equal(c.cmds.at(-1).cmd, 'stop');
});

test('detectCrash: slip (opt-in) when the range does not shrink', () => {
  const s = (t, drivenCm, distanceCm) => ({ t, drivenCm, cmdCm: drivenCm, distanceCm, hasEnc: true });
  assert.equal(detectCrash([s(0, 0, 80), s(900, 10, 79)]).crash, false, 'off by default');
  assert.equal(detectCrash([s(0, 0, 80), s(900, 10, 79)], { slip: true }).reason, 'slip');
  assert.equal(detectCrash([s(0, 0, 80), s(900, 10, 70)], { slip: true }).crash, false);
  assert.equal(detectCrash([s(0, 0, 80)]).crash, false);
});

test('sample expressions fit the 200-byte limit and parse mirrored encoders', async () => {
  const full = buildSampleExpr(SENSOR_EXPR);
  assert.ok(full.expr.length <= 200, `${full.expr.length} bytes`);
  assert.ok(!/import/.test(full.expr));
  assert.deepEqual(full.keys, ['distance', 'encL', 'encR', 'ax', 'ay', 'az', 'yaw', 'shake']);
  const p = parseSample(full.keys, [55, 360, -360, 0.1, 0.2, -9.8, 12, 3]);
  assert.deepEqual(p, { distanceCm: 55, encL: 360, encR: 360, yaw: 12, shake: 3, acc: { x: 0.1, y: 0.2, z: -9.8 } });
  assert.throws(() => buildSampleExpr({}));

  // falls back to the confirmed sensors once when the full list fails
  const sent = [];
  const robot = {
    query: async (expr) => {
      sent.push(expr);
      if (/EM_get_angle/.test(expr)) throw new Error("AttributeError: 'module' object has no attribute");
      return [42, 7];
    },
  };
  const sample = makeBleSampler(robot, SENSOR_EXPR);
  const a = await sample();
  const b = await sample();
  assert.equal(a.distanceCm, 42);
  assert.equal(b.yaw, 7);
  assert.equal(sent.length, 3);
  assert.equal(sent[2], buildSampleExpr(BLE_SENSORS_MIN).expr);
});

test('SimRobot sensorSample: encoders follow motion, impact shows on the accelerometer', async () => {
  const stub = () => {};
  const sim = new SimRobot({ log: stub, onStatus: stub, timeScale: 20, obstacles: [{ kind: 'circle', x: 150, y: 60, r: 5, low: true }] });
  await sim.connect();
  try {
    const sample = makeSimSampler(sim);
    const s0 = await sample();
    assert.equal(s0.acc.z, -9.8);
    assert.ok(s0.distanceCm > 80, 'low obstacle is invisible to the ultrasonic');
    await sim.straight(10, { wait: true });
    const s1 = await sample();
    assert.ok(Math.abs((s1.encL - s0.encL) * CM_PER_DEG - 10) < 0.3, `encL ${s1.encL}`);
    assert.ok(Math.abs(s1.encL - s1.encR) < 0.5);
    await sim.turn(90, { wait: true });
    const s2 = await sample();
    assert.ok(s2.encL - s1.encL > 0 && s2.encR - s1.encR < 0, 'clockwise turn: left forward, right back');
    await sim.turn(-90, { wait: true });
    const r = await driveLeg({ submit: async (c) => { if (c.cmd === 'drive') await sim.drive(c.args.left, c.args.right); else if (c.cmd === 'stop') await sim.stop(); return { ok: true }; } },
      { cm: 40, sample });
    assert.ok(r.reason === 'crash' || r.reason === 'stall', r.reason);
    assert.ok(r.backedCm > 3, `backed ${r.backedCm}`);
    assert.equal(sim.motion, null);
  } finally {
    await sim.disconnect();
  }
});

test('SimRobot sensorSample stays exact while spinning with drive()', async () => {
  const stub = () => {};
  const sim = new SimRobot({ log: stub, onStatus: stub, timeScale: 10 });
  await sim.connect();
  try {
    const sample = makeSimSampler(sim);
    const s0 = await sample();
    for (let i = 0; i < 8; i++) {
      await sim.drive(20, -20);
      await sample.clock.sleep(150);
      const s = await sample();
      const yaw = ((sim.state.heading - sim.startHeading) % 360 + 540) % 360 - 180;
      assert.ok(Math.abs(s.yaw - yaw) <= 0.05, `yaw ${s.yaw} vs ${yaw}`);
      assert.equal(s.distanceCm, sim.distanceNow(), "distance at the sampled heading");
      // encoders: opposite wheel travel matching the rotation
      const turned = ((s.encL - s0.encL) - (s.encR - s0.encR)) * CM_PER_DEG / 12 * (180 / Math.PI);
      assert.ok(Math.abs(((turned - s.yaw + s0.yaw + 540) % 360) - 180) < 0.5, `enc ${turned} yaw ${s.yaw}`);
    }
    assert.ok(Math.abs(sim.state.heading - sim.startHeading) > 20, 'it turned');
    await sim.stop();
  } finally {
    await sim.disconnect();
  }
});

test('driveLeg detects a glancing hit: one wheel caught while the other keeps going', async () => {
  // left wheel held at 12 cm (door frame); the right wheel drives on, the body pivots
  const w = { t: 0, l: 0, r: 0, rpm: 0, cmds: [] };
  const adv = (ms) => {
    const d = ((w.rpm / 60) * WHEEL_CM * ms) / 1000;
    w.l = Math.min(w.l + d, 12); w.r += d; w.t += ms;
  };
  const bus = { submit: async (c) => { w.cmds.push(c); if (c.cmd === 'drive') w.rpm = c.args.left; if (c.cmd === 'stop') w.rpm = 0; return { ok: true }; } };
  const sample = async () => {
    adv(200);
    const heading = (((w.l - w.r) / 12) * 180) / Math.PI;
    return { distanceCm: 300, encL: w.l / CM_PER_DEG + 0.7, encR: w.r / CM_PER_DEG - 0.7, yaw: Math.round(heading), acc: { x: 0.1, y: 0, z: -9.6 }, shake: 2 };
  };
  const r = await driveLeg(bus, { cm: 40, sample, clock: { now: () => w.t, sleep: async (ms) => adv(ms) } });
  assert.ok(r.reason === 'crash' || r.reason === 'stall', r.reason);
  assert.ok(['stall', 'twist', 'wheel'].includes(r.detail), r.detail);
  assert.ok(w.r - 12 < 8, `right wheel ran on ${(w.r - 12).toFixed(1)} cm past the catch`);
  assert.ok(r.details && typeof r.details.value === 'number');
  assert.equal(w.cmds.at(-1).cmd, 'stop');
});

test('turnInPlace: closed-loop gyro turns within 2.5 deg under latency, integer and unbounded yaw', async () => {
  const stub = () => {};
  const sim = new SimRobot({ log: stub, onStatus: stub, timeScale: 25, yawInteger: true, yawMode: 'unbounded', encNoiseDeg: 1, latencyMs: 200 });
  const bus = { submit: async (c) => { if (c.cmd === 'drive') await sim.drive(c.args.left, c.args.right); else if (c.cmd === 'stop') await sim.stop(); return { ok: true }; } };
  await sim.connect();
  try {
    const sample = makeSimSampler(sim);
    let spinSign = -1; // wrong on purpose: the first turn must discover it
    for (const deg of [90, -150, 170, 20]) {
      const h0 = sim.state.heading;
      const r = await turnInPlace(bus, { deg, sample, spinSign });
      spinSign = r.spinSign;
      const truth = sim.state.heading - h0;
      assert.equal(r.reason, 'done', `${deg}: ${r.reason}`);
      assert.ok(Math.abs(truth - deg) <= 2.5, `${deg}: turned ${truth.toFixed(1)}`);
      assert.ok(Math.abs(truth - r.achievedDeg) <= 1.5, `${deg}: reported ${r.achievedDeg}, true ${truth.toFixed(1)}`);
      assert.equal(sim.motion, null);
    }
    assert.equal(spinSign, 1, 'reversed spin learned');
    const none = await turnInPlace(bus, { deg: 45, sample: async () => ({ distanceCm: 50 }) });
    assert.equal(none.reason, 'nosensor');
  } finally {
    await sim.disconnect();
  }
});

test('turnInPlace stops on a stall (wheels blocked)', async () => {
  let t = 0, rpm = 0;
  const bus = { submit: async (c) => { if (c.cmd === 'drive') rpm = c.args.left; if (c.cmd === 'stop') rpm = 0; return { ok: true }; } };
  const sample = async () => { t += 150; return { yaw: 3, encL: 0, encR: 0 }; };
  const r = await turnInPlace(bus, { deg: 90, sample, clock: { now: () => t, sleep: async (ms) => { t += ms; } } });
  assert.equal(r.reason, 'stall');
  assert.equal(rpm, 0);
});

test('driveLeg learns the stop distance: legs stop overshooting (field: +3 to 4 cm per leg)', async () => {
  const stub = () => {};
  const sim = new SimRobot({ log: stub, onStatus: stub, timeScale: 10, stopCoastCm: 3.5, latencyMs: 150 });
  const bus = { submit: async (c) => { if (c.cmd === 'drive') await sim.drive(c.args.left, c.args.right); else if (c.cmd === 'stop') await sim.stop(); return { ok: true }; } };
  await sim.connect();
  try {
    const sample = makeSimSampler(sim);
    const coast = { cm: 0 };
    const errs = [];
    for (let i = 0; i < 4; i++) {
      sim.state.y = 190; sim.state.x = 150; sim.state.heading = -90; // fresh run-up along the room
      const r = await driveLeg(bus, { cm: 35, sample, coast });
      await sample.clock.sleep(400);
      sim.tick();
      assert.equal(r.reason, 'done');
      errs.push(190 - sim.state.y - 35);
    }
    assert.ok(errs[0] > 2, `first leg overshoots ${errs[0].toFixed(1)}`);
    assert.ok(Math.abs(errs[3]) < 1.5, `after learning ${errs.map((e) => e.toFixed(1))}`);
    assert.ok(coast.cm > 2 && coast.cm < 6, `coast ${coast.cm}`);
  } finally {
    await sim.disconnect();
  }
});

test('turnInPlace: large turns across +-180 with a slow link (field: -168 achieved -150)', async () => {
  const stub = () => {};
  // 250 ms: the slowest full poll measured on hardware (the turn uses the light ~120 ms one)
  const sim = new SimRobot({ log: stub, onStatus: stub, timeScale: 10, yawInteger: true, latencyMs: 250 });
  sim.state.heading = sim.startHeading + 168; // yaw reads 168, the turn crosses the wrap
  const bus = { submit: async (c) => { if (c.cmd === 'drive') await sim.drive(c.args.left, c.args.right); else if (c.cmd === 'stop') await sim.stop(); return { ok: true }; } };
  await sim.connect();
  try {
    const sample = makeSimSampler(sim);
    for (const deg of [-168, 168, -175]) {
      const h0 = sim.state.heading;
      const r = await turnInPlace(bus, { deg, sample });
      const truth = sim.state.heading - h0;
      assert.equal(r.reason, 'done', `${deg}: ${r.reason} ${r.detail}`);
      assert.ok(Math.abs(truth - deg) <= 3, `${deg}: turned ${truth.toFixed(1)}`);
    }
  } finally {
    await sim.disconnect();
  }
});

// Field v0.7.0: "wheel {value:-8.13, drivenCm:13.3, yawDelta:0, encHeading:-8.13}"
// ended a leg as a crash and left a phantom obstacle. One wheel slipped.
function slipWorld({ gyro = true, twistAt = Infinity } = {}) {
  const w = { t: 0, x: 0, rpm: 0, cmds: [] };
  const adv = (ms) => { w.x += ((w.rpm / 60) * WHEEL_CM * ms) / 1000; w.t += ms; };
  w.bus = { submit: async (c) => { w.cmds.push(c); if (c.cmd === 'drive') w.rpm = c.args.left; if (c.cmd === 'stop') w.rpm = 0; return { ok: true }; } };
  w.sample = async () => {
    adv(180);
    // the right wheel spins 25 % more than the ground it covers after 8 cm
    const extra = Math.max(0, w.x - 8) * 0.25;
    const s = { distanceCm: 300, encL: w.x / CM_PER_DEG, encR: (w.x + extra) / CM_PER_DEG, acc: { x: 0.1, y: 0, z: -9.6 }, shake: 2 };
    if (gyro) s.yaw = w.x > twistAt ? 15 : 0;
    return s;
  };
  w.clock = { now: () => w.t, sleep: async (ms) => adv(ms) };
  return w;
}

test('driveLeg: encoder mismatch with the gyro at 0 is wheel slip, not a crash', async () => {
  const w = slipWorld();
  const r = await driveLeg(w.bus, { cm: 29, sample: w.sample, clock: w.clock });
  assert.equal(r.reason, 'done', `${r.reason} ${r.detail} ${JSON.stringify(r.details)}`);
  assert.ok(r.slip && Math.abs(r.slip.encHeading) > 8 && r.slip.yawDelta === 0, JSON.stringify(r.slip));
  assert.ok(Math.abs(w.x - 29) < 2, `drove ${w.x.toFixed(1)} (the slower wheel counts)`);
  assert.equal(r.yawDelta, 0);
});

test('driveLeg: without a gyro an encoder mismatch is still a glancing hit; with a twist it is a crash', async () => {
  const n = slipWorld({ gyro: false });
  const rn = await driveLeg(n.bus, { cm: 29, sample: n.sample, clock: n.clock });
  assert.equal(rn.reason, 'crash');
  assert.equal(rn.detail, 'wheel');
  const t = slipWorld({ twistAt: 12 });
  const rt = await driveLeg(t.bus, { cm: 29, sample: t.sample, clock: t.clock });
  assert.equal(rt.reason, 'crash');
  assert.equal(rt.detail, 'twist');
});
