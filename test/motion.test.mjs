// Run with: npm test
import { test } from 'node:test';
import assert from 'node:assert/strict';
import {
  driveLeg, detectCrash, buildSampleExpr, parseSample, makeBleSampler, makeSimSampler,
  SENSOR_EXPR, BLE_SENSORS, CM_PER_DEG,
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
  assert.equal(rh.detail, 'heading');
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

test('detectCrash: slip when the range does not shrink', () => {
  const s = (t, drivenCm, distanceCm) => ({ t, drivenCm, cmdCm: drivenCm, distanceCm, hasEnc: true });
  assert.equal(detectCrash([s(0, 0, 80), s(900, 10, 79)]).reason, 'slip');
  assert.equal(detectCrash([s(0, 0, 80), s(900, 10, 70)]).crash, false);
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
  assert.equal(sent[2], buildSampleExpr(BLE_SENSORS).expr);
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
