// Run with: npm test
import test from 'node:test';
import assert from 'node:assert/strict';
import {
  scan, findOpenings, describeScan, driveToward, explore, normAngle,
  sweepScan, resampleSweep, spinRpmForRate, encoderRotationDeg,
} from '../js/scan.js';
import { SimRobot, SIM_ROOM, SIM_OBSTACLES, raycast, collides } from '../js/robot-sim.js';
import { CommandBus, makeCommand } from '../js/bus.js';

// Fake bus: tracks heading from turn commands, answers distance from a function of it.
function fakeBus({ dist = () => 100, failTurnAt = -1, failRead = () => false, onTurn } = {}) {
  const bus = { heading: 0, turns: [], stops: 0, reads: 0, straights: [] };
  bus.submit = async (c) => {
    if (c.cmd === 'read') {
      bus.reads++;
      return failRead(bus.heading) ? { ok: false, error: 'timeout' } : { ok: true, value: dist(normAngle(bus.heading)) };
    }
    if (c.cmd === 'turn') {
      if (bus.turns.length === failTurnAt) return { ok: false, error: 'not connected' };
      bus.turns.push(c.args.deg);
      bus.heading += c.args.deg;
      onTurn?.(bus);
      return { ok: true };
    }
    if (c.cmd === 'straight') { bus.straights.push(c.args.cm); return { ok: true }; }
    return { ok: true };
  };
  bus.stop = async () => { bus.stops++; return { ok: true }; };
  return bus;
}

test('normAngle', () => {
  assert.equal(normAngle(0), 0);
  assert.equal(normAngle(210), -150);
  assert.equal(normAngle(-180), 180);
  assert.equal(normAngle(540), 180);
  assert.equal(normAngle(-30), -30);
  assert.equal(normAngle(360), 0);
});

test('scan: angles, full rotation, returns to start heading', async () => {
  const bus = fakeBus({ dist: (a) => 100 + a });
  const seen = [];
  const r = await scan(bus, { steps: 12, settleMs: 0, makeCommand, onPoint: (p, i) => seen.push(i) });
  assert.equal(r.steps, 12);
  assert.equal(r.points.length, 12);
  assert.deepEqual(r.points.map((p) => p.angle), [0, 30, 60, 90, 120, 150, 180, -150, -120, -90, -60, -30]);
  assert.deepEqual(r.points.map((p) => p.cm), r.points.map((p) => 100 + (p.angle === 180 ? 180 : p.angle)));
  assert.equal(bus.turns.reduce((a, b) => a + b, 0), 360);
  assert.ok(bus.turns.every((d) => d === 30));
  assert.equal(normAngle(bus.heading), 0);
  assert.deepEqual(seen, [...Array(12).keys()]);
  assert.ok(r.durationMs >= 0 && r.startedAt > 0);
});

test('scan: odd step count still sums to 360', async () => {
  const bus = fakeBus();
  const r = await scan(bus, { steps: 7, settleMs: 0 });
  assert.ok(Math.abs(bus.turns.reduce((a, b) => a + b, 0) - 360) < 1e-9);
  assert.equal(r.points[1].angle, normAngle(360 / 7));
});

test('scan: failed reads become null, scan continues', async () => {
  const bus = fakeBus({ failRead: (h) => h === 90 });
  const r = await scan(bus, { steps: 4, settleMs: 0 });
  assert.deepEqual(r.points.map((p) => p.cm), [100, null, 100, 100]);
});

test('scan: failed turn aborts with a clear error and stops', async () => {
  const bus = fakeBus({ failTurnAt: 2 });
  await assert.rejects(scan(bus, { steps: 12, settleMs: 0 }), /turn 3\/12 failed \(not connected\)/);
  assert.equal(bus.stops, 1);
  assert.equal(bus.reads, 3);
});

test('scan: abort stops the robot and throws AbortError', async () => {
  const ac = new AbortController();
  const bus = fakeBus({ onTurn: (b) => { if (b.turns.length === 3) ac.abort(); } });
  await assert.rejects(scan(bus, { steps: 12, settleMs: 0, signal: ac.signal }), { name: 'AbortError' });
  assert.equal(bus.stops, 1);
  assert.equal(bus.turns.length, 3);

  const pre = new AbortController(); pre.abort();
  const bus2 = fakeBus();
  await assert.rejects(scan(bus2, { signal: pre.signal, settleMs: 0 }), { name: 'AbortError' });
  assert.equal(bus2.reads, 0);
});

test('scan: abort during the settle delay', async () => {
  const ac = new AbortController();
  const bus = fakeBus();
  const p = scan(bus, { steps: 12, settleMs: 200, signal: ac.signal });
  setTimeout(() => ac.abort(), 30);
  await assert.rejects(p, { name: 'AbortError' });
  assert.equal(bus.turns.length, 1);
  assert.equal(bus.stops, 1);
});

const pts = (cms) => cms.map((cm, i) => ({ angle: normAngle(i * (360 / cms.length)), cm }));

test('findOpenings: groups, widths, min cm, sorting', () => {
  //           0   30  60   90   120  150 180 -150 -120 -90 -60 -30
  const o = findOpenings(pts([20, 30, 120, 300, 80, 20, 20, 60, 20, 20, 20, 20]));
  assert.equal(o.length, 2);
  assert.deepEqual({ angle: o[0].angle, widthDeg: o[0].widthDeg, cm: o[0].cm }, { angle: 90, widthDeg: 90, cm: 80 });
  assert.deepEqual({ angle: o[1].angle, widthDeg: o[1].widthDeg, cm: o[1].cm }, { angle: -150, widthDeg: 30, cm: 60 });
  assert.ok(o[0].score > o[1].score);
});

test('findOpenings: wrap-around at +-180 and at 0', () => {
  // open at 150, 180, -150
  let o = findOpenings(pts([20, 20, 20, 20, 20, 100, 100, 100, 20, 20, 20, 20]));
  assert.equal(o.length, 1);
  assert.equal(o[0].angle, 180);
  assert.equal(o[0].widthDeg, 90);
  // open at -60, -30, 0, 30 (run spans the array start)
  o = findOpenings(pts([200, 150, 20, 20, 20, 20, 20, 20, 20, 20, 90, 100]));
  assert.equal(o.length, 1);
  assert.equal(o[0].angle, -15);
  assert.equal(o[0].widthDeg, 120);
  assert.equal(o[0].cm, 90);
});

test('findOpenings: all open, none open, nulls, minCm', () => {
  assert.deepEqual(findOpenings(pts([300, 300, 300, 300])).map((o) => o.widthDeg), [360]);
  assert.deepEqual(findOpenings(pts([10, 10, 10, 10])), []);
  assert.deepEqual(findOpenings(pts([null, 100, null, null])).map((o) => o.angle), [90]);
  assert.equal(findOpenings(pts([60, 10, 10, 10]), { minCm: 70 }).length, 0);
  assert.deepEqual(findOpenings([]), []);
  // order of input does not matter
  const shuffled = pts([20, 30, 120, 300, 80, 20, 20, 60, 20, 20, 20, 20]).reverse();
  assert.equal(findOpenings(shuffled)[0].angle, 90);
});

test('findOpenings: prefers wide and deep openings', () => {
  // narrow very deep at 90 vs wide medium at -90
  const o = findOpenings(pts([20, 20, 20, 300, 20, 20, 20, 20, 100, 100, 100, 20]));
  assert.equal(o[0].angle, -90);
});

test('describeScan: dense sweeps are summarised as at most 12 directions', () => {
  const dense = Array.from({ length: 120 }, (_, i) => ({ angle: normAngle(i * 3), cm: i * 3 === 90 ? 25 : 150 }));
  const s = describeScan(dense);
  const dirs = s.match(/: (.*?)\. Nearest/)[1].split(' ');
  assert.equal(dirs.length, 12);
  assert.ok(s.length < 400, `length ${s.length}`);
  assert.match(s, /90:25/);
  assert.match(s, /Nearest 25cm at 90°/);
});

test('describeScan: compact and explicit', () => {
  const p = pts([85, 40, 120, 300, 80, 20, null, 60, 20, 20, 20, 20]);
  const s = describeScan(p);
  assert.ok(s.length < 400, `length ${s.length}`);
  assert.match(s, /\+ = right\/clockwise/);
  assert.match(s, /0:85 30:40 60:120 90:300\+/);
  assert.match(s, /180:\?/);
  assert.match(s, /Nearest 20cm/);
  assert.match(s, /Open: 90° w90 >=80cm/);
  assert.match(describeScan(pts([10, 10, 10, 10])), /Open: none\./);
});

test('driveToward: turns, re-measures and clamps the distance', async () => {
  let bus = fakeBus({ dist: () => 300 });
  let r = await driveToward(bus, { angle: 60, cm: 150 });
  assert.deepEqual(bus.turns, [60]);
  assert.deepEqual(bus.straights, [100]);
  assert.equal(r.ok, true); assert.equal(r.droveCm, 100); assert.equal(r.turnedDeg, 60);

  bus = fakeBus({ dist: () => 70 });
  r = await driveToward(bus, { angle: 0, cm: 80 });
  assert.deepEqual(bus.turns, []);
  assert.deepEqual(bus.straights, [50]);

  bus = fakeBus({ dist: () => 300 });
  await driveToward(bus, { angle: -200, cm: 30 });
  assert.deepEqual(bus.turns, [160]);
  assert.deepEqual(bus.straights, [30]);

  bus = fakeBus({ dist: () => 22 });
  r = await driveToward(bus, { angle: 0 });
  assert.equal(r.ok, true); assert.equal(r.droveCm, 0); assert.match(r.note, /blocked/);
  assert.deepEqual(bus.straights, []);

  bus = fakeBus({ failTurnAt: 0 });
  r = await driveToward(bus, { angle: 90 });
  assert.equal(r.ok, false); assert.match(r.error, /turn failed/);
});

test('explore: scans and moves toward the best opening', async () => {
  const bus = fakeBus({ dist: (a) => (a === 90 ? 300 : 20) });
  const r = await explore(bus, { maxMoves: 2, settleMs: 0 });
  assert.equal(r.moves, 2);
  assert.equal(r.log[0].opening.angle, 90);
  assert.deepEqual(bus.straights, [40, 40]);
});

// --- simulator ---------------------------------------------------------

const stub = () => {};

test('sim raycast and collision geometry', () => {
  const pouf = SIM_OBSTACLES.find((o) => o.label === 'pouf');
  assert.ok(pouf && SIM_OBSTACLES.length >= 3);
  // from 50 cm right of the pouf centre, facing left
  assert.ok(Math.abs(raycast(pouf.x + 50, pouf.y, 180) - (50 - pouf.r)) < 1e-6);
  // empty room: wall distance
  assert.ok(Math.abs(raycast(150, 100, -90, SIM_ROOM, []) - 100) < 1e-6);
  assert.equal(collides(pouf.x + pouf.r + 5, pouf.y), true);
  assert.equal(collides(150, 100), false);
});

test('sim: distance sees obstacles, movement stops at contact', async () => {
  const sim = new SimRobot({ log: stub, onStatus: stub, timeScale: 20 });
  await sim.connect();
  try {
    const pouf = SIM_OBSTACLES.find((o) => o.label === 'pouf');
    Object.assign(sim.state, { x: pouf.x + 50, y: pouf.y, heading: 180 });
    assert.equal(await sim.distance(), 50 - pouf.r - 6);
    await sim.straight(60, { wait: true });
    assert.ok(Math.abs(sim.state.x - (pouf.x + pouf.r + 9)) < 1.5, `x ${sim.state.x}`);
    assert.ok(await sim.distance() <= 4);
    await sim.straight(-20, { wait: true }); // can back away from contact
    assert.ok(sim.state.x > pouf.x + pouf.r + 25);
  } finally {
    await sim.disconnect();
  }
});

test('integration: scan with SimRobot + CommandBus finds the pouf ahead and the chair behind', async () => {
  const sim = new SimRobot({ log: stub, onStatus: stub, timeScale: 20 });
  const bus = new CommandBus({ log() {} });
  bus.setRobot(sim);
  await sim.connect();
  try {
    // facing left (-x) toward the pouf; the chair is behind at the same height
    Object.assign(sim.state, { x: 120, y: 150, heading: 180 });
    const { points } = await scan(bus, { steps: 12, settleMs: 0 });
    const at = (a) => points.find((p) => p.angle === a).cm;
    const nearest = [...points].sort((a, b) => a.cm - b.cm)[0];
    assert.equal(nearest.angle, 0);
    assert.ok(Math.abs(at(0) - 29) < 2, `front ${at(0)}`);
    assert.ok(Math.abs(at(180) - 102) < 3, `back ${at(180)}`);   // chair at x = 228
    assert.ok(Math.abs(at(-90) - 44) < 3, `left ${at(-90)}`);    // floor-side wall
    assert.ok(at(90) > 90, `right ${at(90)}`);                   // open toward the sofa and top wall
    assert.ok(Math.abs(normAngle(sim.state.heading - 180)) < 1, `heading ${sim.state.heading}`);
    const best = findOpenings(points)[0];
    assert.ok(best.angle > 0, `best ${best.angle}`);
    const d = await driveToward(bus, { angle: best.angle, cm: 30 });
    assert.equal(d.ok, true);
    assert.equal(d.droveCm, 30);
  } finally {
    await sim.disconnect();
  }
});

// --- continuous sweep ----------------------------------------------------

test('spin and encoder geometry', () => {
  // 45 deg/s: rim speed = 2 pi 6 cm / 8 s = 4.71 cm/s, i.e. 13.85 RPM on a 6.5 cm wheel
  assert.ok(Math.abs(spinRpmForRate(45) - 13.846) < 0.01);
  // one full robot turn: each wheel rolls pi * 12 cm = 664.6 degrees of wheel rotation
  const wheelDeg = (Math.PI * 12) / (Math.PI * 6.5) * 360;
  assert.ok(Math.abs(encoderRotationDeg(wheelDeg, -wheelDeg) - 360) < 1e-9);
});

const pos360 = (a) => ((a % 360) + 360) % 360;
// Ramp room: the distance encodes the true direction (50 cm at 0°, 229.5 cm at 359°).
const rampRoom = (h) => 50 + pos360(h) / 2;
const truthOf = (cm) => (cm - 50) * 2;

// A robot spinning in real time `scale` times faster than commanded, with a
// sampler that reads distance `latencyMs` before the yaw.
function sweepWorld({ scale = 40, latencyMs = 0, yawSign = 1, yawOffset = 170, yaw = true, enc = false,
  room = rampRoom, periodMs = 5, failSampleAt = -1, failDriveAt = -1, onSample } = {}) {
  const w = { log: [{ t: performance.now(), heading: 0, rate: 0 }], drives: [], stops: 0, events: [], samples: 0 };
  w.heading = (t) => {
    let e = w.log[0];
    for (const x of w.log) if (x.t <= t) e = x;
    return e.heading + (e.rate * Math.max(0, t - e.t)) / 1000;
  };
  const setRate = (rate) => {
    const t = performance.now();
    w.log.push({ t, heading: w.heading(t), rate });
  };
  w.bus = {
    submit: async (c) => {
      if (c.cmd === 'drive') {
        if (w.drives.length === failDriveAt) return { ok: false, error: 'cancelled by stop' };
        w.drives.push(c.args);
        w.events.push('drive');
        const rpm = (c.args.left - c.args.right) / 2;
        setRate(((rpm * 360 * 6.5) / (60 * 12)) * scale);
      } else if (c.cmd === 'stop') {
        w.stops++; w.events.push('stop'); setRate(0);
      }
      return { ok: true };
    },
    stop: async () => { w.stops++; w.events.push('stop'); setRate(0); return { ok: true }; },
  };
  w.sample = async () => {
    await new Promise((r) => setTimeout(r, periodMs));
    if (w.samples++ === failSampleAt) throw new Error('link lost');
    onSample?.(w);
    const t = performance.now();
    const h = w.heading(t);
    const s = { t, distanceCm: Math.round(room(w.heading(t - latencyMs)) * 10) / 10 };
    if (yaw) s.yaw = normAngle(yawSign * h + yawOffset);
    if (enc) {
      const wheel = (h * 12) / 6.5;
      Object.assign(s, { encL: 1000 + wheel, encR: -500 - wheel });
    }
    return s;
  };
  return w;
}

const angleErrors = (points) => points
  .filter((p) => { const tr = truthOf(p.cm); return tr > 6 && tr < 354; })
  .map((p) => Math.abs(normAngle(p.angle - truthOf(p.cm))));

test('sweepScan: yaw with latency compensation, wrap at ±180, full coverage, stops', async () => {
  const w = sweepWorld({ latencyMs: 20 });
  const seen = [];
  const r = await sweepScan(w.bus, { returnToStart: false, sample: w.sample, latencyMs: 20, onPoint: (p) => seen.push(p) });
  assert.equal(r.method, 'sweep');
  assert.equal(r.rotationSource, 'yaw');
  assert.equal(r.yawSign, 1);
  assert.equal(r.coverageDeg, 360);
  assert.ok(Math.abs(r.totalTurnDeg - w.heading(performance.now())) < 2, `turned ${r.totalTurnDeg}`);
  assert.equal(r.turnedDeg, normAngle(r.totalTurnDeg));
  assert.ok(r.turnedDeg >= 15 && r.turnedDeg < 60, `net ${r.turnedDeg}`);
  assert.ok(w.drives.every((d) => d.leg === true));
  assert.ok(r.points.length >= 40, `points ${r.points.length}`);
  assert.ok(seen.length >= r.points.length);
  const err = angleErrors(r.points);
  assert.ok(Math.max(...err) <= 3, `max error ${Math.max(...err)}`);
  // sorted, normalised, distinct
  for (let i = 1; i < r.points.length; i++) assert.ok(r.points[i].angle > r.points[i - 1].angle);
  assert.ok(r.points.every((p) => p.angle > -180 && p.angle <= 180));
  // clockwise spin: left forward, right backward; ends with a stop and no drive after it
  assert.ok(w.drives.every((d) => d.left > 0 && d.right === -d.left));
  assert.equal(w.events.at(-1), 'stop');
  assert.equal(w.stops, 1);
});

test('sweepScan: without compensation the same latency shifts the angles', async () => {
  const w = sweepWorld({ latencyMs: 20 });
  const r = await sweepScan(w.bus, { returnToStart: false, sample: w.sample, latencyMs: 0 });
  const err = angleErrors(r.points);
  const mean = err.reduce((a, b) => a + b, 0) / err.length;
  assert.ok(mean > 5, `mean error ${mean}`);
});

test('sweepScan: trusts the calibrated gyro and reports a spin against the command', async () => {
  // gyro and spin disagree (field test 2026-10-04: swapped wheel mapping); the
  // angles follow the gyro, never a guessed flip, and the sweep says so
  const w = sweepWorld({ yawSign: -1, yawOffset: -100 });
  const r = await sweepScan(w.bus, { returnToStart: false, sample: w.sample, latencyMs: 0 });
  assert.equal(r.reversed, true);
  assert.equal(r.yawSign, 1);
  assert.equal(r.coverageDeg, 360);
});

test('sweepScan: counterclockwise with negative speed', async () => {
  const w = sweepWorld();
  const r = await sweepScan(w.bus, { returnToStart: false, sample: w.sample, latencyMs: 0, speedDegS: -45 });
  assert.ok(w.drives.every((d) => d.left < 0 && d.right === -d.left));
  assert.equal(r.reversed, false);
  assert.ok(r.totalTurnDeg <= -380, `turned ${r.totalTurnDeg}`);
  assert.ok(r.turnedDeg < 0 && r.turnedDeg > -60, `net ${r.turnedDeg}`);
  const err = angleErrors(r.points);
  assert.ok(Math.max(...err) <= 3, `max error ${Math.max(...err)}`);
});

test('sweepScan: falls back to encoders, then to commanded rate x time', async () => {
  let w = sweepWorld({ yaw: false, enc: true });
  let r = await sweepScan(w.bus, { returnToStart: false, sample: w.sample, latencyMs: 0 });
  assert.equal(r.rotationSource, 'encoder');
  assert.equal(r.coverageDeg, 360);
  let err = angleErrors(r.points);
  assert.ok(Math.max(...err) <= 3, `encoder max error ${Math.max(...err)}`);

  // time: tell the sweep a track width that matches the faster test world
  w = sweepWorld({ yaw: false, scale: 6 });
  r = await sweepScan(w.bus, { returnToStart: false, sample: w.sample, latencyMs: 0, trackCm: 2, speedDegS: 600 });
  assert.equal(r.rotationSource, 'time');
  assert.equal(r.coverageDeg, 360);
  err = angleErrors(r.points);
  const mean = err.reduce((a, b) => a + b, 0) / err.length;
  assert.ok(mean <= 6, `time mean error ${mean}`); // coarse: command delays are not modelled
  assert.equal(w.events.at(-1), 'stop');
});

test('sweepScan: drops invalid readings, keeps no-echo, merges duplicates', async () => {
  const room = (h) => (pos360(h) < 90 ? 1 : pos360(h) < 180 ? 300 : 120);
  const w = sweepWorld({ room });
  const r = await sweepScan(w.bus, { returnToStart: false, sample: w.sample, latencyMs: 0, mergeDeg: 5 });
  assert.ok(r.points.every((p) => p.cm > 2));
  assert.ok(!r.points.some((p) => p.angle > 3 && p.angle < 87), 'invalid sector dropped');
  assert.ok(r.points.some((p) => p.cm === 300));
  for (let i = 1; i < r.points.length; i++) assert.ok(r.points[i].angle - r.points[i - 1].angle >= 2.5);
});

test('sweepScan: abort stops the robot and throws AbortError', async () => {
  const ac = new AbortController();
  const w = sweepWorld({ onSample: (x) => { if (x.samples === 10) ac.abort(); } });
  await assert.rejects(sweepScan(w.bus, { sample: w.sample, signal: ac.signal }), { name: 'AbortError' });
  assert.ok(w.stops >= 1);
  assert.equal(w.events.at(-1), 'stop');

  const pre = new AbortController(); pre.abort();
  const w2 = sweepWorld();
  await assert.rejects(sweepScan(w2.bus, { sample: w2.sample, signal: pre.signal }), { name: 'AbortError' });
  assert.equal(w2.samples, 0);
  assert.equal(w2.events.at(-1), 'stop');
});

test('sweepScan: a failing sample, a bus stop or a timeout still stops the robot', async () => {
  let w = sweepWorld({ failSampleAt: 5 });
  await assert.rejects(sweepScan(w.bus, { sample: w.sample }), /link lost/);
  assert.equal(w.events.at(-1), 'stop');

  w = sweepWorld({ failDriveAt: 2, periodMs: 40 });
  await assert.rejects(sweepScan(w.bus, { sample: w.sample, keepaliveMs: 30 }), { name: 'AbortError' });
  assert.equal(w.events.at(-1), 'stop');

  w = sweepWorld();
  const hang = () => new Promise(() => {});
  await assert.rejects(sweepScan(w.bus, { sample: hang, sampleTimeoutMs: 50 }), /sample timeout/);
  assert.equal(w.events.at(-1), 'stop');

  w = sweepWorld({ scale: 1 });
  const r = await sweepScan(w.bus, { returnToStart: false, sample: w.sample, maxDurationMs: 150 });
  assert.ok(r.coverageDeg < 360, `coverage ${r.coverageDeg}`);
  assert.equal(w.events.at(-1), 'stop');
  assert.equal(w.stops, 1);
});

test('resampleSweep: even bins, nearest echo per bin, null for empty bins', () => {
  const pts = [
    { angle: 0.8, cm: 80 }, { angle: -1.2, cm: 60 }, { angle: 4, cm: 100 },
    { angle: 179, cm: 40 }, { angle: -179.5, cm: 30 }, { angle: 90, cm: null },
  ];
  const b = resampleSweep(pts, 3);
  assert.equal(b.length, 120);
  assert.deepEqual(b.slice(0, 3).map((p) => p.angle), [0, 3, 6]);
  assert.equal(b.find((p) => p.angle === 0).cm, 60);
  assert.equal(b.find((p) => p.angle === 3).cm, 100);
  assert.equal(b.find((p) => p.angle === 180).cm, 30);
  assert.equal(b.find((p) => p.angle === 90).cm, null);
  assert.equal(resampleSweep([], 30).length, 12);
});

test('findOpenings: dense uneven sweep points', () => {
  // open from 60 to 120 degrees, readings every 2.5 to 4.5 degrees
  const pts = [];
  for (let a = 0, i = 0; a < 360; a += i++ % 2 ? 2.5 : 4.5) pts.push({ angle: normAngle(a), cm: a >= 60 && a <= 120 ? 200 : 30 });
  const o = findOpenings(pts);
  assert.equal(o.length, 1);
  assert.ok(Math.abs(o[0].angle - 90) <= 3, `angle ${o[0].angle}`);
  assert.ok(Math.abs(o[0].widthDeg - 63) <= 5, `width ${o[0].widthDeg}`);
});

test('integration: sweepScan with SimRobot and the motion sampler', async (t) => {
  const motion = await import('../js/motion.js').catch(() => null);
  if (!motion?.makeSimSampler || typeof SimRobot.prototype.sensorSample !== 'function') {
    t.skip('js/motion.js makeSimSampler or SimRobot.sensorSample not available yet');
    return;
  }
  const sim = new SimRobot({ log: stub, onStatus: stub, timeScale: 25 });
  const bus = new CommandBus({ log() {} });
  bus.setRobot(sim);
  await sim.connect();
  try {
    // facing left (-x) toward the pouf, chair behind (as in the step scan test)
    Object.assign(sim.state, { x: 120, y: 150, heading: 180 });
    const r = await sweepScan(bus, { returnToStart: false, sample: motion.makeSimSampler(sim), makeCommand: bus.stamped() });
    assert.equal(r.rotationSource, 'yaw');
    assert.ok(r.coverageDeg >= 359, `coverage ${r.coverageDeg}`);
    const near = (a) => Math.min(...r.points.filter((p) => Math.abs(normAngle(p.angle - a)) <= 6).map((p) => p.cm));
    assert.ok(Math.abs(near(0) - 29) < 4, `front ${near(0)}`);
    assert.ok(Math.abs(near(180) - 102) < 6, `back ${near(180)}`);
    assert.ok(Math.abs(near(-90) - 44) < 5, `left ${near(-90)}`);
    assert.ok(near(90) > 90, `right ${near(90)}`);
  } finally {
    await sim.disconnect();
  }
});

test('findOpenings ignores empty readings inside an open area', () => {
  const pts = [];
  for (let a = -180; a < 180; a += 5) pts.push({ angle: a, cm: a % 20 === 0 ? null : 120 });
  const o = findOpenings(pts);
  assert.equal(o.length, 1);
  assert.equal(o[0].widthDeg, 360);
});

test('sweepScan: turns back to the starting direction afterwards (SimRobot)', async () => {
  const { SimRobot } = await import('../js/robot-sim.js');
  const { CommandBus } = await import('../js/bus.js');
  const { makeSimSampler } = await import('../js/motion.js');
  const stub = () => {};
  const sim = new SimRobot({ log: stub, onStatus: stub, timeScale: 15 });
  const bus = new CommandBus({ log: stub });
  bus.setRobot(sim);
  await sim.connect();
  try {
    const h0 = sim.state.heading;
    const r = await sweepScan(bus, { sample: makeSimSampler(sim), latencyMs: 0 });
    assert.ok(r.totalTurnDeg >= 380, 'the sweep itself still covers more than a full turn');
    assert.ok(r.returned?.ok, `return turn ${JSON.stringify(r.returned)}`);
    const off = ((sim.state.heading - h0) % 360 + 540) % 360 - 180;
    assert.ok(Math.abs(off) <= 4, `ends ${off.toFixed(1)}° from the start heading`);
    assert.ok(Math.abs(r.turnedDeg - off) <= 3, `reported ${r.turnedDeg}, true ${off.toFixed(1)}`);
  } finally { await sim.disconnect(); }
});
