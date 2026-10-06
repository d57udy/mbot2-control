// Run with: npm test
import test from 'node:test';
import assert from 'node:assert/strict';
import { GridMap } from '../js/gridmap.js';
import { scoreScan, matchScan, relocalize, fusePose, likelihoodField } from '../js/localize.js';
import { SimRobot } from '../js/robot-sim.js';
import { Navigator } from '../js/navigate.js';
import { PoseTracker } from '../js/pose.js';
import { scan } from '../js/scan.js';
import { CommandBus } from '../js/bus.js';
import { makeSimSampler } from '../js/motion.js';

const stub = () => {};
const angErr = (a, b) => Math.abs((((a - b) % 360) + 540) % 360 - 180);
const d = (a, b) => Math.hypot(a.x - b.x, a.y - b.y);

// Synthetic map from an inside(x, y) test: inside cells free, a 1-cell rim occupied.
function mapOf(inside) {
  const m = new GridMap({});
  for (let k = 0; k < m.L.length; k++) {
    const c = m.centre(k);
    if (inside(c.x, c.y)) { m.L[k] = -2; continue; }
    for (const [dx, dy] of [[5, 0], [-5, 0], [0, 5], [0, -5], [5, 5], [-5, -5], [5, -5], [-5, 5]]) {
      if (inside(c.x + dx, c.y + dy)) { m.L[k] = 3; break; }
    }
  }
  m.touch();
  return m;
}

// Exact beams against inside(): sensor 6 cm ahead, no echo beyond 250 cm.
function castScan(inside, p, n = 36) {
  return Array.from({ length: n }, (_, i) => {
    const a = (i * 360) / n, t = ((p.heading + a) * Math.PI) / 180;
    const sx = p.x + 6 * Math.sin(t), sy = p.y + 6 * Math.cos(t);
    let r = 0;
    while (r < 300 && inside(sx + r * Math.sin(t), sy + r * Math.cos(t))) r += 0.5;
    return { angle: a > 180 ? a - 360 : a, cm: r >= 250 ? 300 : Math.round(r) };
  });
}

// L-shaped room with a box: asymmetric.
const lRoom = (x, y) => Math.abs(x) < 150 && Math.abs(y) < 100 && !(x > 50 && y > 20)
  && !(x > -90 && x < -60 && y > -60 && y < -30);
const square = (x, y) => Math.abs(x) < 120 && Math.abs(y) < 120;

test('localize: score is highest at the true pose; sign conventions', () => {
  const m = mapOf(lRoom);
  const truth = { x: 20, y: -30, heading: 25 };
  const pts = castScan(lRoom, truth);
  const s = scoreScan(m, truth, pts);
  assert.ok(s > scoreScan(m, { ...truth, x: 35 }, pts));
  assert.ok(s > scoreScan(m, { ...truth, heading: 35 }, pts));
  // a single beam straight ahead (heading 90 = +x) ending on the right wall at x = 150
  const one = [{ angle: 0, cm: 150 - 6 - 0 }];
  assert.ok(scoreScan(m, { x: 0, y: 0, heading: 90 }, one) > 0.7);
  assert.ok(scoreScan(m, { x: 0, y: 0, heading: 0 }, one) < 0.2);   // +y: past the wall at y = 100
  // the field is cached per map version
  assert.equal(likelihoodField(m), likelihoodField(m));
  // nulls are ignored
  assert.equal(scoreScan(m, truth, [{ angle: 0, cm: null }]), 0);
});

test('localize: matchScan recovers a 15 cm / 10 deg offset', () => {
  const m = mapOf(lRoom);
  for (const truth of [{ x: 20, y: -30, heading: 25 }, { x: -100, y: 40, heading: -120 }]) {
    const pts = castScan(lRoom, truth);
    const guess = { x: truth.x + 12, y: truth.y - 9, heading: truth.heading - 10 };
    const t0 = performance.now();
    const r = matchScan(m, guess, pts);
    const ms = performance.now() - t0;
    assert.ok(d(r.pose, truth) < 5, `pose ${JSON.stringify(r.pose)} truth ${JSON.stringify(truth)}`);
    assert.ok(angErr(r.pose.heading, truth.heading) < 3);
    assert.ok(r.confidence > 0.6, `confidence ${r.confidence}`);
    assert.ok(ms < 100, `${ms} ms`);
  }
  // nothing usable: the guess comes back with confidence 0
  const r = matchScan(m, { x: 1, y: 2, heading: 3 }, [{ angle: 0, cm: null }]);
  assert.deepEqual(r.pose, { x: 1, y: 2, heading: 3 });
  assert.equal(r.confidence, 0);
});

test('localize: a featureless corridor gives low confidence along it', () => {
  const corridor = (x, y) => Math.abs(x) < 40 && Math.abs(y) < 390;
  const m = mapOf(corridor);
  const pts = castScan(corridor, { x: 0, y: 0, heading: 0 }).map((p) => ({ ...p, cm: p.cm >= 250 ? 300 : p.cm }));
  const r = matchScan(m, { x: 5, y: 10, heading: 3 }, pts);
  assert.ok(Math.abs(r.pose.x) < 5 && angErr(r.pose.heading, 0) < 3);
  assert.ok(r.confidence < 0.3, `confidence ${r.confidence}`);
});

test('localize: relocalize finds the pose in an asymmetric room, not in a symmetric one', () => {
  const m = mapOf(lRoom);
  const truth = { x: 20, y: -30, heading: 25 };
  const t0 = performance.now();
  const r = relocalize(m, castScan(lRoom, truth));
  const ms = performance.now() - t0;
  assert.ok(d(r.pose, truth) < 5 && angErr(r.pose.heading, truth.heading) < 3, JSON.stringify(r.pose));
  assert.ok(r.confidence > 0.5, `confidence ${r.confidence}`);
  assert.ok(r.runnerUp && d(r.runnerUp.pose, truth) > 30 || angErr(r.runnerUp.pose.heading, truth.heading) > 30);
  assert.ok(ms < 500, `${ms} ms`);

  const sym = mapOf(square);
  const s = relocalize(sym, castScan(square, { x: 40, y: 30, heading: 10 }));
  assert.ok(s.confidence < 0.2, `confidence ${s.confidence}`);
  // the four rotations of the square fit equally well; the answer is one of them
  const ok = [0, 90, 180, -90].some((a) => {
    const t = (a * Math.PI) / 180;
    const p = { x: 40 * Math.cos(t) + 30 * Math.sin(t), y: -40 * Math.sin(t) + 30 * Math.cos(t) };
    return d(s.pose, p) < 6 && angErr(s.pose.heading, 10 + a) < 4;
  });
  assert.ok(ok, JSON.stringify(s.pose));

  assert.equal(relocalize(new GridMap({}), castScan(lRoom, truth)).pose, null);
});

test('localize: relocalize on a full 800 x 800 cm map stays under 500 ms', () => {
  const big = (x, y) => Math.abs(x) < 390 && Math.abs(y) < 390 && !(x > 100 && x < 160 && y > -50 && y < 30);
  const m = mapOf(big);
  const t0 = performance.now();
  const r = relocalize(m, castScan(big, { x: 50, y: 0, heading: 45 }));
  const ms = performance.now() - t0;
  assert.ok(ms < 500, `${ms} ms`);
  assert.ok(r.pose);
});

// Sim frame: map x = sim x - 150, map y = 100 - sim y, map heading = sim heading + 90.
async function simScan(sim, p, n) {
  const pts = [];
  for (let i = 0; i < n; i++) {
    const a = (i * 360) / n;
    Object.assign(sim.state, { x: p.x + 150, y: 100 - p.y, heading: p.heading + a - 90 });
    pts.push({ angle: a > 180 ? a - 360 : a, cm: await sim.distance() });
  }
  return pts;
}

test('localize: real sim scans, map built by integration, match after drift', async () => {
  const sim = new SimRobot({ log: stub, onStatus: stub });
  const m = new GridMap({});
  for (const p of [{ x: 0, y: 0 }, { x: 60, y: 0 }, { x: -60, y: 10 }, { x: 0, y: -60 }, { x: 100, y: 30 }, { x: -100, y: -20 }]) {
    m.integrateScan({ ...p, heading: 0 }, await simScan(sim, { ...p, heading: 0 }, 24), { freeBeamDeg: 15 });
  }
  for (const truth of [{ x: 30, y: -20, heading: 15 }, { x: -40, y: 20, heading: -70 }]) {
    const pts = await simScan(sim, truth, 36);
    const guess = { x: truth.x - 15, y: truth.y + 5, heading: truth.heading + 10 };
    const r = matchScan(m, guess, pts);
    assert.ok(d(r.pose, truth) < 5, `pose ${JSON.stringify(r.pose)} truth ${JSON.stringify(truth)}`);
    assert.ok(angErr(r.pose.heading, truth.heading) < 3, `heading ${r.pose.heading}`);
    assert.ok(r.confidence > 0.4, `confidence ${r.confidence}`);
    // 12 beams as the navigator scans: still close
    const r12 = matchScan(m, guess, await simScan(sim, truth, 12));
    assert.ok(d(r12.pose, truth) < 8 && angErr(r12.pose.heading, truth.heading) < 5, JSON.stringify(r12));
  }
  const g = relocalize(m, await simScan(sim, { x: 30, y: -20, heading: 15 }, 36));
  assert.ok(d(g.pose, { x: 30, y: -20 }) < 6 && angErr(g.pose.heading, 15) < 4, JSON.stringify(g));
});

test('localize: fusePose blends by confidence, rejects weak matches, honours the gyro', () => {
  const odom = { x: 0, y: 0, heading: 10 };
  const match = { pose: { x: 10, y: -10, heading: 20 }, confidence: 1 };
  let f = fusePose(odom, match, { odomWeight: 0.5 });
  assert.equal(f.source, 'scan');
  assert.deepEqual(f.pose, { x: 5, y: -5, heading: 15 });
  f = fusePose(odom, { ...match, confidence: 0.2 }, { minConfidence: 0.4 });
  assert.equal(f.source, 'odom');
  assert.deepEqual(f.pose, odom);
  assert.match(f.reason, /low confidence/);
  assert.equal(fusePose(odom, null).source, 'odom');
  // gyro: sets the heading, vetoes a match that disagrees
  f = fusePose(odom, match, { yawDeg: 18, odomWeight: 0 });
  assert.equal(f.source, 'scan');
  assert.deepEqual(f.pose, { x: 10, y: -10, heading: 18 });
  f = fusePose(odom, { ...match, pose: { ...match.pose, heading: 60 } }, { yawDeg: 12 });
  assert.equal(f.source, 'odom');
  assert.equal(f.pose.heading, 12);
  // heading blend wraps across +-180
  f = fusePose({ x: 0, y: 0, heading: 170 }, { pose: { x: 0, y: 0, heading: -170 }, confidence: 1 }, { odomWeight: 0.5 });
  assert.ok(angErr(f.pose.heading, 180) < 1e-9);
});

// Realistic ultrasonic for a SimRobot instance (sim frame: x right, y down,
// heading 0 = +x, clockwise). 25 deg cone, nearest echo wins; surfaces hit at
// more than specDeg from their normal send no echo back; thin posts answer
// near the axis only; gaussian-ish noise; a fraction of random short readings;
// no echo -> 190 (what the real sensor reports beyond its range).
function realisticSensor(sim, { beamDeg = 25, specDeg = 55, noiseCm = 1.5, falseRate = 0.05, rand = Math.random } = {}) {
  const SENSOR_CM = 6;
  const cast = (ox, oy, dx, dy) => {
    let best = { t: Infinity, inc: 0, thin: false };
    const hit = (t, nx, ny) => { if (t > 1e-6 && t < best.t) best = { t, inc: Math.acos(Math.min(1, Math.abs(dx * nx + dy * ny))) * 180 / Math.PI, thin: false }; };
    const { w, h } = sim.room;
    if (dx > 0) hit((w - ox) / dx, 1, 0); else if (dx < 0) hit(-ox / dx, 1, 0);
    if (dy > 0) hit((h - oy) / dy, 0, 1); else if (dy < 0) hit(-oy / dy, 0, 1);
    for (const o of sim.obstacles) {
      if (o.low) continue;
      if (o.kind === 'circle') {
        const fx = ox - o.x, fy = oy - o.y, b = fx * dx + fy * dy, disc = b * b - (fx * fx + fy * fy - o.r * o.r);
        if (disc < 0) continue;
        const t = -b - Math.sqrt(disc);
        if (t <= 0 || t >= best.t) continue;
        const px = ox + dx * t - o.x, py = oy + dy * t - o.y, l = Math.hypot(px, py) || 1;
        best = { t, inc: Math.acos(Math.min(1, Math.abs(dx * px / l + dy * py / l))) * 180 / Math.PI, thin: o.r < 6 };
        continue;
      }
      for (const [x, ok] of [[o.x, true], [o.x + o.w, true]]) {
        if (!dx || !ok) continue;
        const t = (x - ox) / dx, y = oy + dy * t;
        if (y >= o.y && y <= o.y + o.h) hit(t, 1, 0);
      }
      for (const y of [o.y, o.y + o.h]) {
        if (!dy) continue;
        const t = (y - oy) / dy, x = ox + dx * t;
        if (x >= o.x && x <= o.x + o.w) hit(t, 0, 1);
      }
    }
    return best;
  };
  sim.distanceNow = () => {
    const { x, y, heading } = sim.state;
    const r0 = (heading * Math.PI) / 180;
    const ox = x + Math.cos(r0) * SENSOR_CM, oy = y + Math.sin(r0) * SENSOR_CM;
    let cm = Infinity;
    for (let off = -beamDeg / 2; off <= beamDeg / 2 + 1e-9; off += 1) {
      const r = ((heading + off) * Math.PI) / 180;
      const b = cast(ox, oy, Math.cos(r), Math.sin(r));
      if (b.thin ? Math.abs(off) > 8 : b.inc > specDeg) continue;
      cm = Math.min(cm, b.t);
    }
    if (cm < Infinity) cm += noiseCm * (rand() + rand() + rand() - 1.5) * 1.4;
    if (rand() < falseRate) cm = 10 + rand() * (Math.min(cm, 150) - 10);
    if (!(cm < 150)) return 190;
    return Math.round(Math.max(3, cm) * 10) / 10;
  };
}

// Field-like odometry errors on a SimRobot instance: per-leg encoder scale
// (slip, sigma slipSigma, right wheel 3 % more), encoder noise, integer yaw,
// 120 ms sensor latency, 3 cm coasting after a stop, and gyro drift.
function fieldErrors(sim, { rand, slipSigma = 0.04, driftDegMin = 2 }) {
  Object.assign(sim, { encNoiseDeg: 1, yawInteger: true, latencyMs: 120, stopCoastCm: 3 });
  const gauss = () => (rand() + rand() + rand() + rand() - 2) * 1.73;
  const go = sim.go.bind(sim);
  sim.go = (vLin, vAng, secs, label) => {
    if (vLin && !sim.motion) { const f = 1 + slipSigma * gauss(); sim.encScale = [f, f * 1.03]; }
    return go(vLin, vAng, secs, label);
  };
  const yawNow = sim.yawNow.bind(sim);
  const t0 = sim.simSecs();
  sim.yawNow = () => {
    const v = yawNow() + (driftDegMin * (sim.simSecs() - t0)) / 60;
    return Math.round((((v % 360) + 540) % 360) - 180);
  };
}

function rng(seed) {
  let s = seed >>> 0;
  return () => { s = (s * 1664525 + 1013904223) >>> 0; return s / 2 ** 32; };
}

// Owner report (v0.7.x): after a few goals "Nach Hause" ended about 30 cm from
// the start. The home check matches a sweep at the start against the
// reference scan taken there and drives a short correction leg.
test('localize: mission with odometry and gyro drift returns home within 8 cm', async () => {
  const rand = rng(5);
  const sim = new SimRobot({ log: () => {}, onStatus: () => {}, timeScale: 20 });
  fieldErrors(sim, { rand });
  realisticSensor(sim, { rand });
  const bus = new CommandBus({ log: () => {} });
  bus.setRobot(sim);
  await sim.connect();
  const map = new GridMap({});
  const pose = new PoseTracker();
  const events = [];
  const nav = new Navigator({ bus, map, pose, scan, settleMs: 0, sample: makeSimSampler(sim), useYaw: true, beamDeg: 25, maxRangeCm: 150,
    onEvent: (e) => events.push(e) });
  const truth = () => ({ x: sim.state.x - 150, y: 100 - sim.state.y, heading: ((sim.state.heading + 90) % 360 + 540) % 360 - 180 });
  try {
    await nav.scanHere({});
    assert.equal(nav.anchors.length, 1, 'reference scan kept');
    for (const g of [{ x: -110, y: -10 }, { x: 40, y: -75 }, { x: 125, y: 5 }]) {
      const r = await nav.goTo(g);
      assert.equal(r.reached, true, r.note);
    }
    const h = await nav.goHome({});
    assert.equal(h.ok, true, h.note);
    const t = truth();
    assert.ok(Math.hypot(t.x, t.y) < 8, `home ${JSON.stringify(t)}`);
    assert.ok(angErr(t.heading, 0) < 5, `heading ${t.heading}`);
    const checks = events.filter((e) => e.type === 'home-check');
    assert.ok(checks.length >= 1 && checks.at(-1).ref === 'anchor' && checks.at(-1).confidence >= 0.5, JSON.stringify(checks));
    assert.ok(checks.at(-1).residualCm <= 3 || checks.length === 3);
    assert.deepEqual(h.homeCheck, checks.at(-1));
    // routine matches are mostly accepted
    const loc = events.filter((e) => e.type === 'localized');
    assert.ok(loc.filter((e) => e.applied).length >= loc.length * 0.6, JSON.stringify(loc.map((e) => [e.ref, Math.round(e.confidence * 100), e.applied])));
  } finally {
    await sim.disconnect();
  }
});
