// Run with: npm test
import test from 'node:test';
import assert from 'node:assert/strict';
import { GridMap } from '../js/gridmap.js';
import { scoreScan, matchScan, relocalize, fusePose, likelihoodField } from '../js/localize.js';
import { SimRobot } from '../js/robot-sim.js';

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
