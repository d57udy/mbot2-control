// Run with: npm test
import test from 'node:test';
import assert from 'node:assert/strict';
import { GridMap } from '../js/gridmap.js';
import { planPath, simplifyPath, pathToMoves, lineOfSight } from '../js/planner.js';
import { SimRobot, SIM_OBSTACLES, SIM_ROBOT } from '../js/robot-sim.js';
import { CommandBus } from '../js/bus.js';
import { Navigator } from '../js/navigate.js';
import { PoseTracker } from '../js/pose.js';
import { scan } from '../js/scan.js';

// Known free square of +-half cm with optional occupied rectangles.
function room({ half = 150, walls = [] } = {}) {
  const m = new GridMap({});
  for (let k = 0; k < m.L.length; k++) {
    const c = m.centre(k);
    if (Math.abs(c.x) < half && Math.abs(c.y) < half) m.L[k] = -2;
  }
  for (const w of walls) {
    for (let k = 0; k < m.L.length; k++) {
      const c = m.centre(k);
      if (c.x >= w.x0 && c.x <= w.x1 && c.y >= w.y0 && c.y <= w.y1) m.L[k] = 3;
    }
  }
  m.touch();
  return m;
}

const len = (p) => p.slice(1).reduce((s, q, i) => s + Math.hypot(q.x - p[i].x, q.y - p[i].y), 0);

test('planner: straight path in open space', () => {
  const m = room();
  const p = planPath(m, { x: 0, y: 0 }, { x: 0, y: 100 });
  assert.ok(p && p.length > 2);
  assert.deepEqual(p[0], { x: 0, y: 0 });
  assert.ok(Math.abs(p.at(-1).y - 100) < 5);
  assert.ok(len(p) < 110);
  const s = simplifyPath(p, m);
  assert.equal(s.length, 2);
});

test('planner: goes around a wall and keeps the inflation distance', () => {
  // wall across y = 50 from x = -100 to 40 (bounds are tested against cell centres)
  const m = room({ walls: [{ x0: -100, x1: 40, y0: 45, y1: 55 }] });
  const p = planPath(m, { x: 0, y: 0 }, { x: 0, y: 100 }, { inflateCm: 14 });
  assert.ok(p);
  assert.ok(p.some((q) => q.x > 40 + 10), 'passes right of the wall end');
  for (const q of p) assert.ok(m.isTraversable(q.x, q.y, { inflateCm: 14 }) || Math.hypot(q.x, q.y) < 20);
  const s = simplifyPath(p, m, { inflateCm: 14 });
  assert.ok(s.length >= 3 && s.length < p.length);
  for (let i = 1; i < s.length; i++) assert.ok(lineOfSight(m, s[i - 1], s[i], { inflateCm: 14 }));
  assert.ok(len(s) <= len(p) + 1e-6);
});

test('planner: no path when walled in, unknown handling', () => {
  // closed ring around the start
  const ring = room({ walls: [
    { x0: -60, x1: 60, y0: 55, y1: 60 }, { x0: -60, x1: 60, y0: -60, y1: -55 },
    { x0: -60, x1: -55, y0: -60, y1: 60 }, { x0: 55, x1: 60, y0: -60, y1: 60 },
  ] });
  assert.equal(planPath(ring, { x: 0, y: 0 }, { x: 0, y: 120 }), null);

  // only the start area is known; the goal lies in unknown space
  const m = room({ half: 40 });
  assert.ok(planPath(m, { x: 0, y: 0 }, { x: 0, y: 150 }, { allowUnknown: true }));
  assert.equal(planPath(m, { x: 0, y: 0 }, { x: 0, y: 150 }, { allowUnknown: false }), null);
  // unknown costs more: a known detour beats a short unknown shortcut
  const d = room({ half: 0 });
  for (let k = 0; k < d.L.length; k++) {
    const c = d.centre(k);
    // known U-shaped corridor: down x=-60, across y=0..., the direct line x=0 is unknown
    if ((Math.abs(c.x) <= 60 && c.y >= -10 && c.y <= 10) || (Math.abs(Math.abs(c.x) - 60) <= 10 && c.y >= 0 && c.y <= 100)
      || (Math.abs(c.x) <= 60 && c.y >= 90 && c.y <= 110)) d.L[k] = -2;
  }
  d.touch();
  const u = planPath(d, { x: 0, y: 0 }, { x: 0, y: 100 }, { inflateCm: 0 });
  assert.ok(u.some((q) => Math.abs(q.x) > 40), 'takes the known corridor');
});

test('planner: start next to an obstacle may leave, blocked goal snaps', () => {
  const m = room({ walls: [{ x0: -50, x1: 50, y0: 6, y1: 14 }] });
  // the start is within the inflation of the wall, the robot can still back off
  const p = planPath(m, { x: 0, y: 0 }, { x: 0, y: -80 });
  assert.ok(p, 'escapes the inflated start');
  // goal on the wall: snapped to a traversable cell within 30 cm
  const q = planPath(m, { x: 0, y: -40 }, { x: 0, y: 10 });
  assert.ok(q);
  const end = q.at(-1);
  assert.ok(Math.hypot(end.x, end.y - 10) <= 30 && m.isTraversable(end.x, end.y, { inflateCm: 14 }));
  // goal deep inside a large block cannot snap
  const big = room({ walls: [{ x0: -100, x1: 100, y0: 60, y1: 140 }] });
  assert.equal(planPath(big, { x: 0, y: 0 }, { x: 0, y: 100 }), null);
  // out of the map
  assert.equal(planPath(m, { x: 900, y: 0 }, { x: 0, y: 0 }), null);
});

test('pathToMoves: turns, splits, ends at the waypoints', () => {
  const pose = { x: 0, y: 0, heading: 0 };
  let mv = pathToMoves(pose, [{ x: 0, y: 0 }, { x: 0, y: 100 }], { maxSegCm: 40 });
  assert.deepEqual(mv.map((m) => m.cm), [33, 34, 33]);
  assert.deepEqual(mv.map((m) => m.turnDeg), [0, 0, 0]);
  assert.deepEqual(mv.at(-1).to, { x: 0, y: 100 });

  mv = pathToMoves({ x: 0, y: 0, heading: 90 }, [{ x: 0, y: 0 }, { x: 0, y: 30 }, { x: -30, y: 30 }]);
  assert.deepEqual(mv.map((m) => [m.turnDeg, m.cm]), [[-90, 30], [-90, 30]]);
  assert.ok(Math.abs(mv[1].to.x + 30) < 0.01 && Math.abs(mv[1].to.y - 30) < 0.01);

  // behind: +-180, never more
  mv = pathToMoves({ x: 0, y: 0, heading: 10 }, [{ x: 0, y: 0 }, { x: 0, y: -20 }]);
  assert.equal(Math.abs(mv[0].turnDeg), 170);
  mv = pathToMoves({ x: 0, y: 0, heading: -170 }, [{ x: 0, y: 0 }, { x: 10, y: 10 }]);
  assert.equal(mv[0].turnDeg, -145);
  assert.ok(mv.every((m) => m.turnDeg >= -180 && m.turnDeg <= 180));
  // the start waypoint and tiny segments are skipped
  assert.deepEqual(pathToMoves(pose, [{ x: 0, y: 0 }, { x: 0.3, y: 0.3 }]), []);
  assert.deepEqual(pathToMoves(pose, null), []);
});

test('planner: keeps distance where there is room, still fits through a gap', () => {
  // a post at (0, 50): the path may pass it but prefers more than the inflation
  const m = room({ walls: [{ x0: -3, x1: 3, y0: 47, y1: 53 }] });
  const p = simplifyPath(planPath(m, { x: 0, y: 0 }, { x: 0, y: 100 }), m);
  let min = Infinity;
  for (let i = 1; i < p.length; i++) {
    for (let t = 0; t <= 1; t += 0.02) min = Math.min(min, m.clearance(p[i - 1].x + (p[i].x - p[i - 1].x) * t, p[i - 1].y + (p[i].y - p[i - 1].y) * t));
  }
  assert.ok(min > 20, `clearance ${min}`);
  // a 45 cm gap between two walls is still passable with inflation 14
  const g = room({ walls: [{ x0: -150, x1: -25, y0: 45, y1: 55 }, { x0: 25, x1: 150, y0: 45, y1: 55 }] });
  assert.ok(planPath(g, { x: 0, y: 0 }, { x: 0, y: 100 }));
});

test('planner: escaping the inflated start never moves closer to the obstacle', () => {
  // start 10 cm left of a post; the goal lies beyond the post
  const m = room({ walls: [{ x0: 8, x1: 12, y0: -2, y1: 2 }] });
  const start = { x: 0, y: 0 };
  const p = planPath(m, start, { x: 60, y: 0 });
  assert.ok(p);
  const c0 = m.clearance(start.x, start.y);
  for (const q of p.slice(1)) assert.ok(m.clearance(q.x, q.y) >= c0 - 1e-6, `(${q.x}, ${q.y}) at ${m.clearance(q.x, q.y)}`);
});

test('planner: weak hit evidence is avoided when there is room', () => {
  const m = room();
  for (let x = -20; x <= 20; x += 5) m.L[m.index(x, 50)] = 0.3;   // below the occupied threshold
  m.touch();
  assert.equal(m.cell(0, 50), 'unknown');
  const p = planPath(m, { x: 0, y: 0 }, { x: 0, y: 100 });
  assert.ok(p.every((q) => Math.hypot(q.x, q.y - 50) > 14 || Math.abs(q.x) > 20), 'goes around the suspect arc');
});

// Home bug (v0.5 owner report): scans from later poses erased the thin table
// leg and the home path ran through it. Sim frame: map x = sim x - 150,
// map y = 100 - sim y, map heading = sim heading + 90.
async function simScan(sim, p, n = 12) {
  const pts = [];
  for (let i = 0; i < n; i++) {
    const a = (i * 360) / n;
    Object.assign(sim.state, { x: p.x + 150, y: 100 - p.y, heading: p.heading + a - 90 });
    pts.push({ angle: a > 180 ? a - 360 : a, cm: await sim.distance() });
  }
  return pts;
}

// Smallest distance from a polyline (map frame) to the sim obstacles' surfaces.
function trueClearance(path, obstacles) {
  let min = Infinity;
  for (let i = 1; i < path.length; i++) {
    const a = path[i - 1], b = path[i], L = Math.hypot(b.x - a.x, b.y - a.y) || 1;
    for (let t = 0; t <= L; t += 1) {
      const sx = a.x + ((b.x - a.x) * t) / L + 150, sy = 100 - (a.y + ((b.y - a.y) * t) / L);
      for (const o of obstacles) {
        const d = o.kind === 'circle' ? Math.hypot(sx - o.x, sy - o.y) - o.r
          : Math.hypot(sx - Math.min(o.x + o.w, Math.max(o.x, sx)), sy - Math.min(o.y + o.h, Math.max(o.y, sy)));
        min = Math.min(min, d);
      }
    }
  }
  return min;
}

test('planner: home path keeps clear of the table leg after scans from other poses', async () => {
  const sim = new SimRobot({ log: () => {}, onStatus: () => {} });
  const m = new GridMap({});
  const opts = { beamDeg: 16, freeBeamDeg: 30 };
  // the poses of the reproduction: start, past the leg, beyond it
  for (const p of [{ x: 0, y: 0, heading: 0 }, { x: 24, y: 24, heading: 45 }, { x: 33, y: 48, heading: 21 }, { x: 86, y: 67, heading: 70 }]) {
    m.integrateScan(p, await simScan(sim, p), opts);
  }
  const leg = { x: 215 - 150, y: 100 - 55 };
  assert.ok(m.clearance(leg.x, leg.y) <= 10, `the leg is still in the map (${m.clearance(leg.x, leg.y)})`);
  const p = simplifyPath(planPath(m, { x: 112, y: 78 }, { x: 0, y: 0 }, { inflateCm: 14 }), m, { inflateCm: 14 });
  assert.ok(p);
  const c = trueClearance(p, SIM_OBSTACLES);
  assert.ok(c > SIM_ROBOT.radiusCm + 3, `home path ${c.toFixed(1)} cm from an obstacle`);
});

// legMode 'straight': blocking legs, independent of the drive-leg crash detection
test('planner: scan, drive past the leg and go home in the sim without coming close', async () => {
  const sim = new SimRobot({ log: () => {}, onStatus: () => {}, timeScale: 80 });
  const bus = new CommandBus({ log: () => {} });
  bus.setRobot(sim);
  await sim.connect();
  let minClear = Infinity;
  sim.onChange = (s) => { minClear = Math.min(minClear, trueClearance([{ x: s.x - 150, y: 100 - s.y }, { x: s.x - 150, y: 100 - s.y }], SIM_OBSTACLES)); };
  const map = new GridMap({});
  const plans = [];
  const nav = new Navigator({ bus, map, pose: new PoseTracker(), scan, settleMs: 0, legMode: 'straight', steps: 8,
    onEvent: (e) => { if (e.type === 'plan') plans.push(e.path); } });
  try {
    await nav.scanHere({});
    const r = await nav.goTo({ x: 110, y: 75 });
    assert.equal(r.ok, true, r.note);
    const before = plans.length;
    const h = await nav.goHome({});
    assert.equal(h.ok, true, h.note);
    for (const path of plans.slice(before)) {
      const c = trueClearance(path, SIM_OBSTACLES);
      assert.ok(c > SIM_ROBOT.radiusCm, `home plan ${c.toFixed(1)} cm from an obstacle`);
    }
    assert.ok(minClear > SIM_ROBOT.radiusCm + 1, `came within ${minClear.toFixed(1)} cm`);
  } finally {
    await sim.disconnect();
  }
});
