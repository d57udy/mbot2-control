// Run with: npm test
import test from 'node:test';
import assert from 'node:assert/strict';
import { GridMap } from '../js/gridmap.js';
import { planPath, simplifyPath, pathToMoves, lineOfSight } from '../js/planner.js';

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
