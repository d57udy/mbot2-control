// Run with: npm test
import test from 'node:test';
import assert from 'node:assert/strict';
import { GridMap } from '../js/gridmap.js';

const O = { x: 0, y: 0, heading: 0 };

test('gridmap: starts unknown, cell geometry, out of range', () => {
  const m = new GridMap({ cellCm: 5, sizeCm: 800 });
  assert.equal(m.n, 160);
  assert.equal(m.cell(0, 0), 'unknown');
  assert.equal(m.cell(1000, 0), 'unknown');
  assert.equal(m.bounds, null);
  assert.equal(m.isTraversable(0, 0), false);
  assert.equal(m.isTraversable(0, 0, { allowUnknown: true }), true);
  assert.equal(m.isTraversable(1000, 0, { allowUnknown: true }), false);
  assert.deepEqual(m.centre(m.index(0, 0)), { x: 2.5, y: 2.5 });
  assert.deepEqual(m.centre(m.index(-0.1, -0.1)), { x: -2.5, y: -2.5 });
  assert.match(m.describe(O), /empty/);
});

test('gridmap: a far hit is suspect after one scan, confirmed after a second one', () => {
  const m = new GridMap({});
  m.integrateScan(O, [{ angle: 0, cm: 100 }]);   // sensor at y = 6, hit at y = 106
  assert.equal(m.cell(0, 0), 'free');            // robot footprint
  assert.equal(m.cell(0, 50), 'free');
  assert.equal(m.cell(0, 95), 'free');
  assert.equal(m.kind(0, 106), 'suspect');       // one scan, far: not confirmed
  assert.equal(m.cell(0, 106), 'unknown');       // legacy view of a suspect cell
  assert.ok(m.L[m.index(0, 106)] > 0.15);
  assert.equal(m.cell(0, 130), 'unknown');       // behind the hit
  assert.equal(m.cell(-40, 50), 'unknown');      // outside the beam
  assert.equal(m.cell(0, -40), 'unknown');
  // the arc is centre-weighted: edge cells hold less hit evidence
  const edge = m.cellInfo(Math.sin((7 * Math.PI) / 180) * 100, 6 + Math.cos((7 * Math.PI) / 180) * 100);
  assert.ok(edge.hits > 0 && edge.hits < m.cellInfo(0, 106).hits);
  // a second scan from elsewhere confirms it
  m.integrateScan({ x: 20, y: 0, heading: 0 }, [{ angle: -9.4, cm: 101 }]);
  assert.equal(m.kind(0, 106), 'occupied');
  assert.equal(m.cell(0, 106), 'occupied');
  const b = m.bounds;
  assert.ok(b.minY < 0 && b.maxY >= 105 && b.minX < -5 && b.maxX > 5);
});

test('gridmap: one scan never confirms, not even a close head-on hit (v0.8)', () => {
  const m = new GridMap({});
  m.integrateScan(O, [{ angle: 0, cm: 40 }]);
  assert.equal(m.kind(0, 46), 'suspect');
  assert.equal(m.cellInfo(0, 46).near, true);
  m.integrateScan(O, [{ angle: 0, cm: 40 }]);
  assert.equal(m.kind(0, 46), 'occupied');
  // the arc edges are not head-on
  assert.equal(m.kind(Math.sin((7 * Math.PI) / 180) * 46, Math.cos((7 * Math.PI) / 180) * 46), 'suspect');
});

test('gridmap: heading and angle rotate the beam clockwise; failed reads are ignored', () => {
  const m = new GridMap({});
  for (let i = 0; i < 2; i++) m.integrateScan({ x: 0, y: 0, heading: 90 }, [{ angle: 0, cm: 50 }, { angle: 90, cm: 40 }, { angle: 180, cm: null }]);
  assert.equal(m.cell(30, 0), 'free');           // heading 90 = right
  assert.equal(m.cell(56, 0), 'occupied');       // two scans: confirmed
  assert.equal(m.cell(0, -46), 'occupied');      // 90 + 90 = behind the start
  assert.equal(m.cell(-30, 0), 'unknown');       // null reading
});

test('gridmap: no echo and readings beyond maxRange clear up to maxRange', () => {
  const m = new GridMap({});
  m.integrateScan(O, [{ angle: 0, cm: 300 }, { angle: 180, cm: 280 }], { maxRangeCm: 150 });
  assert.equal(m.cell(0, 150), 'free');
  assert.equal(m.cell(0, -150), 'free');
  assert.equal(m.cell(0, 165), 'unknown');
  let occ = 0;
  m.forEachCell((x, y, s) => { if (s === 'occupied') occ++; });
  assert.equal(occ, 0);
});

test('gridmap: repeated evidence clamps, free evidence erodes a false hit', () => {
  const m = new GridMap({});
  for (let i = 0; i < 50; i++) m.integrateScan(O, [{ angle: 0, cm: 60 }]);
  const k = m.index(0, 66);
  assert.ok(m.L[k] <= 4);
  assert.equal(m.cell(0, 66), 'occupied');
  // the obstacle moved away: long readings through it clear it again
  for (let i = 0; i < 20; i++) m.integrateScan(O, [{ angle: 0, cm: 200 }]);
  assert.equal(m.cell(0, 66), 'free');
});

test('gridmap: freeBeamDeg widens only the free cone, weaker and with a shorter reach', () => {
  const m = new GridMap({});
  const at = (deg, r) => m.cell(Math.sin((deg * Math.PI) / 180) * r, 6 + Math.cos((deg * Math.PI) / 180) * r);
  const info = (deg, r) => m.cellInfo(Math.sin((deg * Math.PI) / 180) * r, 6 + Math.cos((deg * Math.PI) / 180) * r);
  // not looked at: unknown after one and two passes, free after three
  for (let i = 0; i < 3; i++) {
    m.integrateScan(O, [{ angle: 0, cm: 100 }], { beamDeg: 16, freeBeamDeg: 30, robotRadiusCm: 0 });
    if (i < 2) assert.equal(at(13, 50), 'unknown');
  }
  assert.ok(info(13, 50).misses > 0);
  assert.equal(at(13, 50), 'free');
  assert.equal(at(13, 90), 'unknown');           // beyond 0.7 of the range
  assert.equal(info(13, 90).misses, 0);
  assert.notEqual(at(13, 100), 'occupied');
  assert.equal(at(0, 100), 'occupied');          // three scans hit it
});

test('gridmap: one contribution per cell per scan', () => {
  const m = new GridMap({});
  m.beginScan();
  for (let i = 0; i < 10; i++) m.integrateScan(O, [{ angle: 0, cm: 100 }, { angle: 2, cm: 100 }]);
  m.endScan();
  const c = m.cellInfo(0, 106);
  assert.ok(c.hits <= 1 + 1e-6, `hits ${c.hits}`);
  assert.equal(c.scans, 1);
  assert.equal(m.kind(0, 106), 'suspect');
  assert.ok(m.cellInfo(0, 50).misses <= 1 + 1e-6);
  // without beginScan every call is a scan of its own
  m.integrateScan(O, [{ angle: 0, cm: 100 }]);
  assert.equal(m.cellInfo(0, 106).scans, 2);
  assert.equal(m.kind(0, 106), 'occupied');
  assert.equal(m.scans, 2);
});

test('gridmap: a phantom is seen through and goes; grazing cones do not erase a real hit', () => {
  const m = new GridMap({});
  // random short reflection at 70 cm, then two scans that see through it
  m.integrateScan(O, [{ angle: 0, cm: 70 }]);
  assert.equal(m.kind(0, 76), 'suspect');
  m.integrateScan(O, [{ angle: 0, cm: 200 }]);
  assert.ok(m.cellInfo(0, 76).p < 0.55);
  m.integrateScan(O, [{ angle: 0, cm: 200 }]);
  assert.equal(m.kind(0, 76), 'free');
  // a thin leg 60 cm ahead seen by two scans; later beams only graze it
  const t = new GridMap({});
  for (let i = 0; i < 2; i++) t.integrateScan(O, [{ angle: 0, cm: 70 }], { robotRadiusCm: 0 });
  assert.equal(t.kind(0, 76), 'occupied');
  const side = { x: -50, y: 76, heading: 90 };
  for (let i = 0; i < 10; i++) {
    // gap fillers and beam edges (outside the +-4 deg core of a 16 deg beam)
    t.integrateScan(side, [{ angle: 12, cm: 200 }, { angle: -12, cm: 200 }], { beamDeg: 16, freeBeamDeg: 30, robotRadiusCm: 0 });
    t.integrateScan(side, [{ angle: 6, cm: 200 }], { beamDeg: 16, robotRadiusCm: 0 });
  }
  assert.equal(t.kind(0, 76), 'occupied');
  // the core of a beam through it does clear it (the obstacle was moved away)
  for (let i = 0; i < 6; i++) t.integrateScan(side, [{ angle: 0, cm: 200 }], { beamDeg: 16, robotRadiusCm: 0 });
  assert.notEqual(t.kind(0, 76), 'occupied');
  // the robot footprint erodes hit evidence only slowly (pose errors)
  const m2 = new GridMap({});
  m2.add(m2.index(3, 3), 2);
  m2.touch();
  m2.markFree(0, 0, 9);
  assert.equal(m2.cell(3, 3), 'occupied');
});

test('gridmap: a well-confirmed wall survives specular ghosts', () => {
  const m = new GridMap({});
  // wall at y = 106 seen head-on by 8 scans
  for (let i = 0; i < 8; i++) m.integrateScan({ x: (i % 3) * 5, y: 0, heading: 0 }, [{ angle: 0, cm: 100 }]);
  assert.equal(m.kind(0, 106), 'occupied');
  // 3 oblique scans whose reading passes the wall (reflection away, long echo)
  for (let i = 0; i < 3; i++) m.integrateScan(O, [{ angle: 0, cm: 200 }]);
  assert.equal(m.kind(0, 106), 'occupied');
  const c = m.cellInfo(0, 106);
  assert.ok(c.misses > 0 && c.hits > c.misses, JSON.stringify(c));
  // evidence is capped: a wall that was really removed fades eventually
  for (let i = 0; i < 20; i++) m.integrateScan(O, [{ angle: 0, cm: 200 }]);
  assert.equal(m.kind(0, 106), 'free');
});

test('gridmap: cleanup removes unconfirmed single-scan hits only', () => {
  const m = new GridMap({});
  m.integrateScan(O, [{ angle: 0, cm: 100 }]);   // one scan: suspect
  for (let i = 0; i < 2; i++) m.integrateScan(O, [{ angle: 90, cm: 40 }]);   // two scans: confirmed
  m.integrateScan({ x: 0, y: 0, heading: 180 }, [{ angle: 0, cm: 120 }]);
  m.integrateScan({ x: 0, y: 0, heading: 180 }, [{ angle: 0, cm: 120 }]); // two scans: confirmed
  assert.equal(m.kind(0, 106), 'suspect');
  const v = m.version;
  const n = m.cleanup();
  assert.ok(n > 0 && m.version > v);
  assert.notEqual(m.kind(0, 106), 'suspect');
  assert.equal(m.kind(46, 0), 'occupied');
  assert.equal(m.kind(0, -126), 'occupied');
  assert.equal(m.cell(0, 50), 'free');            // misses stay
});

test('gridmap: cellInfo, aging, legacy log-odds writes and add()', () => {
  const m = new GridMap({ decayAfterScans: 2, decayRate: 0.5 });
  m.beginScan(); m.integrateScan(O, [{ angle: 0, cm: 40 }]); m.endScan();
  m.integrateScan(O, [{ angle: 0, cm: 40 }]);
  const c = m.cellInfo(0, 46);
  assert.deepEqual(Object.keys(c).sort(), ['contact', 'contactAge', 'evidence', 'hits', 'lastSeen', 'misses', 'near', 'p', 'scans', 'scansAgo', 'state', 'weakHits', 'weakReadings', 'x', 'y'].sort());
  assert.equal(c.state, 'occupied');
  assert.equal(c.scansAgo, 0);
  assert.equal(m.cellInfo(5000, 0), null);
  // not observed for more than 2 scans: fades toward unknown
  for (let i = 0; i < 8; i++) m.integrateScan({ x: 0, y: -200, heading: 180 }, [{ angle: 0, cm: 50 }]);
  assert.equal(m.kind(0, 46), 'unknown');
  assert.equal(m.cellInfo(0, -256).state, 'occupied');   // observed every scan
  // L written from outside (old saved maps) is imported on touch()
  const o = new GridMap({});
  o.L[o.index(0, 50)] = 3; o.L[o.index(0, 0)] = -2; o.L[o.index(0, 20)] = 0.3;
  o.touch();
  assert.equal(o.kind(0, 50), 'occupied');
  assert.equal(o.kind(0, 0), 'free');
  assert.equal(o.kind(0, 20), 'suspect');
  // add(): contacts (+2 and more) confirm, negative values are misses
  o.add(o.index(30, 30), 4);
  o.add(o.index(0, 50), -1);
  o.touch();
  assert.equal(o.kind(30, 30), 'occupied');
  assert.ok(o.cellInfo(0, 50).misses > 0);
});

test('gridmap: distance field matches brute force and is cached', () => {
  const m = new GridMap({ sizeCm: 200 });
  const occ = [[-50, 20], [30, -60], [70, 70], [-95, -95]];
  for (const [x, y] of occ) m.L[m.index(x, y)] = 2;
  m.L[m.index(10, 10)] = 0.3;  // suspect only
  m.touch();
  const df = m.distanceField();
  for (let k = 0; k < m.L.length; k += 7) {
    const c = m.centre(k);
    const want = Math.min(...occ.map(([x, y]) => { const o = m.centre(m.index(x, y)); return Math.hypot(o.x - c.x, o.y - c.y); }));
    assert.ok(Math.abs(df[k] - want) < 1e-3, `${k}: ${df[k]} vs ${want}`);
  }
  assert.equal(m.distanceField(), df);
  assert.equal(m.clearance(10, 10, 0.15), 0);
  assert.ok(m.clearance(10, 10) > 30);
  assert.equal(new GridMap({ sizeCm: 100 }).distanceField()[0], Infinity);
  m.touch();
  assert.notEqual(m.distanceField(), df);
});

test('gridmap: inflation and traversability', () => {
  const m = new GridMap({});
  for (let i = 0; i < 2; i++) m.integrateScan(O, [{ angle: 0, cm: 100 }], { beamDeg: 1 });
  assert.equal(m.isTraversable(0, 50), true);
  assert.equal(m.isTraversable(0, 90), false);                  // within 14 cm of the hit arc at 100..110
  assert.equal(m.isTraversable(0, 90, { inflateCm: 5 }), true);
  assert.equal(m.isTraversable(0, 106, { inflateCm: 0 }), false);  // occupied itself
  assert.equal(m.isTraversable(0, 115, { allowUnknown: true }), false); // unknown, but inflated
  assert.equal(m.isTraversable(0, 140, { allowUnknown: true }), true);
  assert.equal(m.isTraversable(0, 140), false);
});

test('gridmap: frontiers cluster free cells next to unknown, largest first', () => {
  const m = new GridMap({});
  // two cones joined through the robot footprint form one cluster
  m.integrateScan(O, [{ angle: 90, cm: 300 }, { angle: 0, cm: 300 }], { maxRangeCm: 100, beamDeg: 16 });
  let f = m.frontiers();
  assert.equal(f.length, 1);
  assert.ok(f[0].x > 10 && f[0].y > 10 && f[0].size > 50);
  // a separate, shorter cone far away is a second, smaller cluster
  m.integrateScan({ x: -200, y: -200, heading: 180 }, [{ angle: 0, cm: 300 }], { maxRangeCm: 40, beamDeg: 16 });
  f = m.frontiers();
  assert.equal(f.length, 2);
  assert.ok(f[0].size >= f[1].size);
  assert.ok(f[1].x < -150 && f[1].y < -200);
  assert.ok(f.every((a) => Number.isFinite(a.x) && Number.isFinite(a.y)));
  assert.ok(m.frontiers({ minCells: 1000 }).length === 0);
  // a closed box has no frontier inside
  const box = new GridMap({ sizeCm: 100 });
  box.L.fill(-2);
  assert.equal(box.frontiers().length, 0);
});

test('gridmap: describe is compact and relative to the heading', () => {
  const m = new GridMap({});
  for (let i = 0; i < 2; i++) m.integrateScan(O, [{ angle: 0, cm: 50 }, { angle: 90, cm: 120 }, { angle: 180, cm: 300 }, { angle: -90, cm: 300 }]);
  const s = m.describe({ x: 0, y: 0, heading: 90 });
  assert.ok(s.length < 400, `${s.length}`);
  assert.match(s, /m² known/);
  assert.match(s, /left 5\d/);          // the hit ahead of the start is left of a robot facing right
  assert.match(s, /ahead 12\d/);
  assert.match(s, /frontier/);
});

test('gridmap: clear, forEachCell, JSON round trip (v2 evidence, v1 read)', () => {
  const m = new GridMap({});
  m.integrateScan(O, [{ angle: 30, cm: 80 }]);
  for (let i = 0; i < 2; i++) m.integrateScan(O, [{ angle: 0, cm: 40 }]);
  const v = m.version;
  const j = JSON.parse(JSON.stringify(m));
  assert.equal(j.v, 2);
  const m2 = GridMap.fromJSON(j);
  let a = 0, b = 0;
  const kinds = new Set();
  m.forEachCell((x, y, s, p, kind) => {
    a++; kinds.add(kind);
    assert.ok(p > 0 && p < 1);
    assert.equal(m2.cell(x, y), s);
    assert.equal(m2.kind(x, y), kind);
  });
  m2.forEachCell(() => b++);
  assert.equal(a, b);
  assert.ok(a > 20);
  assert.ok(kinds.has('free') && kinds.has('suspect') && kinds.has('occupied'));
  assert.equal(m2.cellInfo(0, 46).near, true);
  const m3 = new GridMap({});
  m3.copyFrom(m2);
  assert.deepEqual(m3.cellInfo(0, 46), m2.cellInfo(0, 46));
  assert.throws(() => m3.copyFrom(new GridMap({ sizeCm: 400 })), /size/);
  // version 1 (log-odds only)
  const old = GridMap.fromJSON({ v: 1, cellCm: 5, sizeCm: 800, cells: [m.index(0, 50), 2.5, m.index(0, 0), -2] });
  assert.equal(old.kind(0, 50), 'occupied');
  assert.equal(old.kind(0, 0), 'free');
  m.clear();
  assert.ok(m.version > v);
  assert.equal(m.bounds, null);
  let c = 0; m.forEachCell(() => c++);
  assert.equal(c, 0);
  assert.equal(m.cellInfo(0, 46).hits, 0);
});

// Synthetic room with the measured sensor model: 25 deg beam (nearest echo
// wins), readings at or beyond 150 cm mean nothing (reported as 190), surfaces
// hit at more than 55 deg from their normal reflect away (specular ghost: the
// beam sees past them), the thin table leg answers only within 8 deg of the
// axis, and 5 % of the readings are random short reflections.
const ROOM = { x0: -150, x1: 150, y0: -100, y1: 100 };
const LEG = { x: 65, y: 45, r: 4 };
const SOFA = { x0: -130, x1: -40, y0: 55, y1: 100 };
const CHAIR = { x0: 78, x1: 112, y0: -52, y1: -18 };

function rng(seed) {
  let s = seed >>> 0;
  return () => { s = (s * 1664525 + 1013904223) >>> 0; return s / 2 ** 32; };
}

// Distance and incidence (deg from the surface normal) of a ray against the scene.
function castRay(ox, oy, dx, dy, boxes) {
  let best = { t: Infinity, inc: 0, thin: false };
  const seg = (t, nx, ny) => { if (t > 1e-6 && t < best.t) best = { t, inc: Math.acos(Math.min(1, Math.abs(dx * nx + dy * ny))) * 180 / Math.PI, thin: false }; };
  // room walls from the inside
  if (dx > 0) seg((ROOM.x1 - ox) / dx, 1, 0); else if (dx < 0) seg((ROOM.x0 - ox) / dx, 1, 0);
  if (dy > 0) seg((ROOM.y1 - oy) / dy, 0, 1); else if (dy < 0) seg((ROOM.y0 - oy) / dy, 0, 1);
  for (const b of boxes) {
    for (const [x, nx] of [[b.x0, 1], [b.x1, 1]]) {
      if (!dx) continue;
      const t = (x - ox) / dx, y = oy + dy * t;
      if (y >= b.y0 && y <= b.y1) seg(t, nx, 0);
    }
    for (const [y, ny] of [[b.y0, 1], [b.y1, 1]]) {
      if (!dy) continue;
      const t = (y - oy) / dy, x = ox + dx * t;
      if (x >= b.x0 && x <= b.x1) seg(t, 0, ny);
    }
  }
  const fx = ox - LEG.x, fy = oy - LEG.y, bq = fx * dx + fy * dy, disc = bq * bq - (fx * fx + fy * fy - LEG.r * LEG.r);
  if (disc >= 0) { const t = -bq - Math.sqrt(disc); if (t > 0 && t < best.t) best = { t, inc: 0, thin: true }; }
  return best;
}

function sweep(pose, { chair, rand }) {
  const boxes = chair ? [SOFA, CHAIR] : [SOFA];
  const points = [];
  for (let a = 0; a < 360; a += 5) {
    const dir = pose.heading + a;
    const sx = pose.x + 6 * Math.sin((dir * Math.PI) / 180), sy = pose.y + 6 * Math.cos((dir * Math.PI) / 180);
    let cm = Infinity;
    for (let off = -12.5; off <= 12.5; off += 0.5) {
      const t = ((dir + off) * Math.PI) / 180;
      const h = castRay(sx, sy, Math.sin(t), Math.cos(t), boxes);
      if (h.thin ? Math.abs(off) > 8 : h.inc > 55) continue;   // weak / specular: no echo from this ray
      cm = Math.min(cm, h.t);
    }
    if (rand() < 0.05) cm = 15 + rand() * (Math.min(cm, 150) - 15);   // random reflection
    points.push({ angle: a > 180 ? a - 360 : a, cm: cm >= 150 ? 190 : Math.round(cm * 10) / 10 });
  }
  return points;
}

// cm from (x, y) to the nearest real surface (chair optional)
function surfaceDist(x, y, chair) {
  const box = (b) => Math.hypot(x - Math.min(b.x1, Math.max(b.x0, x)), y - Math.min(b.y1, Math.max(b.y0, y)));
  let d = Math.min(x - ROOM.x0, ROOM.x1 - x, y - ROOM.y0, ROOM.y1 - y, box(SOFA), Math.hypot(x - LEG.x, y - LEG.y) - LEG.r);
  if (chair) d = Math.min(d, box(CHAIR));
  return d;
}

test('gridmap: scans from a few poses keep walls and the leg, drop phantoms and a removed chair', () => {
  const m = new GridMap({});
  const rand = rng(7);
  const poses = [{ x: 0, y: 0, heading: 0 }, { x: 40, y: -20, heading: 30 }, { x: -50, y: 0, heading: -60 },
    { x: 20, y: 30, heading: 90 }, { x: -90, y: -50, heading: 10 }, { x: 110, y: 20, heading: 200 }, { x: 0, y: -60, heading: 45 }];
  const opts = { beamDeg: 25, maxRangeCm: 150 };
  // the chair is there for the first three scans, then removed
  for (let i = 0; i < 3; i++) m.integrateScan(poses[i], sweep(poses[i], { chair: true, rand }), opts);
  const chairBefore = [];
  m.forEachCell((x, y, s, p, kind) => { if (kind === 'occupied' && surfaceDist(x, y, false) > 8) chairBefore.push([x, y]); });
  assert.ok(chairBefore.length > 3, 'the chair was mapped');
  for (let i = 0; i < 11; i++) {
    const p = poses[i % poses.length];
    m.integrateScan(p, sweep(p, { chair: false, rand }), opts);
  }
  // phantoms: hit evidence more than 25 cm from any real surface (random
  // reflections, the removed chair); bias: confirmed cells 8 to 25 cm in
  // front of a surface (a 25 deg cone rounds corners and oblique walls)
  const phantoms = { occupied: [], suspect: [] };
  let bias = 0;
  m.forEachCell((x, y, s, p, kind) => {
    const d = surfaceDist(x, y, false);
    if (d > 25 && (kind === 'occupied' || kind === 'suspect')) phantoms[kind].push([x, y]);
    if (d > 8 && d <= 25 && kind === 'occupied') bias++;
  });
  // wall points within range of a pose with a confirmed cell within 5 cm
  let wall = 0, wallOk = 0, wallSeen = 0;
  const pts = [];
  for (let t = -145; t <= 145; t += 5) pts.push({ x: t, y: ROOM.y0 }, { x: t, y: ROOM.y1 });
  for (let t = -95; t <= 95; t += 5) pts.push({ x: ROOM.x0, y: t }, { x: ROOM.x1, y: t });
  for (let t = SOFA.x0; t <= SOFA.x1; t += 5) pts.push({ x: t, y: SOFA.y0 });
  for (const q of pts) {
    if (!poses.some((p) => Math.hypot(q.x - p.x, q.y - p.y) < 120)) continue;
    wall++;
    let ok = false, seen = false;
    for (let dx = -5; dx <= 5; dx += 5) {
      for (let dy = -5; dy <= 5; dy += 5) {
        const k = m.kind(q.x + dx, q.y + dy);
        if (k === 'occupied') ok = true;
        if (k === 'occupied' || k === 'suspect') seen = true;
      }
    }
    if (ok) wallOk++;
    if (seen) wallSeen++;
  }
  assert.equal(phantoms.occupied.length, 0, `confirmed phantoms ${JSON.stringify(phantoms.occupied)}`);
  assert.ok(phantoms.suspect.length <= 8, `suspect phantoms ${phantoms.suspect.length}`);
  assert.ok(bias <= 22, `confirmed cells in front of surfaces ${bias}`);
  // one sweep never confirms: wall stretches seen from one pose only stay suspect
  assert.ok(wallOk / wall > 0.6, `walls confirmed ${wallOk}/${wall}`);
  assert.ok(wallSeen / wall > 0.85, `walls seen ${wallSeen}/${wall}`);
  // nothing of the removed chair is left confirmed
  m.forEachCell((x, y, s, p, kind) => {
    if (kind === 'occupied') assert.ok(!(x > CHAIR.x0 - 3 && x < CHAIR.x1 + 3 && y > CHAIR.y0 - 3 && y < CHAIR.y1 + 3), `chair cell ${x},${y}`);
  });
  // the leg stays confirmed
  let leg = false;
  m.forEachCell((x, y, s, p, kind) => { if (kind === 'occupied' && Math.hypot(x - LEG.x, y - LEG.y) < 9) leg = true; });
  assert.ok(leg, 'leg confirmed');
  // cleanup drops the remaining single-scan suspects and keeps everything confirmed
  const before = m.stats().occupied;
  m.cleanup();
  let left = 0;
  m.forEachCell((x, y, s, p, kind) => { if (kind === 'suspect' && surfaceDist(x, y, false) > 25) left++; });
  assert.ok(left <= phantoms.suspect.length / 2, `after cleanup ${left}`);
  assert.equal(m.stats().occupied, before);
});

test('gridmap: contacts block planning without evidence; only driving through, age or cleanup clear them', () => {
  const m = new GridMap({});
  m.integrateScan(O, [{ angle: 0, cm: 200 }], { maxRangeCm: 150 });    // free ahead
  const before = m.cellInfo(0, 25);
  assert.equal(m.markContact(0, 25), m.index(0, 25));
  assert.equal(m.markContact(5000, 0), -1);
  assert.equal(m.contactAt(0, 25), true);
  assert.equal(m.kind(0, 25), 'contact');
  assert.equal(m.cell(0, 25), 'occupied');
  assert.equal(m.clearance(0, 25), 0);
  assert.equal(m.isTraversable(0, 15), false);                   // inflated
  const info = m.cellInfo(0, 25);
  assert.equal(info.contact, true);
  assert.equal(info.contactAge.scans, 0);
  assert.equal(info.evidence, 'free');
  assert.equal(info.hits, before.hits);                         // evidence untouched
  assert.equal(info.misses, before.misses);
  // idempotent: marking again adds nothing
  for (let i = 0; i < 5; i++) m.markContact(0, 25);
  assert.equal(m.contacts().length, 1);
  assert.equal(m.cellInfo(0, 25).hits, before.hits);
  const kinds = [];
  m.forEachCell((x, y, s, p, kind) => { if (kind === 'contact') kinds.push([x, y, s]); });
  assert.deepEqual(kinds, [[2.5, 27.5, 'occupied']]);
  assert.equal(m.stats().contacts, 1);
  // the ultrasonic never clears it: many close scans that see through it
  for (let i = 0; i < 10; i++) m.integrateScan(O, [{ angle: 0, cm: 120 }]);
  assert.equal(m.contactAt(0, 25), true);
  // driving past it (path 20 cm to the side) leaves it; driving through clears it
  assert.equal(m.clearContactsAlong([{ x: 20, y: 0 }, { x: 20, y: 60 }], 9), 0);
  assert.equal(m.contactAt(0, 25), true);
  const v = m.version;
  assert.equal(m.clearContactsAlong([{ x: 0, y: 0 }, { x: 0, y: 60 }], 9), 1);
  assert.ok(m.version > v);
  assert.equal(m.kind(0, 25), 'free');
  m.markContact(-40, 0);
  assert.equal(m.clearContactsNear(-45, 5, 9), 1);
  assert.equal(m.clearContactsNear(-45, 5, 9), 0);
  // age limit in scans (default 20) and in time
  const e = new GridMap({ contactScans: 3 });
  e.markContact(30, 30);
  for (let i = 0; i < 2; i++) e.integrateScan(O, [{ angle: 180, cm: 50 }]);
  assert.equal(e.contactAt(30, 30), true);
  e.integrateScan(O, [{ angle: 180, cm: 50 }]);
  assert.equal(e.contactAt(30, 30), false);
  assert.equal(new GridMap({}).contactScans, 20);
  const t = new GridMap({ contactMs: 0 });
  t.markContact(30, 30);
  t.integrateScan(O, [{ angle: 180, cm: 50 }]);
  assert.equal(t.contactAt(30, 30), false);
  // cleanup and clear remove all contacts; contacts are not saved
  m.markContact(-40, 0);
  m.markContact(40, 0);
  assert.ok(m.cleanup() >= 2);
  assert.equal(m.contacts().length, 0);
  m.markContact(-40, 0);
  assert.equal(GridMap.fromJSON(JSON.parse(JSON.stringify(m))).contacts().length, 0);
  m.clear();
  assert.equal(m.contacts().length, 0);
});

test('gridmap: a single forward reading is weak evidence; a second one or a sweep makes it suspect; a miss clears it', () => {
  const m = new GridMap({});
  const opts = { beamDeg: 25, maxRangeCm: 150, weak: true, robotRadiusCm: 0 };
  m.integrateScan(O, [{ angle: 0, cm: 60 }], opts);              // a glitch, maybe
  assert.equal(m.kind(0, 66), 'weak');
  assert.equal(m.cell(0, 66), 'unknown');
  const l = m.L[m.index(0, 66)];
  assert.ok(l >= 0.05 && l <= 0.15, `L ${l}`);                    // below the suspect threshold
  assert.equal(m.cellInfo(0, 66).weakReadings, 1);
  // a second agreeing reading: suspect, never confirmed by weak readings alone
  m.integrateScan(O, [{ angle: 0, cm: 60 }], opts);
  assert.equal(m.kind(0, 66), 'suspect');
  for (let i = 0; i < 5; i++) m.integrateScan(O, [{ angle: 0, cm: 60 }], opts);
  assert.notEqual(m.kind(0, 66), 'occupied');
  // one sweep through it clears a single weak reading
  const g = new GridMap({});
  g.integrateScan(O, [{ angle: 0, cm: 60 }], opts);
  g.integrateScan(O, [{ angle: 0, cm: 140 }], { beamDeg: 25, maxRangeCm: 150, robotRadiusCm: 0 });
  assert.ok(['free', 'unknown'].includes(g.kind(0, 66)), g.kind(0, 66));
  // a weak reading plus a sweep hit: suspect
  const s = new GridMap({});
  s.integrateScan(O, [{ angle: 0, cm: 60 }], opts);
  s.integrateScan(O, [{ angle: 0, cm: 60 }], { beamDeg: 25, maxRangeCm: 150, robotRadiusCm: 0 });
  assert.equal(s.kind(0, 66), 'suspect');
});

test('gridmap: far walls (near the 150 cm range) are confirmed by two sweeps', () => {
  const m = new GridMap({});
  for (let i = 0; i < 2; i++) m.integrateScan({ x: 2 * i, y: 0, heading: 0 }, [{ angle: 0, cm: 140 }], { beamDeg: 25, maxRangeCm: 150 });
  assert.equal(m.kind(0, 146), 'occupied');
  // a phantom seen once and seen through by two later sweeps disappears
  const p = new GridMap({});
  p.integrateScan(O, [{ angle: 0, cm: 70 }], { beamDeg: 25, maxRangeCm: 150 });
  assert.equal(p.kind(0, 76), 'suspect');
  for (let i = 0; i < 2; i++) p.integrateScan(O, [{ angle: 0, cm: 140 }], { beamDeg: 25, maxRangeCm: 150 });
  assert.ok(['free', 'unknown'].includes(p.kind(0, 76)), p.kind(0, 76));
  assert.ok(p.L[p.index(0, 76)] < 0.05);
});
