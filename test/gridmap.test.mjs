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

test('gridmap: a hit ahead makes the beam free and the arc occupied', () => {
  const m = new GridMap({});
  m.integrateScan(O, [{ angle: 0, cm: 100 }]);   // sensor at y = 6, hit at y = 106
  assert.equal(m.cell(0, 0), 'free');            // robot footprint
  assert.equal(m.cell(0, 50), 'free');
  assert.equal(m.cell(0, 95), 'free');
  assert.equal(m.cell(0, 106), 'occupied');
  assert.equal(m.cell(0, 130), 'unknown');       // behind the hit
  assert.equal(m.cell(-40, 50), 'unknown');      // outside the beam
  assert.equal(m.cell(0, -40), 'unknown');
  // the arc is centre-weighted: edge cells are weaker than the centre
  const edge = m.index(Math.sin((7.5 * Math.PI) / 180) * 100, 6 + Math.cos((7.5 * Math.PI) / 180) * 100);
  assert.ok(m.L[edge] > 0 && m.L[edge] < m.L[m.index(0, 106)]);
  const b = m.bounds;
  assert.ok(b.minY < 0 && b.maxY >= 105 && b.minX < -5 && b.maxX > 5);
});

test('gridmap: heading and angle rotate the beam clockwise; failed reads are ignored', () => {
  const m = new GridMap({});
  m.integrateScan({ x: 0, y: 0, heading: 90 }, [{ angle: 0, cm: 50 }, { angle: 90, cm: 40 }, { angle: 180, cm: null }]);
  assert.equal(m.cell(30, 0), 'free');           // heading 90 = right
  assert.equal(m.cell(56, 0), 'occupied');
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
  m.integrateScan(O, [{ angle: 0, cm: 100 }], { beamDeg: 16, freeBeamDeg: 30 });
  const at = (deg, r) => m.cell(Math.sin((deg * Math.PI) / 180) * r, 6 + Math.cos((deg * Math.PI) / 180) * r);
  // not looked at: one pass leaves it unknown, a second pass makes it free
  assert.equal(at(13, 50), 'unknown');
  assert.ok(m.L[m.index(Math.sin((13 * Math.PI) / 180) * 50, 6 + Math.cos((13 * Math.PI) / 180) * 50)] < 0);
  m.integrateScan(O, [{ angle: 0, cm: 100 }], { beamDeg: 16, freeBeamDeg: 30, robotRadiusCm: 0 });
  assert.equal(at(13, 50), 'free');
  assert.equal(at(13, 90), 'unknown');           // beyond 0.7 of the range
  assert.notEqual(at(13, 100), 'occupied');
  assert.equal(at(0, 100), 'occupied');
});

test('gridmap: hit evidence survives free cones that only graze it', () => {
  // a thin leg 60 cm ahead, seen once by the beam centre
  const m = new GridMap({});
  m.integrateScan(O, [{ angle: 0, cm: 60 }], { beamDeg: 16, robotRadiusCm: 0 });
  const k = m.index(0, 66);
  assert.equal(m.stateOf(k), 'occupied');
  // later scans from the side: widened gap fillers and beam edges pass over it many times
  const side = { x: -50, y: 66, heading: 90 };
  for (let i = 0; i < 10; i++) {
    m.integrateScan(side, [{ angle: 12, cm: 200 }, { angle: -12, cm: 200 }], { beamDeg: 16, freeBeamDeg: 30, robotRadiusCm: 0 });
  }
  for (let i = 0; i < 3; i++) m.integrateScan(side, [{ angle: 6, cm: 200 }], { beamDeg: 16, robotRadiusCm: 0 });
  assert.equal(m.stateOf(k), 'occupied');
  // the core of a beam through it does clear it (the obstacle moved away)
  for (let i = 0; i < 3; i++) m.integrateScan(side, [{ angle: 0, cm: 200 }], { beamDeg: 16, robotRadiusCm: 0 });
  assert.equal(m.stateOf(k), 'free');
  // the robot footprint erodes hit evidence only slowly (pose errors)
  const m2 = new GridMap({});
  m2.L[m2.index(3, 3)] = 1.2;
  m2.markFree(0, 0, 9);
  assert.equal(m2.cell(3, 3), 'occupied');
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
  m.integrateScan(O, [{ angle: 0, cm: 100 }], { beamDeg: 1 });
  assert.equal(m.isTraversable(0, 50), true);
  assert.equal(m.isTraversable(0, 95), false);                  // within 14 cm of the hit at 106
  assert.equal(m.isTraversable(0, 95, { inflateCm: 5 }), true);
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
  m.integrateScan(O, [{ angle: 0, cm: 50 }, { angle: 90, cm: 120 }, { angle: 180, cm: 300 }, { angle: -90, cm: 300 }]);
  const s = m.describe({ x: 0, y: 0, heading: 90 });
  assert.ok(s.length < 400, `${s.length}`);
  assert.match(s, /m² known/);
  assert.match(s, /left 5\d/);          // the hit ahead of the start is left of a robot facing right
  assert.match(s, /ahead 12\d/);
  assert.match(s, /frontier/);
});

test('gridmap: clear, forEachCell, JSON round trip', () => {
  const m = new GridMap({});
  m.integrateScan(O, [{ angle: 30, cm: 80 }]);
  const v = m.version;
  const j = JSON.parse(JSON.stringify(m));
  const m2 = GridMap.fromJSON(j);
  let a = 0, b = 0;
  m.forEachCell((x, y, s, p) => { a++; assert.ok(p > 0 && p < 1); assert.equal(m2.cell(x, y), s); });
  m2.forEachCell(() => b++);
  assert.equal(a, b);
  assert.ok(a > 20);
  m.clear();
  assert.ok(m.version > v);
  assert.equal(m.bounds, null);
  let c = 0; m.forEachCell(() => c++);
  assert.equal(c, 0);
});
