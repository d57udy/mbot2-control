// Run with: npm test
import test from 'node:test';
import assert from 'node:assert/strict';
import { scan, findOpenings, describeScan, driveToward, explore, normAngle } from '../js/scan.js';
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
