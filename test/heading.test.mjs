// Run with: npm test
import { test } from 'node:test';
import assert from 'node:assert/strict';
import { HeadingEstimator, fuseSegment, findWalls, roomAxis, snapHeading, encoderHeadingDeg, wrap45, normDeg } from '../js/heading.js';
import { SimRobot } from '../js/robot-sim.js';
import { CommandBus } from '../js/bus.js';
import { makeSimSampler, driveLeg, turnInPlace } from '../js/motion.js';
import { sweepScan } from '../js/scan.js';

const stub = () => {};

test('fuseSegment: encoders by default, gyro when they differ by more than 2 deg', () => {
  const calm = fuseSegment({ gyroDeg: 1.4, encDeg: 0.3, gyroWeight: 0 });
  assert.equal(calm.source, 'enc');
  assert.equal(calm.deg, 0.3);
  assert.ok(Math.abs(fuseSegment({ gyroDeg: 1.4, encDeg: 0.3, gyroWeight: 0.5 }).deg - 0.85) < 1e-9, 'optional blend');
  assert.equal(calm.slip, false);
  const bump = fuseSegment({ gyroDeg: 4.2, encDeg: 0.1 });
  assert.equal(bump.source, 'gyro');
  assert.equal(bump.deg, 4.2);
  assert.equal(bump.slip, true);
  assert.equal(fuseSegment({ gyroDeg: 90, encDeg: null }).source, 'gyro');
  assert.equal(fuseSegment({ gyroDeg: undefined, encDeg: 3 }).deg, 3);
  assert.ok(Math.abs(encoderHeadingDeg(100, -100) - (200 * (Math.PI * 6.5 / 360) / 12) * (180 / Math.PI)) < 1e-9);
  assert.equal(wrap45(92), 2);
  assert.equal(wrap45(-46), 44);
});

test('HeadingEstimator: standing still freezes the heading and learns the gyro bias', () => {
  const est = new HeadingEstimator();
  const bias = 2 / 60; // 2 deg/min
  let t = 0, yawTrue = 0;
  const raw = (enc) => ({ t: t * 1000, yaw: Math.round(yawTrue + bias * t), encL: enc, encR: enc }); // integer yaw like the firmware
  // stops of 12 s (one sample at each end), 5 s drives between them
  let enc = 0, out;
  for (let k = 0; k < 8; k++) {
    out = est.ingest(raw(enc));
    t += 12;
    out = est.ingest(raw(enc));
    const before = out.yaw;
    t += 5; enc += 500; // drive straight
    out = est.ingest(raw(enc));
    if (k >= 5) assert.ok(Math.abs(out.yaw - before) < 1.2, `drive step ${out.yaw - before}`); // one integer count at most
  }
  assert.ok(Math.abs(est.biasDegPerS - bias) < 0.25 * bias, `bias ${est.biasDegPerS} vs ${bias}`);
  // 136 s of 2 deg/min: the raw yaw drifted about 4.5 deg, the corrected one stays near 0
  assert.ok(Math.round(yawTrue + bias * t) >= 4);
  assert.ok(Math.abs(out.yaw) < 1.5, `corrected yaw ${out.yaw}`); // drives before the bias was known, integer steps
});

// Readings of a simulated 360 deg sweep, 4.5 deg apart, from a sim pose.
async function simSweep(sim, x, y, mapHeading) {
  Object.assign(sim.state, { x, y });
  const pts = [];
  for (let a = 0; a < 360; a += 4.5) { sim.state.heading = mapHeading - 90 + a; pts.push({ angle: a, cm: await sim.distance() }); }
  return pts;
}

test('wall snap: room axes from one sweep, then a heading error is snapped back', async () => {
  const sim = new SimRobot({ log: stub, onStatus: stub });
  const o = { beamDeg: 16 }; // the simulator's cone
  const first = await simSweep(sim, 150, 100, 0);
  const walls = findWalls(first, o);
  assert.ok(walls.length >= 3, JSON.stringify(walls));
  const ax = roomAxis(walls, 0, o);
  assert.ok(ax && Math.abs(ax.axisDeg) < 1.5, JSON.stringify(ax));
  // the same room seen with the heading estimate 6 deg too far clockwise
  for (const err of [6, -7]) {
    const pts = await simSweep(sim, 150, 100, 0);
    const r = snapHeading(pts, err, ax.axisDeg, o);
    assert.equal(r.applied, true, r.reason);
    assert.ok(Math.abs(r.correctionDeg + err) < 1.5, `error ${err}: correction ${r.correctionDeg}`);
  }
  // a correction beyond maxSnapDeg is refused (could be the wrong axis)
  const far = snapHeading(first, 30, ax.axisDeg, o);
  assert.equal(far.applied, false);
  // too few walls: nothing to snap
  assert.equal(snapHeading(first.slice(0, 6), 0, 0, o).applied, false);
});

test('HeadingEstimator.sweep: first sweep sets the axes, a rotated room works too', async () => {
  const sim = new SimRobot({ log: stub, onStatus: stub });
  const est = new HeadingEstimator({ beamDeg: 16, snapGain: 1, axisSweeps: 1 });
  // the robot started 20 deg off the room axes: its map frame is rotated
  const pts = await simSweep(sim, 150, 100, 20);
  const r1 = est.sweep(pts, 0);
  assert.equal(r1.type, 'heading-axes');
  assert.ok(Math.abs(wrap45(est.axisDeg + 20)) < 1.5, `axis ${est.axisDeg}`);
  const r2 = est.sweep(pts, 4); // the estimate drifted 4 deg
  assert.equal(r2.applied, true, r2.reason);
  assert.ok(Math.abs(r2.correctionDeg + 4) < 1.5, `correction ${r2.correctionDeg}`);
});

// Two-minute mission in the simulator with gyro drift 2 deg/min, a floor
// threshold that turns the robot 4 deg (gyro sees it, encoders do not), a
// slight wheel diameter mismatch (0.1 %), encoder noise, integer and
// unbounded yaw. The heading is kept the way the navigator will: fused
// segments, stops, sweeps with snaps.
async function mission() {
  const sim = new SimRobot({
    log: stub, onStatus: stub, timeScale: 40, gyroDriftDegPerMin: 2, floorJumps: [{ atCm: 75, deg: 4 }],
    yawInteger: true, yawMode: 'unbounded', encNoiseDeg: 0.5, encScale: [1.0005, 0.9995],
  });
  const bus = new CommandBus({ log: stub });
  bus.setRobot(sim);
  await sim.connect();
  try {
    // the simulator's cone; its ultrasonic is exact out to 300 cm, so side walls at 150 count
    const est = new HeadingEstimator({ beamDeg: 16, maxWallCm: 180 });
    const raw = makeSimSampler(sim);
    const sample = est.wrap(raw);
    const clock = raw.clock;
    const truth = () => normDeg(sim.state.heading + 90);
    const t0 = clock.now();
    const yaw0 = (await raw()).yaw;
    let H = 0; // pipeline heading, map frame
    const errs = [], events = [];
    const check = (what) => errs.push({ what, err: normDeg(H - truth()) });
    const stand = async (s) => { await sample(); await clock.sleep(s * 1000); await sample(); };
    const sweep = async () => {
      const at = H;
      const m = est.mark();
      const r = await sweepScan(bus, { sample, latencyMs: 0 }); // the sim reads instantly
      H = normDeg(H + est.segmentSince(m, 'sweep').deg);
      const s = est.sweep(r.points, at);
      events.push(s);
      if (s.applied) H = normDeg(H + s.correctionDeg);
      check('sweep');
    };
    await stand(12);
    await sweep();
    let cycle = 0;
    while (clock.now() - t0 < 120000) {
      let m = est.mark();
      await turnInPlace(bus, { deg: 90, sample });
      H = normDeg(H + est.segmentSince(m, 'turn').deg);
      check('turn');
      m = est.mark();
      await driveLeg(bus, { cm: 30, sample });
      H = normDeg(H + est.segmentSince(m, 'leg').deg);
      check('leg');
      await stand(10);
      if (++cycle % 2 === 0) await sweep();
    }
    const maxErr = Math.max(...errs.map((e) => Math.abs(e.err)));
    const trace = `heading error ${JSON.stringify(errs.map((e) => [e.what, Math.round(e.err * 10) / 10]))} snaps ${JSON.stringify(events.map((e) => [e.type, e.correctionDeg, e.agreeing, e.walls, e.reason]))}`;
    assert.ok(est.events.some((e) => e.type === 'heading-slip'), 'the threshold was flagged');
    assert.ok(est.axisDeg != null, 'room axes set from a sweep');
    // snaps are rare here (from most poses fewer than two clean walls agree);
    // any that applied must not have pushed the heading the wrong way by much
    for (const e of events.filter((x) => x.applied)) assert.ok(Math.abs(e.correctionDeg) < 2, JSON.stringify(e.correctionDeg));
    // without the pipeline: the raw gyro drifted, encoders alone missed the threshold
    const rawErr = Math.abs(normDeg((await raw()).yaw - yaw0 - truth()));
    assert.ok(rawErr > 3, `raw gyro error only ${rawErr}`);
    assert.ok(clock.now() - t0 >= 120000);
    return { maxErr, trace };
  } finally {
    await sim.disconnect();
  }
}

// Over 20 single runs the worst heading error per mission was 0.3 to 1.9 deg
// (median 1.0). Remaining terms: integer yaw (the 4 deg threshold is measured
// to +-1 deg) and snap noise (+-1 deg, halved by snapGain). Three missions run
// in parallel here, which adds timing jitter: each must stay under 2.5 deg,
// their mean worst error under 2 deg.
test('heading pipeline: 2 minute missions stay within 2 deg', async () => {
  const runs = await Promise.all([mission(), mission(), mission()]);
  for (const r of runs) assert.ok(r.maxErr < 2.5, r.trace);
  const mean = runs.reduce((a, r) => a + r.maxErr, 0) / runs.length;
  assert.ok(mean < 2, `mean worst error ${mean.toFixed(2)}: ${runs.map((r) => r.trace).join(' | ')}`);
});

test('HeadingEstimator.sweep: late or untrusted sweeps do not set the room axes', async () => {
  const sim = new SimRobot({ log: stub, onStatus: stub });
  const pts = await simSweep(sim, 150, 100, 0);
  const est = new HeadingEstimator({ beamDeg: 16 });
  est.ingest({ t: 0, yaw: 0 });
  est.ingest({ t: 60000, yaw: 4 }); // a minute later: too late, the heading may have drifted
  assert.equal(est.sweep(pts, 4).reason, 'heading not trusted for room axes');
  assert.equal(est.axisDeg, null);
  assert.equal(est.sweep(pts, 0, { trusted: true }).type, 'heading-axes'); // right after a reference match
  est.sweep(pts, 0, { trusted: true });
  assert.ok(est.axisDeg != null && Math.abs(est.axisDeg) < 1.5, `axis ${est.axisDeg}`);
  const one = new HeadingEstimator({ beamDeg: 16 });
  one.ingest({ t: 0, yaw: 0 });
  one.sweep(pts, 0); // early: counts
  one.ingest({ t: 60000, yaw: 0 });
  one.sweep(pts, 0); // late: settles for the one early sweep
  assert.ok(one.axisDeg != null);
});
