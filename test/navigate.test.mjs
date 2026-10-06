// Run with: npm test
import { describe, test } from 'node:test';
import assert from 'node:assert/strict';
import { Navigator } from '../js/navigate.js';
import { PoseTracker } from '../js/pose.js';
import { GridMap } from '../js/gridmap.js';
import { scan } from '../js/scan.js';
import { CommandBus } from '../js/bus.js';
import { SimRobot, collides, SIM_OBSTACLES } from '../js/robot-sim.js';
import { makeSimSampler } from '../js/motion.js';

const stub = () => {};

// timeScale 15: the sim robot moves timeScale times faster than real time, so
// an event-loop stall of 50 ms under the parallel sim tests lets a leg run on
// for 0.75 s of robot time before its stop lands; at 30 that was 1.5 s
// (up to 20 cm) and legs ended close to walls.
// Sim at its default start (150, 100) facing up (-90) = map origin, heading 0.
// Map x = sim x - 150, map y = 100 - sim y, map heading = sim heading + 90.
async function setup({ attach = false, timeScale = 15, simOpts = {}, ...opts } = {}) {
  // the sim reads distance and yaw at the same instant: no sweep latency to compensate
  opts = { localize: false, sweepLatencyMs: 0, ...opts };
  const sim = new SimRobot({ log: stub, onStatus: stub, timeScale, ...simOpts });
  const bus = new CommandBus({ log: stub });
  bus.setRobot(sim);
  await sim.connect();
  // closest approach: smallest robot radius that would touch something
  const track = { minClear: Infinity };
  sim.onChange = (s) => {
    for (let r = 9; r < 14; r += 0.5) if (collides(s.x, s.y, sim.room, sim.obstacles, r)) { track.minClear = Math.min(track.minClear, r); break; }
  };
  const map = new GridMap({});
  const pose = new PoseTracker();
  if (attach) pose.attach(bus);
  const events = [];
  const nav = new Navigator({ bus, map, pose, scan, settleMs: 0, sample: makeSimSampler(sim), onEvent: (e) => events.push(e), ...opts });
  const truth = () => ({ x: sim.state.x - 150, y: 100 - sim.state.y, heading: ((sim.state.heading + 90) % 360 + 540) % 360 - 180 });
  return { sim, bus, map, pose, nav, events, truth, track };
}

const d = (a, b) => Math.hypot(a.x - b.x, a.y - b.y);

// Smallest distance from a map-frame path to a real obstacle surface (sim frame).
function trueClearance(path, obstacles) {
  let min = Infinity;
  for (let i = 1; i < path.length; i++) {
    const a = path[i - 1], b = path[i], L = Math.hypot(b.x - a.x, b.y - a.y) || 1;
    for (let t = 0; t <= L; t += 1) {
      const sx = a.x + ((b.x - a.x) * t) / L + 150, sy = 100 - (a.y + ((b.y - a.y) * t) / L);
      for (const o of obstacles) {
        const dd = o.kind === 'circle' ? Math.hypot(sx - o.x, sy - o.y) - o.r
          : Math.hypot(sx - Math.min(o.x + o.w, Math.max(o.x, sx)), sy - Math.min(o.y + o.h, Math.max(o.y, sy)));
        min = Math.min(min, dd);
      }
    }
  }
  return min;
}
const angErr = (a, b) => Math.abs((((a - b) % 360) + 540) % 360 - 180);

describe('navigator with SimRobot', { concurrency: true }, () => {
  test('goTo around the pouf, then goHome', async () => {
    const { sim, nav, pose, events, truth, track } = await setup();
    try {
      const goal = { x: -125, y: -60 };   // behind the pouf seen from the start
      const r = await nav.goTo(goal, { tolCm: 10 });
      assert.equal(r.ok, true, r.note);
      assert.equal(r.reached, true);
      assert.ok(d(truth(), goal) <= 15, `truth ${JSON.stringify(truth())}`);
      assert.ok(d(truth(), pose.pose) < 5 && angErr(truth().heading, pose.pose.heading) < 3, 'estimate matches truth');
      assert.ok(track.minClear > 10, `came within ${track.minClear} cm of contact`);
      assert.ok(events.some((e) => e.type === 'scan') && events.some((e) => e.type === 'plan') && events.some((e) => e.type === 'leg'));
      assert.equal(events.at(-1).type, 'arrived');
      assert.ok(pose.trail.length > 2);

      const h = await nav.goHome({});
      assert.equal(h.ok, true, h.note);
      assert.ok(d(truth(), { x: 0, y: 0 }) <= 15, `home ${JSON.stringify(truth())}`);
      assert.ok(angErr(truth().heading, 0) < 3, `heading ${truth().heading}`);
      assert.ok(track.minClear > 10);
    } finally {
      await sim.disconnect();
    }
  });

  test('goTo around the chair with the pose tracker attached to the bus', async () => {
    const { sim, nav, pose, truth, track } = await setup({ attach: true });
    try {
      // past the table leg to above the chair. The old goal (130, -40) sat in the
      // 38 cm gap between the chair and the right wall: with 14 cm inflation
      // that leaves a 2-cell corridor, which one misplaced cell from a scan
      // closes ("no path"), so the outcome depended on where the scans landed.
      const goal = { x: 120, y: 15 };
      const r = await nav.goTo(goal, { tolCm: 10 });
      assert.equal(r.ok, true, r.note);
      assert.ok(d(truth(), goal) <= 15, `truth ${JSON.stringify(truth())}`);
      // applied once: the estimate follows the truth
      assert.ok(d(truth(), pose.pose) < 5, `est ${JSON.stringify(pose.pose)} truth ${JSON.stringify(truth())}`);
      assert.ok(track.minClear > 10, `came within ${track.minClear} cm of contact`);
    } finally {
      await sim.disconnect();
    }
  });

  test('explore grows the known area without contact', async () => {
    const { sim, nav, map, truth, pose, track } = await setup({ steps: 8 });
    try {
      await nav.scanHere({});
      const before = map.stats().knownM2;
      const r = await nav.explore({ maxMoves: 3 });
      assert.equal(r.ok, true, r.note);
      assert.ok(r.moves >= 1);
      assert.ok(Number.isInteger(r.frontiersLeft));
      assert.ok(map.stats().knownM2 > before + 0.3, `known ${before} -> ${map.stats().knownM2}`);
      assert.ok(track.minClear > 10, `came within ${track.minClear} cm of contact`);
      assert.ok(d(truth(), pose.pose) < 5);
      assert.ok(nav.describe().length < 600);
      assert.match(nav.describe(), /^Pose: x -?\d+ cm/);
    } finally {
      await sim.disconnect();
    }
  });

  test('abort mid-task stops the robot and throws AbortError', async () => {
    const { sim, nav } = await setup();
    try {
      const ac = new AbortController();
      const p = nav.goTo({ x: -125, y: -60 }, { signal: ac.signal });
      setTimeout(() => ac.abort(), 400);
      await assert.rejects(p, { name: 'AbortError' });
      await new Promise((r) => setTimeout(r, 80));
      assert.equal(sim.motion, null);
      assert.equal(nav.busy, false);
    } finally {
      await sim.disconnect();
    }
  });

  test('a stop on the bus cancels the task without throwing', async () => {
    const { sim, bus, nav, events } = await setup();
    try {
      const p = nav.goTo({ x: -125, y: -60 });
      setTimeout(() => bus.stop('ui'), 300);
      const r = await p;
      assert.equal(r.ok, false);
      assert.match(r.note, /cancelled by stop/);
      assert.equal(events.at(-1).type, 'error');
      await new Promise((res) => setTimeout(res, 80));
      assert.equal(sim.motion, null);
    } finally {
      await sim.disconnect();
    }
  });

  test('crash into an obstacle the ultrasonic cannot see: detect, back off, mark, replan', async () => {
    // a low box straight ahead of the start, at map (0, 40); scans see past it
    const low = { kind: 'circle', x: 150, y: 60, r: 5, low: true, label: 'low box' };
    const { sim, nav, map, pose, events, truth } = await setup({ localize: true, simOpts: { obstacles: [...SIM_OBSTACLES, low] } });
    try {
      const goal = { x: 0, y: 75 };
      let atCrash = null;
      const prev = nav.onEvent;
      nav.onEvent = (e) => {
        if (e.type === 'crash' && !atCrash) { const c = nav.contacts[3]; atCrash = { c, cell: map.cell(c.x, c.y) }; }
        prev(e);
      };
      const r = await nav.goTo(goal, { tolCm: 10 });
      const crash = events.findIndex((e) => e.type === 'crash');
      assert.ok(crash >= 0, 'crash detected');
      assert.match(events[crash].reason, /stall|jolt/);
      assert.ok(events[crash].backedCm > 3, `backed ${events[crash].backedCm}`);
      // contact marked in the map between the robot and the box when the crash is reported
      const c = atCrash.c; // middle of the first contact
      assert.ok(Math.abs(c.x) < 4 && c.y > 30 && c.y < 40, `contact ${JSON.stringify(c)}`);
      assert.equal(atCrash.cell, 'occupied');
      assert.ok(events.slice(crash).some((e) => e.type === 'scan'), 'rescanned after the crash');
      assert.ok(events.slice(crash).some((e) => e.type === 'plan'), 'replanned');
      assert.ok(events.slice(crash).some((e) => e.type === 'localized' && !e.error), 'relocalized after the crash (js/localize.js)');
      assert.ok(events.filter((e) => e.type === 'crash').length <= 2);
      assert.equal(r.ok, true, r.note);
      assert.ok(d(truth(), goal) <= 15, `truth ${JSON.stringify(truth())}`);
      assert.ok(d(truth(), pose.pose) < 10, `est ${JSON.stringify(pose.pose)} truth ${JSON.stringify(truth())}`);
      assert.equal(sim.motion, null);
    } finally {
      await sim.disconnect();
    }
  });

  test('scans relocalize through matchScan/fusePose and clear the uncertain flag', async () => {
    const calls = [];
    let truthFn;
    const localizer = {
      matchScan: (map, guess, beams, opts) => { calls.push({ guess, n: beams.length, opts }); return { pose: truthFn(), score: 1, confidence: 0.9 }; },
      fusePose: (odom, m) => ({ pose: m.pose, source: 'scan' }),
    };
    const { sim, nav, pose, events, truth } = await setup({ localizer, localize: true, steps: 8, scanMode: 'step' });
    truthFn = truth;
    try {
      await nav.scanHere({});
      assert.equal(calls.length, 0, 'empty map: integrate only');
      pose.x += 12; pose.y -= 9; // odometry drift
      nav.poseUncertain = true;
      await nav.scanHere({});
      assert.equal(calls.length, 1);
      assert.equal(calls[0].opts.xyWindowCm, 60, 'wide search while uncertain');
      assert.ok(calls[0].n >= 4 && calls[0].n <= 8);
      const ev = events.find((e) => e.type === 'localized');
      assert.ok(ev && Math.abs(ev.correction.dx + 12) < 1 && Math.abs(ev.correction.dy - 9) < 1, JSON.stringify(ev));
      assert.ok(d(truth(), pose.pose) < 1);
      assert.equal(nav.poseUncertain, false);
    } finally {
      await sim.disconnect();
    }
  });

  // Field log (v0.5.4): turns -125, +104, -122, +104 ... and no progress. Reproduced
  // when mbot2.turn rotates the other way than commanded while the gyro keeps the
  // estimate right: every correction is executed backwards. Emulated here with
  // turnError -2 plus integer, unbounded yaw, encoder noise and 200 ms latency.
  const FIELD = { turnError: -2, yawInteger: true, yawMode: 'unbounded', encNoiseDeg: 1, latencyMs: 200 };
  for (const turnMode of ['gyro', 'blocking']) {
    test(`field oscillation regression (${turnMode} turns)`, async () => {
      // timeScale 10: the emulated latency must dominate event-loop jitter under test load
      const { sim, nav, pose, events, truth } = await setup({ attach: true, useYaw: true, localize: true, turnMode, timeScale: 10, simOpts: FIELD });
      try {
        const goal = { x: 110, y: 70 };
        const r = await nav.goTo(goal, { tolCm: 10, maxLegs: 10 });
        assert.equal(r.ok, true, r.note);
        assert.ok(d(truth(), goal) <= 15, `truth ${JSON.stringify(truth())}`);
        assert.ok(d(truth(), pose.pose) < 10 && angErr(truth().heading, pose.pose.heading) < 5, `est ${JSON.stringify(pose.pose)} truth ${JSON.stringify(truth())}`);
        const turns = events.filter((e) => e.type === 'turn');
        assert.ok(turns.length && turns.every((e) => e.mode === turnMode));
        // no back-and-forth. The field run flipped the sign of every large turn
        // (9 flips in 10 legs); a detour may need one, blocking mode one more
        // to undo its wrong-way turn before it has learned the sign.
        const big = turns.filter((e) => Math.abs(e.target) > 60).map((e) => e.target);
        const flips = big.filter((t, i) => i > 0 && Math.sign(t) !== Math.sign(big[i - 1])).length;
        assert.ok(flips <= (turnMode === 'blocking' ? 2 : 1), `large turns ${big}`);
        if (turnMode === 'blocking') assert.ok(events.some((e) => e.type === 'warning' && /opposite sign/.test(e.note)), 'learned the turn sign');
        else assert.ok(turns.every((e) => Math.abs(e.achieved - e.target) <= 3 || Math.abs(e.target) <= 1.5), JSON.stringify(turns.map((e) => [e.target, e.achieved])));
        assert.ok(!events.some((e) => e.type === 'crash'), `no false crash: ${JSON.stringify(events.filter((e) => e.type === 'crash'))}`);
      } finally {
        await sim.disconnect();
      }
    });
  }

  // Without periodic scans the table leg was never hit by a beam before the
  // home plan (6.6 cm from it). goHome now scans first when the robot moved
  // more than taskScanCm since the last scan. Sweep scans only: a step scan
  // has blind gaps between its beams (45 deg apart at 8 steps, 16 deg wide)
  // where a 4 cm leg can hide, so one scan cannot guarantee seeing it.
  for (const mode of ['sweep']) {
    test(`task-start scan keeps the home route clear of the table leg (${mode} scans)`, async () => {
      const { sim, nav, events } = await setup({});
      try {
        await nav.scanHere({});
        const r = await nav.goTo({ x: 110, y: 75 });
        assert.equal(r.ok, true, r.note);
        const before = events.length;
        const moved = nav.movedSinceScan();
        const h = await nav.goHome({});
        assert.equal(h.ok, true, h.note);
        const after = events.slice(before);
        // scanned first if the robot had moved more than 50 cm since the last scan
        if (moved > 50) assert.equal(after.find((e) => e.type === 'scan')?.reason, 'task-start');
        else assert.notEqual(after.find((e) => e.type === 'scan')?.reason, 'task-start');
        const plans = after.filter((e) => e.type === 'plan').map((e) => e.path);
        assert.ok(plans.length);
        for (const path of plans) {
          const c = trueClearance(path, sim.obstacles);
          assert.ok(c > 10, `home plan ${c.toFixed(1)} cm from an obstacle`);
        }
        assert.ok(!events.some((e) => e.type === 'scan' && /periodic/.test(e.reason)), 'no periodic scans');
      } finally {
        await sim.disconnect();
      }
    });
  }

  // The driver now compensates the firmware's reversed mbot2.turn, so turns
  // arrive with the right sign and only a scale error: the learned turn sign
  // must stay 1 (no double compensation).
  test('blocking turns with the right sign and 10 % error keep turnSign 1', async () => {
    const { sim, nav, events, pose, truth } = await setup({ attach: true, useYaw: true, turnMode: 'blocking', timeScale: 10,
      simOpts: { turnError: 0.1, yawInteger: true, yawMode: 'unbounded', latencyMs: 150 } });
    try {
      const r = await nav.goTo({ x: 110, y: 70 }, { tolCm: 10 });
      assert.equal(r.ok, true, r.note);
      assert.equal(nav.turnSign, 1);
      assert.ok(!events.some((e) => e.type === 'warning'), JSON.stringify(events.filter((e) => e.type === 'warning')));
      assert.ok(events.some((e) => e.type === 'turn' && Math.abs(e.target) >= 20), 'made a real turn');
      assert.ok(angErr(truth().heading, pose.pose.heading) < 3, 'gyro keeps the heading');
    } finally {
      await sim.disconnect();
    }
  });

  test('legs learn the stop distance (field: every leg 3 to 4 cm long)', async () => {
    const { sim, nav, events, pose, truth } = await setup({ simOpts: { stopCoastCm: 3.5 } });
    try {
      const r = await nav.goTo({ x: 110, y: 70 }, { tolCm: 10 });
      assert.equal(r.ok, true, r.note);
      const legs = events.filter((e) => e.type === 'leg' && e.reason === 'done');
      assert.ok(legs.length >= 3);
      assert.ok(legs[0].droveCm - legs[0].cm > 2, `first leg ${legs[0].cm} -> ${legs[0].droveCm}`);
      // the systematic 3.5 cm is learned away; what remains is stop-timing jitter
      // of the 15x sim under the parallel tests (up to 2.7 cm seen), not a bias
      const errs = legs.slice(2).map((l) => l.droveCm - l.cm);
      const mean = errs.reduce((a, b) => a + b, 0) / errs.length;
      assert.ok(Math.abs(mean) < 1.5 && errs.every((e) => Math.abs(e) < 3.2), `later legs off by ${errs.map((e) => e.toFixed(1))}`);
      assert.ok(d(truth(), pose.pose) < 5, 'the overshoot is in the pose');
    } finally {
      await sim.disconnect();
    }
  });

  test('a slipping wheel (encoders disagree, gyro straight) is no crash and leaves no contact', async () => {
    const { sim, nav, events, pose, truth } = await setup({ simOpts: { encScale: [1, 1.2] } });
    try {
      const r = await nav.goTo({ x: 110, y: 70 }, { tolCm: 10 });
      assert.equal(r.ok, true, r.note);
      assert.ok(!events.some((e) => e.type === 'crash'), JSON.stringify(events.filter((e) => e.type === 'crash')));
      assert.ok(events.some((e) => e.type === 'warning' && e.kind === 'slip'));
      assert.equal(nav.contacts.length, 0);
      assert.ok(d(truth(), pose.pose) < 8 && angErr(truth().heading, pose.pose.heading) < 3, `est ${JSON.stringify(pose.pose)} truth ${JSON.stringify(truth())}`);
    } finally {
      await sim.disconnect();
    }
  });

  test('yaw correction keeps the heading on the gyro', async () => {
    const { sim, nav, pose, truth } = await setup({ useYaw: true });
    try {
      const r = await nav.goTo({ x: 40, y: 40 }, { tolCm: 10 });
      assert.equal(r.ok, true, r.note);
      assert.notEqual(pose.yawRef, null);
      assert.ok(angErr(truth().heading, pose.pose.heading) < 1.5);
    } finally {
      await sim.disconnect();
    }
  });
});

// Scripted bus: a wall across y = wallY ahead, straights only track y.
function fakeBus({ wallY = 1e9, failTurn = false } = {}) {
  const bus = { y: 0, heading: 0, cmds: [], stops: 0 };
  bus.stamped = () => (cmd, args, src) => ({ cmd, args, src, gen: 7 });
  bus.submit = async (c) => {
    bus.cmds.push(c);
    if (c.cmd === 'turn') {
      if (failTurn) return { ok: false, error: 'robot not connected' };
      bus.heading = (bus.heading + c.args.deg) % 360;
      return { ok: true };
    }
    if (c.cmd === 'straight') { bus.y += c.args.cm * Math.cos((bus.heading * Math.PI) / 180); return { ok: true }; }
    if (c.cmd === 'read') {
      const cos = Math.cos((bus.heading * Math.PI) / 180);
      return { ok: true, value: cos > 0.9 ? Math.min(300, (wallY - bus.y) / cos - 6) : 300 };
    }
    return { ok: true };
  };
  bus.stop = async () => { bus.stops++; return { ok: true }; };
  return bus;
}

test('navigator: shortens a leg to keep safetyCm, then reports blocked', async () => {
  const bus = fakeBus({ wallY: 80 });
  const map = new GridMap({});
  const pose = new PoseTracker();
  const events = [];
  const nav = new Navigator({ bus, map, pose, scan, settleMs: 0, steps: 8, legMode: 'straight', inflateCm: 14, onEvent: (e) => events.push(e) });
  const r = await nav.goTo({ x: 0, y: 70 }, { tolCm: 5, maxLegs: 4 });
  assert.equal(r.ok, false);
  assert.equal(r.reached, false);
  const straights = bus.cmds.filter((c) => c.cmd === 'straight').map((c) => c.args.cm);
  assert.ok(straights.length >= 1);
  // never closer than safetyCm (20) to the reading: wall at 80, sensor 6 ahead
  assert.ok(bus.y <= 80 - 6 - 20 + 0.5, `y ${bus.y}`);
  assert.ok(Math.abs(pose.pose.y - bus.y) < 1e-6, 'self-applied pose');
  assert.ok(bus.cmds.every((c) => c.gen === 7 && c.src === 'agent'), 'one stamped makeCommand');
  assert.ok(events.some((e) => e.type === 'blocked'));
  assert.match(r.note, /blocked|no path|gave up|goal blocked/);
  // blocked after a scan here: back off and rescan before giving up (field:
  // five 'blocked' in one second, then 'blocked: 10.1 cm ahead')
  const backs = events.filter((e) => e.type === 'backoff');
  assert.ok(backs.length >= 1 && backs.length <= 2, `backoffs ${backs.length}`);
  assert.ok(bus.cmds.some((c) => c.cmd === 'straight' && c.args.cm === -10));
  const firstBack = events.indexOf(backs[0]);
  assert.ok(events.slice(firstBack).some((e) => e.type === 'scan' && /backed off/.test(e.reason)), 'rescanned after backing off');
});

test('navigator: failures return ok:false instead of throwing', async () => {
  const nav = new Navigator({ bus: fakeBus({ failTurn: true }), map: new GridMap({}), pose: new PoseTracker(), scan, settleMs: 0, steps: 4 });
  const r = await nav.goTo({ x: 50, y: 0 });
  assert.equal(r.ok, false);
  assert.match(r.note, /failed/);
  const bad = await nav.goTo({ x: 'a', y: 0 });
  assert.equal(bad.ok, false);
  const s = await nav.scanHere({});
  assert.equal(s.ok, false);
  const pre = new AbortController(); pre.abort();
  const bus = fakeBus();
  const nav2 = new Navigator({ bus, map: new GridMap({}), pose: new PoseTracker(), scan, settleMs: 0 });
  await assert.rejects(nav2.explore({ signal: pre.signal }), { name: 'AbortError' });
  assert.equal(bus.stops >= 1, true);
});

test('navigator: knownReach stops a leg along an unseen obstacle edge, not in open unknown space', () => {
  const map = new GridMap({});
  const nav = new Navigator({ bus: fakeBus(), map, pose: new PoseTracker(), scan, settleMs: 0 });
  const free = (x0, x1, y0, y1) => {
    for (let x = x0; x <= x1; x += 2.5) for (let y = y0; y <= y1; y += 2.5) map.add(map.index(x, y), -3);
    map.touch();
  };
  free(-30, 30, -10, 40);   // known, wide enough for the footprint up to y = 40
  free(-4, 4, 40, 80);      // only a narrow strip beyond: the sides are unknown
  assert.equal(nav.knownReach({ x: 0, y: 0 }, { x: 0, y: 80 }, 80), 80, 'open unknown space does not stop the leg');
  // an obstacle corner seen at a grazing angle, next to the unknown cells at x = 12
  map.add(map.index(22, 52), 3); map.touch();
  const r = nav.knownReach({ x: 0, y: 0 }, { x: 0, y: 80 }, 80);
  assert.ok(r >= 30 && r <= 45, `reach ${r}`);
  free(-30, 30, 40, 85);
  assert.equal(nav.knownReach({ x: 0, y: 0 }, { x: 0, y: 80 }, 80), 80, 'all known');
});

test('navigator: crash contacts are marked once, survive the sonar, clear when driven through', () => {
  const map = new GridMap({});
  const nav = new Navigator({ bus: fakeBus(), map, pose: new PoseTracker(), scan, settleMs: 0 });
  assert.equal(typeof map.markContact, 'function', 'gridmap contact layer');
  const pts = nav.addContact(30); // robot at (0, 0) facing +y: contact row at y = 30
  const mid = pts[3];
  const info = () => map.cellInfo(mid.x, mid.y);
  assert.equal(info().contact, true);
  assert.equal(map.cell(mid.x, mid.y), 'occupied', 'the planner treats a contact as an obstacle');
  const hits = info().hits;
  for (let i = 0; i < 5; i++) nav.plan({ x: 0, y: 80 });
  assert.equal(info().hits, hits, 'planning does not add evidence');
  assert.equal(info().contact, true);
  // a low obstacle looks see-through to the ultrasonic: close pass-through scans must not clear it
  for (let i = 0; i < 2; i++) {
    map.beginScan();
    map.integrateScan({ x: 0, y: 0, heading: 0 }, [{ angle: 0, cm: 120 }], { beamDeg: 25 });
    map.endScan();
  }
  assert.equal(info().contact, true, 'survives sonar pass-through scans');
  // the robot drove through the cell on a leg that ended normally
  assert.ok(map.clearContactsAlong([{ x: 0, y: 0 }, { x: 0, y: 50 }], 9) >= 1);
  assert.equal(info().contact, false, 'cleared by driving through');
  assert.notEqual(map.cell(mid.x, mid.y), 'occupied');
  nav.addContact(30);
  assert.equal(info().contact, true);
  map.cleanup();
  assert.equal(info().contact, false, 'cleared by cleanup()');
  assert.equal(map.contacts().length, 0);
});
