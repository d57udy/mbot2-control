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

// Sim at its default start (150, 100) facing up (-90) = map origin, heading 0.
// Map x = sim x - 150, map y = 100 - sim y, map heading = sim heading + 90.
async function setup({ attach = false, timeScale = 30, simOpts = {}, ...opts } = {}) {
  opts = { localize: false, ...opts };
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
      const r = await nav.goTo(goal, { tolCm: 10 });
      const crash = events.findIndex((e) => e.type === 'crash');
      assert.ok(crash >= 0, 'crash detected');
      assert.match(events[crash].reason, /stall|jolt/);
      assert.ok(events[crash].backedCm > 3, `backed ${events[crash].backedCm}`);
      // contact remembered in the map between the robot and the box
      const c = nav.contacts[3]; // middle of the first contact
      assert.ok(Math.abs(c.x) < 4 && c.y > 30 && c.y < 40, `contact ${JSON.stringify(c)}`);
      assert.equal(map.cell(c.x, c.y), 'occupied');
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
