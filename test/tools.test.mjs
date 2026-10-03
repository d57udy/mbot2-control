import { test } from 'node:test';
import assert from 'node:assert/strict';
import { TOOLS, EMOTIONS, createToolExecutor, parseColor, toolsFor } from '../js/tools.js';
import { makeCommand, EYE_EFFECTS } from '../js/bus.js';

function fakeBus({ distance = 200, fail = {} } = {}) {
  const sent = [];
  return {
    sent,
    async submit(c) {
      sent.push(c);
      if (fail[c.cmd]) return { id: c.id, ok: false, error: fail[c.cmd] };
      if (c.cmd === 'read' && c.args.sensor === 'distance') return { id: c.id, ok: true, value: distance };
      if (c.cmd === 'read' && c.args.sensor === 'battery') return { id: c.id, ok: true, value: 87 };
      if (c.cmd === 'read' && c.args.sensor === 'floor') return { id: c.id, ok: true, value: { line: 6 } };
      return { id: c.id, ok: true };
    },
  };
}

const POINTS = [
  { angle: 0, cm: 30 }, { angle: 90, cm: 150 }, { angle: 180, cm: 12 }, { angle: 270, cm: 80 },
];

function setup(opts = {}) {
  const bus = fakeBus(opts);
  const calls = { scan: [], drive: [], emotions: [], scans: [] };
  const ex = createToolExecutor({
    bus,
    makeCommand,
    scan: async (b, o) => { calls.scan.push(o); return { points: opts.points ?? POINTS }; },
    findOpenings: (pts, { minCm }) => pts.filter((p) => p.cm >= minCm).map((p) => ({ angle: p.angle, widthDeg: 90, cm: p.cm, score: p.cm })),
    describeScan: (pts) => `scan ${pts.length}`,
    driveToward: async (b, o) => { calls.drive.push(o); return { ok: true, droveCm: o.cm - 1 }; },
    onEmotion: (e) => calls.emotions.push(e),
    onScan: (p, o) => calls.scans.push([p, o]),
  });
  return { bus, ex, calls };
}

test('tool schemas are well formed', () => {
  const names = new Set();
  for (const t of TOOLS) {
    assert.match(t.name, /^[a-z_]+$/);
    assert.ok(!names.has(t.name), `duplicate ${t.name}`);
    names.add(t.name);
    assert.ok(t.description.length > 10 && t.description.length < 400);
    assert.equal(t.input_schema.type, 'object');
    assert.equal(typeof t.input_schema.properties, 'object');
    for (const r of t.input_schema.required ?? []) assert.ok(r in t.input_schema.properties);
  }
  for (const n of ['move_straight', 'turn', 'stop', 'scan_surroundings', 'drive_toward', 'read_distance', 'read_battery',
    'read_floor', 'express_emotion', 'set_back_leds', 'play_sound', 'play_tone', 'show_text',
    'navigate_to', 'explore_room', 'go_home', 'describe_map']) assert.ok(names.has(n), n);
  assert.doesNotThrow(() => JSON.stringify(TOOLS));
});

test('emotions only use whitelisted eye effects', () => {
  for (const [name, e] of Object.entries(EMOTIONS)) {
    assert.ok(EYE_EFFECTS.includes(e.eyes), `${name}: ${e.eyes}`);
    assert.equal(e.led.length, 3);
  }
});

test('move_straight maps to straight with wait and agent src', async () => {
  const { bus, ex } = setup();
  const r = await ex.execute('move_straight', { cm: 30 });
  assert.equal(r.ok, true);
  const c = bus.sent.at(-1);
  assert.equal(c.cmd, 'straight');
  assert.deepEqual(c.args, { cm: 30, wait: true });
  assert.equal(c.src, 'agent');
  assert.equal(bus.sent[0].cmd, 'read'); // distance check first
});

test('move_straight clamps and keeps 20 cm from obstacles', async () => {
  let { bus, ex } = setup({ distance: 300 });
  await ex.execute('move_straight', { cm: 500 });
  assert.equal(bus.sent.at(-1).args.cm, 100);
  ({ bus, ex } = setup({ distance: 50 }));
  const r = await ex.execute('move_straight', { cm: 60 });
  assert.equal(r.moved_cm, 30);
  assert.match(r.note, /shortened/);
  ({ bus, ex } = setup({ distance: 15 }));
  const blocked = await ex.execute('move_straight', { cm: 10 });
  assert.equal(blocked.ok, false);
  assert.ok(!bus.sent.some((c) => c.cmd === 'straight'));
  ({ bus, ex } = setup({ distance: 5 }));
  await ex.execute('move_straight', { cm: -500 });
  assert.equal(bus.sent.at(-1).args.cm, -100); // backward needs no distance check
});

test('invalid inputs are rejected before the bus', async () => {
  const { bus, ex } = setup();
  assert.equal((await ex.execute('move_straight', { cm: 'los' })).ok, false);
  assert.equal((await ex.execute('turn', {})).ok, false);
  assert.equal((await ex.execute('rm_rf', {})).ok, false);
  assert.equal((await ex.execute('express_emotion', { emotion: 'evil' })).ok, false);
  assert.equal((await ex.execute('set_back_leds', { color: 'chartreuse-ish' })).ok, false);
  assert.equal((await ex.execute('set_back_leds', { colors: ['red', 'red'] })).ok, false);
  assert.equal((await ex.execute('play_sound', { kind: 'explosion' })).ok, false);
  assert.equal((await ex.execute('toString', {})).ok, false);
  assert.equal(bus.sent.length, 0);
});

test('turn clamps and uses wait', async () => {
  const { bus, ex } = setup();
  await ex.execute('turn', { degrees: -720 });
  assert.deepEqual(bus.sent[0].args, { deg: -360, wait: true });
  await ex.execute('turn', { degrees: '45' });
  assert.equal(bus.sent[1].args.deg, 45);
});

test('sensors and simple commands', async () => {
  const { bus, ex } = setup({ distance: 42 });
  assert.deepEqual(await ex.execute('read_distance', {}), { ok: true, cm: 42 });
  assert.deepEqual(await ex.execute('read_battery', {}), { ok: true, percent: 87 });
  assert.deepEqual(await ex.execute('read_floor', { colors: true }), { ok: true, floor: { line: 6 } });
  assert.equal(bus.sent[2].args.colors, true);
  await ex.execute('stop', {});
  assert.equal(bus.sent[3].cmd, 'stop');
  await ex.execute('play_tone', { freq: 99999, secs: 5 });
  assert.deepEqual(bus.sent[4].args, { freq: 4000, secs: 1 });
  await ex.execute('play_sound', { kind: 'happy' });
  assert.equal(bus.sent.filter((c) => c.cmd === 'beep').length, 4);
});

test('colours: names, hex, arrays of five', async () => {
  assert.deepEqual(parseColor('Rot'), [255, 0, 0]);
  assert.deepEqual(parseColor('#00ff80'), [0, 255, 128]);
  assert.equal(parseColor('blurple'), null);
  const { bus, ex } = setup();
  await ex.execute('set_back_leds', { color: 'blue' });
  assert.deepEqual(bus.sent[0], { ...bus.sent[0], cmd: 'led', args: { r: 0, g: 0, b: 255 } });
  await ex.execute('set_back_leds', { colors: ['red', 'green', 'blue', '#ffffff', 'off'] });
  assert.equal(bus.sent[1].cmd, 'leds');
  assert.deepEqual(bus.sent[1].args.colors[3], [255, 255, 255]);
});

test('show_text strips control and quote characters', async () => {
  const { bus, ex } = setup();
  const r = await ex.execute('show_text', { text: 'Hallo "Welt"\n\')\\x' + 'y'.repeat(60) });
  assert.equal(r.ok, true);
  const t = bus.sent[0].args.text;
  assert.ok(t.length <= 40);
  assert.doesNotMatch(t, /["'\\\n]/);
});

test('express_emotion is best effort', async () => {
  const { bus, ex, calls } = setup({ fail: { eyes_effect: 'unknown eye effect' } });
  const r = await ex.execute('express_emotion', { emotion: 'happy' });
  assert.equal(r.ok, true);
  assert.equal(r.parts.leds, true);
  assert.equal(r.parts.eyes, 'unknown eye effect');
  assert.deepEqual(calls.emotions, ['happy']);
  assert.deepEqual(bus.sent.map((c) => c.cmd), ['led', 'beep', 'eyes_effect']);
  const all = setup({ fail: { led: 'x', beep: 'x', eyes_effect: 'x' } });
  assert.equal((await all.ex.execute('express_emotion', { emotion: 'happy' })).ok, false);
});

test('scan_surroundings returns compact points and openings', async () => {
  const { ex, calls } = setup();
  const ac = new AbortController();
  const r = await ex.execute('scan_surroundings', { steps: 16 }, { signal: ac.signal });
  assert.equal(r.ok, true);
  assert.equal(calls.scan[0].steps, 16);
  assert.equal(calls.scan[0].signal, ac.signal);
  assert.deepEqual(r.points[1], [90, 150]);
  assert.equal(r.openings[0].angle, 90);
  assert.equal(r.summary, 'scan 4');
  assert.equal(calls.scans.length, 1);
  await ex.execute('scan_surroundings', { steps: 7 });
  assert.equal(calls.scan[1].steps, 12);
});

test('drive_toward needs a fresh scan and respects obstacles', async () => {
  const { ex, calls } = setup();
  assert.equal((await ex.execute('drive_toward', { angle: 90, cm: 50 })).ok, false);
  await ex.execute('scan_surroundings', {});
  const blocked = await ex.execute('drive_toward', { angle: 175, cm: 50 });
  assert.equal(blocked.ok, false);
  assert.match(blocked.error, /obstacle/);
  const r = await ex.execute('drive_toward', { angle: 90, cm: 200 });
  assert.equal(r.ok, true);
  assert.equal(r.cm, 99); // reports what the driver actually drove
  assert.deepEqual({ ...calls.drive[0], makeCommand: undefined }, { angle: 90, cm: 100, makeCommand: undefined });
  // the scan is consumed by the move
  assert.equal((await ex.execute('drive_toward', { angle: 90, cm: 20 })).ok, false);
  await ex.execute('scan_surroundings', {});
  await ex.execute('drive_toward', { angle: -90, cm: 100 }); // 270 = -90, 80 cm free
  assert.equal(calls.drive[1].cm, 60);
  assert.equal(calls.drive[1].angle, -90);
  await ex.execute('scan_surroundings', {});
  await ex.execute('turn', { degrees: 10 });
  assert.equal((await ex.execute('drive_toward', { angle: 90, cm: 20 })).ok, false);
});

test('aborted signal and thrown errors become results', async () => {
  const ac = new AbortController();
  ac.abort();
  const { bus, ex } = setup();
  assert.deepEqual(await ex.execute('turn', { degrees: 10 }, { signal: ac.signal }), { ok: false, error: 'aborted' });
  assert.equal(bus.sent.length, 0);
  const broken = createToolExecutor({ bus, makeCommand, scan: async () => { throw new Error('ble lost'); } });
  assert.deepEqual(await broken.execute('scan_surroundings', {}), { ok: false, error: 'ble lost' });
});

test('works with the real scan.js', async () => {
  const scanMod = await import('../js/scan.js');
  const bus = fakeBus({ distance: 120 });
  const ex = createToolExecutor({ bus, makeCommand, ...scanMod });
  const s = await ex.execute('scan_surroundings', { steps: 8 });
  assert.equal(s.ok, true, s.error);
  assert.equal(s.points.length, 8);
  assert.equal(typeof s.summary, 'string');
  const d = await ex.execute('drive_toward', { angle: 45, cm: 50 });
  assert.equal(d.ok, true, d.error);
  assert.ok(bus.sent.some((c) => c.cmd === 'straight' && c.src === 'agent'));
});

const NAV = ['navigate_to', 'explore_room', 'go_home', 'describe_map'];

function fakeNavigator({ goTo = { ok: true, reached: true, legs: 2, note: 'arrived' } } = {}) {
  const calls = [];
  return {
    calls,
    pose: { pose: { x: 10.4, y: -3.6, heading: 44.7 } }, // a PoseTracker
    async scanHere(o) { calls.push(['scanHere', o]); return { points: POINTS }; },
    async goTo(goal, o) { calls.push(['goTo', goal, o]); return { ...goTo, pose: { x: goal.x + 0.4, y: goal.y - 0.4, heading: 90.2 } }; },
    async explore(o) { calls.push(['explore', o]); return { ok: true, moves: o.maxMoves, frontiersLeft: 2 }; },
    async goHome(o) { calls.push(['goHome', o]); return { ok: true, reached: true, legs: 1, pose: { x: 1, y: -1, heading: 0 } }; },
    describe() { calls.push(['describe']); return 'pose (10,-4) 45deg; known 3 m2'; },
  };
}

test('toolsFor offers navigation tools only with a navigator', () => {
  const on = toolsFor({ navigation: true }).map((t) => t.name);
  const off = toolsFor({ navigation: false }).map((t) => t.name);
  for (const n of NAV) { assert.ok(on.includes(n), n); assert.ok(!off.includes(n), n); }
  assert.equal(toolsFor().length, TOOLS.length - NAV.length);
  assert.equal(on.length, TOOLS.length);
  const nav = TOOLS.find((t) => t.name === 'navigate_to').input_schema;
  assert.deepEqual(nav.required, ['x_cm', 'y_cm']);
  assert.equal(nav.properties.x_cm.maximum, 400);
  assert.equal(nav.properties.y_cm.minimum, -400);
  assert.equal(TOOLS.find((t) => t.name === 'explore_room').input_schema.properties.max_moves.maximum, 8);
});

test('navigation tools without a navigator fail cleanly', async () => {
  const { bus, ex } = setup();
  for (const n of NAV) assert.deepEqual(await ex.execute(n, { x_cm: 10, y_cm: 10 }), { ok: false, error: 'navigation not available' });
  assert.equal(bus.sent.length, 0);
});

test('navigation tools map onto the navigator with the signal', async () => {
  const bus = fakeBus();
  const nav = fakeNavigator();
  const ex = createToolExecutor({ bus, makeCommand, navigator: nav, findOpenings: () => [], describeScan: () => 's' });
  const ac = new AbortController();
  const r = await ex.execute('navigate_to', { x_cm: 1000, y_cm: '-50.4' }, { signal: ac.signal });
  assert.deepEqual(nav.calls[0], ['goTo', { x: 400, y: -50 }, { signal: ac.signal }]);
  assert.deepEqual(r, { ok: true, reached: true, goal: { x: 400, y: -50 }, legs: 2, pose: { x: 400, y: -50, heading: 90 }, note: 'arrived' });
  assert.equal((await ex.execute('navigate_to', { x_cm: 'da', y_cm: 1 })).ok, false);
  assert.equal(nav.calls.length, 1);

  const e = await ex.execute('explore_room', { max_moves: 20 }, { signal: ac.signal });
  assert.deepEqual(nav.calls[1], ['explore', { signal: ac.signal, maxMoves: 8 }]);
  assert.deepEqual(e, { ok: true, moves: 8, frontiers_left: 2, pose: { x: 10, y: -4, heading: 45 } });
  await ex.execute('explore_room', {});
  assert.equal(nav.calls[2][1].maxMoves, 4);

  const h = await ex.execute('go_home', {}, { signal: ac.signal });
  assert.deepEqual(nav.calls[3], ['goHome', { signal: ac.signal }]);
  assert.deepEqual(h.pose, { x: 1, y: -1, heading: 0 });
  assert.deepEqual(await ex.execute('describe_map', {}), { ok: true, map: 'pose (10,-4) 45deg; known 3 m2' });
  assert.equal(bus.sent.length, 0); // the navigator drives, not the executor
});

test('navigation failures and aborts become results', async () => {
  const bus = fakeBus();
  const nav = fakeNavigator({ goTo: { ok: false, reached: false, legs: 1, note: 'no path' } });
  const ex = createToolExecutor({ bus, makeCommand, navigator: nav });
  const r = await ex.execute('navigate_to', { x_cm: 50, y_cm: 50 });
  assert.equal(r.ok, false);
  assert.equal(r.error, 'no path');
  assert.equal(r.reached, false);
  assert.equal(r.note, undefined);
  const ac = new AbortController();
  ac.abort();
  assert.deepEqual(await ex.execute('go_home', {}, { signal: ac.signal }), { ok: false, error: 'aborted' });
  assert.equal(nav.calls.length, 1);
  nav.goTo = async () => { const err = new Error('aborted'); err.name = 'AbortError'; throw err; };
  assert.deepEqual(await ex.execute('navigate_to', { x_cm: 0, y_cm: 10 }), { ok: false, error: 'aborted' });
});

test('scan_surroundings goes through the navigator so it lands in the map', async () => {
  const bus = fakeBus();
  const nav = fakeNavigator();
  const scans = [];
  const ex = createToolExecutor({
    bus, makeCommand, navigator: nav,
    scan: async () => { throw new Error('should use the navigator'); },
    findOpenings: (pts, { minCm }) => pts.filter((p) => p.cm >= minCm).map((p) => ({ angle: p.angle, widthDeg: 90, cm: p.cm })),
    describeScan: (pts) => `scan ${pts.length}`,
    onScan: (p) => scans.push(p),
  });
  const ac = new AbortController();
  const r = await ex.execute('scan_surroundings', { steps: 8 }, { signal: ac.signal });
  assert.equal(r.ok, true, r.error);
  assert.deepEqual(nav.calls[0], ['scanHere', { signal: ac.signal, steps: 8 }]);
  assert.deepEqual(r.points[1], [90, 150]);
  assert.equal(r.summary, 'scan 4');
  assert.equal(r.openings[0].angle, 90);
  assert.equal(scans.length, 1);
  // drive_toward still works off that scan, without a driveToward helper
  const d = await ex.execute('drive_toward', { angle: 90, cm: 40 });
  assert.equal(d.ok, true, d.error);
  assert.deepEqual(bus.sent.map((c) => c.cmd), ['turn', 'straight']);
});
