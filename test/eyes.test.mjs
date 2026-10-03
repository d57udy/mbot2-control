// Eye LED commands: script generation, whitelist, quiet period. Run with: npm test
import { test } from 'node:test';
import assert from 'node:assert/strict';
import { BleRobot, EYE_EFFECT_NEEDS_BRI } from '../js/robot-ble.js';
import { CommandBus, makeCommand, EYE_EFFECTS, LED_EFFECTS } from '../js/bus.js';

// Decodes the Python source from a script frame (see protocol.js buildScriptFrame).
const decode = (frame) => {
  const n = frame[8] | (frame[9] << 8);
  return { mode: frame[5], idx: frame[6] | (frame[7] << 8), script: new TextDecoder().decode(frame.slice(10, 10 + n)) };
};

// BleRobot with a fake characteristic; `reply(script)` decides what the robot answers
// (undefined = no reply). Chunks are reassembled per write call sequence.
function fakeRobot(reply = () => undefined) {
  const logs = [];
  const sent = [];
  const robot = new BleRobot({ log: (m) => logs.push(m), onStatus: () => {}, chunkSize: 4096, chunkDelayMs: 0, helpers: true });
  robot.connected = true;
  robot.writeChar = {
    async writeValueWithoutResponse(bytes) {
      const f = decode(bytes);
      sent.push(f);
      const v = reply(f.script);
      if (v !== undefined && f.mode === 1) {
        setTimeout(() => robot.resolveReply({ idx: f.idx, value: v, raw: JSON.stringify({ ret: v }) }), 1);
      }
    },
  };
  return { robot, sent, logs };
}

// Robot with the helper installed (hasattr check answers [true, true]).
const helperReply = (onFx) => (s) => {
  if (s.startsWith('exec(') || s.startsWith('setattr(mbot2')) return null;
  if (s.startsWith('[hasattr(mbot2')) return [true, true];
  if (s.startsWith('mbot2._eyes')) return 'list';
  if (s.startsWith('mbot2._fx')) return onFx(s);
  return undefined;
};

test('whitelists keep the known names and every name is a safe identifier', () => {
  assert.equal(EYE_EFFECTS.length, 13);
  for (const n of [...EYE_EFFECTS, ...LED_EFFECTS]) assert.match(n, /^[a-z_]+$/);
  for (const n of EYE_EFFECT_NEEDS_BRI) assert.ok(EYE_EFFECTS.includes(n), n);
});

test('eye effect uses the helper and returns the measured run time', async () => {
  const { robot, sent } = fakeRobot(helperReply(() => 2400));
  const r = await robot.eyesEffect('happy');
  assert.deepEqual(r, { name: 'happy', ms: 2400 });
  const fx = sent.find((f) => f.script.startsWith('mbot2._fx'));
  assert.equal(fx.script, 'mbot2._fx("happy")');
  assert.equal(fx.mode, 1, 'effects are sent with reply so errors come back');
  // the helper is uploaded in pieces (each under MAX_SCRIPT) and then executed
  const src = sent.filter((f) => f.script.startsWith("setattr(mbot2,'_s',mbot2._s+"))
    .map((f) => JSON.parse(f.script.slice("setattr(mbot2,'_s',mbot2._s+".length, -1))).join('');
  assert.ok(src.includes('except TypeError:f(b)'), 'helper retries with a brightness argument');
  assert.equal(robot.effectEstimateMs('happy'), 2700);
  // second call does not reinstall the helper
  await robot.eyesEffect('dizzy');
  assert.equal(sent.filter((f) => f.script === 'exec(mbot2._s)').length, 1);
});

test('helper errors surface as exceptions', async () => {
  const { robot } = fakeRobot(helperReply((s) => (s.includes('naughty') ? 'missing' : "error: TypeError('x')")));
  await assert.rejects(robot.eyesEffect('naughty'), /does not exist/);
  await assert.rejects(robot.eyesEffect('wink'), /TypeError/);
  await assert.rejects(robot.eyesEffect('happy_effect()\nimport os'), /bad effect name/);
});

test('without helper, direct calls pass brightness where the firmware needs it', async () => {
  const { robot, sent } = fakeRobot((s) => (s.includes('_effect(') ? null : undefined));
  // helper install times out (no reply); shorten by stubbing query for the install
  const q = robot.query.bind(robot);
  robot.query = (s, t) => (s.startsWith('exec(') || s.startsWith('setattr(mbot2') ? Promise.reject(new Error('timeout')) : q(s, t));
  await robot.eyesEffect('happy');
  await robot.eyesEffect('thinking');
  const scripts = sent.map((f) => f.script);
  assert.ok(scripts.includes('cyberpi.ultrasonic2.happy_effect(100)'));
  assert.ok(scripts.includes('cyberpi.ultrasonic2.thinking_effect()'));
});

test('effect timeout resolves as running and a late reply is logged, not unmatched', async () => {
  const { robot, logs } = fakeRobot(() => undefined);
  const p = robot.queryUntilDone('cyberpi.ultrasonic2.dizzy_effect()', 20, 'eyes dizzy');
  const r = await p;
  assert.deepEqual(r, { done: false });
  const idx = [...robot.pending.keys()][0];
  robot.resolveReply({ idx, value: null, raw: '{"ret":null}' });
  assert.ok(logs.some((l) => /eyes dizzy finished late/.test(l)));
  assert.ok(!logs.some((l) => /unmatched/.test(l)));
  assert.equal(robot.pending.size, 0);
});

test('eyes: first call reports the mode, later calls are fire and forget', async () => {
  const { robot, sent, logs } = fakeRobot(helperReply(() => 0));
  await robot.eyes(100, 0);
  await robot.eyes(30, 60);
  const eyeFrames = sent.filter((f) => f.script.startsWith('mbot2._eyes'));
  assert.deepEqual(eyeFrames.map((f) => [f.script, f.mode]), [['mbot2._eyes(100,0)', 1], ['mbot2._eyes(30,60)', 0]]);
  assert.ok(logs.some((l) => /led_show\(\[8 values\],1\)/.test(l)));
});

test('eye_led all uses the documented 8-value led_show', async () => {
  const { robot, sent } = fakeRobot();
  await robot.eyeLed('all', 40);
  await robot.eyeLed(3, 70);
  assert.deepEqual(sent.map((f) => f.script), [
    'cyberpi.ultrasonic2.led_show([40,40,40,40,40,40,40,40],1)',
    'cyberpi.ultrasonic2.set_bri(70,3,1)',
  ]);
});

test('probeEyeEffects builds one hasattr expression', async () => {
  const { robot, sent } = fakeRobot((s) => (s.startsWith('[n for n') ? ['happy'] : undefined));
  assert.deepEqual(await robot.probeEyeEffects(['happy', 'wink']), ['happy']);
  assert.equal(sent[0].script, `[n for n in ["happy","wink"] if hasattr(cyberpi.ultrasonic2,n+'_effect')]`);
  assert.throws(() => robot.probeEyeEffects(['x()']), /bad effect name/);
});

// --- bus quiet period ---------------------------------------------------------

function busWithFake() {
  let release;
  const calls = [];
  const robot = {
    connected: true,
    distance: async () => { calls.push('distance'); return 42; },
    battery: async () => { calls.push('battery'); return 80; },
    eyesEffect: (name) => { calls.push(`eyes ${name}`); return new Promise((r) => { release = r; }); },
    effectEstimateMs: () => 5000,
  };
  const logs = [];
  const bus = new CommandBus({ log: (m) => logs.push(m) });
  bus.setRobot(robot);
  return { bus, calls, logs, done: (v) => release(v) };
}

test('distance reads during an eye effect return the cached value without touching the robot', async () => {
  const { bus, calls, done } = busWithFake();
  assert.equal((await bus.submit(makeCommand('read', { sensor: 'distance' }))).value, 42);
  const fx = bus.submit(makeCommand('eyes_effect', { name: 'happy' }));
  await Promise.resolve();
  assert.ok(bus.sensorsQuiet());
  assert.ok(bus.sensorQuietUntil > Date.now() + 4000);
  const r = await bus.submit(makeCommand('read', { sensor: 'distance' }));
  assert.deepEqual([r.ok, r.value], [true, 42]);
  assert.equal(calls.filter((c) => c === 'distance').length, 1);
  // a second effect is refused while one runs
  const busy = await bus.submit(makeCommand('eyes_effect', { name: 'wink' }));
  assert.equal(busy.ok, false);
  assert.match(busy.error, /busy/);
  done({ name: 'happy', ms: 2000 });
  assert.equal((await fx).ok, true);
  assert.ok(!bus.sensorsQuiet(), 'reply ends the quiet period');
  await bus.submit(makeCommand('read', { sensor: 'distance' }));
  assert.equal(calls.filter((c) => c === 'distance').length, 2);
});

test('an effect still running after timeout keeps the quiet period', async () => {
  const { bus, done } = busWithFake();
  const fx = bus.submit(makeCommand('eyes_effect', { name: 'dizzy' }));
  await Promise.resolve();
  done({ name: 'dizzy', running: true });
  await fx;
  assert.ok(bus.sensorsQuiet());
});

test('unknown eye effect is rejected before reaching the robot', async () => {
  const { bus, calls } = busWithFake();
  const r = await bus.submit(makeCommand('eyes_effect', { name: 'happy_effect();import os' }));
  assert.equal(r.ok, false);
  assert.equal(calls.length, 0);
  assert.ok(!bus.sensorsQuiet());
});

test('robots without effectEstimateMs (simulator) still work', async () => {
  const bus = new CommandBus({ log: () => {} });
  bus.setRobot({ connected: true, eyesEffect: async () => undefined });
  const r = await bus.submit(makeCommand('eyes_effect', { name: 'thinking' }));
  assert.equal(r.ok, true);
  assert.ok(!bus.sensorsQuiet());
});
