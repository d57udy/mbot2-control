// Regression tests for the v0.3 code review findings.
import { test } from 'node:test';
import assert from 'node:assert/strict';
import { CommandBus, makeCommand } from '../js/bus.js';
import { SimRobot } from '../js/robot-sim.js';
import { BleRobot } from '../js/robot-ble.js';
import { scan, findOpenings, describeScan, driveToward } from '../js/scan.js';
import { createToolExecutor } from '../js/tools.js';

const stub = () => {};

test('a second "Stopp" fires again after the dedupe window', async () => {
  globalThis.window ??= {};
  window.webkitSpeechRecognition = function () {};
  const { VoiceListener } = await import('../js/voice.js');
  let stops = 0;
  const v = new VoiceListener({ onStop: () => stops++ });
  const ev = (t) => ({ results: [Object.assign([{ transcript: t }], { isFinal: true })] });
  v.handle(ev('Stopp'));
  v.handle(ev('Stopp'));             // Android duplicate, same moment
  assert.equal(stops, 1);
  v.stopFiredAt -= 2000;             // later in the session
  v.handle(ev('Stopp'));
  assert.equal(stops, 2);
});

test('BLE disconnect settles queued commands instead of hanging', async () => {
  const ble = new BleRobot({ log: stub, onStatus: stub, chunkDelayMs: 0 });
  let release;
  ble.writeChar = { writeValueWithoutResponse: () => new Promise((r) => { release = r; }) };
  ble.connected = true;
  const first = ble.run('mbot2.forward(10,1)');
  const queued = ble.query('cyberpi.get_battery()', 60000);
  await new Promise((r) => setTimeout(r, 10));
  ble.handleDisconnect();
  release();
  await first.catch(() => {});
  await assert.rejects(queued, /disconnected/);
});

test('stop generation cancels stamped commands but not later ones', async () => {
  const sim = new SimRobot({ log: stub, onStatus: stub, timeScale: 20 });
  const bus = new CommandBus({ log: stub });
  bus.setRobot(sim);
  await sim.connect();
  const mk = bus.stamped();
  await bus.stop('ui');
  const r = await bus.submit(mk('turn', { deg: 90 }, 'agent'));
  assert.equal(r.ok, false);
  assert.match(r.error, /cancelled by stop/);
  const r2 = await bus.submit(bus.stamped()('turn', { deg: 10 }, 'agent'));
  assert.equal(r2.ok, true);
  const soft = bus.stopGen;
  await bus.stop('ui', { soft: true });
  assert.equal(bus.stopGen, soft, 'soft stop keeps the generation');
  await sim.disconnect();
});

test('STOPP during drive_toward cancels the drive after the turn', async () => {
  const sim = new SimRobot({ log: stub, onStatus: stub, timeScale: 4 });
  const bus = new CommandBus({ log: stub });
  bus.setRobot(sim);
  await sim.connect();
  const ex = createToolExecutor({ bus, makeCommand, scan, findOpenings, describeScan, driveToward });
  const s = await ex.execute('scan_surroundings', { steps: 8 });
  assert.equal(s.ok, true);
  const start = { x: sim.state.x, y: sim.state.y };
  const p = ex.execute('drive_toward', { angle: 90, cm: 40 });
  setTimeout(() => bus.stop('ui'), 30); // during the 90 degree turn
  const r = await p;
  await new Promise((res) => setTimeout(res, 200));
  const moved = Math.hypot(sim.state.x - start.x, sim.state.y - start.y);
  await sim.disconnect();
  assert.ok(moved < 2, `robot drove ${moved.toFixed(1)} cm after stop`);
  assert.equal(r.ok, false);
});

test('invalid numbers are rejected, not sent to the robot', async () => {
  const sim = new SimRobot({ log: stub, onStatus: stub });
  const bus = new CommandBus({ log: stub });
  bus.setRobot(sim);
  await sim.connect();
  const r = await bus.submit(makeCommand('straight', { cm: 'abc' }));
  await sim.disconnect();
  assert.equal(r.ok, false);
  assert.match(r.error, /invalid number/);
});

test('a stalled Claude request times out with a clear error', async () => {
  const { ConversationAgent } = await import('../js/agent.js');
  const fetchImpl = (url, { signal }) => new Promise((_, rej) => {
    signal.addEventListener('abort', () => rej(Object.assign(new Error('aborted'), { name: 'AbortError' })));
  });
  const agent = new ConversationAgent({ apiKey: 'k', fetchImpl, requestTimeoutMs: 50, executor: { execute: async () => ({ ok: true }) } });
  await assert.rejects(agent.send('hallo'), /Zeitüberschreitung/);
  assert.equal(agent.busy, false);
});

test('turn and straight carry the chosen speed', async () => {
  const sent = [];
  const fake = { connected: true, turn: async (d, o) => sent.push(['turn', d, o.speed]), straight: async (c, o) => sent.push(['straight', c, o.speed]) };
  const bus = new CommandBus({ log: stub });
  bus.setRobot(fake);
  bus.settings.speed = 30;
  await bus.submit(makeCommand('turn', { deg: 90 }));
  await bus.submit(makeCommand('straight', { cm: 20, speed: 80 }));
  assert.deepEqual(sent, [['turn', 90, 30], ['straight', 20, 80]]);
});

test('BLE turn compensates the firmware turn sign (+90 = clockwise)', async () => {
  const { FIRMWARE_TURN_SIGN } = await import('../js/robot-ble.js');
  const sent = [];
  const r = new BleRobot({ log: stub, onStatus: stub });
  r.run = async (s) => sent.push(s);
  r.query = async (s) => sent.push(s);
  await r.turn(90, { speed: 40 });
  await r.turn(-45, { wait: true, speed: 40 });
  assert.equal(FIRMWARE_TURN_SIGN, -1);
  assert.deepEqual(sent, ['mbot2.turn(-90,40)', 'mbot2.turn(45,40)']);
});
