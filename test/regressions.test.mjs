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

test('calibration: joystick wheel mapping, turn sign and gyro sign', async () => {
  const { parseSample } = await import('../js/motion.js');
  const sent = [];
  const r = new BleRobot({ log: stub, onStatus: stub });
  r.run = async (s) => sent.push(s);
  r.query = async (s) => { sent.push(s); return 90; };
  // default (standard mBot2, firmware 44.01.013): joystick forward = EM1 +, EM2 -
  await r.drive(40, 40);
  await r.drive(30, -30);
  await r.turn(90);
  assert.deepEqual(sent.splice(0), ['mbot2.drive_speed(40,-40)', 'mbot2.drive_speed(30,30)', 'mbot2.turn(-90,50)']);
  // connectors swapped, firmware turn normal, gyro reversed
  Object.assign(r.wheels, { swap: true, turnSign: 1, yawSign: -1 });
  await r.drive(40, 20);
  await r.turn(90);
  assert.equal(await r.yaw(), -90);
  assert.deepEqual(sent.splice(0, 2), ['mbot2.drive_speed(20,-40)', 'mbot2.turn(90,50)']);
  // encoders and yaw in samples follow the same calibration
  const keys = ['encL', 'encR', 'yaw'];
  assert.deepEqual(parseSample(keys, [100, -100, 10], { mirrored: true }), { distanceCm: undefined, encL: 100, encR: 100, yaw: 10, shake: undefined });
  assert.deepEqual(parseSample(keys, [100, -100, 10], { mirrored: true, swap: true, yawSign: -1 }), { distanceCm: undefined, encL: 100, encR: 100, yaw: -10, shake: undefined });
});

test('readings at or beyond the ultrasonic range draw no wall', async () => {
  const { GridMap } = await import('../js/gridmap.js');
  const map = new GridMap({ cellCm: 5, sizeCm: 800 });
  // the real sensor reports ~190 cm when it sees nothing
  map.integrateScan({ x: 0, y: 0, heading: 0 }, [{ angle: 0, cm: 191 }], { maxRangeCm: 150, beamDeg: 16 });
  assert.equal(map.cell(0, 6 + 191), 'unknown', 'no obstacle at the far reading');
  assert.equal(map.cell(0, 100), 'free', 'free space up to the range');
  const fresh = new GridMap({ cellCm: 5, sizeCm: 800 });
  fresh.integrateScan({ x: 0, y: 0, heading: 0 }, [{ angle: 0, cm: 90 }], { maxRangeCm: 150, beamDeg: 16 });
  assert.equal(fresh.cell(0, 6 + 90), 'occupied', 'a real echo inside the range is an obstacle');
});
