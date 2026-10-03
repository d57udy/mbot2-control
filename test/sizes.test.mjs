// Every script the BLE driver sends must fit the robot's frame limit
// (measured: 200 bytes ok, 300 bytes silently dropped on 44.01.013).
import { test } from 'node:test';
import assert from 'node:assert/strict';
import { BleRobot, MAX_SCRIPT } from '../js/robot-ble.js';
import { EYE_EFFECTS, LED_EFFECTS } from '../js/bus.js';

test('all generated scripts fit MAX_SCRIPT', async () => {
  const seen = [];
  const r = new BleRobot({ log() {}, onStatus() {} });
  r.connected = true;
  r.query = async (s) => {
    seen.push(s);
    if (s.startsWith('str(')) return '';
    if (s.includes('hasattr(mbot2')) return [1, true, true].slice(-2).length === 2 && s.includes("'_wr'") ? [1, true] : [true, true];
    if (s.includes('get_line_sta')) return [0, 0, [0, 0, 0, 0]];
    if (s.includes('get_color_sta')) return [['white', 'white', 'white', 'white'], [[0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0]]];
    return [];
  };
  r.run = async (s) => { seen.push(s); };
  r.queryUntilDone = async (s) => { seen.push(s); return { done: true, value: null, ms: 1 }; };
  await r.installWatchdog();
  await r.ensureEyeHelper();
  await r.floor(true);
  await r.ledAll(Array.from({ length: 5 }, () => [255, 255, 255]));
  await r.diagnose();
  await r.probeEyeEffects?.(EYE_EFFECTS).catch(() => {});
  for (const n of LED_EFFECTS) await r.ledEffect(n).catch(() => {});
  await r.display('x'.repeat(40));
  const over = seen.filter((s) => new TextEncoder().encode(s).length > MAX_SCRIPT);
  assert.deepEqual(over, [], `too long: ${over.map((s) => s.length)}`);
  assert.ok(seen.some((s) => s === 'exec(mbot2._s)'), 'long helpers are sent in pieces');
});

test('the driver refuses an oversized script instead of letting the robot drop it', async () => {
  const r = new BleRobot({ log() {}, onStatus() {}, chunkDelayMs: 0 });
  r.connected = true;
  r.writeChar = { writeValueWithoutResponse: async () => {} };
  await assert.rejects(r.query(`len("${'x'.repeat(300)}")`), /too long/);
  assert.equal(r.pending.size, 0);
});
