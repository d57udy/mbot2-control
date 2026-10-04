import { test } from 'node:test';
import assert from 'node:assert/strict';
import { pointsFromLog, profile, estimateLatency, findPost, postLatency } from '../js/scanlab.js';

// Synthetic sweep: robot spins at rateDegS, a post at 0° / 60 cm and a wall at
// 90° / 40 cm; each reply arrives trueLatency ms after the distance was measured.
function sweepLog(rateDegS, trueLatency, { periodMs = 120 } = {}) {
  const log = [];
  const world = (a) => {
    a = ((a % 360) + 540) % 360 - 180;
    if (Math.abs(a) < 8) return 60;
    if (a > 75 && a < 105) return 40 / Math.cos(((a - 90) * Math.PI) / 180);
    return 190; // nothing: the sensor reports ~190
  };
  for (let t = 0; t <= 9000; t += periodMs) {
    const yawNow = Math.round(rateDegS * t / 1000); // integer gyro like the firmware
    const angleWhenMeasured = rateDegS * (t - trueLatency) / 1000;
    log.push({ t, yaw: ((yawNow + 540) % 360) - 180, cm: world(angleWhenMeasured) });
  }
  return log;
}

test('pointsFromLog unwraps yaw and applies latency', () => {
  const pts = pointsFromLog(sweepLog(45, 100), 100);
  const post = findPost(pts);
  assert.ok(Math.abs(post.angle) <= 3, `post at ${post.angle}`);
  assert.equal(post.cm, 60);
});

test('wrong latency shifts the post in the turning direction', () => {
  const cw = findPost(pointsFromLog(sweepLog(45, 100), 0));
  const ccw = findPost(pointsFromLog(sweepLog(-45, 100), 0));
  assert.ok(cw.angle > 2 && ccw.angle < -2, `cw ${cw.angle} ccw ${ccw.angle}`);
});

test('estimateLatency recovers the true latency from a cw/ccw pair', () => {
  const { best } = estimateLatency(sweepLog(45, 110), sweepLog(-45, 110));
  assert.ok(Math.abs(best.latencyMs - 110) <= 40, `estimated ${best.latencyMs}`);
});

test('profile ignores readings beyond the range', () => {
  const bins = profile([{ angle: 0, cm: 190 }, { angle: 10, cm: 50 }], 2, 150);
  assert.equal(bins.filter((b) => b != null).length, 1);
});

test('postLatency recovers the latency from the bottle angles', () => {
  const r = postLatency(sweepLog(45, 110), sweepLog(-45, 110));
  assert.ok(Math.abs(r.latencyMs - 110) <= 40, `post latency ${r.latencyMs}`);
});
