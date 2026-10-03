// Run with: npm test
import test from 'node:test';
import assert from 'node:assert/strict';
import { PoseTracker, normDeg } from '../js/pose.js';
import { CommandBus, makeCommand } from '../js/bus.js';

const near = (a, b, eps = 1e-6, msg) => assert.ok(Math.abs(a - b) <= eps, msg ?? `${a} != ${b}`);

test('pose: turns and straights in the map frame', () => {
  const p = new PoseTracker();
  p.applyStraight(10);
  assert.deepEqual(p.pose, { x: 0, y: 10, heading: 0 });
  p.applyTurn(90);       // clockwise = right
  p.applyStraight(20);
  near(p.x, 20); near(p.y, 10);
  p.applyTurn(-270);     // -180 net turn
  near(p.heading, 180);
  p.applyStraight(5);
  near(p.x, 20); near(p.y, 5);
  p.applyTurn(45);
  near(p.heading, -135);
  assert.equal(normDeg(-180), 180);
  assert.equal(normDeg(540), 180);
  assert.equal(normDeg(-190), 170);
});

test('pose: reset, trail is capped and skips tiny moves', () => {
  const p = new PoseTracker({ x: 5, y: 6, heading: 370 });
  assert.deepEqual(p.pose, { x: 5, y: 6, heading: 10 });
  p.reset();
  assert.deepEqual(p.pose, { x: 0, y: 0, heading: 0 });
  for (let i = 0; i < 700; i++) p.applyStraight(2);
  assert.equal(p.trail.length, 500);
  near(p.trail.at(-1).y, 1400);
  const n = p.trail.length;
  p.applyStraight(0.2);
  assert.equal(p.trail.length, n);
});

test('pose: differential drive', () => {
  const p = new PoseTracker();
  p.applyDrive(60, 60, 1);                 // one wheel turn forward
  near(p.y, Math.PI * 6.5); near(p.x, 0);
  p.reset();
  p.applyDrive(30, -30, 1);                // spin right in place
  near(p.x, 0); near(p.y, 0);
  near(p.heading, ((Math.PI * 6.5 * 0.5 * 2) / 12) * (180 / Math.PI), 1e-6);
  p.reset();
  // quarter circle to the right: arc ends right and ahead, heading 90
  const vDiff = (Math.PI / 2) * 12;        // wheel speed difference * t for 90 degrees
  p.applyDrive(60 + (vDiff / (Math.PI * 6.5)) * 30, 60 - (vDiff / (Math.PI * 6.5)) * 30, 1);
  near(p.heading, 90, 1e-6);
  assert.ok(p.x > 0 && p.y > 0);
});

test('pose: correctHeading uses the first yaw as reference', () => {
  const p = new PoseTracker();
  p.applyTurn(30);
  p.correctHeading(130);     // reference: yaw 130 = heading 30
  near(p.heading, 30);
  p.applyTurn(90);           // estimate 120, gyro says 115
  p.correctHeading(215);
  near(p.heading, 115);
  p.correctHeading(225, 0.5);
  near(p.heading, 120);
  p.correctHeading(-100);    // wraps: yaw -100 (= 260) -> heading 160
  near(p.heading, 160);
  p.correctHeading(NaN);
  near(p.heading, 160);
  p.reset();
  assert.equal(p.yawRef, null);
});

test('pose: attach follows successful turn/straight only and reproduces the bus clamp', async () => {
  const robot = {
    connected: true,
    dist: 300,
    fail: false,
    async turn() { if (this.fail) throw new Error('nope'); },
    async straight() {},
    async distance() { return this.dist; },
    async stop() {},
  };
  const bus = new CommandBus({ log() {} });
  bus.setRobot(robot);
  const p = new PoseTracker();
  const off = p.attach(bus);
  assert.equal(p.bus, bus);
  await bus.submit(makeCommand('turn', { deg: 90, wait: true }));
  await bus.submit(makeCommand('straight', { cm: 30, wait: true }));
  near(p.x, 30); near(p.y, 0); near(p.heading, 90);
  robot.fail = true;
  await bus.submit(makeCommand('turn', { deg: 90 }));
  near(p.heading, 90);
  await bus.submit(makeCommand('read', { sensor: 'distance' }));
  await bus.submit(makeCommand('move', { dir: 'forward' }));
  near(p.x, 30);
  // fresh reading of 40 cm: the bus drives at most 40 - 15 = 25
  robot.dist = 40;
  await bus.submit(makeCommand('read', { sensor: 'distance' }));
  await bus.submit(makeCommand('straight', { cm: 60, wait: true }));
  near(p.x, 55);
  await bus.submit(makeCommand('straight', { cm: -500 }));  // clamped to -100
  near(p.x, -45);
  off();
  assert.equal(p.bus, null);
  robot.fail = false;
  await bus.submit(makeCommand('turn', { deg: 45 }));
  near(p.heading, 90);
});
