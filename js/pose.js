// Dead-reckoning pose in the map frame: x right, y forward of the start,
// heading in degrees clockwise from +y (same sign as `turn`), -180..180.

import { LIMITS } from './bus.js';

import { ODOMETRY } from './motion.js';

const wheelCm = () => Math.PI * ODOMETRY.wheelDiameterCm; // calibrated in the app (Strecken-Test)
const TRACK_CM = 12;
const TRAIL_MAX = 500;

export function normDeg(a) {
  const r = ((((a + 180) % 360) + 360) % 360) - 180;
  return r === -180 ? 180 : r;
}

const rad = (d) => (d * Math.PI) / 180;

export class PoseTracker {
  constructor({ x = 0, y = 0, heading = 0 } = {}) {
    this.bus = null; // set by attach(); Navigator checks it to avoid applying moves twice
    this.trail = [];
    this.reset({ x, y, heading });
  }

  get pose() { return { x: this.x, y: this.y, heading: this.heading }; }

  reset({ x = 0, y = 0, heading = 0 } = {}) {
    this.x = x;
    this.y = y;
    this.heading = normDeg(heading);
    this.yawRef = null; // yaw reading that corresponds to heading 0, set on the first correction
    this.trail = [{ x, y }];
  }

  applyTurn(deg) {
    this.heading = normDeg(this.heading + (Number(deg) || 0));
  }

  applyStraight(cm) {
    const d = Number(cm) || 0;
    this.x += d * Math.sin(rad(this.heading));
    this.y += d * Math.cos(rad(this.heading));
    this.mark();
  }

  // Differential drive, wheel RPM forward-positive; left faster turns right (clockwise).
  applyDrive(leftRpm, rightRpm, dtSec) {
    const vl = (leftRpm / 60) * wheelCm(), vr = (rightRpm / 60) * wheelCm();
    const v = (vl + vr) / 2;
    const dh = ((vl - vr) / TRACK_CM) * (180 / Math.PI) * dtSec;
    const mid = rad(this.heading + dh / 2); // midpoint heading for the arc
    this.x += v * dtSec * Math.sin(mid);
    this.y += v * dtSec * Math.cos(mid);
    this.heading = normDeg(this.heading + dh);
    this.mark();
  }

  // Gyro yaw (degrees, clockwise positive). The first call after reset only
  // records the reference; later calls replace (weight 1) or blend the heading.
  correctHeading(yawDeg, weight = 1) {
    const yaw = Number(yawDeg);
    if (!Number.isFinite(yaw)) return;
    if (this.yawRef == null) { this.yawRef = normDeg(yaw - this.heading); return; }
    const err = normDeg(normDeg(yaw - this.yawRef) - this.heading);
    this.heading = normDeg(this.heading + err * weight);
  }

  mark() {
    const last = this.trail.at(-1);
    if (last && Math.hypot(last.x - this.x, last.y - this.y) < 1) return;
    this.trail.push({ x: this.x, y: this.y });
    if (this.trail.length > TRAIL_MAX) this.trail.splice(0, this.trail.length - TRAIL_MAX);
  }

  // Successful turn/straight commands update the pose. The bus may shorten a
  // forward straight to stay obstacleCm away from a fresh distance reading;
  // that clamp is reproduced here from bus.lastDistance (the result carries no
  // driven distance). A move cut short by a stop or a collision is not seen.
  attach(bus) {
    this.bus = bus;
    const off = bus.onCommand((c, r) => {
      if (!r?.ok) return;
      const a = c.args ?? {};
      if (c.cmd === 'turn') {
        const deg = Math.round(Math.min(LIMITS.maxTurnDeg, Math.max(-LIMITS.maxTurnDeg, Number(a.deg ?? 90))));
        if (Number.isFinite(deg)) this.applyTurn(deg);
      } else if (c.cmd === 'straight') {
        let cm = Math.round(Math.min(LIMITS.maxStraightCm, Math.max(-LIMITS.maxStraightCm, Number(a.cm ?? 0))));
        if (!Number.isFinite(cm)) return;
        const { value, at } = bus.lastDistance ?? {};
        if (cm > 0 && bus.guard && value != null && at <= (c.ts ?? 0) + 50 && (c.ts ?? 0) - at < LIMITS.distanceFreshMs) {
          cm = Math.max(0, Math.min(cm, Math.floor(value - LIMITS.obstacleCm)));
        }
        this.applyStraight(cm);
      }
    });
    return () => { off(); if (this.bus === bus) this.bus = null; };
  }
}
