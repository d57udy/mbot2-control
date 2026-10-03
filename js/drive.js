// Continuous drive controller shared by joystick, buttons and voice.
// Producers set a target (throttle/steer or wheel RPM); a 20 Hz loop ramps
// toward it and streams `drive` commands, which double as the watchdog
// heartbeat on the robot. Parameters from research/05-smooth-driving.md.

import { makeCommand } from './bus.js';

export const DRIVE = {
  hz: 20,
  deadzone: 0.08,
  expoThrottle: 0.35,
  expoSteer: 0.5,
  spinCap: 0.6,          // in-place spin limited to 60 % of max
  steerAtFull: 0.5,      // steering authority at full throttle
  accelRpmS: 300,
  decelRpmS: 600,
  minRpm: 6,             // below this send 0
  changeRpm: 3,          // resend when a wheel changes this much
  keepaliveMs: 120,      // resend while moving, below the 400 ms watchdog
  slowCm: 40,            // forward throttle scales down from here
  stopCm: 15,
  staleMs: 1000,
  staleCap: 0.3,         // forward cap when no fresh distance reading
};

const expo = (v, e) => (1 - e) * v + e * v * v * v;

// Joystick x/y in [-1, 1] to forward-positive wheel fractions.
export function mixArcade(x, y) {
  const len = Math.hypot(x, y);
  if (len < DRIVE.deadzone) return [0, 0];
  const k = (len - DRIVE.deadzone) / (1 - DRIVE.deadzone) / len;
  const t = expo(y * k, DRIVE.expoThrottle);
  let s = expo(x * k, DRIVE.expoSteer);
  const at = Math.abs(t);
  s *= 1 - (1 - DRIVE.steerAtFull) * at;
  if (at < 0.15) s *= DRIVE.spinCap + (1 - DRIVE.spinCap) * (at / 0.15);
  let l = t + s, r = t - s;
  const m = Math.max(1, Math.abs(l), Math.abs(r));
  l /= m; r /= m;
  return [l, r];
}

export class DriveStream {
  constructor({ bus, maxRpm, onWheels }) {
    this.bus = bus;
    this.maxRpm = maxRpm;   // () => number
    this.onWheels = onWheels;
    this.target = [0, 0];   // fractions -1..1
    this.current = [0, 0];  // RPM
    this.sent = null;
    this.sentAt = 0;
    this.timer = null;
    this.until = 0;
    this.stopTimer = null;
  }

  // Fractions of max speed, forward-positive. holdMs ends the drive after a time (voice).
  set(l, r, holdMs = 0) {
    this.target = [l, r];
    this.until = holdMs ? performance.now() + holdMs : 0;
    clearTimeout(this.stopTimer);
    if (!this.timer) {
      this.last = performance.now();
      this.timer = setInterval(() => this.tick(), 1000 / DRIVE.hz);
    }
  }

  release() { this.set(0, 0); }

  // Emergency: no ramp, no further frames. Caller sends the stop command.
  halt() {
    clearInterval(this.timer);
    clearTimeout(this.stopTimer);
    this.timer = null;
    this.target = [0, 0];
    this.current = [0, 0];
    this.sent = null;
    this.onWheels?.(0, 0);
  }

  // Forward speed factor from the latest ultrasonic reading.
  guardFactor() {
    const bus = this.bus;
    if (!bus.guard) return 1;
    const { value, at } = bus.lastDistance;
    if (value == null || Date.now() - at > DRIVE.staleMs) return DRIVE.staleCap;
    if (value <= DRIVE.stopCm) return 0;
    if (value >= DRIVE.slowCm) return 1;
    return (value - DRIVE.stopCm) / (DRIVE.slowCm - DRIVE.stopCm);
  }

  tick() {
    const now = performance.now();
    const dt = Math.min(0.2, (now - this.last) / 1000);
    this.last = now;
    if (this.until && now > this.until) { this.target = [0, 0]; this.until = 0; }

    const max = this.maxRpm();
    let [tl, tr] = this.target.map((f) => f * max);
    const fwd = (tl + tr) / 2;
    if (fwd > 0) {
      const g = this.guardFactor();
      const turn = (tl - tr) / 2;
      tl = fwd * g + turn;
      tr = fwd * g - turn;
    }

    this.current = this.current.map((c, i) => {
      const tgt = i === 0 ? tl : tr;
      const towardZero = Math.abs(tgt) < Math.abs(c) || (c !== 0 && Math.sign(tgt) !== Math.sign(c));
      const step = (towardZero ? DRIVE.decelRpmS : DRIVE.accelRpmS) * dt;
      return Math.abs(tgt - c) <= step ? tgt : c + Math.sign(tgt - c) * step;
    });

    const out = this.current.map((v) => (Math.abs(v) < DRIVE.minRpm ? 0 : Math.round(v)));
    const moving = out[0] !== 0 || out[1] !== 0;
    const changed = !this.sent || Math.abs(out[0] - this.sent[0]) >= DRIVE.changeRpm
      || Math.abs(out[1] - this.sent[1]) >= DRIVE.changeRpm
      || (!moving && (this.sent[0] !== 0 || this.sent[1] !== 0));
    const due = moving && now - this.sentAt >= DRIVE.keepaliveMs;

    if (changed || due) {
      this.sent = out;
      this.sentAt = now;
      this.onWheels?.(out[0], out[1]);
      this.bus.submit(makeCommand('drive', { left: out[0], right: out[1] }, 'ui', 500)).then((r) => {
        if (!r.ok && r.error === 'robot not connected') this.halt();
      });
    }

    // Idle: stop the loop and send a final EM_stop as a belt-and-braces stop.
    if (!moving && this.target[0] === 0 && this.target[1] === 0) {
      clearInterval(this.timer);
      this.timer = null;
      this.stopTimer = setTimeout(() => this.bus.stop('ui', { soft: true }), 300);
    }
  }
}
