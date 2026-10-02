// Simulated robot with the same interface as BleRobot. Lets the page, voice
// parser and later AI agents be tested without hardware.
// Room is 300 x 200 cm; speed is treated as RPM on a 6.5 cm wheel.

const WHEEL_CM = Math.PI * 6.5;
const TRACK_CM = 12;
const ROOM = { w: 300, h: 200 };
const SIM_COLORS = ['white', 'black', 'black', 'white'];

export class SimRobot {
  constructor({ log, onStatus, onChange }) {
    this.kind = 'sim';
    this.log = log;
    this.onStatus = onStatus;
    this.onChange = onChange;
    this.state = {
      x: 150, y: 100, heading: -90, leds: Array.from({ length: 5 }, () => [0, 0, 0]),
      eyes: [0, 0], floorLight: 'off', label: '',
    };
    this.motion = null; // {vLin cm/s, vAng deg/s, until}
    this.connected = false;
    this.watchdog = true;
    this.lastDrive = 0;
    this.timer = null;
  }

  async connect() {
    this.connected = true;
    this.timer = setInterval(() => this.tick(), 50);
    this.last = performance.now();
    this.onStatus('connected');
    this.log('Simulator connected (watchdog emulated)');
  }

  async disconnect() {
    clearInterval(this.timer);
    this.connected = false;
    this.motion = null;
    this.onStatus('disconnected');
  }

  tick() {
    const now = performance.now();
    const dt = (now - this.last) / 1000;
    this.last = now;
    const m = this.motion;
    if (!m) return;
    if (now >= m.until) this.motion = null;
    const s = this.state;
    s.heading += m.vAng * dt;
    const rad = (s.heading * Math.PI) / 180;
    s.x = Math.min(ROOM.w - 8, Math.max(8, s.x + Math.cos(rad) * m.vLin * dt));
    s.y = Math.min(ROOM.h - 8, Math.max(8, s.y + Math.sin(rad) * m.vLin * dt));
    this.onChange?.(s);
  }

  go(vLin, vAng, secs, label) {
    if (label) this.log(`> sim ${label}`);
    this.motion = { vLin, vAng, until: performance.now() + secs * 1000 };
    return Promise.resolve();
  }

  move(dir, speed, secs) {
    const v = (speed / 60) * WHEEL_CM * (dir === 'backward' ? -1 : 1);
    return this.go(v, 0, secs, `${dir}(${speed},${secs})`);
  }

  spin(dir, speed, secs) {
    return this.go(0, (dir === 'left' ? -1 : 1) * speed * 2, secs, `spin ${dir}(${speed},${secs})`);
  }

  turn(deg) {
    return this.go(0, Math.sign(deg) * 180, Math.abs(deg) / 180, `turn(${deg})`);
  }

  // Differential drive; the 0.4 s timeout mirrors the robot-side watchdog.
  drive(left, right) {
    const vl = (left / 60) * WHEEL_CM, vr = (right / 60) * WHEEL_CM;
    const vAng = ((vl - vr) / TRACK_CM) * (180 / Math.PI);
    return this.go((vl + vr) / 2, vAng, left || right ? 0.4 : 0);
  }

  stop() {
    this.motion = null;
    this.log('> sim stop');
    return Promise.resolve();
  }

  async battery() { return 87; }

  // Distance from the front of the robot to the wall it faces.
  async distance() {
    const { x, y, heading } = this.state;
    const rad = (heading * Math.PI) / 180;
    const dx = Math.cos(rad), dy = Math.sin(rad);
    const tx = dx > 0 ? (ROOM.w - x) / dx : dx < 0 ? -x / dx : Infinity;
    const ty = dy > 0 ? (ROOM.h - y) / dy : dy < 0 ? -y / dy : Infinity;
    return Math.round(Math.min(300, tx, ty) * 10) / 10;
  }

  changed() { this.onChange?.(this.state); return Promise.resolve(); }

  led(r, g, b, id) {
    this.state.leds = this.state.leds.map((c, i) => (!id || id === i + 1 ? [r, g, b] : c));
    return this.changed();
  }

  ledAll(colors) { this.state.leds = colors.map((c) => [...c]); return this.changed(); }

  ledOff() { return this.led(0, 0, 0); }

  ledBrightness() { return Promise.resolve(); }

  ledEffect(name) { this.log(`> sim led effect ${name}`); return Promise.resolve(); }

  eyes(left, right) { this.state.eyes = [left, right]; return this.changed(); }

  eyeLed(id, bri) { this.log(`> sim eye led ${id} = ${bri}`); return Promise.resolve(); }

  eyesEffect(name) { this.log(`> sim eyes ${name}`); return Promise.resolve(); }

  floorLight(color) { this.state.floorLight = color; return this.changed(); }

  async floor(withColors) {
    const gray = [82, 12, 15, 80];
    return {
      line: 0b0110,
      offset: -4,
      gray,
      names: withColors ? [...SIM_COLORS] : null,
      rgb: withColors ? gray.map((g) => [g * 2.5 | 0, g * 2.5 | 0, g * 2.4 | 0]) : null,
    };
  }

  async diagnose() { return ['sim', [], [], [], []]; }

  async raw(script) { this.log(`> sim raw ${script}`); return null; }
}

export const SIM_ROOM = ROOM;
