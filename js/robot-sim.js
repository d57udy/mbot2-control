// Simulated robot with the same interface as BleRobot. Lets the page, voice
// parser and later AI agents be tested without hardware.
// Room is 300 x 200 cm (x right, y down, heading 0 = +x, clockwise positive);
// speed is treated as RPM on a 6.5 cm wheel.

const WHEEL_CM = Math.PI * 6.5;
const TRACK_CM = 12;
const ROOM = { w: 300, h: 200 };
const SIM_COLORS = ['white', 'black', 'black', 'white'];
const RADIUS_CM = 9;        // collision circle around the robot centre
const SENSOR_CM = 6;        // ultrasonic sits this far ahead of the centre
const RANGE = { min: 3, max: 300 };
const BEAM_DEG = [-8, -4, 0, 4, 8]; // the ultrasonic cone, nearest echo wins

// Furniture so scans have something to find. Boxes are axis-aligned.
const OBSTACLES = [
  { kind: 'box', x: 20, y: 0, w: 90, h: 45, label: 'sofa' },
  { kind: 'box', x: 228, y: 118, w: 34, h: 34, label: 'chair' },
  { kind: 'circle', x: 215, y: 55, r: 4, label: 'table leg' },
  { kind: 'circle', x: 70, y: 150, r: 15, label: 'pouf' },
];

// Distance along a unit ray to the first hit, or Infinity.
function rayBox(ox, oy, dx, dy, b) {
  let t0 = -Infinity, t1 = Infinity;
  for (const [o, d, lo, hi] of [[ox, dx, b.x, b.x + b.w], [oy, dy, b.y, b.y + b.h]]) {
    if (Math.abs(d) < 1e-9) {
      if (o < lo || o > hi) return Infinity;
    } else {
      const a = (lo - o) / d, c = (hi - o) / d;
      t0 = Math.max(t0, Math.min(a, c));
      t1 = Math.min(t1, Math.max(a, c));
    }
  }
  if (t1 < Math.max(t0, 0)) return Infinity;
  return Math.max(t0, 0);
}

function rayCircle(ox, oy, dx, dy, c) {
  const fx = ox - c.x, fy = oy - c.y;
  const b = fx * dx + fy * dy;
  const disc = b * b - (fx * fx + fy * fy - c.r * c.r);
  if (disc < 0) return Infinity;
  const t = -b - Math.sqrt(disc);
  if (t >= 0) return t;
  return -b + Math.sqrt(disc) >= 0 ? 0 : Infinity;
}

function rayWalls(ox, oy, dx, dy, room) {
  const tx = dx > 0 ? (room.w - ox) / dx : dx < 0 ? -ox / dx : Infinity;
  const ty = dy > 0 ? (room.h - oy) / dy : dy < 0 ? -oy / dy : Infinity;
  return Math.max(0, Math.min(tx, ty));
}

export function raycast(ox, oy, headingDeg, room = ROOM, obstacles = OBSTACLES) {
  const rad = (headingDeg * Math.PI) / 180;
  const dx = Math.cos(rad), dy = Math.sin(rad);
  let t = rayWalls(ox, oy, dx, dy, room);
  for (const o of obstacles) t = Math.min(t, o.kind === 'circle' ? rayCircle(ox, oy, dx, dy, o) : rayBox(ox, oy, dx, dy, o));
  return t;
}

// True if a robot centred at (x, y) overlaps a wall or an obstacle.
export function collides(x, y, room = ROOM, obstacles = OBSTACLES, r = RADIUS_CM) {
  if (x < r || y < r || x > room.w - r || y > room.h - r) return true;
  return obstacles.some((o) => {
    if (o.kind === 'circle') return Math.hypot(x - o.x, y - o.y) < o.r + r;
    const nx = Math.min(o.x + o.w, Math.max(o.x, x)), ny = Math.min(o.y + o.h, Math.max(o.y, y));
    return Math.hypot(x - nx, y - ny) < r;
  });
}

export class SimRobot {
  // timeScale > 1 runs motion faster than real time (tests).
  constructor({ log, onStatus, onChange, timeScale = 1, obstacles = OBSTACLES, room = ROOM }) {
    this.kind = 'sim';
    this.timeScale = timeScale;
    this.obstacles = obstacles;
    this.room = room;
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

  // Integrates up to now but never past the end of the motion, so turns land
  // on the exact angle. Linear motion is sub-stepped and stops at contact.
  tick() {
    const now = performance.now();
    const m = this.motion;
    const end = m ? Math.min(now, m.until) : now;
    const dt = (Math.max(0, end - this.last) / 1000) * this.timeScale;
    this.last = now;
    if (!m) return;
    if (now >= m.until) this.motion = null;
    const s = this.state;
    const n = Math.max(1, Math.ceil(Math.abs(m.vLin * dt)));
    for (let i = 0; i < n; i++) {
      s.heading += (m.vAng * dt) / n;
      const rad = (s.heading * Math.PI) / 180;
      const x = s.x + (Math.cos(rad) * m.vLin * dt) / n, y = s.y + (Math.sin(rad) * m.vLin * dt) / n;
      if (m.vLin && collides(x, y, this.room, this.obstacles) && !collides(s.x, s.y, this.room, this.obstacles)) {
        // stop at contact, but still finish the rotation part of this step
        s.heading += (m.vAng * dt * (n - i - 1)) / n;
        this.motion = null;
        break;
      }
      s.x = x; s.y = y;
    }
    this.onChange?.(s);
  }

  go(vLin, vAng, secs, label) {
    if (label) this.log(`> sim ${label}`);
    if (this.timer) this.tick(); // settle the previous motion first
    this.motion = { vLin, vAng, until: performance.now() + (secs * 1000) / this.timeScale };
    return Promise.resolve();
  }

  async settle(secs) {
    await new Promise((r) => setTimeout(r, (secs * 1000) / this.timeScale + 60));
    this.tick();
  }

  move(dir, speed, secs) {
    const v = (speed / 60) * WHEEL_CM * (dir === 'backward' ? -1 : 1);
    return this.go(v, 0, secs, `${dir}(${speed},${secs})`);
  }

  spin(dir, speed, secs) {
    return this.go(0, (dir === 'left' ? -1 : 1) * speed * 2, secs, `spin ${dir}(${speed},${secs})`);
  }

  async turn(deg, { wait = false, speed = 50 } = {}) {
    const secs = (Math.abs(deg) / 180) * (50 / speed);
    await this.go(0, (Math.sign(deg) * Math.abs(deg)) / secs, secs, `turn(${deg},${speed})`);
    if (wait) await this.settle(secs);
  }

  async straight(cm, { wait = false, speed = 50 } = {}) {
    const v = (speed / 60) * WHEEL_CM * Math.sign(cm);
    const secs = Math.abs(cm) / Math.abs(v || 1);
    await this.go(v, 0, secs, `straight(${cm},${speed})`);
    if (wait) await this.settle(secs);
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

  // Ultrasonic: nearest echo within a narrow cone from the sensor at the
  // front, against walls and obstacles; 300 means nothing in range.
  async distance() {
    if (this.timer) this.tick();
    const { x, y, heading } = this.state;
    const rad = (heading * Math.PI) / 180;
    const ox = x + Math.cos(rad) * SENSOR_CM, oy = y + Math.sin(rad) * SENSOR_CM;
    const t = Math.min(...BEAM_DEG.map((d) => raycast(ox, oy, heading + d, this.room, this.obstacles)));
    return Math.round(Math.min(RANGE.max, Math.max(RANGE.min, t)) * 10) / 10;
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
export const SIM_OBSTACLES = OBSTACLES;
export const SIM_ROBOT = { radiusCm: RADIUS_CM, sensorCm: SENSOR_CM };
