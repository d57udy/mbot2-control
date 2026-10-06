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
const DEG_PER_CM = 360 / WHEEL_CM; // wheel angle per cm of wheel travel
const IMPACT_S = 0.25;      // how long a collision shows on the accelerometer
const G = 9.8;

// Furniture so scans have something to find. Boxes are axis-aligned.
const OBSTACLES = [
  { kind: 'box', x: 20, y: 0, w: 90, h: 45, label: 'sofa' },
  { kind: 'box', x: 228, y: 118, w: 34, h: 34, label: 'chair' },
  { kind: 'circle', x: 215, y: 55, r: 4, label: 'table leg' },
  { kind: 'circle', x: 70, y: 150, r: 15, label: 'pouf' },
];
// An obstacle with low: true blocks the robot but is below the ultrasonic beam.

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
  // encScale: per-wheel encoder scale [left, right], e.g. [1.03, 1] to emulate
  // a slipping or worn wheel for odometry fusion tests.
  // Hardware imperfections (all off by default):
  //   turnError     blocking turn() rotates deg * (1 + turnError)
  //   yawMode       'wrap' (-180..180) or 'unbounded' (keeps counting)
  //   yawInteger    yaw reported as an integer, like get_yaw() on 44.01.013
  //   encNoiseDeg   uniform noise on each encoder reading
  //   latencyMs     sensor query round trip for makeSimSampler (read halfway)
  //   stopCoastCm   distance a linear motion rolls on after stop()
  //   gyroDriftDegPerMin  gyro bias: the yaw reading drifts this much per
  //                 minute of simulated time, moving or not
  //   floorJumps    [{ atCm, deg }]: after atCm of driving the body turns by
  //                 deg (threshold, rug edge); the gyro sees it, the encoders do not
  constructor({ log, onStatus, onChange, timeScale = 1, obstacles = OBSTACLES, room = ROOM, encScale = [1, 1],
    turnError = 0, yawMode = 'wrap', yawInteger = false, encNoiseDeg = 0, latencyMs = 0, stopCoastCm = 0,
    gyroDriftDegPerMin = 0, floorJumps = [] }) {
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
    this.startHeading = this.state.heading;
    this.enc = [0, 0];      // cumulative wheel angles in degrees, forward-positive
    this.encScale = encScale;
    Object.assign(this, { turnError, yawMode, yawInteger, encNoiseDeg, latencyMs, stopCoastCm, gyroDriftDegPerMin });
    this.floorJumps = floorJumps.map((j) => ({ ...j, done: false }));
    this.odoCm = 0;          // distance driven, for floorJumps
    this.gyroT0 = this.simSecs();
    this.impact = null;     // { at: sim seconds, ms2 } of the last collision
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
      this.turnWheels(0, (m.vAng * dt) / n);
      const rad = (s.heading * Math.PI) / 180;
      const x = s.x + (Math.cos(rad) * m.vLin * dt) / n, y = s.y + (Math.sin(rad) * m.vLin * dt) / n;
      if (m.vLin && collides(x, y, this.room, this.obstacles) && !collides(s.x, s.y, this.room, this.obstacles)) {
        // stop at contact, but still finish the rotation part of this step;
        // the wheels stall and the accelerometer sees the impact
        s.heading += (m.vAng * dt * (n - i - 1)) / n;
        this.turnWheels(0, (m.vAng * dt * (n - i - 1)) / n);
        this.impact = { at: this.simSecs(), ms2: Math.max(8, Math.abs(m.vLin) / 100 / 0.02) };
        this.motion = null;
        break;
      }
      this.turnWheels((m.vLin * dt) / n, 0);
      this.odoCm += Math.abs((m.vLin * dt) / n);
      for (const j of this.floorJumps) if (!j.done && this.odoCm >= j.atCm) { j.done = true; s.heading += j.deg; }
      s.x = x; s.y = y;
    }
    this.onChange?.(s);
  }

  simSecs() { return (performance.now() * this.timeScale) / 1000; }

  // Wheel travel for cm forward and deg of clockwise rotation.
  turnWheels(cm, deg) {
    const arc = ((deg * Math.PI) / 180) * (TRACK_CM / 2);
    this.enc[0] += (cm + arc) * DEG_PER_CM * this.encScale[0];
    this.enc[1] += (cm - arc) * DEG_PER_CM * this.encScale[1];
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
    await this.go(0, (deg * (1 + this.turnError)) / secs, secs, `turn(${deg},${speed})`);
    if (wait) await this.settle(secs);
  }

  async straight(cm, { wait = false, speed = 50 } = {}) {
    const v = (speed / 60) * WHEEL_CM * Math.sign(cm);
    const secs = Math.abs(cm) / Math.abs(v || 1);
    await this.go(v, 0, secs, `straight(${cm},${speed})`);
    if (wait) await this.settle(secs);
  }

  // Differential drive; the timeout mirrors the robot-side watchdog. It is
  // 0.4 s of wall-clock time at any timeScale: the page resends drive frames
  // on a wall-clock rhythm, and a window in simulated time (5 ms at
  // timeScale 80) would stall the wheels between frames under test load.
  drive(left, right) {
    const vl = (left / 60) * WHEEL_CM, vr = (right / 60) * WHEEL_CM;
    const vAng = ((vl - vr) / TRACK_CM) * (180 / Math.PI);
    return this.go((vl + vr) / 2, vAng, left || right ? 0.4 * this.timeScale : 0);
  }

  // stopCoastCm: linear motion rolls on this far after a stop (field: legs
  // overshoot by 3 to 4 cm); rotation stops at once.
  stop() {
    if (this.timer) this.tick();
    const m = this.motion;
    if (m && m.vLin && this.stopCoastCm > 0) {
      // the whole stop distance within 0.1 s, so a reading 150 ms later sees it
      const secs = 0.1;
      this.motion = { vLin: Math.sign(m.vLin) * (this.stopCoastCm / secs), vAng: 0, until: performance.now() + (secs * 1000) / this.timeScale };
    } else this.motion = null;
    this.log('> sim stop');
    return Promise.resolve();
  }

  async battery() { return 87; }

  // Heading relative to the heading at construction, clockwise positive, -180..180.
  async yaw() { return this.yawNow(); }

  yawNow() {
    const drift = (this.gyroDriftDegPerMin * (this.simSecs() - this.gyroT0)) / 60;
    const raw = this.state.heading - this.startHeading + drift;
    const d = this.yawMode === 'unbounded' ? raw : ((raw % 360) + 540) % 360 - 180;
    return this.yawInteger ? Math.round(d) : Math.round(d * 10) / 10;
  }

  // Ultrasonic: nearest echo within a narrow cone from the sensor at the
  // front, against walls and obstacles; 300 means nothing in range.
  async distance() {
    if (this.timer) this.tick();
    return this.distanceNow();
  }

  distanceNow() {
    const { x, y, heading } = this.state;
    const rad = (heading * Math.PI) / 180;
    const ox = x + Math.cos(rad) * SENSOR_CM, oy = y + Math.sin(rad) * SENSOR_CM;
    const seen = this.obstacles.filter((o) => !o.low);
    const t = Math.min(...BEAM_DEG.map((d) => raycast(ox, oy, heading + d, this.room, seen)));
    return Math.round(Math.min(RANGE.max, Math.max(RANGE.min, t)) * 10) / 10;
  }

  // Everything the motion sampler reads in one poll (js/motion.js): wheel
  // angles in degrees (forward-positive), acceleration in m/s² with gravity
  // on -z and a spike along -x for IMPACT_S after a collision, yaw, shake.
  sensorSample() {
    if (this.timer) this.tick();
    const now = this.simSecs();
    const hit = this.impact && now - this.impact.at < IMPACT_S ? this.impact.ms2 : 0;
    return {
      t: now * 1000,
      distanceCm: this.distanceNow(),
      encL: Math.round((this.enc[0] + this.encNoiseDeg * (2 * Math.random() - 1)) * 10) / 10,
      encR: Math.round((this.enc[1] + this.encNoiseDeg * (2 * Math.random() - 1)) * 10) / 10,
      acc: { x: -hit, y: 0, z: -G },
      yaw: this.yawNow(),
      shake: hit ? Math.min(100, hit * 5) : 0,
    };
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
