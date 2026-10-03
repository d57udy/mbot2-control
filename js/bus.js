// Transport-agnostic command bus with a safety layer.
// Every producer (buttons, voice, later a relay or an LLM) submits the same
// JSON command; only the bus talks to the robot driver.
//
// Command schema v1:
//   { v: 1, id, ts, src: 'ui'|'voice'|'remote'|'agent', cmd, args, timeout_ms }
// cmd / args:
//   move     { dir: 'forward'|'backward', speed?, secs? }
//   spin     { dir: 'left'|'right', speed?, secs? }
//   turn     { deg, wait?, speed? }            (+ right, - left; gyro turn; wait = resolve when done)
//   straight { cm, wait?, speed? }             gyro-straight distance, negative = backward
//   drive    { left, right }             wheel RPM, forward-positive, continuous;
//                                        robot watchdog stops it 0.4 s after the
//                                        last drive command, so resend to keep going
//   stop     {}                          (jumps the queue)
//   read     { sensor: 'battery'|'distance'|'floor'|'yaw', colors? }  yaw: gyro heading in degrees
//   led      { r, g, b, id? }            back LED 1..5, or all
//   leds     { colors: [[r,g,b] x5] }
//   led_off / led_brightness { value } / led_effect { name }
//   eyes     { left, right }             ultrasonic eye LEDs 0..100
//   eye_led  { id: 1..8|'all', bri }     experimental per-LED form
//   eyes_effect { name }
//   floor_light { color }               quad RGB sensor fill light
//   display  { text }
//   beep     { freq?, secs? }
// Result: { id, ok, value?, error? }

export const LIMITS = {
  maxSpeed: 100,     // RPM for timed moves, mbot2 tops out near 200
  maxDriveRpm: 150,  // RPM for continuous joystick drive
  maxSecs: 2,        // longest single timed move
  maxTurnDeg: 360,
  maxStraightCm: 100,
  obstacleCm: 15,    // forward moves refused below this
  distanceFreshMs: 1500,
  defaultTimeoutMs: 2000,
};

// Names from research/04-leds-and-rgb-sensor.md; effects are whitelisted
// because they are interpolated into Python source. All 13 eye effects exist
// as cyberpi.ultrasonic2.<name>_effect in CyberPiOS 44.01.011 (research/06).
export const LED_EFFECTS = ['rainbow', 'spoondrift', 'meteor_blue', 'meteor_green', 'flash_red', 'flash_orange', 'firefly'];
export const EYE_EFFECTS = ['happy', 'new_happy', 'wink', 'naughty', 'aggrieved', 'raises_brow', 'look_left',
  'look_right', 'eye_left', 'eye_right', 'thinking', 'dizzy', 'standby'];
export const FLOOR_LIGHTS = ['off', 'white', 'red', 'green', 'blue'];

const byte = (v) => Math.min(255, Math.max(0, Number(v) || 0)) | 0;

let seq = 0;
export function makeCommand(cmd, args = {}, src = 'ui', timeout_ms = LIMITS.defaultTimeoutMs) {
  return { v: 1, id: `${src}-${Date.now().toString(36)}-${seq++}`, ts: Date.now(), src, cmd, args, timeout_ms };
}

const clamp = (v, lo, hi) => {
  const n = Number(v);
  if (!Number.isFinite(n)) throw new Error(`invalid number ${v}`);
  return Math.min(hi, Math.max(lo, n));
};

export class CommandBus {
  constructor({ log, onSensor }) {
    this.log = log;
    this.onSensor = onSensor;
    this.robot = null;
    this.settings = { speed: 50 };
    this.guard = true;
    this.stopGen = 0;
    this.lastDistance = { value: null, at: 0 };
    this.listeners = new Set();
    // Blocking LED animations occupy the robot's script executor; until this
    // time, sensor reads return the last value instead of queueing behind them.
    this.sensorQuietUntil = 0;
    this.effectsRunning = 0;
    this.lastReads = {};
  }

  sensorsQuiet() { return Date.now() < this.sensorQuietUntil; }

  // Runs a blocking animation with a quiet period; one at a time so taps do
  // not pile up minutes of animation on the robot.
  async runEffect(key, fn) {
    if (this.effectsRunning) throw new Error('busy: an LED effect is still running');
    const est = this.robot.effectEstimateMs?.(key) ?? 3000;
    this.effectsRunning++;
    this.sensorQuietUntil = Math.max(this.sensorQuietUntil, Date.now() + est);
    let res;
    try {
      res = await fn();
      return res;
    } finally {
      this.effectsRunning--;
      // finished on the robot (reply arrived): end the quiet period now
      if (!res?.running) this.sensorQuietUntil = Date.now();
    }
  }

  setRobot(robot) { this.robot = robot; }

  onCommand(fn) { this.listeners.add(fn); return () => this.listeners.delete(fn); }

  // Returns a result object; never throws.
  async submit(c) {
    const r = await this.execute(c).then(
      (value) => ({ id: c.id, ok: true, value }),
      (e) => ({ id: c.id, ok: false, error: e.message }),
    );
    for (const fn of this.listeners) fn(c, r);
    if (!r.ok && !/dropped by stop/.test(r.error)) this.log(`! ${c.cmd} (${c.src}): ${r.error}`);
    return r;
  }

  // A real stop bumps the generation, which cancels every command stamped
  // with an older one (multi-step AI tools and scans). soft: the drive
  // stream's end-of-motion stop, which must not cancel anything.
  stop(src = 'ui', { soft = false } = {}) {
    if (!soft) this.stopGen++;
    return this.submit(makeCommand('stop', {}, src));
  }

  // makeCommand that stamps the current generation; use one per task.
  stamped() {
    const gen = this.stopGen;
    return (cmd, args, src, timeout) => ({ ...makeCommand(cmd, args, src, timeout), gen });
  }

  async execute(c) {
    const robot = this.robot;
    if (!robot?.connected) throw new Error('robot not connected');
    if (c.v !== 1) throw new Error(`unsupported schema version ${c.v}`);
    if (c.cmd !== 'stop' && c.gen != null && c.gen !== this.stopGen) throw new Error('cancelled by stop');
    if (c.cmd !== 'stop' && Date.now() - c.ts > (c.timeout_ms ?? LIMITS.defaultTimeoutMs)) {
      throw new Error('expired');
    }
    const a = c.args ?? {};
    const speed = clamp(a.speed ?? this.settings.speed, 1, LIMITS.maxSpeed);
    const secs = clamp(a.secs ?? 0.5, 0.05, LIMITS.maxSecs);

    switch (c.cmd) {
      case 'stop':
        return robot.stop();
      case 'drive': {
        let l = Math.round(clamp(a.left ?? 0, -LIMITS.maxDriveRpm, LIMITS.maxDriveRpm));
        let r = Math.round(clamp(a.right ?? 0, -LIMITS.maxDriveRpm, LIMITS.maxDriveRpm));
        if (l + r > 0 && this.obstacleAhead()) { l = 0; r = 0; }
        return robot.drive(l, r);
      }
      case 'move':
        if (a.dir === 'forward') this.checkObstacle();
        return robot.move(a.dir === 'backward' ? 'backward' : 'forward', speed, secs);
      case 'spin':
        return robot.spin(a.dir === 'left' ? 'left' : 'right', speed, secs);
      case 'turn': {
        const deg = Math.round(clamp(a.deg ?? 90, -LIMITS.maxTurnDeg, LIMITS.maxTurnDeg));
        return robot.turn(deg, { wait: !!a.wait, speed });
      }
      case 'straight': {
        let cm = Math.round(clamp(a.cm ?? 0, -LIMITS.maxStraightCm, LIMITS.maxStraightCm));
        if (cm > 0) {
          this.checkObstacle();
          // never plan to drive closer than the stop distance to a fresh reading
          const { value, at } = this.lastDistance;
          if (this.guard && value != null && Date.now() - at < LIMITS.distanceFreshMs) {
            cm = Math.min(cm, Math.floor(value - LIMITS.obstacleCm));
            if (cm <= 0) throw new Error(`obstacle at ${value} cm`);
          }
        }
        return robot.straight(cm, { wait: !!a.wait, speed });
      }
      case 'read': {
        if (!['battery', 'distance', 'floor', 'yaw'].includes(a.sensor)) throw new Error(`unknown sensor ${a.sensor}`);
        if (this.sensorsQuiet()) {
          return a.sensor === 'distance' ? this.lastDistance.value : (this.lastReads[a.sensor] ?? null);
        }
        if (a.sensor === 'battery') {
          const v = await robot.battery();
          this.lastReads.battery = v;
          this.onSensor?.('battery', v);
          return v;
        }
        if (a.sensor === 'yaw') {
          if (!robot.yaw) throw new Error('yaw not supported');
          const v = Number(await robot.yaw());
          this.lastReads.yaw = v;
          return v;
        }
        if (a.sensor === 'distance') {
          const v = await robot.distance();
          this.lastDistance = { value: Number(v), at: Date.now() };
          this.onSensor?.('distance', v);
          return v;
        }
        const v = await robot.floor(!!a.colors);
        this.lastReads.floor = v;
        this.onSensor?.('floor', v);
        return v;
      }
      case 'led': {
        const id = a.id == null || a.id === 'all' ? null : clamp(a.id, 1, 5) | 0;
        return robot.led(byte(a.r), byte(a.g), byte(a.b), id);
      }
      case 'leds': {
        const colors = (a.colors ?? []).slice(0, 5).map(([r, g, b]) => [byte(r), byte(g), byte(b)]);
        if (colors.length !== 5) throw new Error('leds needs 5 colours');
        return robot.ledAll(colors);
      }
      case 'led_off':
        return robot.ledOff();
      case 'led_brightness':
        return robot.ledBrightness(clamp(a.value ?? 100, 0, 100) | 0);
      case 'led_effect':
        if (!LED_EFFECTS.includes(a.name)) throw new Error(`unknown led effect ${a.name}`);
        return this.runEffect(`led:${a.name}`, () => robot.ledEffect(a.name));
      case 'eyes':
        return robot.eyes(clamp(a.left ?? 0, 0, 100) | 0, clamp(a.right ?? 0, 0, 100) | 0);
      case 'eye_led':
        return robot.eyeLed(a.id === 'all' ? 'all' : clamp(a.id ?? 1, 1, 8) | 0, clamp(a.bri ?? 0, 0, 100) | 0);
      case 'eyes_effect':
        if (!EYE_EFFECTS.includes(a.name)) throw new Error(`unknown eye effect ${a.name}`);
        return this.runEffect(a.name, () => robot.eyesEffect(a.name));
      case 'floor_light':
        if (!FLOOR_LIGHTS.includes(a.color)) throw new Error(`unknown floor light ${a.color}`);
        return robot.floorLight(a.color);
      case 'display':
        return robot.display(String(a.text ?? '').slice(0, 40));
      case 'beep':
        return robot.beep(clamp(a.freq ?? 700, 100, 4000) | 0, clamp(a.secs ?? 0.2, 0.05, 1));
      default:
        throw new Error(`unknown command ${c.cmd}`);
    }
  }

  // Uses the most recent reading only if it is fresh; a stale reading never blocks.
  obstacleAhead() {
    const { value, at } = this.lastDistance;
    return this.guard && value != null && Date.now() - at < LIMITS.distanceFreshMs && value < LIMITS.obstacleCm;
  }

  checkObstacle() {
    if (this.obstacleAhead()) throw new Error(`obstacle at ${this.lastDistance.value} cm`);
  }
}
