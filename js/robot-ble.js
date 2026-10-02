// Robot driver over Web Bluetooth. Owns the only GATT connection; every
// write goes through one queue because Chrome rejects overlapping GATT ops.
import {
  UUID, ONLINE_FRAME, MODE_NO_REPLY, MODE_REPLY, buildScriptFrame, F3Parser, toHex,
} from './protocol.js';

const sleep = (ms) => new Promise((r) => setTimeout(r, ms));
const py = (s) => JSON.stringify(String(s)); // JSON string literal is valid Python
const MODE_IMMEDIATE = 3; // runs outside the robot's script queue (research/05)

// Robot-side dead-man switch: stops the motors if no drive frame arrives for
// 400 ms. State lives on the mbot2 module so it persists between frames, and
// re-sending it does not start a second thread. See research/05-smooth-driving.md.
const WATCHDOG_SRC = [
  'import _thread,time,mbot2 as M',
  'M._wto=400',
  'M._t=time.ticks_ms();M._mv=0',
  'def _d(a,b):',
  ' M._t=time.ticks_ms()',
  ' M._mv=1 if (a or b) else 0',
  ' M.drive_speed(a,b)',
  'def _wd():',
  ' while M._wr:',
  '  try:',
  '   if M._mv and time.ticks_diff(time.ticks_ms(),M._t)>M._wto:',
  '    M._mv=0;M.EM_stop()',
  '  except Exception:',
  '   pass',
  '  time.sleep_ms(50)',
  'M._d=_d',
  "if not getattr(M,'_wr',0):",
  ' M._wr=1',
  ' _thread.start_new_thread(_wd,())',
].join('\n');

export class BleRobot {
  constructor({ log, onStatus, chunkSize = 20, chunkDelayMs = 8, debugAllDevices = false }) {
    this.kind = 'ble';
    this.log = log;
    this.onStatus = onStatus;
    this.chunkSize = chunkSize;
    this.chunkDelayMs = chunkDelayMs;
    this.debugAllDevices = debugAllDevices;
    this.device = null;
    this.writeChar = null;
    this.parser = new F3Parser();
    this.pending = new Map(); // idx -> {resolve, reject, timer}
    this.idx = 1;
    this.queue = [];
    this.pumping = false;
    this.connected = false;
    this.watchdog = false;
    this.ignoreIdx = new Set();
    // Wheel mapping: the motors are mirrored, so forward is EM1 +, EM2 -.
    this.wheels = { swap: false, mirrored: true };
  }

  // --- connection -------------------------------------------------------

  async pickDevice() {
    const opts = this.debugAllDevices
      ? { acceptAllDevices: true, optionalServices: UUID.serviceCandidates }
      : {
        filters: [{ namePrefix: 'Makeblock' }, { namePrefix: 'CyberPi' }, { namePrefix: 'mBot' }],
        optionalServices: UUID.serviceCandidates,
      };
    this.device = await navigator.bluetooth.requestDevice(opts);
    this.device.addEventListener('gattserverdisconnected', () => this.handleDisconnect());
    this.log(`Selected device "${this.device.name}"`);
  }

  // Reuses the same BluetoothDevice so reconnecting needs no chooser.
  async connect() {
    if (!this.device) await this.pickDevice();
    this.onStatus('connecting');
    const server = await this.device.gatt.connect();

    const services = await server.getPrimaryServices().catch(() => []);
    this.log(`Visible services: ${services.map((s) => s.uuid).join(', ') || 'none'}`);

    let notifyChar = null;
    for (const svc of services) {
      try {
        this.writeChar = await svc.getCharacteristic(UUID.write);
        notifyChar = await svc.getCharacteristic(UUID.notify);
        this.log(`Using service ${svc.uuid}`);
        break;
      } catch { /* try next */ }
    }
    if (!notifyChar) throw new Error('ffe2/ffe3 not found. Turn on "show all devices" and check the log.');

    await notifyChar.startNotifications();
    notifyChar.addEventListener('characteristicvaluechanged', (e) => {
      const bytes = new Uint8Array(e.target.value.buffer);
      for (const frame of this.parser.feed(bytes)) this.resolveReply(frame);
    });

    this.connected = true;
    await this.handshake();
    await this.installWatchdog();
    this.onStatus('connected');
  }

  async handshake() {
    this.onStatus('handshake');
    await this.enqueue(() => this.writeRaw(ONLINE_FRAME));
    await sleep(500);
    this.idx = 1;
    for (let i = 0; i < 12; i++) {
      try {
        const v = await this.query('cyberpi.get_bri()', 500);
        this.log(`Live mode ready (brightness ${v}, try ${i + 1})`);
        return;
      } catch { /* retry */ }
    }
    this.log('No handshake reply. Continuing anyway; the robot often still accepts commands.');
  }

  async installWatchdog() {
    this.watchdog = false;
    try {
      await this.query(`exec(${py(WATCHDOG_SRC)})`, 3000);
      const [wr, hasD] = await this.query("[mbot2._wr,hasattr(mbot2,'_d')]", 1500);
      this.watchdog = wr === 1 && hasD === true;
    } catch (e) {
      this.log(`Watchdog install failed: ${e.message}`);
    }
    this.log(this.watchdog
      ? 'Watchdog active: motors stop 0.4 s after the last drive command.'
      : 'No watchdog: if Bluetooth drops during joystick driving the robot may keep going.');
    return this.watchdog;
  }

  async disconnect() {
    try { await this.stop(); } catch { /* ignore */ }
    this.device?.gatt?.disconnect();
  }

  handleDisconnect() {
    this.connected = false;
    this.writeChar = null;
    this.queue = [];
    for (const p of this.pending.values()) { clearTimeout(p.timer); p.reject(new Error('disconnected')); }
    this.pending.clear();
    this.onStatus('disconnected');
  }

  // --- low level --------------------------------------------------------

  async writeRaw(frame) {
    if (!this.writeChar) throw new Error('not connected');
    for (let i = 0; i < frame.length; i += this.chunkSize) {
      await this.writeChar.writeValueWithoutResponse(frame.slice(i, i + this.chunkSize));
      if (this.chunkDelayMs) await sleep(this.chunkDelayMs);
    }
  }

  // urgent tasks jump the queue and drop anything not yet sent (e.g. drive bursts).
  // key: a pending task with the same key is replaced (latest drive frame wins).
  enqueue(task, { urgent = false, key = null } = {}) {
    return new Promise((resolve, reject) => {
      const item = { task, resolve, reject, key };
      const same = key && this.queue.findIndex((q) => q.key === key);
      if (key && same >= 0) {
        this.queue[same].resolve(undefined);
        this.queue[same] = item;
      } else if (urgent) {
        for (const q of this.queue) q.reject(new Error('dropped by stop'));
        this.queue = [item];
      } else {
        this.queue.push(item);
      }
      this.pump();
    });
  }

  async pump() {
    if (this.pumping) return;
    this.pumping = true;
    while (this.queue.length) {
      const { task, resolve, reject } = this.queue.shift();
      try { resolve(await task()); } catch (e) { reject(e); }
    }
    this.pumping = false;
  }

  nextIdx() {
    const i = this.idx;
    this.idx = (this.idx % 0xfffe) + 1;
    return i;
  }

  run(script, opts = {}) {
    return this.enqueue(() => {
      const frame = buildScriptFrame(script, this.nextIdx(), MODE_NO_REPLY);
      if (!opts.quiet) this.log(`> ${script}`, toHex(frame));
      return this.writeRaw(frame);
    }, opts);
  }

  query(script, timeoutMs = 1500) {
    return new Promise((resolve, reject) => {
      this.enqueue(async () => {
        const idx = this.nextIdx();
        const timer = setTimeout(() => {
          this.pending.delete(idx);
          reject(new Error(`timeout: ${script}`));
        }, timeoutMs);
        this.pending.set(idx, { resolve, reject, timer });
        await this.writeRaw(buildScriptFrame(script, idx, MODE_REPLY));
      }).catch(reject);
    });
  }

  resolveReply({ idx, value, raw }) {
    const p = this.pending.get(idx);
    if (!p && this.ignoreIdx.delete(idx)) return;
    if (!p) { this.log(`< unmatched reply idx ${idx}: ${raw}`); return; }
    clearTimeout(p.timer);
    this.pending.delete(idx);
    p.resolve(value);
  }

  // --- robot primitives (all motion is timed so the robot stops by itself) ---

  move(dir, speed, secs) {
    const fn = dir === 'backward' ? 'backward' : 'forward';
    return this.run(`mbot2.${fn}(${speed},${secs})`);
  }

  spin(dir, speed, secs) {
    const fn = dir === 'left' ? 'turn_left' : 'turn_right';
    return this.run(`mbot2.${fn}(${speed},${secs})`);
  }

  turn(deg) { return this.run(`mbot2.turn(${deg})`); }

  // Immediate mode first so it skips anything the robot is still executing,
  // then a normal copy in case this firmware ignores mode 3 for scripts.
  stop() {
    return this.enqueue(async () => {
      const idx = this.nextIdx();
      this.ignoreIdx.add(idx);
      await this.writeRaw(buildScriptFrame('mbot2.EM_stop()', idx, MODE_IMMEDIATE));
      await this.writeRaw(buildScriptFrame('mbot2.EM_stop()', this.nextIdx(), MODE_NO_REPLY));
      this.log('> mbot2.EM_stop() (immediate + queued)');
    }, { urgent: true });
  }

  // Continuous wheel speeds in RPM, forward-positive. Each call is also the
  // watchdog heartbeat. Only the newest pending frame is kept.
  drive(left, right) {
    let [a, b] = this.wheels.swap ? [right, left] : [left, right];
    if (this.wheels.mirrored) b = -b;
    const fn = this.watchdog ? 'mbot2._d' : 'mbot2.drive_speed';
    return this.run(`${fn}(${a},${b})`, { key: 'drive', quiet: true });
  }

  battery() { return this.query('cyberpi.get_battery()'); }

  distance() { return this.query('cyberpi.ultrasonic2.get(1)'); }

  // --- lights and floor sensor (research/04-leds-and-rgb-sensor.md) ---

  // id 1..5 for one back LED, omitted for all five.
  led(r, g, b, id) {
    return this.run(id ? `cyberpi.led.on(${r},${g},${b},id=${id})` : `cyberpi.led.on(${r},${g},${b})`);
  }

  // Five colours in one frame: [[r,g,b], ...] for LEDs 1..5.
  ledAll(colors) {
    return this.run(`[${colors.map(([r, g, b], i) => `cyberpi.led.on(${r},${g},${b},id=${i + 1})`).join(',')}]`);
  }

  ledOff() { return this.run('cyberpi.led.off("all")'); }

  ledBrightness(v) { return this.run(`cyberpi.led.set_bri(${v})`); }

  ledEffect(name) { return this.run(`cyberpi.led.play(${py(name)})`); }

  eyes(left, right) {
    return this.run(`[cyberpi.ultrasonic2.set_both_led_bri(${left},${right}),cyberpi.ultrasonic2.led_show()]`);
  }

  eyeLed(id, bri) { return this.run(`cyberpi.ultrasonic2.set_bri(${bri},${id === 'all' ? '"all"' : id},1)`); }

  eyesEffect(name) { return this.run(`cyberpi.ultrasonic2.${name}_effect()`); }

  floorLight(color) {
    return this.run(color === 'off' ? 'cyberpi.quad_rgb_sensor.off_led(1)' : `cyberpi.quad_rgb_sensor.set_led(${py(color)},1)`);
  }

  // Probes in physical order L2 L1 R1 R2 (sensor ids 4 3 2 1). Colour reads
  // make the sensor switch its fill light, so they are optional.
  async floor(withColors) {
    const q = 'cyberpi.quad_rgb_sensor';
    const parts = ['q.get_line_sta(1)', 'q.get_offset_track(1)', '[q.get_gray(i) for i in (4,3,2,1)]'];
    if (withColors) {
      parts.push('[q.get_color_sta(i) for i in (4,3,2,1)]',
        '[[q.get_red(i),q.get_green(i),q.get_blue(i)] for i in (4,3,2,1)]');
    }
    const v = await this.query(`(lambda q:[${parts.join(',')}])(${q})`, 2500);
    if (!Array.isArray(v)) throw new Error(`unexpected floor reply: ${JSON.stringify(v)}`);
    const [line, offset, gray, names, rgb] = v;
    return { line, offset, gray, names: names ?? null, rgb: rgb ?? null };
  }

  diagnose() {
    return this.query('[cyberpi.get_firmware_version(),dir(cyberpi.led),dir(cyberpi.ultrasonic2),dir(cyberpi.quad_rgb_sensor),dir(mbot2)]', 5000);
  }

  display(text) { return this.run(`cyberpi.display.show_label(${py(text)},24,"center")`); }

  beep(freq = 700, secs = 0.2) { return this.run(`cyberpi.audio.play_tone(${freq},${secs})`); }

  raw(script) { return this.query(script, 3000); }
}
