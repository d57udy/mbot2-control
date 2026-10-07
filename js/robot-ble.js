// Robot driver over Web Bluetooth. Owns the only GATT connection; every
// write goes through one queue because Chrome rejects overlapping GATT ops.
import {
  UUID, ONLINE_FRAME, MODE_NO_REPLY, MODE_REPLY, buildScriptFrame, F3Parser, toHex,
} from './protocol.js';

const eyeList = (l, r) => `[${l},${l},${l},${l},${r},${r},${r},${r}]`;
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

// Eye LED helpers (research/06-eye-leds.md). In CyberPiOS 44.01.011,
// cyberpi.ultrasonic2 is the module mbuild_modules/led_ultrasonic_sensor.py and
// eight of the thirteen *_effect functions take a required brightness argument,
// so calling them with no arguments raises TypeError. _fx tries both forms and
// returns the run time in ms or an error string, so failures become visible
// even if Live Mode does not report exceptions. _eyes prefers the documented
// 8-value led_show and falls back to the form DrorSh saw on 44.01.013.
const EYE_HELPER_SRC = [
  'import time,cyberpi as C,mbot2 as M',
  'def _fx(n,b=100):',
  " f=getattr(C.ultrasonic2,n+'_effect',None)",
  " if f is None:return 'missing'",
  ' t=time.ticks_ms()',
  ' try:',
  '  try:f()',
  '  except TypeError:f(b)',
  ' except Exception as e:',
  "  return 'error: '+repr(e)",
  ' return time.ticks_diff(time.ticks_ms(),t)',
  'def _eyes(l,r):',
  ' u=C.ultrasonic2',
  ' try:',
  '  u.led_show([l,l,l,l,r,r,r,r],1)',
  "  return 'list'",
  ' except Exception:',
  '  pass',
  ' u.set_both_led_bri(l,r)',
  ' try:u.led_show()',
  ' except Exception:pass',
  " return 'both'",
  'M._fx=_fx',
  'M._eyes=_eyes',
].join('\n');

// Signatures read from the 44.01.011 firmware image: these take (led_bri, index=1),
// the others take (index=1). Only used when the helper could not be installed.
export const EYE_EFFECT_NEEDS_BRI = new Set(['happy', 'wink', 'naughty', 'aggrieved', 'look_left', 'look_right',
  'eye_left', 'eye_right']);
// Measured on firmware 44.01.013: 200-byte scripts work, 300-byte scripts are
// silently dropped. Longer code goes through execLong; long replies through queryLong.
export const MAX_SCRIPT = 200;
export const FIRMWARE_TURN_SIGN = 1; // mbot2.turn(+deg) turns clockwise (as documented); see turn()
const byteLen = (s) => new TextEncoder().encode(s).length;
const EFFECT_DEFAULT_MS = 6000; // generous: the reply ends the quiet period early
const safeName = (n) => { if (!/^[a-z_]+$/.test(n)) throw new Error(`bad effect name ${n}`); return n; };

export class BleRobot {
  // helpers: install the robot-side watchdog and eye helper (exec of ~400 byte scripts).
  // helpers default off: on 44.01.013 importing modules from Live Mode reboots the robot
  // (research/07-hardware-session.md), so the watchdog and eye helper cannot be installed.
  constructor({ log, onStatus, chunkSize = 20, chunkDelayMs = 8, debugAllDevices = false, helpers = false }) {
    this.helpers = helpers;
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
    this.eyeHelperFor = null; // writeChar the helper was installed over (new connection = reinstall)
    this.eyeHelper = false;
    this.eyeModeLogged = null;
    this.effectMs = {}; // measured effect durations, name -> ms
    // Wheel mapping: the motors are mirrored, so forward is EM1 +, EM2 -.
    // Robot calibration (settings "Roboter-Kalibrierung"): motor mounting,
    // connector swap, the firmware's mbot2.turn sign and the gyro sign.
    // Defaults for a standard mBot2 with firmware 44.01.013, anchored on what is
    // visible (joystick right turns right, 2026-10-04): EM1 left, EM2 mirrored,
    // mbot2.turn as documented, and the gyro counts COUNTERclockwise positive.
    this.wheels = { swap: false, mirrored: true, turnSign: FIRMWARE_TURN_SIGN, yawSign: -1 };
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
    if (this.helpers) await this.installWatchdog();
    if (!this.connected) throw new Error('Verbindung beim Einrichten verloren');
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

  // Diagnoses link problems: latency, how long a frame may be, whether the
  // robot is jammed after a long frame, and whether threads are available.
  // Read-only sensor discovery for motion tracking: which methods exist
  // (hasattr only, no dir() and no imports) and what they return.
  async sensorTest(report) {
    const t = async (label, script) => {
      const t0 = performance.now();
      try {
        const v = await this.query(script, 2500);
        report(`OK  ${label}: ${JSON.stringify(v)} (${Math.round(performance.now() - t0)} ms)`);
      } catch (e) {
        report(`--  ${label}: ${e.message.split(':')[0]}`);
      }
    };
    report('Sensor-Test (nur lesen, Roboter bewegt sich nicht)');
    await t('Firmware', 'cyberpi.get_firmware_version()');
    const names = (obj, list) => `[n for n in ${JSON.stringify(list)} if hasattr(${obj},n)]`;
    await t('mbot2 hat', names('mbot2', ['EM_get_angle', 'EM_get_speed', 'EM_reset_angle', 'EM_get_power', 'get_speed', 'get_angle']));
    await t('cyberpi hat', names('cyberpi', ['get_yaw', 'get_roll', 'get_pitch', 'get_acc', 'get_gyro', 'get_rotation', 'reset_yaw', 'reset_rotation', 'get_shakeval', 'is_shake', 'timer']));
    await t('yaw', 'cyberpi.get_yaw()');
    await t('roll, pitch', '[cyberpi.get_roll(),cyberpi.get_pitch()]');
    await t('acc x,y,z', "[cyberpi.get_acc('x'),cyberpi.get_acc('y'),cyberpi.get_acc('z')]");
    await t('gyro x,y,z', "[cyberpi.get_gyro('x'),cyberpi.get_gyro('y'),cyberpi.get_gyro('z')]");
    await t('rotation z', "cyberpi.get_rotation('z')");
    await t('Encoder Winkel EM1,EM2', '[mbot2.EM_get_angle("EM1"),mbot2.EM_get_angle("EM2")]');
    await t('Encoder Winkel 1,2', '[mbot2.EM_get_angle(1),mbot2.EM_get_angle(2)]');
    await t('Shake', 'cyberpi.get_shakeval()');
    await t('Encoder Tempo 1,2', '[mbot2.EM_get_speed(1),mbot2.EM_get_speed(2)]');
    await t('Abstand + yaw', '[cyberpi.ultrasonic2.get(1),cyberpi.get_yaw()]');
    report('Sensor-Test fertig. Bitte den Roboter jetzt von Hand etwa 90° im Uhrzeigersinn drehen und den Test erneut starten.');
  }

  // Drives both wheels slowly for about 1 s (forward, mirrored motors) and
  // reads the encoders with both port forms, to find the one that works.
  async encoderTest(report) {
    const read = async (label) => {
      try {
        const v = await this.query('[mbot2.EM_get_angle("EM1"),mbot2.EM_get_angle("EM2"),mbot2.EM_get_angle(1),mbot2.EM_get_angle(2),mbot2.EM_get_speed(1),mbot2.EM_get_speed(2)]', 2500);
        report(`${label}: Winkel "EM1","EM2" = ${v[0]}, ${v[1]} | Winkel 1,2 = ${v[2]}, ${v[3]} | Tempo 1,2 = ${v[4]}, ${v[5]}`);
      } catch (e) { report(`${label}: Fehler ${e.message.split(':')[0]}`); }
    };
    report('Encoder-Test: Räder drehen 1 s langsam vorwärts (Roboter anheben oder Platz lassen)');
    await read('vorher');
    try {
      await this.run('mbot2.drive_speed(30,-30)');
      await new Promise((r) => setTimeout(r, 500));
      await read('während');
      await new Promise((r) => setTimeout(r, 500));
    } finally {
      await this.stop();
    }
    await new Promise((r) => setTimeout(r, 300));
    await read('nachher');
    report('Encoder-Test fertig. Erwartet: nach 1 s bei 30 RPM etwa 180° Radwinkel (EM2 negativ, weil gespiegelt).');
  }

  // Checks the sign and accuracy of the firmware's blocking mbot2.turn():
  // a +90 command should raise get_yaw() (clockwise positive) by about 90.
  // Detects motor mounting, connector swap and the mbot2.turn sign, using the
  // gyro (clockwise positive after yawSign) as the reference. Spins in place;
  // with unmirrored motors the first step drives a few cm forward.
  async calibrate(report) {
    // Reference: the wheel mapping as set (check first that the joystick
    // turns right when pushed right). Derives motor mirroring, the gyro sign
    // and the mbot2.turn sign from that. Spins in place; with unmirrored
    // motors the first step drives a few cm forward.
    const rawYaw = async () => Number(await this.query('cyberpi.get_yaw()', 2000));
    const dyaw = (a, b) => ((b - a + 540) % 360) - 180;
    const pause = (ms) => new Promise((r) => setTimeout(r, ms));
    const out = { ...this.wheels };
    report('Kalibrierung: Voraussetzung ist, dass der Joystick nach rechts den Roboter nach rechts dreht (sonst „Räder tauschen“ umschalten). Roboter dreht sich kurz auf der Stelle.');
    try {
      // 1. same RPM on both motors: mirrored motors spin the robot, unmirrored drive straight
      let y0 = await rawYaw();
      await this.run('mbot2.drive_speed(25,25)');
      await pause(700);
      await this.stop();
      await pause(400);
      let d = dyaw(y0, await rawYaw());
      out.mirrored = Math.abs(d) > 15;
      report(`1. drive_speed(25,25): Gyro ${d}° → Motoren ${out.mirrored ? 'gespiegelt' : 'nicht gespiegelt'}`);
      // 2. right turn through the (trusted) wheel mapping: its gyro sign is the gyro sign
      this.wheels = { ...this.wheels, mirrored: out.mirrored };
      y0 = await rawYaw();
      await this.drive(25, -25);
      await pause(700);
      await this.stop();
      await pause(400);
      d = dyaw(y0, await rawYaw());
      out.yawSign = d >= 0 ? 1 : -1;
      report(`2. Rechtsdrehung über die Radzuordnung: Gyro ${d}° → Gyro ${out.yawSign === 1 ? 'normal' : 'umgekehrt'}`);
      // 3. firmware turn: raw mbot2.turn(45) should turn right
      y0 = await rawYaw();
      await this.query('mbot2.turn(45)', 6000);
      await pause(300);
      d = dyaw(y0, await rawYaw()) * out.yawSign;
      out.turnSign = d >= 0 ? 1 : -1;
      report(`3. mbot2.turn(45): ${d}° (rechts positiv) → eingebaute Drehung ${out.turnSign === 1 ? 'normal' : 'umgekehrt'}`);
      await this.query(`mbot2.turn(${-45 * out.turnSign})`, 6000); // back
      if (Math.abs(d) < 20) report('! Drehung zu klein gemessen; Ergebnis bitte prüfen (Akku? Untergrund?)');
    } catch (e) {
      report(`! Kalibrierung abgebrochen: ${e.message.split(':')[0]}`);
      await this.stop().catch(() => {});
      return null;
    } finally {
      await this.stop().catch(() => {});
    }
    this.wheels = out;
    report(`Kalibrierung fertig: gespiegelt ${out.mirrored}, getauscht ${out.swap} (nicht geprüft), Drehung ${out.turnSign === 1 ? 'normal' : 'umgekehrt'}, Gyro ${out.yawSign === 1 ? 'normal' : 'umgekehrt'}`);
    return out;
  }

  async turnTest(report) {
    const yaw = async () => Number(await this.query('cyberpi.get_yaw()', 2000));
    report('Dreh-Test: mbot2.turn(90), dann mbot2.turn(-90) (Roboter dreht sich auf der Stelle)');
    try {
      const y0 = await yaw();
      await this.query('mbot2.turn(90)', 8000);
      await new Promise((r) => setTimeout(r, 300));
      const y1 = await yaw();
      await this.query('mbot2.turn(-90)', 8000);
      await new Promise((r) => setTimeout(r, 300));
      const y2 = await yaw();
      const d1 = ((y1 - y0 + 540) % 360) - 180, d2 = ((y2 - y1 + 540) % 360) - 180;
      report(`yaw ${y0} → ${y1} → ${y2}: turn(90) ergab ${d1}°, turn(-90) ergab ${d2}°`);
      report(d1 > 45 ? 'Ergebnis: mbot2.turn dreht wie erwartet (+ = im Uhrzeigersinn).'
        : d1 < -45 ? 'Ergebnis: mbot2.turn dreht ANDERSHERUM als kommandiert (bestätigt die Ursache des Zickzacks).'
          : 'Ergebnis: unklar (kaum Drehung gemessen).');
    } catch (e) {
      report(`! Dreh-Test: ${e.message.split(':')[0]}`);
      await this.stop().catch(() => {});
    }
  }

  async connectionTest(report) {
    const ms = (t0) => Math.round(performance.now() - t0);
    const t = async (label, script, timeout = 4000) => {
      const t0 = performance.now();
      try {
        const v = await this.query(script, timeout);
        report(`OK  ${label}: ${ms(t0)} ms → ${JSON.stringify(v).slice(0, 40)}`);
        return true;
      } catch (e) {
        report(`FEHLER ${label}: ${e.message.split(':')[0]} nach ${ms(t0)} ms`);
        return false;
      }
    };
    report(`Test: Chunk ${this.chunkSize} B, Pause ${this.chunkDelayMs} ms, Helfer ${this.helpers ? 'an' : 'aus'}`);
    for (let i = 1; i <= 3; i++) await t(`kurz ${i}`, 'cyberpi.get_bri()');
    await t('Abstand', 'cyberpi.ultrasonic2.get(1)');
    this.maxScript = 1000; // the test probes past the normal limit on purpose
    try {
      for (const n of [120, 200, 220, 240, 250, 260, 280]) {
        const ok = await t(`Anfrage ${n} Bytes`, `len("${'x'.repeat(n - 7)}")`, 4000);
        await t('  danach kurz', 'cyberpi.get_bri()');
        if (!ok) { report(`→ Anfragen: Grenze zwischen vorherigem Wert und ${n} Bytes`); break; }
      }
    } finally { this.maxScript = undefined; }
    for (const n of [100, 200, 300, 500]) {
      const ok = await t(`Antwort ${n} Zeichen`, `"x"*${n}`, 5000);
      await t('  danach kurz', 'cyberpi.get_bri()', 6000);
      if (!ok) { report(`→ Antworten: Grenze unter ${n} Zeichen`); break; }
    }
    await t('Firmware', 'cyberpi.get_firmware_version()');
    await t('_thread vorhanden', "__import__('_thread').get_ident()>0");
    report('Test fertig');
  }

  async installWatchdog() {
    this.watchdog = false;
    try {
      await this.execLong(WATCHDOG_SRC);
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
    for (const q of this.queue) q.reject(new Error('disconnected'));
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
      // A hidden tab stretches timers to ~1 s; the pause is not needed on macOS
      // Chrome (research/07), so skip it there rather than stall every frame.
      if (this.chunkDelayMs && !(typeof document !== 'undefined' && document.hidden)) await sleep(this.chunkDelayMs);
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

  checkSize(script) {
    const max = this.maxScript ?? MAX_SCRIPT;
    if (byteLen(script) > max) throw new Error(`script too long for the robot (${byteLen(script)} > ${max} bytes)`);
  }

  nextIdx() {
    const i = this.idx;
    this.idx = (this.idx % 0xfffe) + 1;
    return i;
  }

  run(script, opts = {}) {
    return this.enqueue(() => {
      this.checkSize(script);
      const frame = buildScriptFrame(script, this.nextIdx(), MODE_NO_REPLY);
      if (!opts.quiet) this.log(`> ${script}`, toHex(frame));
      return this.writeRaw(frame);
    }, opts);
  }

  query(script, timeoutMs = 1500) {
    return new Promise((resolve, reject) => {
      this.enqueue(async () => {
        this.checkSize(script);
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

  // wait: send with reply so the promise resolves when the robot has finished
  // the (blocking) gyro turn. Used by scans and the AI agent.
  // speed: wheel RPM (Makeblock signature turn(angle, speed=50)).
  // wheels.turnSign keeps "+ = clockwise/right" for every caller. On firmware
  // 44.01.013 mbot2.turn(+90) turns clockwise; the Dreh-Test reading of -87
  // came from the gyro counting counterclockwise positive (wheels.yawSign).
  turn(deg, { wait = false, speed = 50 } = {}) {
    const s = `mbot2.turn(${(this.wheels.turnSign ?? FIRMWARE_TURN_SIGN) * deg},${speed})`;
    return wait ? this.query(s, 2000 + Math.abs(deg) * 30 * (50 / speed)) : this.run(s);
  }

  // Drive straight for cm (negative = backward), gyro-corrected on the robot.
  // straightScale: the firmware assumes a 6.5 cm wheel; with a measured
  // effective diameter D the app sets 6.5 / D so the real travel matches cm.
  straight(cm, { wait = false, speed = 50 } = {}) {
    const s = `mbot2.straight(${Math.round(cm * (this.straightScale ?? 1) * 10) / 10},${speed})`;
    return wait ? this.query(s, 2000 + Math.abs(cm) * 150 * (50 / speed)) : this.run(s);
  }

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

  // Gyro heading in degrees from the CyberPi IMU (sign and range UNVERIFIED on hardware).
  async yaw() { return Number(await this.query('cyberpi.get_yaw()')) * (this.wheels.yawSign ?? 1); }

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

  // led.play blocks the robot until the animation ends; the reply marks the end.
  async ledEffect(name) {
    const key = `led:${safeName(name)}`;
    const r = await this.queryUntilDone(`cyberpi.led.play(${py(name)})`, 12000, `led ${name}`);
    if (!r.done) return { name, running: true };
    this.effectMs[key] = r.ms + 300;
    return { name, ms: r.ms };
  }

  // Long blocking scripts (effects): resolves { done: true, value } when the
  // robot replies, or { done: false } after timeoutMs while it may still run.
  // A late reply is logged instead of showing up as "unmatched".
  queryUntilDone(script, timeoutMs, label = script) {
    return new Promise((resolve, reject) => {
      const t0 = Date.now();
      this.enqueue(async () => {
        const idx = this.nextIdx();
        this.checkSize(script);
        const entry = {
          resolve: (value) => resolve({ done: true, value, ms: Date.now() - t0 }),
          reject,
          timer: setTimeout(() => {
            resolve({ done: false });
            const late = {
              resolve: (v) => this.log(`< ${label} finished late after ${Date.now() - t0} ms: ${JSON.stringify(v)}`),
              reject: () => {},
              timer: setTimeout(() => this.pending.delete(idx), 30000),
            };
            this.pending.set(idx, late);
          }, timeoutMs),
        };
        this.pending.set(idx, entry);
        this.log(`> ${script}`);
        await this.writeRaw(buildScriptFrame(script, idx, MODE_REPLY));
      }).catch(reject);
    });
  }

  async ensureEyeHelper() {
    if (!this.helpers) return false;
    if (this.eyeHelperFor === this.writeChar) return this.eyeHelper;
    this.eyeHelperFor = this.writeChar;
    this.eyeHelper = false;
    this.eyeModeLogged = null;
    try {
      await this.execLong(EYE_HELPER_SRC);
      const [fx, eyes] = await this.query("[hasattr(mbot2,'_fx'),hasattr(mbot2,'_eyes')]", 1500);
      this.eyeHelper = fx === true && eyes === true;
    } catch (e) {
      this.log(`Eye helper install failed: ${e.message}`);
    }
    if (!this.eyeHelper) this.log('Eye helper not available; eye effects use direct calls, errors may stay silent.');
    return this.eyeHelper;
  }

  // Both eyes, 0..100 each. Eye A = LEDs 1-4 = "left" (UNVERIFIED which side).
  async eyes(left, right) {
    if (await this.ensureEyeHelper()) {
      if (!this.eyeModeLogged) {
        this.eyeModeLogged = true;
        const mode = await this.query(`mbot2._eyes(${left},${right})`, 3000);
        this.log(`Eyes use ${mode === 'list' ? 'led_show([8 values],1)' : 'set_both_led_bri(l,r)+led_show()'} (${JSON.stringify(mode)})`);
        return mode;
      }
      return this.run(`mbot2._eyes(${left},${right})`, { quiet: true });
    }
    return this.run(`cyberpi.ultrasonic2.led_show(${eyeList(left, right)},1)`);
  }

  // One eye LED (1..8) or all; set_bri = set_single_led_bri(led_bri, led_index, index) on 44.01.011.
  eyeLed(id, bri) {
    if (id === 'all') return this.run(`cyberpi.ultrasonic2.led_show(${eyeList(bri, bri)},1)`);
    return this.run(`cyberpi.ultrasonic2.set_bri(${bri},${id},1)`);
  }

  effectEstimateMs(name) { return this.effectMs[name] ?? EFFECT_DEFAULT_MS; }

  // Runs one eye animation and resolves when it has finished on the robot.
  // Returns { name, ms } or { name, running: true } if no reply came in time.
  async eyesEffect(name) {
    safeName(name);
    const script = (await this.ensureEyeHelper())
      ? `mbot2._fx(${py(name)})`
      : `cyberpi.ultrasonic2.${name}_effect(${EYE_EFFECT_NEEDS_BRI.has(name) ? 100 : ''})`;
    const r = await this.queryUntilDone(script, 12000, `eyes ${name}`);
    if (!r.done) return { name, running: true };
    if (r.value === 'missing') throw new Error(`eye effect ${name} does not exist on this firmware`);
    if (typeof r.value === 'string' && r.value.startsWith('error')) throw new Error(`eye effect ${name}: ${r.value}`);
    const ms = Number.isFinite(r.value) ? r.value : r.ms;
    this.effectMs[name] = ms + 300;
    return { name, ms };
  }

  // Which candidate effects exist on this firmware (hasattr on the robot).
  probeEyeEffects(names) {
    const list = `[${names.map((n) => py(safeName(n))).join(',')}]`;
    return this.query(`[n for n in ${list} if hasattr(cyberpi.ultrasonic2,n+'_effect')]`, 3000);
  }

  floorLight(color) {
    return this.run(color === 'off' ? 'cyberpi.quad_rgb_sensor.off_led(1)' : `cyberpi.quad_rgb_sensor.set_led(${py(color)},1)`);
  }

  // Probes in physical order L2 L1 R1 R2 (sensor ids 4 3 2 1). Colour reads
  // make the sensor switch its fill light, so they are optional.
  async floor(withColors) {
    const q = 'cyberpi.quad_rgb_sensor';
    // two queries with colours: one combined script would exceed MAX_SCRIPT
    const v = await this.query(`(lambda q:[q.get_line_sta(1),q.get_offset_track(1),[q.get_gray(i) for i in (4,3,2,1)]])(${q})`, 2500);
    if (!Array.isArray(v)) throw new Error(`unexpected floor reply: ${JSON.stringify(v)}`);
    const [line, offset, gray] = v;
    let names = null, rgb = null;
    if (withColors) {
      const c = await this.query(`(lambda q:[[q.get_color_sta(i) for i in (4,3,2,1)],[[q.get_red(i),q.get_green(i),q.get_blue(i)] for i in (4,3,2,1)]])(${q})`, 2500);
      if (Array.isArray(c)) [names, rgb] = c;
    }
    return { line, offset, gray, names, rgb };
  }

  async diagnose() {
    const out = [await this.query('cyberpi.get_firmware_version()', 3000)];
    for (const obj of ['cyberpi.led', 'cyberpi.ultrasonic2', 'cyberpi.quad_rgb_sensor', 'mbot2']) {
      out.push(await this.queryLong(`dir(${obj})`));
    }
    return out;
  }

  // Runs code longer than MAX_SCRIPT: the source is assembled in pieces in
  // mbot2._s on the robot (memory only), then executed.
  async execLong(src, timeoutMs = 3000) {
    const piece = (p) => `setattr(mbot2,'_s',mbot2._s+${py(p)})`;
    const pieces = [];
    let cur = '';
    for (const ch of src) {
      if (byteLen(piece(cur + ch)) > MAX_SCRIPT) { pieces.push(cur); cur = ch; } else cur += ch;
    }
    if (cur) pieces.push(cur);
    await this.query("setattr(mbot2,'_s','')", 1500);
    for (const p of pieces) await this.query(piece(p), 1500);
    const r = await this.query('exec(mbot2._s)', timeoutMs);
    this.run("setattr(mbot2,'_s','')", { quiet: true }).catch(() => {});
    return r;
  }

  // Reads a long result in 150-character slices of its str() form.
  async queryLong(expr, slice = 150) {
    let text = '';
    for (let i = 0; i < 40; i++) {
      const part = await this.query(`str(${expr})[${i * slice}:${(i + 1) * slice}]`, 2000);
      text += part ?? '';
      if (!part || part.length < slice) break;
    }
    return text;
  }

  display(text) { return this.run(`cyberpi.display.show_label(${py(text)},24,"center")`); }

  beep(freq = 700, secs = 0.2) { return this.run(`cyberpi.audio.play_tone(${freq},${secs})`); }

  raw(script) { return this.query(script, 3000); }
}
