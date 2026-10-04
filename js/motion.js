// Straight legs driven with streamed wheel speeds while polling sensors, so a
// leg can end early on an obstacle, a stall or a crash and report the
// distance actually driven. Sensor calls and units: research/08-motion-sensors.md.
//
// Samples: { distanceCm, encL, encR, acc: { x, y, z }, yaw, shake }, every
// field optional. encL/encR are cumulative wheel angles in degrees,
// forward-positive (the sampler undoes the mirrored motor). Missing encoders
// fall back to time x commanded speed; missing IMU fields disable the
// detectors that need them.
//
// Every drive command carries args.leg = true so the app can skip its own
// drive integration: the caller applies the returned droveCm to the pose.

import { makeCommand as defaultMakeCommand } from './bus.js';

const WHEEL_CM = Math.PI * 6.5;
const TRACK_CM = 12;
export const CM_PER_DEG = WHEEL_CM / 360;
const NO_ECHO_CM = 300;

export const MOTION = {
  hz: 6,              // polls per second (each poll is one ~90 ms query on BLE)
  minRpm: 12,         // start and creep speed
  rampMs: 500,        // minRpm -> speed
  slowCm: 8,          // slow down over the last cm
  doneCm: 0.5,
  stallMs: 400,       // window for stall: progress below stallRatio of commanded
  stallRatio: 0.3,
  stallMinCm: 1.5,    // judge only when this much was commanded in the window
  // Slip (range ahead does not shrink while the wheels turn) is off by
  // default: driving past a side object keeps the nearest echo in the cone at
  // a constant range, which looked like slip and caused false crashes.
  slip: false,
  slipMs: 800,
  slipCm: 8,
  slipRatio: 0.15,
  slipMaxRangeCm: 150,
  joltMs2: 6,         // change of the acceleration vector between two samples
  shakeLimit: 40,     // cyberpi.get_shakeval() 0..100
  headingDeg: 12,     // yaw change not explained by the wheels
  // Glancing hits on straight legs (one wheel caught, the body twisted).
  // Noise at 4 to 6 Hz: integer yaw +-1 deg, encoders +-1 wheel deg (0.5 deg
  // of heading), closed-loop wheels hold a straight line within a few deg.
  twistDeg: 10,       // yaw change on a straight leg
  wheelDeg: 8,        // encoder-implied heading change on a straight leg
  settleMs: 150,      // wait after a stop before the final reading, so the coast is counted
  coastGain: 0.7,     // how fast the learned stop distance follows the measured overshoot
  maxCoastCm: 10,
  backoffCm: 5,
  backoffRpm: 25,
  timeoutFactor: 2.5, // of the nominal leg time
  maxSampleErrors: 3,
};

// Candidate read expressions (UNVERIFIED on hardware except distance and yaw).
export const SENSOR_EXPR = {
  distance: 'cyberpi.ultrasonic2.get(1)',
  encL: 'mbot2.EM_get_angle("EM1")',
  encR: 'mbot2.EM_get_angle("EM2")',
  ax: "cyberpi.get_acc('x')",
  ay: "cyberpi.get_acc('y')",
  az: "cyberpi.get_acc('z')",
  yaw: 'cyberpi.get_yaw()',
  shake: 'cyberpi.get_shakeval()',
};

// All confirmed on firmware 44.01.013 (research/07): distance, yaw (clockwise
// positive, degrees), acceleration (m/s², z = -9.6 at rest), shake, and the
// encoders: EM_get_angle("EM1"/"EM2") in wheel degrees, EM1 positive and EM2
// negative when driving forward (mirrored motors), 245° after about 1.2 s at 30 RPM.
export const BLE_SENSORS = { ...SENSOR_EXPR };
export const BLE_SENSORS_MIN = { distance: SENSOR_EXPR.distance, yaw: SENSOR_EXPR.yaw };

const FIELDS = ['distance', 'encL', 'encR', 'ax', 'ay', 'az', 'yaw', 'shake'];
const MAX_SCRIPT = 200;

const normDeg = (a) => { const r = ((((a + 180) % 360) + 360) % 360) - 180; return r === -180 ? 180 : r; };
const num = (v) => (v == null || v === '' ? undefined : Number.isFinite(Number(v)) ? Number(v) : undefined);
const clamp = (v, lo, hi) => Math.min(hi, Math.max(lo, v));

// One list expression for the given descriptor, shortened with a lambda.
export function buildSampleExpr(sensors) {
  const keys = FIELDS.filter((k) => typeof sensors?.[k] === 'string');
  if (!keys.length) throw new Error('no sensor expressions');
  const items = keys.map((k) => sensors[k]);
  const plain = `[${items.join(',')}]`;
  const short = `(lambda c,m:[${items.map((s) => s.replace(/\bcyberpi\./g, 'c.').replace(/\bmbot2\./g, 'm.')).join(',')}])(cyberpi,mbot2)`;
  const expr = short.length < plain.length ? short : plain;
  if (expr.length > MAX_SCRIPT) throw new Error(`sample expression is ${expr.length} bytes (max ${MAX_SCRIPT})`);
  return { expr, keys };
}

// Turns a reply list into a sample; mirrored/swap match robot.wheels.
export function parseSample(keys, values, { mirrored = true, swap = false, yawSign = 1 } = {}) {
  const v = {};
  keys.forEach((k, i) => { v[k] = num(values?.[i]); });
  // raw: encL = EM1, encR = EM2. EM2 is the mirrored motor, so un-mirror it
  // first, then map motors to sides (swap: EM1 is the right wheel)
  let encL = v.encL, encR = v.encR;
  if (mirrored && encR != null) encR = -encR;
  if (swap) [encL, encR] = [encR, encL];
  const s = { distanceCm: v.distance, encL, encR, yaw: v.yaw != null ? v.yaw * yawSign : undefined, shake: v.shake };
  if (v.ax != null || v.ay != null || v.az != null) s.acc = { x: v.ax ?? 0, y: v.ay ?? 0, z: v.az ?? 0 };
  return s;
}

// Sampler over BleRobot.query. If the full expression fails (a name the
// firmware lacks), falls back once to `fallback` and keeps using it.
export function makeBleSampler(robot, sensors = BLE_SENSORS, { fallback = BLE_SENSORS_MIN, timeoutMs = 1200, log } = {}) {
  let cur = buildSampleExpr(sensors);
  let canFall = fallback && fallback !== sensors;
  const sample = async () => {
    let values;
    try {
      values = await robot.query(cur.expr, timeoutMs);
    } catch (e) {
      if (!canFall || /timeout|not connected/.test(e?.message ?? '')) throw e;
      log?.(`! motion sensors failed (${e.message}); using ${Object.keys(fallback).join(', ')}`);
      canFall = false;
      cur = buildSampleExpr(fallback);
      sensors = fallback;
      values = await robot.query(cur.expr, timeoutMs);
    }
    if (!Array.isArray(values)) throw new Error(`unexpected sample reply ${values}`);
    // encoder and gyro mapping follow the robot calibration (settings)
    return { t: performance.now(), ...parseSample(cur.keys, values, robot.wheels ?? {}) };
  };
  return sample;
}

// Sampler over SimRobot.sensorSample(), with a clock in simulated time so
// legs behave the same at any timeScale.
// sim.latencyMs emulates the BLE round trip: the robot reads its sensors
// halfway, the reply (and t) arrives at the end.
export function makeSimSampler(sim) {
  const k = () => sim.timeScale || 1;
  const clock = {
    now: () => performance.now() * k(),
    sleep: (ms) => new Promise((r) => setTimeout(r, ms / k())),
  };
  const sample = async () => {
    const lat = sim.latencyMs || 0;
    if (!lat) return sim.sensorSample();
    await clock.sleep(lat / 2);
    const s = sim.sensorSample();
    await clock.sleep(lat / 2);
    return { ...s, t: clock.now() };
  };
  sample.clock = clock;
  return sample;
}

const realClock = { now: () => performance.now(), sleep: (ms) => new Promise((r) => setTimeout(r, ms)) };

// Looks back over the sample history (newest last) for a crash or stall.
// Each entry: { t, drivenCm, cmdCm, distanceCm?, acc?, shake?, yawDelta?, encHeading?, hasEnc,
// dL?, dR?, cmdL?, cmdR? } (per-wheel cm, signed). expected.straight enables the
// glancing-hit checks (twist, wheel) that assume equal wheel commands.
export function detectCrash(samples, expected = {}) {
  const o = { ...MOTION, ...expected };
  const n = samples.length;
  if (n < 2) return { crash: false, reason: null };
  const s = samples[n - 1], p = samples[n - 2];
  const before = (ms) => { for (let i = n - 2; i >= 0; i--) if (s.t - samples[i].t >= ms) return samples[i]; return null; };

  if (s.acc && p.acc) {
    const d = Math.hypot(s.acc.x - p.acc.x, s.acc.y - p.acc.y, s.acc.z - p.acc.z);
    if (d > o.joltMs2) return { crash: true, reason: 'jolt', value: d };
  }
  if (s.shake != null && s.shake > o.shakeLimit) return { crash: true, reason: 'jolt', value: s.shake };
  // Stall must show in two consecutive windows: a single sample can lag the
  // commanded progress by half a round trip (150 to 270 ms on BLE).
  const stalled = (j) => {
    const e = samples[j];
    if (!e?.hasEnc) return null;
    let w = null;
    for (let i = j - 1; i >= 0; i--) if (e.t - samples[i].t >= o.stallMs) { w = samples[i]; break; }
    if (!w) return null;
    const want = e.cmdCm - w.cmdCm, got = e.drivenCm - w.drivenCm;
    if (want >= o.stallMinCm && got < o.stallRatio * want) return { value: got / want };
    // one wheel held back (door frame, chair leg) while the other keeps going
    for (const [k, c, side] of [['dL', 'cmdL', 'left'], ['dR', 'cmdR', 'right']]) {
      if (e[k] == null || w[k] == null || e[c] == null || w[c] == null) continue;
      const wantW = e[c] - w[c], gotW = (e[k] - w[k]) * Math.sign(wantW);
      if (Math.abs(wantW) >= o.stallMinCm && gotW < o.stallRatio * Math.abs(wantW)) return { value: gotW / Math.abs(wantW), wheel: side };
    }
    return null;
  };
  const st = stalled(n - 1);
  if (st && stalled(n - 2)) return { crash: true, reason: 'stall', ...st };
  if (o.straight && s.yawDelta != null && Math.abs(s.yawDelta) > o.twistDeg) {
    return { crash: true, reason: 'twist', value: s.yawDelta };
  }
  if (o.straight && s.hasEnc && Math.abs(s.encHeading ?? 0) > o.wheelDeg) {
    return { crash: true, reason: 'wheel', value: s.encHeading };
  }
  if (s.yawDelta != null) {
    const div = Math.abs(normDeg(s.yawDelta - (s.encHeading ?? 0)));
    const tol = o.headingDeg + (o.headingFrac ?? 0) * Math.abs(s.yawDelta);
    if (div > tol) return { crash: true, reason: 'heading', value: div };
  }
  const v = o.slip && before(o.slipMs);
  const ok = (x) => x != null && x > 0 && x < o.slipMaxRangeCm;
  if (v && ok(v.distanceCm) && ok(s.distanceCm)) {
    const moved = s.drivenCm - v.drivenCm;
    if (moved >= o.slipCm && v.distanceCm - s.distanceCm < o.slipRatio * moved) return { crash: true, reason: 'slip', value: moved };
  }
  return { crash: false, reason: null };
}

function abortError(msg = 'leg aborted') {
  const e = new Error(msg);
  e.name = 'AbortError';
  return e;
}

// Drives cm forward at `speed` RPM. Returns { ok, droveCm, reason, detail,
// samples, contactCm, backedCm, encHeading, yawDelta, note }; reason is
// 'done' | 'obstacle' | 'crash' | 'stall' | 'aborted' | 'error'. On crash or
// stall the robot backs off about backoffCm and droveCm is the net distance.
// Always ends with a stop, including on errors. Never throws for abort; the
// caller checks signal itself.
export async function driveLeg(bus, {
  cm, speed = 40, makeCommand = defaultMakeCommand, signal, sample, sensors, onSample,
  stopAtCm = 20, clock, yawSign = 1, cmPerDeg = CM_PER_DEG, opts = {}, coast,
} = {}) {
  // coast: { cm } learned stop distance, shared across legs by the caller.
  // The leg aims coast.cm short of the target; after each completed leg the
  // estimate moves toward the measured overshoot (field: 3 to 4 cm per leg).
  const o = { ...MOTION, ...opts };
  const mk = makeCommand;
  if (!sample && sensors && bus.robot?.query) sample = makeBleSampler(bus.robot, sensors);
  if (!sample) {
    // distance-only fallback through the bus (also feeds the bus obstacle guard)
    sample = async () => {
      const r = await bus.submit(mk('read', { sensor: 'distance' }, 'agent'));
      if (!r.ok) throw new Error(r.error);
      return { distanceCm: num(r.value) };
    };
  }
  clock ??= sample.clock ?? realClock;
  const target = Math.max(0, Number(cm) || 0);
  const vNom = (speed / 60) * WHEEL_CM;
  const samples = [];
  let reason = null, detail = null, details = null, note, contactCm = null, backedCm = 0, overshootCm = null;
  let base = null, last = null, cmdCm = 0, tPrev = null, errors = 0, cancelled = false;

  const send = async (l, r) => {
    const res = await bus.submit(mk('drive', { left: l, right: r, leg: true }, 'agent', 500));
    if (!res.ok) {
      if (/cancelled by stop/.test(res.error ?? '')) { cancelled = true; throw abortError(res.error); }
      throw new Error(res.error);
    }
  };
  const stop = () => bus.submit(mk('stop', {}, 'agent')).catch(() => {});

  // Reads one sample and derives progress relative to the leg start.
  const read = async (rpm) => {
    const raw = await sample();
    const t = clock.now();
    if (tPrev != null) cmdCm += (rpm / 60) * WHEEL_CM * ((t - tPrev) / 1000);
    tPrev = t;
    base ??= raw;
    const hasEnc = raw.encL != null && raw.encR != null && base.encL != null && base.encR != null;
    const dL = hasEnc ? (raw.encL - base.encL) * cmPerDeg : 0, dR = hasEnc ? (raw.encR - base.encR) * cmPerDeg : 0;
    const s = {
      t, hasEnc, cmdCm, cmdL: cmdCm, cmdR: cmdCm,
      dL: hasEnc ? dL : undefined, dR: hasEnc ? dR : undefined,
      drivenCm: hasEnc ? (dL + dR) / 2 : cmdCm,
      encHeading: hasEnc ? ((dL - dR) / TRACK_CM) * (180 / Math.PI) : 0,
      distanceCm: raw.distanceCm, acc: raw.acc, shake: raw.shake,
      yawDelta: raw.yaw != null && base.yaw != null ? normDeg(raw.yaw - base.yaw) * yawSign : undefined,
    };
    samples.push(s);
    last = s;
    try { onSample?.(s); } catch { /* UI errors must not break the leg */ }
    return s;
  };
  const readSafe = async (rpm) => {
    try {
      const s = await read(rpm);
      errors = 0;
      return s;
    } catch (e) {
      if (++errors >= o.maxSampleErrors) throw new Error(`sensor poll failed: ${e?.message ?? e}`);
      return null;
    }
  };

  try {
    if (signal?.aborted) throw abortError();
    await read(0);
    const t0 = clock.now();
    const timeout = (target / Math.max(vNom, 1)) * 1000 * o.timeoutFactor + o.rampMs + 1000;
    const period = 1000 / o.hz;
    let rpm = 0;
    const coastCm = clamp(Number(coast?.cm) || 0, 0, Math.max(0, target - 1));
    while (!reason) {
      const left = target - coastCm - last.drivenCm;
      if (left <= o.doneCm) { reason = 'done'; break; }
      const d = last.distanceCm;
      if (d != null && d > 0 && d < NO_ECHO_CM && d < stopAtCm) { reason = 'obstacle'; break; }
      const el = clock.now() - t0;
      if (el > timeout) { reason = 'stall'; detail = 'timeout'; break; }
      const ramp = o.minRpm + (speed - o.minRpm) * clamp(el / o.rampMs, 0, 1);
      const slow = o.minRpm + (speed - o.minRpm) * clamp(left / o.slowCm, 0, 1);
      rpm = Math.round(clamp(Math.min(ramp, slow), o.minRpm, speed));
      const tick = clock.now();
      await send(rpm, rpm);
      if (signal?.aborted) throw abortError();
      const s = await readSafe(rpm);
      if (signal?.aborted) throw abortError();
      if (s) {
        const c = detectCrash(samples, { ...o, straight: true });
        if (c.crash) {
          reason = c.reason === 'stall' ? 'stall' : 'crash';
          detail = c.reason;
          details = { value: c.value, wheel: c.wheel, drivenCm: s.drivenCm, yawDelta: s.yawDelta, encHeading: s.encHeading };
          contactCm = s.drivenCm;
          break;
        }
      }
      const wait = period - (clock.now() - tick);
      if (wait > 1) await clock.sleep(wait);
    }
    await stop();
    await clock.sleep(o.settleMs);
    await readSafe(0);
    if (reason === 'done' && coast && last.hasEnc) {
      overshootCm = last.drivenCm - target;
      coast.cm = clamp(coastCm + overshootCm * o.coastGain, 0, o.maxCoastCm);
    }
    if (reason === 'crash' || reason === 'stall') {
      // back off so the robot can turn without scraping the obstacle
      const from = last.drivenCm;
      const tb = clock.now();
      const vBack = (o.backoffRpm / 60) * WHEEL_CM;
      const limit = (o.backoffCm / vBack) * 1000 * (last.hasEnc ? 3 : 1);
      while (from - last.drivenCm < o.backoffCm && clock.now() - tb < limit) {
        if (signal?.aborted) throw abortError();
        const tick = clock.now();
        await send(-o.backoffRpm, -o.backoffRpm);
        await readSafe(-o.backoffRpm);
        const wait = period / 2 - (clock.now() - tick);
        if (wait > 1) await clock.sleep(wait);
      }
      await stop();
      await clock.sleep(o.settleMs);
      await readSafe(0);
      backedCm = Math.max(0, from - last.drivenCm);
    }
  } catch (e) {
    if (e?.name === 'AbortError') { reason = 'aborted'; note = cancelled ? e.message : 'aborted'; }
    else { reason = 'error'; note = e?.message ?? String(e); }
  } finally {
    // the robot has no watchdog: a leg must never end with the wheels turning
    await stop();
  }
  return {
    ok: reason === 'done',
    droveCm: last ? last.drivenCm : 0,
    reason, detail, details, note, cancelled, contactCm, backedCm, overshootCm, samples,
    encHeading: last?.hasEnc ? last.encHeading : null,
    yawDelta: last?.yawDelta ?? null,
  };
}

export const TURN = {
  rpm: 30,            // wheel RPM while far from the target (about 100 deg/s)
  creepRpm: 6,        // near the target (about 20 deg/s)
  slowDeg: 40,        // ramp down over the last degrees
  tolDeg: 1.5,
  coastS: 0.05,       // extra lead for the stop on top of the sample age
  reverseDeg: 8,      // rotation against the command that proves a reversed spin
  maxPasses: 4,       // first pass plus corrections
  hz: 10,             // at most this many polls per second (BLE is slower anyway)
  spinUpMs: 600,      // no stall verdict this soon after a pass starts
};

// Closed-loop turn in place: streams spin `drive` commands (leg: true) and
// reads yaw (encoders if yaw is missing) until the heading is within tolDeg of
// the target. Positive deg = clockwise, like `turn`. Stops early on a stall
// or jolt. spinSign maps clockwise onto wheel commands; if the robot turns
// the other way the sign is flipped and returned (reversed: true).
// Returns { ok, achievedDeg, reason: 'done'|'crash'|'stall'|'aborted'|'error'|'nosensor',
// detail, details, spinSign, reversed, passes, samples, note }. Always ends with a stop.
export async function turnInPlace(bus, {
  deg, sample, makeCommand = defaultMakeCommand, signal, clock, spinSign = 1, yawSign = 1,
  cmPerDeg = CM_PER_DEG, opts = {},
} = {}) {
  const o = { ...MOTION, ...TURN, ...opts };
  const mk = makeCommand;
  clock ??= sample?.clock ?? realClock;
  const target = Number(deg) || 0;
  const samples = [];
  let reason = null, detail = null, details = null, note, cancelled = false, reversed = false, passes = 0;
  let base = null, prevYaw = null, yawAcc = 0, last = null, tPrev = null, cmdL = 0, cmdR = 0, errors = 0;
  const rate = (rpm) => (rpm * 360 * WHEEL_CM) / (60 * Math.PI * TRACK_CM); // deg/s of the body

  const send = async (l, r) => {
    const res = await bus.submit(mk('drive', { left: l, right: r, leg: true }, 'agent', 500));
    if (!res.ok) {
      if (/cancelled by stop/.test(res.error ?? '')) { cancelled = true; throw abortError(res.error); }
      throw new Error(res.error);
    }
  };
  const stop = () => bus.submit(mk('stop', {}, 'agent')).catch(() => {});

  // turned: clockwise degrees since the start, from unwrapped yaw or encoders
  const read = async (l, r) => {
    const raw = await sample();
    const t = clock.now();
    if (tPrev != null) {
      const dt = (t - tPrev) / 1000;
      cmdL += (l / 60) * WHEEL_CM * dt;
      cmdR += (r / 60) * WHEEL_CM * dt;
    }
    tPrev = t;
    base ??= raw;
    if (raw.yaw != null) {
      if (prevYaw != null) yawAcc += normDeg(raw.yaw - prevYaw); // works for wrapped and unbounded yaw
      prevYaw = raw.yaw;
    }
    const hasEnc = raw.encL != null && raw.encR != null && base.encL != null && base.encR != null;
    const dL = hasEnc ? (raw.encL - base.encL) * cmPerDeg : undefined, dR = hasEnc ? (raw.encR - base.encR) * cmPerDeg : undefined;
    const encHeading = hasEnc ? ((dL - dR) / TRACK_CM) * (180 / Math.PI) : undefined;
    const yawDelta = base.yaw != null && raw.yaw != null ? yawAcc * yawSign : undefined;
    const s = {
      t, hasEnc, dL, dR, cmdL, cmdR, encHeading, yawDelta, acc: raw.acc, shake: raw.shake,
      // stall over both wheels: travel in the commanded directions
      cmdCm: (Math.abs(cmdL) + Math.abs(cmdR)) / 2,
      drivenCm: hasEnc ? (dL * Math.sign(cmdL || 1) + dR * Math.sign(cmdR || -1)) / 2 : 0,
      turned: yawDelta ?? encHeading,
    };
    samples.push(s);
    last = s;
    return s;
  };
  const readSafe = async (l, r) => {
    try { const s = await read(l, r); errors = 0; return s; } catch (e) {
      if (++errors >= o.maxSampleErrors) throw new Error(`sensor poll failed: ${e?.message ?? e}`);
      return null;
    }
  };

  try {
    if (signal?.aborted) throw abortError();
    if (typeof sample !== 'function') { reason = 'nosensor'; return finish(); }
    await read(0, 0);
    if (last.turned == null) { reason = 'nosensor'; return finish(); }
    const t0 = clock.now();
    // room for the turn at creep speed plus every correction pass on a slow link
    const timeout = (Math.abs(target) / rate(o.creepRpm)) * 1000 + o.maxPasses * 3000;
    while (passes < o.maxPasses && !reason) {
      const remaining0 = target - last.turned;
      if (Math.abs(remaining0) <= o.tolDeg) { reason = 'done'; break; }
      passes++;
      const dir = Math.sign(remaining0); // +1 clockwise
      const startTurned = last.turned;
      const passStart = clock.now();
      let prev = last;
      while (true) {
        if (signal?.aborted) throw abortError();
        if (clock.now() - t0 > timeout) { reason = 'stall'; detail = 'timeout'; break; }
        const remaining = (target - last.turned) * dir;
        // stop early by what the robot turns before the stop lands
        const dt = Math.max(0.05, (last.t - prev.t) / 1000);
        const w = prev === last ? 0 : Math.abs(last.turned - prev.turned) / dt;
        if (remaining <= 0.5 + w * (dt * 0.75 + o.coastS)) break;
        // correction passes creep slower: they start close to the target
        const creep = passes > 1 ? Math.max(3, o.creepRpm / 2) : o.creepRpm;
        // slow down early enough for the poll interval: at 100 deg/s and a 300 ms
        // poll a fixed 40 deg zone is crossed in one sample, the stop then fires
        // 20 to 30 deg early and only correction passes can save the turn
        const slowDeg = Math.max(o.slowDeg, w * dt * 3);
        const rpm = Math.round(creep + (o.rpm - creep) * clamp((remaining - o.tolDeg) / slowDeg, 0, 1));
        const l = dir * spinSign * rpm, r = -dir * spinSign * rpm;
        const tick = clock.now();
        await send(l, r);
        prev = last;
        const s = await readSafe(l, r);
        if (signal?.aborted) throw abortError();
        if (!s) continue;
        const moved = s.turned - startTurned;
        if (!reversed && Math.abs(moved) >= o.reverseDeg && Math.sign(moved) !== dir) {
          // the wheels turn the robot the other way: flip the spin mapping
          spinSign = -spinSign;
          reversed = true;
          passes--; // the reversal does not count as a correction pass
          break;
        }
        // wheels lag at the start of a spin: judge stalls only after spin-up
        const c = clock.now() - passStart > o.spinUpMs ? detectCrash(samples, { ...o, straight: false, headingFrac: 0.25 }) : { crash: false };
        if (c.crash && c.reason !== 'heading') {
          reason = c.reason === 'stall' ? 'stall' : 'crash';
          detail = c.reason;
          details = { value: c.value, wheel: c.wheel, turned: s.turned };
          break;
        }
        const wait = 1000 / o.hz - (clock.now() - tick);
        if (wait > 1) await clock.sleep(wait);
      }
      await stop();
      await clock.sleep(o.settleMs);
      await readSafe(0, 0);
    }
    reason ??= 'done'; // errorDeg tells how close the last pass got
  } catch (e) {
    if (e?.name === 'AbortError') { reason = 'aborted'; note = cancelled ? e.message : 'aborted'; }
    else { reason = 'error'; note = e?.message ?? String(e); }
  } finally {
    if (reason !== 'nosensor') await stop();
  }
  return finish();

  function finish() {
    const achieved = last?.turned ?? 0;
    return {
      ok: reason === 'done', achievedDeg: Math.round(achieved * 10) / 10, targetDeg: target,
      errorDeg: Math.round((target - achieved) * 10) / 10,
      reason, detail, details, note, cancelled, spinSign, reversed, passes, samples,
    };
  }
}
