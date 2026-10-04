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
  slipMs: 800,        // window for slip: range ahead does not shrink while driving
  slipCm: 8,
  slipRatio: 0.15,
  slipMaxRangeCm: 150,
  joltMs2: 6,         // change of the acceleration vector between two samples
  shakeLimit: 40,     // cyberpi.get_shakeval() 0..100
  headingDeg: 12,     // yaw change not explained by the wheels
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

// Confirmed on firmware 44.01.013 (research/07): distance, yaw (clockwise
// positive, degrees), acceleration (m/s², z = -9.6 at rest), shake.
// Encoders exist but the port form is not confirmed yet, so they stay out:
// a wrong form reads 0 and would look like a stall on every leg.
export const BLE_SENSORS = {
  distance: SENSOR_EXPR.distance, yaw: SENSOR_EXPR.yaw,
  ax: SENSOR_EXPR.ax, ay: SENSOR_EXPR.ay, az: SENSOR_EXPR.az, shake: SENSOR_EXPR.shake,
};
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
export function parseSample(keys, values, { mirrored = true, swap = false } = {}) {
  const v = {};
  keys.forEach((k, i) => { v[k] = num(values?.[i]); });
  let encL = v.encL, encR = v.encR;
  if (swap) [encL, encR] = [encR, encL];
  if (mirrored && encR != null) encR = -encR;
  const s = { distanceCm: v.distance, encL, encR, yaw: v.yaw, shake: v.shake };
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
    return { t: performance.now(), ...parseSample(cur.keys, values, sensors) };
  };
  return sample;
}

// Sampler over SimRobot.sensorSample(), with a clock in simulated time so
// legs behave the same at any timeScale.
export function makeSimSampler(sim) {
  const sample = async () => sim.sensorSample();
  const k = () => sim.timeScale || 1;
  sample.clock = {
    now: () => performance.now() * k(),
    sleep: (ms) => new Promise((r) => setTimeout(r, ms / k())),
  };
  return sample;
}

const realClock = { now: () => performance.now(), sleep: (ms) => new Promise((r) => setTimeout(r, ms)) };

// Looks back over the sample history (newest last) for a crash or stall.
// Each entry: { t, drivenCm, cmdCm, distanceCm?, acc?, shake?, yawDelta?, encHeading?, hasEnc }.
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
  if (s.yawDelta != null) {
    const div = Math.abs(normDeg(s.yawDelta - (s.encHeading ?? 0)));
    if (div > o.headingDeg) return { crash: true, reason: 'heading', value: div };
  }
  const w = s.hasEnc && before(o.stallMs);
  if (w) {
    const want = s.cmdCm - w.cmdCm, got = s.drivenCm - w.drivenCm;
    if (want >= o.stallMinCm && got < o.stallRatio * want) return { crash: true, reason: 'stall', value: got / want };
  }
  const v = before(o.slipMs);
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
  stopAtCm = 20, clock, yawSign = 1, cmPerDeg = CM_PER_DEG, opts = {},
} = {}) {
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
  let reason = null, detail = null, note, contactCm = null, backedCm = 0;
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
      t, hasEnc, cmdCm,
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
    while (!reason) {
      const left = target - last.drivenCm;
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
        const c = detectCrash(samples, o);
        if (c.crash) { reason = c.reason === 'stall' ? 'stall' : 'crash'; detail = c.reason; contactCm = s.drivenCm; break; }
      }
      const wait = period - (clock.now() - tick);
      if (wait > 1) await clock.sleep(wait);
    }
    await stop();
    await readSafe(0);
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
    reason, detail, note, cancelled, contactCm, backedCm, samples,
    encHeading: last?.hasEnc ? last.encHeading : null,
    yawDelta: last?.yawDelta ?? null,
  };
}
