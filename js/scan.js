// Environment scan and navigation on top of the command bus.
// Angles are relative to the heading at scan start: 0 = straight ahead,
// positive = right / clockwise, normalised to -180..180.
// The ultrasonic sensor reports 300 cm when nothing is in range.

import { makeCommand as defaultMakeCommand } from './bus.js';

export const NO_ECHO_CM = 300;

export function normAngle(a) {
  const r = ((((a + 180) % 360) + 360) % 360) - 180;
  return r === -180 ? 180 : Math.round(r * 10) / 10 || 0;
}

function abortError(msg = 'scan aborted') {
  const e = new Error(msg);
  e.name = 'AbortError';
  return e;
}

// Resolves with p, or rejects as soon as the signal aborts.
function raceAbort(p, signal) {
  if (!signal) return p;
  if (signal.aborted) return Promise.reject(abortError());
  return new Promise((resolve, reject) => {
    const onAbort = () => reject(abortError());
    signal.addEventListener('abort', onAbort, { once: true });
    p.then(resolve, reject).finally(() => signal.removeEventListener('abort', onAbort));
  });
}

const sleep = (ms) => new Promise((r) => setTimeout(r, ms));

// Spins in place in `steps` equal turns, reading the distance before each turn.
// The last turn brings the robot back to the start heading.
export async function scan(bus, { steps = 12, onPoint, signal, settleMs = 120, makeCommand = defaultMakeCommand } = {}) {
  steps = Math.max(1, Math.round(steps));
  const stepDeg = 360 / steps;
  const startedAt = Date.now();
  const points = [];
  const guarded = async (p) => {
    try {
      return await raceAbort(p, signal);
    } catch (e) {
      if (e.name === 'AbortError') await bus.stop('agent');
      throw e;
    }
  };
  for (let i = 0; i < steps; i++) {
    if (signal?.aborted) await guarded(Promise.resolve());
    const r = await guarded(bus.submit(makeCommand('read', { sensor: 'distance' }, 'agent')));
    const cm = r.ok && Number.isFinite(Number(r.value)) ? Number(r.value) : null;
    const point = { angle: normAngle(i * stepDeg), cm };
    points.push(point);
    onPoint?.(point, i);
    const t = await guarded(bus.submit(makeCommand('turn', { deg: stepDeg, wait: true }, 'agent')));
    if (!t.ok) {
      await bus.stop('agent');
      throw new Error(`scan: turn ${i + 1}/${steps} failed (${t.error}); heading is now about ${normAngle(i * stepDeg)}° from start`);
    }
    if (settleMs > 0) await guarded(sleep(settleMs));
  }
  return { points, steps, startedAt, durationMs: Date.now() - startedAt };
}

const isOpen = (p, minCm) => p.cm != null && p.cm >= minCm;

// Groups circular runs of open points into openings, best first.
// Works with evenly spaced points (scan()) and with the uneven, denser points
// of sweepScan(): each point covers half the gap to each neighbour, capped so a
// hole in the data does not widen an opening much.
export function findOpenings(points, { minCm = 50 } = {}) {
  // In dense sweeps, empty bins and failed reads carry no information and must
  // not split an opening; in sparse step scans a missing reading stays "unknown".
  if (points.length >= 24) points = points.filter((p) => p.cm != null);
  const n = points.length;
  if (!n) return [];
  const pos = (a) => ((a % 360) + 360) % 360;
  const pts = [...points].sort((a, b) => pos(a.angle) - pos(b.angle));
  const maxHalf = Math.max(30, 720 / n) / 2;
  const half = pts.map((p, i) => {
    const gap = n === 1 ? 360 : pos(pts[(i + 1) % n].angle - p.angle);
    return Math.min(gap / 2, maxHalf);
  });
  // share of point i: half the gap behind it plus half the gap ahead
  const share = (i) => half[(i + n - 1) % n] + half[i];
  const score = (o) => {
    // width matters most (the robot must fit); depth beyond 150 cm adds nothing
    const w = Math.min(o.widthDeg, 120) / 120;
    const d = Math.min(o.cm, 150) / 150;
    const h = 1 - Math.abs(o.angle) / 180;
    return Math.round((0.55 * w + 0.35 * d + 0.1 * h) * 100) / 100;
  };
  const make = (run) => {
    const start = pos(pts[run[0]].angle);
    const span = pos(pts[run.at(-1)].angle - start);
    const o = {
      angle: normAngle(start + span / 2),
      widthDeg: Math.round(run.reduce((sum, i) => sum + share(i), 0)),
      cm: Math.min(...run.map((i) => pts[i].cm)),
    };
    o.score = score(o);
    return o;
  };
  if (pts.every((p) => isOpen(p, minCm))) {
    const o = { angle: 0, widthDeg: 360, cm: Math.min(...pts.map((p) => p.cm)) };
    return [{ ...o, score: score(o) }];
  }
  // start just after a closed point so no run is split by the wrap
  const first = pts.findIndex((p) => !isOpen(p, minCm));
  const out = [];
  let run = [];
  for (let k = 1; k <= n; k++) {
    const i = (first + k) % n;
    if (isOpen(pts[i], minCm)) run.push(i);
    else if (run.length) { out.push(make(run)); run = []; }
  }
  if (run.length) out.push(make(run));
  return out.sort((a, b) => b.score - a.score);
}

const fmtCm = (cm) => (cm == null ? '?' : cm >= NO_ECHO_CM ? '300+' : String(Math.round(cm)));

// Compact text for the LLM, e.g.
// "Scan (deg from heading, + = right/clockwise; cm): 0:85 30:40 ... Open: 60° w90 >=120cm; ..."
// Dense sweeps are summarised as at most 12 directions (nearest echo per 30°
// sector) so the text stays short; nearest and openings use every point.
export function describeScan(points, openings = findOpenings(points)) {
  const shown = points.length > 12 ? resampleSweep(points, 30) : points;
  const pts = shown.map((p) => `${p.angle}:${fmtCm(p.cm)}`).join(' ');
  const near = points.filter((p) => p.cm != null).sort((a, b) => a.cm - b.cm)[0];
  const open = openings.slice(0, 3).map((o) => `${o.angle}° w${o.widthDeg} >=${fmtCm(o.cm)}cm`).join('; ');
  return `Scan (deg from heading, + = right/clockwise; cm, 300+ = clear): ${pts}. `
    + (near ? `Nearest ${fmtCm(near.cm)}cm at ${near.angle}°. ` : '')
    + `Open: ${open || 'none'}.`;
}

// Turns to `angle` (relative to the current heading), re-measures, then drives
// at most min(cm, reading - 20, 100) cm. Never throws; reports what happened.
export async function driveToward(bus, { angle = 0, cm = 40, makeCommand = defaultMakeCommand, marginCm = 20 } = {}) {
  const out = { ok: false, turnedDeg: 0, readingCm: null, droveCm: 0 };
  const deg = Math.round(normAngle(angle));
  if (deg !== 0) {
    const t = await bus.submit(makeCommand('turn', { deg, wait: true }, 'agent'));
    if (!t.ok) return { ...out, error: `turn failed: ${t.error}` };
    out.turnedDeg = deg;
  }
  const r = await bus.submit(makeCommand('read', { sensor: 'distance' }, 'agent'));
  if (!r.ok) return { ...out, error: `distance read failed: ${r.error}` };
  out.readingCm = Number(r.value);
  const dist = Math.floor(Math.min(Number(cm) || 0, out.readingCm - marginCm, 100));
  if (dist < 5) return { ...out, ok: true, note: `blocked: ${fmtCm(out.readingCm)} cm ahead` };
  const s = await bus.submit(makeCommand('straight', { cm: dist, wait: true }, 'agent'));
  if (!s.ok) return { ...out, error: `straight failed: ${s.error}` };
  return { ...out, ok: true, droveCm: dist };
}

// Scan, head for the best opening, repeat. Returns a compact log.
export async function explore(bus, { maxMoves = 3, stepCm = 40, minCm = 50, steps = 12, signal, settleMs, onEvent, makeCommand = defaultMakeCommand } = {}) {
  const log = [];
  for (let i = 0; i < maxMoves; i++) {
    const { points } = await scan(bus, { steps, signal, settleMs, makeCommand });
    const best = findOpenings(points, { minCm })[0];
    if (!best) {
      log.push({ move: i + 1, note: 'no opening' });
      onEvent?.(log.at(-1));
      break;
    }
    if (signal?.aborted) { await bus.stop('agent'); throw abortError('explore aborted'); }
    const d = await driveToward(bus, { angle: best.angle, cm: stepCm, makeCommand });
    log.push({ move: i + 1, opening: { angle: best.angle, widthDeg: best.widthDeg, cm: best.cm }, ...d });
    onEvent?.(log.at(-1));
    if (!d.ok) break;
  }
  return { moves: log.length, log };
}

// --- continuous sweep ------------------------------------------------------
// Spins in place at a constant rate with streamed 'drive' commands and samples
// distance plus heading back to back (about 90 ms per sample on hardware), so a
// full turn yields 70 to 130 readings instead of 12 to 16.
// See research/09-sweep-scan.md.

export const WHEEL_DIAMETER_CM = 6.5;
export const TRACK_CM = 12;

// Wheel RPM for an in-place spin at degS. Each wheel moves at
// v = rpm / 60 * pi * D, and the robot turns at (vl - vr) / track = 2v / track rad/s.
export function spinRpmForRate(degS, { wheelCm = WHEEL_DIAMETER_CM, trackCm = TRACK_CM } = {}) {
  return (degS * trackCm * 60) / (360 * wheelCm);
}

// Robot rotation in degrees from wheel rotations in degrees (both forward-positive):
// (encL - encR) / 360 * pi * D / track rad = (encL - encR) * D / (2 * track) degrees.
export function encoderRotationDeg(encL, encR, { wheelCm = WHEEL_DIAMETER_CM, trackCm = TRACK_CM } = {}) {
  return ((encL - encR) * wheelCm) / (2 * trackCm);
}

// Resolves like p, or with onTimeout() after ms; the timer never outlives p.
function within(p, ms, onTimeout) {
  let timer;
  const t = new Promise((resolve, reject) => {
    timer = setTimeout(() => { try { resolve(onTimeout()); } catch (e) { reject(e); } }, ms);
  });
  return Promise.race([p, t]).finally(() => clearTimeout(timer));
}

const wrap180 = (d) => ((((d + 180) % 360) + 360) % 360) - 180;
const fin = (v) => v != null && Number.isFinite(Number(v));

// Linear interpolation of rotation at time t in a time-sorted track, with
// extrapolation from the last segment and clamping before the first entry.
function rotationAt(track, t) {
  if (!track.length) return 0;
  if (t <= track[0].t) return track[0].rot;
  let i = track.length - 1;
  while (i > 0 && track[i - 1].t >= t) i--;
  const a = track[Math.max(0, i - 1)], b = track[i];
  if (b.t === a.t) return b.rot;
  return a.rot + ((b.rot - a.rot) * (t - a.t)) / (b.t - a.t);
}

// Lower median: of two readings the nearer one, which is the safer guess.
const lowerMedian = (xs) => [...xs].sort((a, b) => a - b)[Math.floor((xs.length - 1) / 2)];

// Sorts by angle and merges readings closer than mergeDeg (also across ±180).
function mergePoints(points, mergeDeg) {
  const pts = [...points].sort((a, b) => a.angle - b.angle);
  const groups = [];
  for (const p of pts) {
    const g = groups.at(-1);
    if (g && p.angle - g[0].angle < mergeDeg) g.push(p);
    else groups.push([p]);
  }
  if (groups.length > 1 && groups[0][0].angle + 360 - groups.at(-1)[0].angle < mergeDeg) {
    groups[0] = [...groups.pop().map((p) => ({ ...p, angle: p.angle - 360 })), ...groups[0]];
  }
  return groups.map((g) => ({
    angle: normAngle(g.reduce((s, p) => s + p.angle, 0) / g.length),
    cm: lowerMedian(g.map((p) => p.cm)),
  })).sort((a, b) => a.angle - b.angle);
}

// Even bins of binDeg centred on 0, binDeg, ...; nearest echo per bin because
// the ultrasonic reports the nearest object anywhere in its cone. Empty bins
// get cm: null, so the spacing stays uniform for findOpenings and the map.
export function resampleSweep(points, binDeg = 3) {
  const n = Math.max(1, Math.round(360 / binDeg));
  const step = 360 / n;
  const bins = Array.from({ length: n }, () => null);
  for (const p of points ?? []) {
    if (p?.cm == null || !Number.isFinite(Number(p.cm))) continue;
    const k = ((Math.round(Number(p.angle) / step) % n) + n) % n;
    bins[k] = bins[k] == null ? Number(p.cm) : Math.min(bins[k], Number(p.cm));
  }
  return bins.map((cm, k) => ({ angle: normAngle(k * step), cm }));
}

// Continuous sweep, clockwise (negative speedDegS: counterclockwise). sample() resolves { t (performance.now ms at the
// reply), distanceCm?, yaw?, encL?, encR? }; missing fields are allowed.
// Rotation source, chosen from the first sample: gyro yaw (unwrapped, sign
// detected from the first signDetectDeg of motion), else wheel encoders, else
// commanded rate x time. Each distance is placed at the rotation interpolated
// at t - latencyMs (the ultrasonic value is older than the yaw read with it).
// Always stops the robot; abort and a bus stop throw AbortError.
// sample.clock { now, sleep } (as on makeSimSampler) replaces the wall clock,
// so a fast simulator keeps its time base. Samples are at least minSampleMs
// apart; the wait before each sample always yields to timers, so a sampler
// that answers at once cannot starve the drive stream.
export async function sweepScan(bus, {
  sample, makeCommand = defaultMakeCommand, signal, speedDegS = 45, onPoint, maxDurationMs = 20000,
  latencyMs = 45, targetDeg = 380, keepaliveMs = 300, rampMs = 300, signDetectDeg = 20,
  sampleTimeoutMs = 2000, minSampleMs = 0, mergeDeg = 1, minCm = 2, wheelCm = WHEEL_DIAMETER_CM, trackCm = TRACK_CM,
} = {}) {
  if (typeof sample !== 'function') throw new Error('sweepScan needs a sample() function');
  const geo = { wheelCm, trackCm };
  const dir = Math.sign(speedDegS) || 1; // +1 clockwise, -1 counterclockwise
  const fullRpm = Math.min(40, Math.max(5, spinRpmForRate(Math.abs(speedDegS), geo)));
  const rateOf = (rpm) => (rpm * 360 * wheelCm) / (60 * trackCm);
  const clock = sample.clock ?? { now: () => performance.now(), sleep };
  const now = () => clock.now();
  const t0 = now();

  // drive stream: [{ t, degS }] of what was commanded, for the time fallback
  const commanded = [];
  let rpmNow = null, running = true, driveError = null, wake = null;
  const nap = (ms) => new Promise((r) => { wake = r; clock.sleep(ms).then(r); });
  let remainingDeg = Infinity, streamer = Promise.resolve();
  // starts after the first sample, so the zero heading is read at rest
  const startStream = (tDrive = now()) => (streamer = (async () => {
    while (running) {
      const ramp = Math.min(1, (now() - tDrive) / Math.max(1, rampMs));
      // half speed at the start and for the last few degrees
      const rpm = Math.round(fullRpm * (ramp < 1 || remainingDeg < 15 ? 0.5 : 1) * 10) / 10;
      const r = await bus.submit(makeCommand('drive', { left: dir * rpm, right: -dir * rpm, leg: true }, 'agent'));
      if (!running) break;
      if (!r.ok) { driveError = r.error; commanded.push({ t: now(), degS: 0 }); break; }
      if (rpm !== rpmNow) commanded.push({ t: now(), degS: dir * rateOf(rpm) });
      rpmNow = rpm;
      await nap(ramp < 1 ? Math.min(keepaliveMs, rampMs / 2) : keepaliveMs);
    }
  })());

  const commandedRotation = (t) => {
    let rot = 0;
    for (let i = 0; i < commanded.length; i++) {
      const end = Math.min(t, commanded[i + 1]?.t ?? t);
      if (end > commanded[i].t) rot += commanded[i].degS * (end - commanded[i].t) / 1000;
    }
    return rot;
  };

  let method = null, sign = null, refSign = dir, n = 0, prevYaw = null, yawRaw = 0, enc0 = null;
  const track = [], pending = [], raw = [];
  let rotation = 0;
  const emit = (p) => {
    const angle = sign * rotationAt(track, p.t);
    const point = { angle: normAngle(angle), cm: p.cm };
    raw.push({ ...point, rot: angle });
    onPoint?.(point, raw.length - 1);
  };
  const ingest = (s) => {
    n++;
    const t = fin(s?.t) ? Number(s.t) : now();
    const hasEnc = fin(s?.encL) && fin(s?.encR);
    if (!method) {
      method = fin(s?.yaw) ? 'yaw' : hasEnc ? 'encoder' : 'time';
      // encoder and time are signed by construction; only the gyro sign is unknown
      if (method !== 'yaw') sign = 1;
    }
    let r = null;
    if (hasEnc) {
      enc0 ??= { l: Number(s.encL), r: Number(s.encR) };
      const er = encoderRotationDeg(Number(s.encL) - enc0.l, Number(s.encR) - enc0.r, geo);
      // encoders (sign fixed by the sampler) tell the true turn direction for the yaw check
      if (Math.abs(er) >= 5) refSign = Math.sign(er);
      if (method === 'encoder') r = er;
    }
    if (method === 'yaw' && fin(s?.yaw)) {
      const y = Number(s.yaw);
      if (prevYaw != null) yawRaw += wrap180(y - prevYaw);
      prevYaw = y;
      r = yawRaw;
    }
    if (method === 'time') r = commandedRotation(t);
    if (r != null) {
      track.push({ t, rot: r });
      if (sign == null && Math.abs(r) >= signDetectDeg) sign = Math.sign(r) * refSign;
      rotation = Math.max(rotation, Math.abs(r));
      remainingDeg = targetDeg - rotation;
    }
    if (fin(s?.distanceCm)) pending.push({ t: t - latencyMs, cm: Number(s.distanceCm) });
    if (sign != null) while (pending.length) emit(pending.shift());
  };

  let stopped = false;
  const halt = async (hard) => {
    running = false;
    wake?.();
    // let an in-flight drive land before the stop so nothing restarts the wheels
    const settled = await within(streamer.then(() => true), 1000, () => false);
    const stop = () => (hard ? bus.stop('agent') : bus.submit(makeCommand('stop', {}, 'agent')));
    await stop();
    if (!settled) streamer.then(stop);
    stopped = true;
  };

  try {
    if (signal?.aborted) throw abortError();
    let last = -Infinity;
    const next = async () => {
      const wait = last + minSampleMs - now();
      await raceAbort(clock.sleep(Math.max(0, wait)), signal);
      last = now();
      return raceAbort(within(Promise.resolve().then(sample), sampleTimeoutMs, () => { throw new Error('sweep: sample timeout'); }), signal);
    };
    ingest(await next());
    startStream();
    while (rotation < targetDeg && now() - t0 < maxDurationMs) {
      const s = await next();
      if (driveError) {
        throw /cancelled by stop|dropped by stop/.test(driveError) ? abortError('sweep cancelled by stop') : new Error(`sweep: drive failed (${driveError})`);
      }
      ingest(s);
    }
    await halt(false);
    // one reading at rest measures the coast after the stop, for turnedDeg
    try { ingest(await next()); } catch { /* the sweep itself is complete */ }
  } catch (e) {
    if (!stopped) await halt(true);
    throw e;
  }

  sign ??= 1;
  while (pending.length) emit(pending.shift());
  const valid = raw.filter((p) => p.cm > minCm);
  const points = mergePoints(valid, mergeDeg);
  const totalTurn = track.length ? sign * track.at(-1).rot : 0;
  return {
    points,
    durationMs: Math.round(now() - t0),
    method: 'sweep',
    rotationSource: method ?? 'time',
    yawSign: method === 'yaw' ? sign : null,
    // measured rotation from the start heading to rest, including the coast
    // after the stop (the robot is not turned back): turnedDeg -180..180 for
    // the pose heading, totalTurnDeg unwrapped (about 380 to 390)
    turnedDeg: normAngle(totalTurn),
    totalTurnDeg: Math.round(totalTurn * 10) / 10,
    samples: n,
    coverageDeg: Math.round(Math.min(360, rotation)),
  };
}
