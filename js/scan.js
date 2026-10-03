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
// Assumes evenly spaced points, as produced by scan().
export function findOpenings(points, { minCm = 50 } = {}) {
  const n = points.length;
  if (!n) return [];
  const pos = (a) => ((a % 360) + 360) % 360;
  const pts = [...points].sort((a, b) => pos(a.angle) - pos(b.angle));
  const step = 360 / n;
  const score = (o) => {
    // width matters most (the robot must fit); depth beyond 150 cm adds nothing
    const w = Math.min(o.widthDeg, 120) / 120;
    const d = Math.min(o.cm, 150) / 150;
    const h = 1 - Math.abs(o.angle) / 180;
    return Math.round((0.55 * w + 0.35 * d + 0.1 * h) * 100) / 100;
  };
  const make = (run) => {
    const start = pos(run[0].angle);
    const o = {
      angle: normAngle(start + ((run.length - 1) * step) / 2),
      widthDeg: Math.round(run.length * step),
      cm: Math.min(...run.map((p) => p.cm)),
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
    const p = pts[(first + k) % n];
    if (isOpen(p, minCm)) run.push(p);
    else if (run.length) { out.push(make(run)); run = []; }
  }
  if (run.length) out.push(make(run));
  return out.sort((a, b) => b.score - a.score);
}

const fmtCm = (cm) => (cm == null ? '?' : cm >= NO_ECHO_CM ? '300+' : String(Math.round(cm)));

// Compact text for the LLM, e.g.
// "Scan (deg from heading, + = right/clockwise; cm): 0:85 30:40 ... Open: 60° w90 >=120cm; ..."
export function describeScan(points, openings = findOpenings(points)) {
  const pts = points.map((p) => `${p.angle}:${fmtCm(p.cm)}`).join(' ');
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
