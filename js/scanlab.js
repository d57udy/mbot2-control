// Scan-Labor: diagnose sweep scans on the real robot from raw readings.
// Each sweep keeps its sample log ({ t ms, yaw deg, cm }); angles can then be
// recomputed for any assumed sensor latency, so a clockwise and a
// counterclockwise sweep from the same spot reveal the timing error: the
// latency at which both agree best.

const norm = (a) => { const r = ((((a + 180) % 360) + 360) % 360) - 180; return r === -180 ? 180 : r; };
const wrap180 = (d) => ((d + 540) % 360) - 180;

// Points { angle, cm } from a sample log, assuming the distance value is
// latencyMs older than the reply that carried it. Angle 0 = heading at the
// first sample, clockwise positive (the sampler applies the gyro calibration).
// refYaw: express angles relative to this gyro heading instead (to overlay
// several sweeps in one frame, e.g. the first sweep's start heading).
export function pointsFromLog(log, latencyMs = 45, { maxCm = 300, refYaw = null } = {}) {
  const track = [];
  let rot = 0, prev = null;
  for (const s of log) {
    if (!Number.isFinite(s.yaw)) continue;
    if (prev != null) rot += wrap180(s.yaw - prev);
    prev = s.yaw;
    track.push({ t: s.t, rot });
  }
  if (track.length < 2) return [];
  const first = log.find((s) => Number.isFinite(s.yaw))?.yaw ?? 0;
  const offset = refYaw == null ? 0 : wrap180(first - refYaw);
  const rotAt = (t) => {
    if (t <= track[0].t) return track[0].rot;
    for (let i = 1; i < track.length; i++) {
      const a = track[i - 1], b = track[i];
      if (t <= b.t) return a.rot + ((b.rot - a.rot) * (t - a.t)) / Math.max(1, b.t - a.t);
    }
    return track.at(-1).rot;
  };
  const pts = [];
  for (const s of log) {
    if (!Number.isFinite(s.cm) || s.cm <= 2) continue;
    pts.push({ angle: norm(rotAt(s.t - latencyMs) + offset), cm: Math.min(s.cm, maxCm) });
  }
  return pts;
}

// Nearest echo per angular bin, 360 / binDeg bins starting at -180.
export function profile(points, binDeg = 2, rangeCm = 150) {
  const n = Math.round(360 / binDeg);
  const bins = new Array(n).fill(null);
  for (const p of points) {
    const i = Math.min(n - 1, Math.floor((norm(p.angle) + 180) / binDeg));
    const cm = p.cm >= rangeCm ? null : p.cm;
    if (cm != null && (bins[i] == null || cm < bins[i])) bins[i] = cm;
  }
  return bins;
}

// Mean absolute difference over bins where both profiles see something.
export function mismatch(a, b) {
  let sum = 0, k = 0;
  for (let i = 0; i < a.length; i++) {
    if (a[i] != null && b[i] != null) { sum += Math.abs(a[i] - b[i]); k++; }
  }
  return k >= 8 ? sum / k : Infinity;
}

// Best latency for a clockwise and a counterclockwise sweep from one spot.
export function estimateLatency(cwLog, ccwLog, { fromMs = -100, toMs = 400, stepMs = 10, binDeg = 2, rangeCm = 150 } = {}) {
  let best = null;
  const curve = [];
  for (let L = fromMs; L <= toMs; L += stepMs) {
    const m = mismatch(profile(pointsFromLog(cwLog, L), binDeg, rangeCm), profile(pointsFromLog(ccwLog, L), binDeg, rangeCm));
    curve.push({ latencyMs: L, mismatchCm: Number.isFinite(m) ? m : null });
    if (Number.isFinite(m) && (!best || m < best.mismatchCm)) best = { latencyMs: L, mismatchCm: m };
  }
  if (best) {
    // the minimum is a plateau with coarse samples: take the middle of it
    const flat = curve.filter((c) => c.mismatchCm != null && c.mismatchCm <= best.mismatchCm + 0.5).map((c) => c.latencyMs);
    best = { latencyMs: flat[Math.floor(flat.length / 2)], mismatchCm: Math.round(best.mismatchCm * 10) / 10 };
  }
  return { best, curve: curve.map((c) => ({ ...c, mismatchCm: c.mismatchCm == null ? null : Math.round(c.mismatchCm * 10) / 10 })) };
}

// Bottle test with a cw/ccw pair: computed without latency correction the post
// appears shifted by +rate*L clockwise and -rate*L counterclockwise, so
// L = (angleCw - angleCcw) / 2 / rate. Rate from the gyro track of each sweep.
export function postLatency(cwLog, ccwLog, opts) {
  const rate = (log) => {
    const ys = log.filter((s) => Number.isFinite(s.yaw));
    let rot = 0;
    for (let i = 1; i < ys.length; i++) rot += wrap180(ys[i].yaw - ys[i - 1].yaw);
    const ms = ys.length > 1 ? ys.at(-1).t - ys[0].t : 0;
    return ms > 0 ? Math.abs(rot) / (ms / 1000) : null;
  };
  const a = findPost(pointsFromLog(cwLog, 0), opts), b = findPost(pointsFromLog(ccwLog, 0), opts);
  const r = ((rate(cwLog) ?? 0) + (rate(ccwLog) ?? 0)) / 2;
  if (!a || !b || !r) return null;
  return { latencyMs: Math.round((((a.angle - b.angle) / 2) / r) * 1000), cw: a, ccw: b, rateDegS: Math.round(r) };
}

// Bottle test: the nearest object within ±maxDeg of straight ahead, its
// angle (centre of the near cluster) and angular width.
export function findPost(points, { maxDeg = 60, rangeCm = 150, bandCm = 10 } = {}) {
  const front = points.filter((p) => Math.abs(norm(p.angle)) <= maxDeg && p.cm < rangeCm);
  if (!front.length) return null;
  const minCm = Math.min(...front.map((p) => p.cm));
  const near = front.filter((p) => p.cm <= minCm + bandCm).map((p) => norm(p.angle)).sort((a, b) => a - b);
  const centre = near.reduce((s, a) => s + a, 0) / near.length;
  return { angle: Math.round(centre * 10) / 10, cm: Math.round(minCm), widthDeg: Math.round(near.at(-1) - near[0]), n: near.length };
}

// Polar overlay of several sweeps, robot at the centre, forward up.
export const LAB_COLORS = ['#2f6fde', '#d92d20', '#12805c', '#b54708'];
export function drawOverlay(canvas, scans, { maxCm = 200, rangeCm = 150 } = {}) {
  const ctx = canvas.getContext?.('2d');
  if (!ctx) return;
  const css = typeof getComputedStyle === 'function' ? getComputedStyle(document.documentElement) : null;
  const col = (name, d) => css?.getPropertyValue(name)?.trim() || d;
  const w = canvas.width, h = canvas.height, cx = w / 2, cy = h / 2, k = (Math.min(w, h) / 2 - 10) / maxCm;
  ctx.clearRect(0, 0, w, h);
  ctx.strokeStyle = col('--border', '#ccc');
  ctx.fillStyle = col('--muted', '#888');
  ctx.font = `${Math.round(w / 40)}px system-ui`;
  for (const r of [50, 100, 150, 200]) {
    ctx.beginPath(); ctx.arc(cx, cy, r * k, 0, Math.PI * 2); ctx.stroke();
    ctx.fillText(`${r}`, cx + 3, cy - r * k + 12);
  }
  ctx.beginPath(); ctx.moveTo(cx, cy); ctx.lineTo(cx, cy - maxCm * k); ctx.stroke(); // forward
  scans.forEach((s, i) => {
    ctx.fillStyle = s.color ?? LAB_COLORS[i % LAB_COLORS.length];
    for (const p of s.points) {
      if (p.cm >= rangeCm) continue;
      const a = (p.angle * Math.PI) / 180;
      ctx.beginPath();
      ctx.arc(cx + Math.sin(a) * p.cm * k, cy - Math.cos(a) * p.cm * k, Math.max(2, w / 200), 0, Math.PI * 2);
      ctx.fill();
    }
  });
}
