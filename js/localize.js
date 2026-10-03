// Correlative scan matching on the occupancy grid: score how well a scan fits
// the map at a pose, search a window around a guess, search the whole map for
// a loaded map, and blend a match with odometry.
//
// Frames (docs/PLAN-v0.4.md): x right, y forward of the start, heading in
// degrees clockwise from +y, -180..180. A beam { angle, cm } is relative to
// the pose heading; the sensor sits sensorOffsetCm ahead of the centre along
// the beam direction dir = heading + angle, so a hit lies at
//   x + (sensorOffsetCm + cm) * sin(dir),  y + (sensorOffsetCm + cm) * cos(dir).
// The pose is the pose at scan start (scan() angles are relative to it). A
// match returns that pose in the map frame; the correction is match - guess.
//
// Score: sum over hit beams of a likelihood field (Gaussian of the distance
// from the endpoint to the nearest occupied cell), plus a small reward for
// no-echo beams whose ray stays clear of obstacles. Null beams are skipped.

import { L_THRESH } from './gridmap.js';

const SIGMA_CM = 8;
const UNKNOWN_FIELD = 0.1;   // endpoint in unknown space: no contradiction, little support
const NO_ECHO_WEIGHT = 0.3;  // full reward of a no-echo beam with a clear ray
const NO_ECHO_STEP_CM = 30;  // ray samples of a no-echo beam
const MIN_BEAMS = 4;         // fewer hit beams than this give confidence 0
const FIT_MIN = 0.2, FIT_GOOD = 0.6;   // mean score per beam mapped to 0..1
const MARGIN_GOOD = 0.25;    // relative lead over the runner-up for full confidence

const rad = (d) => (d * Math.PI) / 180;
const clamp01 = (v) => Math.max(0, Math.min(1, v));
export const normDeg = (a) => { const r = ((((a + 180) % 360) + 360) % 360) - 180; return r === -180 ? 180 : r; };
const angDiff = (a, b) => Math.abs(normDeg(a - b));

// Likelihood per cell, cached per map version in map.cache.
export function likelihoodField(map, sigmaCm = SIGMA_CM) {
  const key = `lf${sigmaCm}`;
  let f = map.cache.get(key);
  if (f) return f;
  const d = map.distanceField(L_THRESH);
  f = new Float32Array(d.length);
  const cap = 3 * sigmaCm;
  for (let k = 0; k < d.length; k++) {
    const g = d[k] < cap ? Math.exp(-(d[k] * d[k]) / (2 * sigmaCm * sigmaCm)) : 0;
    f[k] = Math.max(g, Math.abs(map.L[k]) <= L_THRESH ? UNKNOWN_FIELD : 0);
  }
  map.cache.set(key, f);
  return f;
}

// Bilinear field value at a world point (cell centres carry the values).
function sample(map, f, x, y) {
  const u = (x + map.half) / map.cellCm - 0.5, v = (y + map.half) / map.cellCm - 0.5;
  const i = Math.floor(u), j = Math.floor(v), n = map.n;
  if (i < 0 || j < 0 || i + 1 >= n || j + 1 >= n) return 0;
  const fx = u - i, fy = v - j, k = j * n + i;
  return (f[k] * (1 - fx) + f[k + 1] * fx) * (1 - fy) + (f[k + n] * (1 - fx) + f[k + n + 1] * fx) * fy;
}

// Beams split into hits (one endpoint radius) and no-echo rays (sample radii).
function prepare(points, { sensorOffsetCm = 6, maxRangeCm = 250 } = {}) {
  const hits = [], rays = [];
  for (const p of points ?? []) {
    const cm = p?.cm == null ? NaN : Number(p.cm);
    if (!Number.isFinite(cm) || cm < 0) continue;
    const a = Number(p.angle) || 0;
    if (cm < maxRangeCm) hits.push({ a, r: sensorOffsetCm + cm });
    else {
      const rs = [];
      for (let t = NO_ECHO_STEP_CM; t <= maxRangeCm; t += NO_ECHO_STEP_CM) rs.push(sensorOffsetCm + t);
      rays.push({ a, rs });
    }
  }
  return { hits, rays, max: hits.length + NO_ECHO_WEIGHT * rays.length };
}

// Beam offsets from the robot centre for one heading.
function offsets(beams, heading) {
  const hit = beams.hits.map((b) => {
    const t = rad(heading + b.a);
    return [Math.sin(t) * b.r, Math.cos(t) * b.r];
  });
  const ray = beams.rays.map((b) => {
    const t = rad(heading + b.a), s = Math.sin(t), c = Math.cos(t);
    return b.rs.map((r) => [s * r, c * r]);
  });
  return { hit, ray };
}

function scoreAt(map, f, off, x, y) {
  let s = 0;
  for (const [dx, dy] of off.hit) s += sample(map, f, x + dx, y + dy);
  for (const r of off.ray) {
    let worst = 0;
    for (const [dx, dy] of r) worst = Math.max(worst, sample(map, f, x + dx, y + dy));
    s += NO_ECHO_WEIGHT * (1 - worst);
  }
  return s;
}

function confidence(beams, best, runner) {
  if (beams.hits.length < MIN_BEAMS || !(best > 0)) return 0;
  const fit = clamp01((best / beams.max - FIT_MIN) / (FIT_GOOD - FIT_MIN));
  const margin = clamp01((best - Math.max(0, runner ?? 0)) / best / MARGIN_GOOD);
  return Math.round(fit * margin * 1000) / 1000;
}

// Likelihood that the beam endpoints at pose hit occupied cells (sum over beams).
export function scoreScan(map, pose, points, opts = {}) {
  const beams = prepare(points, opts);
  const f = likelihoodField(map, opts.sigmaCm ?? SIGMA_CM);
  return scoreAt(map, f, offsets(beams, pose.heading ?? 0), pose.x, pose.y);
}

// Brute force over a box around centre, then two halving refinements around
// the best pose. Returns the best and the best pose distinct from it.
function search(map, f, beams, c, { xyWindowCm, xyStepCm, angWindowDeg, angStepDeg, distinctCm, distinctDeg }) {
  const cands = [];
  const nA = Math.max(0, Math.round(angWindowDeg / angStepDeg));
  const nX = Math.max(0, Math.round(xyWindowCm / xyStepCm));
  for (let a = -nA; a <= nA; a++) {
    const h = normDeg(c.heading + a * angStepDeg);
    const off = offsets(beams, h);
    for (let j = -nX; j <= nX; j++) {
      for (let i = -nX; i <= nX; i++) {
        const x = c.x + i * xyStepCm, y = c.y + j * xyStepCm;
        cands.push({ x, y, heading: h, score: scoreAt(map, f, off, x, y) });
      }
    }
  }
  let best = cands[0];
  for (const q of cands) if (q.score > best.score) best = q;
  let runner = null;
  for (const q of cands) {
    if (Math.hypot(q.x - best.x, q.y - best.y) <= distinctCm && angDiff(q.heading, best.heading) <= distinctDeg) continue;
    if (!runner || q.score > runner.score) runner = q;
  }
  let st = xyStepCm / 2, sa = angStepDeg / 2;
  for (let r = 0; r < 2; r++, st /= 2, sa /= 2) {
    const c0 = best;
    for (let a = -1; a <= 1; a++) {
      const h = normDeg(c0.heading + a * sa);
      const off = offsets(beams, h);
      for (let j = -1; j <= 1; j++) {
        for (let i = -1; i <= 1; i++) {
          const x = c0.x + i * st, y = c0.y + j * st, s = scoreAt(map, f, off, x, y);
          if (s > best.score) best = { x, y, heading: h, score: s };
        }
      }
    }
  }
  return { best, runner };
}

const roundPose = (p) => ({ x: Math.round(p.x * 10) / 10, y: Math.round(p.y * 10) / 10, heading: Math.round(normDeg(p.heading) * 10) / 10 });

// Searches a window around guess. confidence 0..1 combines the fit (mean
// score per beam) and the lead over the best pose more than distinctCm or
// distinctDeg away inside the window (low in corridors and featureless walls).
export function matchScan(map, guess, points, { xyWindowCm = 40, xyStepCm = 5, angWindowDeg = 20, angStepDeg = 2,
  distinctCm = 20, distinctDeg = 20, sigmaCm = SIGMA_CM, ...beamOpts } = {}) {
  const beams = prepare(points, beamOpts);
  const g = { x: guess.x, y: guess.y, heading: guess.heading ?? 0 };
  if (!beams.hits.length) return { pose: roundPose(g), score: 0, confidence: 0 };
  const f = likelihoodField(map, sigmaCm);
  const { best, runner } = search(map, f, beams, g, { xyWindowCm, xyStepCm, angWindowDeg, angStepDeg, distinctCm, distinctDeg });
  return { pose: roundPose(best), score: best.score, confidence: confidence(beams, best.score, runner?.score) };
}

// Global search for a loaded map: every free cell with room for the robot
// (stride xyStepCm) at every angStepDeg, nearest-cell lookups; the best
// distinct candidates are refined with a local search. confidence compares
// the winner with the best pose more than distinctCm or distinctDeg away.
export function relocalize(map, points, { angStepDeg = 5, xyStepCm = 10, topK = 16, distinctCm = 30, distinctDeg = 30,
  minClearanceCm = 8, sigmaCm = SIGMA_CM, ...beamOpts } = {}) {
  const beams = prepare(points, beamOpts);
  const none = { pose: null, score: 0, confidence: 0, runnerUp: null };
  if (!beams.hits.length) return none;
  const f = likelihoodField(map, sigmaCm);
  const df = map.distanceField(L_THRESH);
  const n = map.n, c = map.cellCm, stride = Math.max(1, Math.round(xyStepCm / c));
  const pos = [];
  for (let j = 0; j < n; j += stride) {
    for (let i = 0; i < n; i += stride) {
      const k = j * n + i;
      if (map.L[k] < -L_THRESH && df[k] >= minClearanceCm) pos.push(k);
    }
  }
  if (!pos.length) return none;
  // coarse pass: nearest-cell lookups with flat index offsets per angle;
  // positions whose offsets could leave the grid take the checked path
  const nAng = Math.max(1, Math.round(360 / angStepDeg)), angDeg = 360 / nAng;
  const nHit = beams.hits.length, nRay = beams.rays.length, nS = nRay ? beams.rays[0].rs.length : 0;
  const tables = [];
  for (let a = 0; a < nAng; a++) {
    const off = offsets(beams, a * angDeg);
    const di = new Int32Array(nHit + nRay * nS), dj = new Int32Array(di.length);
    let q = 0;
    for (const [dx, dy] of off.hit) { di[q] = Math.round(dx / c); dj[q++] = Math.round(dy / c); }
    for (const r of off.ray) for (const [dx, dy] of r) { di[q] = Math.round(dx / c); dj[q++] = Math.round(dy / c); }
    const flat = new Int32Array(di.length);
    let reach = 0;
    for (let t = 0; t < di.length; t++) { flat[t] = dj[t] * n + di[t]; reach = Math.max(reach, Math.abs(di[t]), Math.abs(dj[t])); }
    tables.push({ di, dj, flat, reach });
  }
  const sc = new Float32Array(nAng);
  const perPos = []; // per position: the best angle and the best one distinct from it
  for (const k of pos) {
    const i = k % n, j = (k - i) / n;
    let b1 = 0;
    for (let a = 0; a < nAng; a++) {
      const { di, dj, flat, reach } = tables[a];
      let s = 0, t = 0;
      if (i - reach >= 0 && j - reach >= 0 && i + reach < n && j + reach < n) {
        for (; t < nHit; t++) s += f[k + flat[t]];
        for (let r = 0; r < nRay; r++) {
          let w = 0;
          for (let e = 0; e < nS; e++, t++) { const v = f[k + flat[t]]; if (v > w) w = v; }
          s += NO_ECHO_WEIGHT * (1 - w);
        }
      } else {
        const at = (q) => {
          const ii = i + di[q], jj = j + dj[q];
          return ii < 0 || jj < 0 || ii >= n || jj >= n ? 0 : f[jj * n + ii];
        };
        for (; t < nHit; t++) s += at(t);
        for (let r = 0; r < nRay; r++) {
          let w = 0;
          for (let e = 0; e < nS; e++, t++) { const v = at(t); if (v > w) w = v; }
          s += NO_ECHO_WEIGHT * (1 - w);
        }
      }
      sc[a] = s;
      if (s > sc[b1]) b1 = a;
    }
    let b2 = -1;
    for (let a = 0; a < nAng; a++) {
      if (angDiff(a * angDeg, b1 * angDeg) > distinctDeg && (b2 < 0 || sc[a] > sc[b2])) b2 = a;
    }
    const ctr = map.centre(k);
    perPos.push({ x: ctr.x, y: ctr.y, heading: normDeg(b1 * angDeg), score: sc[b1] });
    if (b2 >= 0) perPos.push({ x: ctr.x, y: ctr.y, heading: normDeg(b2 * angDeg), score: sc[b2] });
  }
  perPos.sort((p, q) => q.score - p.score);
  // greedy non-maximum suppression, then local refinement
  const seeds = [];
  for (const p of perPos) {
    if (seeds.length >= topK) break;
    if (seeds.some((s) => Math.hypot(s.x - p.x, s.y - p.y) <= distinctCm && angDiff(s.heading, p.heading) <= distinctDeg)) continue;
    seeds.push(p);
  }
  const win = { xyWindowCm: stride * c, xyStepCm: c, angWindowDeg: angStepDeg, angStepDeg: 1, distinctCm, distinctDeg };
  const refined = seeds.map((s) => search(map, f, beams, s, win).best).sort((p, q) => q.score - p.score);
  const best = refined[0];
  const runner = refined.find((q) => Math.hypot(q.x - best.x, q.y - best.y) > distinctCm || angDiff(q.heading, best.heading) > distinctDeg) ?? null;
  return {
    pose: roundPose(best),
    score: best.score,
    confidence: confidence(beams, best.score, runner?.score),
    runnerUp: runner ? { pose: roundPose(runner), score: runner.score } : null,
  };
}

// Blends odometry with a match. yawDeg (gyro heading in the map frame), when
// given, sets the heading and vetoes matches whose heading disagrees by more
// than yawTolDeg. Matches below minConfidence are rejected. The match weight
// is confidence * (1 - odomWeight). source: 'scan' when the match was used,
// 'odom' when it was rejected (reason says why); yaw: true if the gyro set the heading.
export function fusePose(odomPose, matchResult, { yawDeg, odomWeight = 0.2, minConfidence = 0.4, yawTolDeg = 20 } = {}) {
  const hasYaw = Number.isFinite(yawDeg);
  const base = { x: odomPose.x, y: odomPose.y, heading: normDeg(hasYaw ? yawDeg : odomPose.heading ?? 0) };
  const odo = (reason) => ({ pose: base, source: 'odom', yaw: hasYaw, weight: 0, reason });
  const m = matchResult?.pose;
  if (!m) return odo('no match');
  const conf = Number(matchResult.confidence) || 0;
  if (conf < minConfidence) return odo(`low confidence ${conf.toFixed(2)}`);
  if (hasYaw && angDiff(m.heading, yawDeg) > yawTolDeg) return odo('match heading disagrees with the gyro');
  const w = clamp01(conf) * clamp01(1 - odomWeight);
  const heading = hasYaw ? base.heading : normDeg(base.heading + w * normDeg(m.heading - base.heading));
  return {
    pose: { x: base.x + w * (m.x - base.x), y: base.y + w * (m.y - base.y), heading },
    source: 'scan',
    yaw: hasYaw,
    weight: w,
  };
}
