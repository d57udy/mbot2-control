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

import { L_THRESH, L_SUSPECT } from './gridmap.js';

const SIGMA_CM = 8;
const SUSPECT_FIELD = 0.9;   // weight of suspect (unconfirmed) obstacles: most of a fresh map
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

// Likelihood per cell, cached per map version in map.cache. Confirmed
// obstacles count fully, suspect ones (not confirmed yet) SUSPECT_FIELD.
export function likelihoodField(map, sigmaCm = SIGMA_CM) {
  const key = `lf${sigmaCm}`;
  let f = map.cache.get(key);
  if (f) return f;
  const d = map.distanceField(L_THRESH), ds = map.distanceField(L_SUSPECT);
  f = new Float32Array(d.length);
  const cap = 3 * sigmaCm, g = (x) => (x < cap ? Math.exp(-(x * x) / (2 * sigmaCm * sigmaCm)) : 0);
  for (let k = 0; k < d.length; k++) {
    f[k] = Math.max(g(d[k]), SUSPECT_FIELD * g(ds[k]), Math.abs(map.L[k]) <= L_THRESH ? UNKNOWN_FIELD : 0);
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

// Matching field (cached per map version): f = Gaussian of the distance to
// the nearest confirmed cell, suspect cells at MATCH_SUSPECT; ll = tempered
// log(floor + (1 - floor) f), the per-reading log-likelihood with a uniform
// floor for random readings and new obstacles (a bounded, robust cost),
// with f taken relative to the map's strongest evidence.
const MATCH_FLOOR = 0.1;
const MATCH_SUSPECT = 0.4;   // suspect cells: a duplicate from one bad sweep must not attract matches
const TEMPER = 0.8;          // neighbouring readings share overlapping cones: temper their product (calibrated: sigma ~ RMS error in the sim)
const AXIS_CM = 9, AXIS_DEG = 5; // a component is determined if its sigma (tempered surface) is within this
function matchField(map, sigmaCm) {
  const key = `mf${sigmaCm}`;
  let m = map.cache.get(key);
  if (m) return m;
  const d = map.distanceField(L_THRESH), ds = map.distanceField(L_SUSPECT);
  const f = new Float32Array(d.length), ll = new Float32Array(d.length);
  const cap = 3 * sigmaCm, g = (x) => (x < cap ? Math.exp(-(x * x) / (2 * sigmaCm * sigmaCm)) : 0);
  let peak = 0;
  for (let k = 0; k < d.length; k++) {
    f[k] = Math.max(g(d[k]), MATCH_SUSPECT * g(ds[k]));
    if (f[k] > peak) peak = f[k];
  }
  // relative to the strongest evidence: a map of suspect cells only (a fresh
  // map, an anchor scan) matches as sharply as a confirmed one
  for (let k = 0; k < d.length; k++) ll[k] = TEMPER * Math.log(MATCH_FLOOR + (1 - MATCH_FLOOR) * (peak ? f[k] / peak : 0));
  m = { f, ll, peak, low: TEMPER * Math.log(MATCH_FLOOR) };
  map.cache.set(key, m);
  return m;
}

// Valid readings only: no echo (>= maxRangeCm) and failed reads say nothing
// (specular dropouts are not free space).
function validBeams(points, { sensorOffsetCm = 6, maxRangeCm = 150, minCm = 2 } = {}) {
  const out = [];
  for (const p of points ?? []) {
    const cm = p?.cm == null ? NaN : Number(p.cm);
    if (Number.isFinite(cm) && cm >= minCm && cm < maxRangeCm) out.push({ a: Number(p.angle) || 0, r: sensorOffsetCm + cm });
  }
  return out;
}

// Correlative scan-to-map matching (Olson 2009, with Karto's odometry prior).
// Every pose of the window (xyStepCm, angStepDeg) is scored with integer
// lookups; the best is refined with bilinear lookups. The score surface,
// as probabilities exp(score - best), gives the pose covariance, which
// captures both noise and ambiguity: a corridor or a door wall gives a long
// ellipse (along-track undetermined) while heading and cross-track stay
// tight. Returns
//   { pose, score, confidence, cov: { xx, xy, yy, hh, major, minor, axisDeg, sigmaDeg },
//     axes: { heading, cross, along }, valid, support, fit, runnerUp, reason? }
// sigmas in cm / deg; axisDeg is the direction of the major (least certain)
// axis, clockwise from +y; axes say which components are well determined.
// confidence (0..1) is high only when all three are. The prior penalises
// distance from the guess (sigmas priorCm / priorDeg, default a third of the
// window) with Karto-style floors (0.5 / 0.9), so it breaks ties without
// overruling the sensor.
export function matchScan(map, guess, points, { xyWindowCm = 40, xyStepCm = 5, angWindowDeg = 20, angStepDeg = 2,
  sigmaCm = SIGMA_CM, minValid = 8, priorCm, priorDeg, nmsCm = 15, nmsDeg = 5, ...beamOpts } = {}) {
  const g = { x: guess.x, y: guess.y, heading: guess.heading ?? 0 };
  const beams = validBeams(points, beamOpts);
  const none = (reason) => ({ pose: roundPose(g), score: 0, confidence: 0, cov: null, axes: { heading: false, cross: false, along: false }, valid: beams.length, support: 0, fit: 0, runnerUp: null, reason });
  if (beams.length < Math.max(1, minValid)) return none('few readings');
  const { f, ll, peak, low } = matchField(map, sigmaCm);
  if (!(peak > 0)) return none('empty map');
  const n = map.n, cell = map.cellCm, half = map.half;
  const step = Math.max(1, Math.round(xyStepCm / cell));          // in cells
  const nX = Math.max(1, Math.round(xyWindowCm / (step * cell)));
  const nA = Math.max(1, Math.round(angWindowDeg / angStepDeg));
  const pCm = priorCm ?? Math.max(5, xyWindowCm / 3), pDeg = priorDeg ?? Math.max(2, angWindowDeg / 3);
  const W = 2 * nX + 1, A = 2 * nA + 1;
  const S = new Float32Array(A * W * W);
  const gi = Math.floor((g.x + half) / cell), gj = Math.floor((g.y + half) / cell);
  const fx0 = g.x - ((gi + 0.5) * cell - half), fy0 = g.y - ((gj + 0.5) * cell - half); // guess offset in its cell
  let bestK = 0, bestS = -Infinity;
  const di = new Int32Array(beams.length), dj = new Int32Array(beams.length);
  for (let a = -nA; a <= nA; a++) {
    const h = g.heading + a * angStepDeg;
    for (let b = 0; b < beams.length; b++) {
      const t = rad(h + beams[b].a);
      di[b] = Math.round((fx0 + beams[b].r * Math.sin(t)) / cell);
      dj[b] = Math.round((fy0 + beams[b].r * Math.cos(t)) / cell);
    }
    const priorA = Math.log(Math.max(0.9, Math.exp(-0.5 * ((a * angStepDeg) / pDeg) ** 2)));
    for (let j = -nX; j <= nX; j++) {
      for (let i = -nX; i <= nX; i++) {
        const ci = gi + i * step, cj = gj + j * step;
        let sc = 0;
        for (let b = 0; b < beams.length; b++) {
          const ii = ci + di[b], jj = cj + dj[b];
          sc += ii < 0 || jj < 0 || ii >= n || jj >= n ? low : ll[jj * n + ii];
        }
        const dcm = Math.hypot(i, j) * step * cell;
        sc += priorA + Math.log(Math.max(0.5, Math.exp(-0.5 * (dcm / pCm) ** 2)));
        const k = ((a + nA) * W + (j + nX)) * W + (i + nX);
        S[k] = sc;
        if (sc > bestS) { bestS = sc; bestK = k; }
      }
    }
  }
  const unpack = (k) => {
    const i = (k % W) - nX, j = (Math.floor(k / W) % W) - nX, a = Math.floor(k / (W * W)) - nA;
    return { dx: i * step * cell, dy: j * step * cell, dh: a * angStepDeg };
  };
  // covariance of the score surface (Olson): p = exp(s - best)
  let sw = 0, mx = 0, my = 0, mh = 0, xx = 0, xy = 0, yy = 0, hh = 0, runner = null;
  const b0 = unpack(bestK);
  for (let k = 0; k < S.length; k++) {
    const u = unpack(k), w = Math.exp(S[k] - bestS);
    sw += w; mx += w * u.dx; my += w * u.dy; mh += w * u.dh;
    xx += w * u.dx * u.dx; xy += w * u.dx * u.dy; yy += w * u.dy * u.dy; hh += w * u.dh * u.dh;
    // runner-up: best pose outside the suppression neighbourhood of the peak
    if ((Math.abs(u.dx - b0.dx) > nmsCm || Math.abs(u.dy - b0.dy) > nmsCm || Math.abs(u.dh - b0.dh) > nmsDeg) && (!runner || S[k] > runner.s)) runner = { s: S[k], ...u };
  }
  mx /= sw; my /= sw; mh /= sw;
  const minVar = (step * cell) ** 2 / 4, minH = angStepDeg ** 2 / 4;
  const cxx = xx / sw - mx * mx + minVar, cyy = yy / sw - my * my + minVar, cxy = xy / sw - mx * my, chh = hh / sw - mh * mh + minH;
  const tr = cxx + cyy, det = cxx * cyy - cxy * cxy, disc = Math.sqrt(Math.max(0, (tr * tr) / 4 - det));
  const major = Math.sqrt(tr / 2 + disc), minor = Math.sqrt(Math.max(0, tr / 2 - disc));
  // major axis direction (clockwise from +y, i.e. atan2(x, y))
  const ex = Math.abs(cxy) > 1e-9 ? tr / 2 + disc - cyy : cxx >= cyy ? 1 : 0, ey = Math.abs(cxy) > 1e-9 ? cxy : cxx >= cyy ? 0 : 1;
  const axisDeg = normDeg((Math.atan2(ex, ey) * 180) / Math.PI);
  const sigmaDeg = Math.sqrt(chh);
  // refine the peak with bilinear lookups
  let best = { x: g.x + b0.dx, y: g.y + b0.dy, heading: g.heading + b0.dh };
  const llAt = (x, y) => TEMPER * Math.log(MATCH_FLOOR + (1 - MATCH_FLOOR) * sample(map, f, x, y) / peak);
  const scoreAtPose = (p) => {
    let sc = 0;
    for (const bm of beams) { const t = rad(p.heading + bm.a); sc += llAt(p.x + bm.r * Math.sin(t), p.y + bm.r * Math.cos(t)); }
    return sc;
  };
  let bestFine = scoreAtPose(best);
  for (let st = (step * cell) / 2, sa = angStepDeg / 2, r = 0; r < 3; r++, st /= 2, sa /= 2) {
    const c0 = best;
    for (let a = -1; a <= 1; a++) for (let j = -1; j <= 1; j++) for (let i = -1; i <= 1; i++) {
      const p = { x: c0.x + i * st, y: c0.y + j * st, heading: c0.heading + a * sa };
      const sc = scoreAtPose(p);
      if (sc > bestFine) { bestFine = sc; best = p; }
    }
  }
  // support: readings that land near mapped structure; fit: their mean field, relative to the map's peak
  let support = 0, fsum = 0;
  for (const bm of beams) {
    const t = rad(best.heading + bm.a), v = sample(map, f, best.x + bm.r * Math.sin(t), best.y + bm.r * Math.cos(t));
    if (v > 0.05 * peak) { support++; fsum += v / peak; }
  }
  const fit = support ? fsum / support : 0;
  const axes = { heading: sigmaDeg <= AXIS_DEG, cross: minor <= AXIS_CM, along: major <= AXIS_CM };
  const gate = (v, good, bad) => clamp01((bad - v) / (bad - good));
  const enough = support >= Math.max(minValid, beams.length * 0.3) ? 1 : 0;
  const confidence = Math.round(enough * clamp01((fit - 0.2) / 0.4) * gate(major, 6, 20) * gate(sigmaDeg, 3, 9) * 1000) / 1000;
  const r1 = (v) => Math.round(v * 10) / 10;
  return {
    pose: roundPose(best), score: bestFine, confidence,
    cov: { xx: r1(cxx), xy: r1(cxy), yy: r1(cyy), hh: r1(chh), major: r1(major), minor: r1(minor), axisDeg: r1(axisDeg), sigmaDeg: r1(sigmaDeg) },
    axes, valid: beams.length, support, fit: Math.round(fit * 100) / 100,
    runnerUp: runner ? { pose: roundPose({ x: g.x + runner.dx, y: g.y + runner.dy, heading: g.heading + runner.dh }), score: runner.s - bestS } : null,
    ...(enough ? {} : { reason: 'little structure' }),
  };
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
