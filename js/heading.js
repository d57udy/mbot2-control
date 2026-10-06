// Heading pipeline against rotational drift (reports/Sonar localization for
// mBot2.md, rotational drift section):
//   1. standstill: while the wheels do not turn, yaw changes are gyro bias;
//      freeze the heading and estimate the bias from long stops
//   2. gyrodometry (Borenstein & Feng 1996), per leg or turn: trust the
//      encoders unless gyro and encoders differ by more than ~2 deg (a bump,
//      a threshold, wheel slip); then trust the gyro and flag the event
//   3. wall snap (Manhattan world): walls in a sweep give the room axes
//      modulo 90 deg; small heading errors are snapped back to them
// Frames: degrees, clockwise positive, like `turn` and the pose heading.

const WHEEL_CM = Math.PI * 6.5;
const TRACK_CM = 12;
const CM_PER_DEG = WHEEL_CM / 360;

export const HEADING = {
  // standstill and bias
  stillEncDeg: 2,        // both wheels moved less than this (wheel degrees) ...
  minStandS: 0.5,        // ... between two samples at least this far apart: standing
  minStillS: 10,         // bias needs this much standing time (integer yaw: 1 count per ~40 s at 1.5 deg/min)
  biasTauS: 300,         // older standstill evidence fades with this time constant
  maxBiasDegPerS: 0.1,   // 6 deg/min: anything larger is not bias (robot pushed by hand)
  // gyrodometry
  segmentTolDeg: 2,      // two quantization steps of the integer yaw
  gyroWeight: 0,         // share of the gyro when both agree; 0 = Borenstein (best in the
                         // 2-minute sim mission: max 1.8 deg vs 3.7 deg with the gyro)
  encLearnRate: 0.15,    // weight of each new agreeing leg in the encoder curve estimate
  encLearnMinLegs: 4,    // apply the learned curve only after this many agreeing legs
  encMaxDegPerCm: 0.05,  // 1.5 deg per 30 cm leg at most
  // wall snap
  beamDeg: 25,           // measured ultrasonic beam
  plateauTolCm: 3,       // readings of one wall plateau lie within this of its floor
  coreTolCm: 1.5,        // the plateau core (for its centre): within this of the floor
  minPlateauReadings: 5,
  minPlateauDeg: 0.5,    // plateau width at least this fraction of the beam ...
  maxPlateauDeg: 3,      // ... and at most this multiple (the edges rise slowly: d / cos)
  edgeCheckDeg: 12,      // past a plateau the readings must follow a wall this far
  edgeSlackDeg: 4,       // angle slack for that curve (beam edge, spacing)
  maxWallCm: 150,        // beyond this the reading means nothing (field setting)
  agreeDeg: 3,           // walls must agree on the axis within this
  distinctDeg: 60,       // ... and two of them must lie this far apart in bearing
  minClean: 1,           // walls in an agreeing group with both plateau edges
  minWalls: 2,
  maxSnapDeg: 12,        // larger corrections could snap to the wrong axis
  axisSweeps: 2,         // sweeps averaged to set the room axes
  axisWindowS: 45,       // untrusted sweeps set the axes only this soon after the start
  snapGain: 0.5,         // fraction of the correction to apply (one snap is about +-1.5 deg noisy)
};

export const normDeg = (a) => { const r = ((((a + 180) % 360) + 360) % 360) - 180; return r === -180 ? 180 : r; };
// -45..45: an angle modulo the 90 deg room symmetry
export const wrap45 = (a) => ((((a + 45) % 90) + 90) % 90) - 45;
const median = (xs) => { const s = [...xs].sort((a, b) => a - b); const m = s.length >> 1; return s.length % 2 ? s[m] : (s[m - 1] + s[m]) / 2; };

// Encoder heading change in degrees (clockwise) from wheel angle changes.
export function encoderHeadingDeg(dEncL, dEncR, { cmPerDeg = CM_PER_DEG, trackCm = TRACK_CM } = {}) {
  return (((dEncL - dEncR) * cmPerDeg) / trackCm) * (180 / Math.PI);
}

// 2. Gyrodometry for one leg or turn. gyroDeg: bias-corrected yaw change,
// encDeg: encoder heading change (either may be null). Returns the heading
// change to apply and whether the segment looked like slip or a bump.
export function fuseSegment({ gyroDeg, encDeg, tolDeg = HEADING.segmentTolDeg, gyroWeight = HEADING.gyroWeight } = {}) {
  const g = Number.isFinite(gyroDeg) ? gyroDeg : null, e = Number.isFinite(encDeg) ? encDeg : null;
  if (g == null && e == null) return { deg: 0, source: 'none', diffDeg: null, slip: false };
  if (g == null) return { deg: e, source: 'enc', diffDeg: null, slip: false };
  if (e == null) return { deg: g, source: 'gyro', diffDeg: null, slip: false };
  const diff = normDeg(g - e);
  // agreeing sources: encoders, nudged toward the gyro by gyroWeight (the
  // integer yaw and the encoder noise are independent; averaging halves both)
  return Math.abs(diff) > tolDeg
    ? { deg: g, source: 'gyro', diffDeg: diff, slip: true }
    : { deg: e + gyroWeight * diff, source: gyroWeight ? 'blend' : 'enc', diffDeg: diff, slip: false };
}

// 3a. Wall plateaus (regions of constant depth) in a sweep. A flat wall
// echoes only while its normal lies inside the beam, so the reading stays at
// the perpendicular distance over about one beam width and rises slowly
// beyond it; the plateau centre is the wall normal. points: [{ angle, cm }]
// relative to the sweep's start heading, any spacing. Returns
// [{ bearingDeg, cm, widthDeg, n }].
export function findWalls(points, opts = {}) {
  const o = { ...HEADING, ...opts };
  const pts = (points ?? [])
    .filter((p) => p && Number.isFinite(p.angle) && Number.isFinite(p.cm) && p.cm > 2 && p.cm < o.maxWallCm)
    .map((p) => ({ a: ((p.angle % 360) + 360) % 360, cm: p.cm }))
    .sort((p, q) => p.a - q.a);
  const n = pts.length;
  if (n < o.minPlateauReadings) return [];
  const at = (i) => pts[((i % n) + n) % n];
  const span = (i, j) => ((((at(j).a - at(i).a) % 360) + 360) % 360) || (j > i ? 360 : 0);
  const hole = (i, j) => span(i, j) > o.beamDeg / 2; // neighbouring readings too far apart
  const walls = [];
  const used = new Uint8Array(n);
  // seeds: local minima, deepest first, so each plateau grows from its floor
  const isMin = (k) => (hole(k - 1, k) || at(k - 1).cm >= at(k).cm) && (hole(k, k + 1) || at(k + 1).cm >= at(k).cm);
  const order = [...pts.keys()].filter(isMin).sort((i, j) => pts[i].cm - pts[j].cm);
  for (const s0 of order) {
    if (used[s0]) continue;
    const d = pts[s0].cm, top = d + o.plateauTolCm;
    let i = s0, j = s0;
    // within [d - tol, d + tol]: a nearer reading is another object in front
    const inside = (k) => at(k).cm <= top && at(k).cm >= d - o.plateauTolCm;
    while (j - i + 1 < n && !hole(j, j + 1) && inside(j + 1)) j++;
    while (j - i + 1 < n && !hole(i - 1, i) && inside(i - 1)) i--;
    // the basin (readings rising away from the plateau) belongs to this candidate
    let bi = i, bj = j;
    while (bj - bi + 1 < n && !hole(bj, bj + 1) && at(bj + 1).cm >= at(bj).cm) bj++;
    while (bj - bi + 1 < n && !hole(bi - 1, bi) && at(bi - 1).cm >= at(bi).cm) bi--;
    for (let k = bi; k <= bj; k++) used[((k % n) + n) % n] = 1;
    const count = j - i + 1, width = span(i, j);
    if (count < o.minPlateauReadings || count >= n) continue;
    if (width < o.beamDeg * o.minPlateauDeg || width > o.beamDeg * o.maxPlateauDeg) continue;
    // A wall rises slowly beyond the plateau: about d / cos(delta) at delta
    // degrees past its edge. A box corner, a chair leg or a doorframe is a
    // point reflector: past its cone the reading jumps to whatever is behind.
    // Both sides must follow the wall curve for edgeCheckDeg; a side cut by a
    // nearer object (reading below the plateau) ends the check there.
    const half = width / 2;
    const wallSide = (k0, step) => {
      for (let k = k0 + step, prev = k0; Math.abs(k - k0) < n; prev = k, k += step) {
        if (hole(Math.min(prev, k), Math.max(prev, k))) return true;
        const delta = step > 0 ? span(k0, k) : span(k, k0);
        if (delta > o.edgeCheckDeg) return true;
        const r = at(k).cm;
        if (r < d - o.plateauTolCm) return true; // occluded
        // angle from the wall normal minus half the beam, plus slack
        const phi = Math.max(0, half + delta - o.beamDeg / 2 + o.edgeSlackDeg);
        if (phi >= 80 || r > d / Math.cos((phi * Math.PI) / 180) + o.plateauTolCm) return false;
      }
      return true;
    };
    if (!wallSide(j, 1) || !wallSide(i, -1)) continue;
    // a cut edge shifts the centre by up to half the missing width
    const cut = (!hole(j, j + 1) && at(j + 1).cm < d - o.plateauTolCm) || (!hole(i - 1, i) && at(i - 1).cm < d - o.plateauTolCm);
    // centre from the core (readings within coreTolCm of the floor): the
    // outer plateau runs on into a room corner on one side and would bias it
    let ci = s0, cj = s0;
    while (cj < j && at(cj + 1).cm <= d + o.coreTolCm) cj++;
    while (ci > i && at(ci - 1).cm <= d + o.coreTolCm) ci--;
    const centre = at(ci).a + span(ci, cj) / 2;
    walls.push({ bearingDeg: Math.round(normDeg(centre) * 10) / 10, cm: d, widthDeg: Math.round(width * 10) / 10, n: count, cut });
  }
  // two plateaus of one wall (a second shallow minimum): keep the deeper
  walls.sort((p, q) => p.cm - q.cm);
  return walls.filter((w, k) => !walls.slice(0, k).some((v) => Math.abs(normDeg(v.bearingDeg - w.bearingDeg)) < o.beamDeg));
}

// 3b. Room axis (-45..45, map frame) from walls seen at scanHeadingDeg: the
// largest group of walls agreeing modulo 90 within agreeDeg.
export function roomAxis(walls, scanHeadingDeg, opts = {}) {
  const o = { ...HEADING, ...opts };
  // A cut edge moves a plateau centre by up to half the missing width:
  // prefer groups of clean walls, and accept a group only with at least
  // minClean clean walls in it.
  const clean = walls.filter((w) => !w.cut);
  if (clean.length >= o.minWalls && clean.length < walls.length) {
    const r = roomAxis(clean, scanHeadingDeg, opts);
    if (r) return r;
  }
  const dirs = walls.map((w) => wrap45(scanHeadingDeg + w.bearingDeg));
  // the largest group agreeing modulo 90 that holds distinct walls: two at
  // clearly different bearings (opposite or perpendicular), not two
  // plateaus of one surface
  let best = [];
  for (const d of dirs) {
    const idx = dirs.map((x, k) => (Math.abs(wrap45(x - d)) <= o.agreeDeg ? k : -1)).filter((k) => k >= 0);
    const bearings = idx.map((k) => walls[k].bearingDeg);
    const distinct = bearings.some((b) => bearings.some((c) => Math.abs(normDeg(b - c)) >= o.distinctDeg));
    const cleanIn = idx.filter((k) => !walls[k].cut).length;
    if (distinct && cleanIn >= o.minClean && idx.length > best.length) best = idx.map((k) => dirs[k]);
  }
  if (best.length < o.minWalls) return null;
  const ref = best[0];
  const axisDeg = wrap45(ref + median(best.map((x) => wrap45(x - ref))));
  return { axisDeg, walls: best.length, of: dirs.length };
}

// 3c. Heading correction from a sweep against known room axes. Returns
// { correctionDeg, walls, confidence, applied, reason } (correctionDeg is to be
// added to the heading the sweep was taken at).
export function snapHeading(points, scanHeadingDeg, axisDeg, opts = {}) {
  const o = { ...HEADING, ...opts };
  const walls = findWalls(points, o);
  const ax = roomAxis(walls, scanHeadingDeg, o);
  const out = { type: 'heading-snap', correctionDeg: 0, walls: walls.length, agreeing: ax?.walls ?? 0, confidence: 0, applied: false, reason: null, wallList: walls };
  if (!ax) { out.reason = `fewer than ${o.minWalls} walls agree`; return out; }
  if (axisDeg == null) { out.reason = 'no room axis yet'; out.axisDeg = ax.axisDeg; return out; }
  const corr = wrap45(axisDeg - ax.axisDeg);
  out.correctionDeg = Math.round(corr * 10) / 10;
  out.confidence = Math.round(Math.min(1, ax.walls / Math.max(o.minWalls + 1, ax.of)) * 100) / 100;
  if (Math.abs(corr) > o.maxSnapDeg) { out.reason = `correction ${Math.round(corr)} deg exceeds ${o.maxSnapDeg}`; return out; }
  out.applied = true;
  return out;
}

// Stateful part: standstill freeze and bias (1), segment fusion (2) and the
// room axes for snapping (3). Feed every sensor sample through wrap() or
// ingest(); read the corrected yaw from the samples or from yaw().
export class HeadingEstimator {
  constructor(opts = {}) {
    this.o = { ...HEADING, ...opts };
    this.reset();
  }

  reset() {
    this.biasDegPerS = 0;
    this.still = { dy: 0, dt: 0 };   // decayed standstill evidence
    this.prev = null;                // last raw sample with yaw
    this.yawRaw = null;              // unwrapped raw yaw
    this.yawC = null;                // corrected yaw (unwrapped)
    this.frozenDeg = 0;              // yaw change discarded while standing
    this.axisDeg = null;             // room axis in the map frame, -45..45
    this.encDegPerCm = 0;            // learned encoder heading error per cm of straight driving
    this.legs = 0;
    this.axisSeen = [];              // axis measurements before the axes are set
    this.t0 = null;                  // time of the first sample, for axisWindowS
    this.lastT = null;
    this.enc = null;                 // last encoder reading
    this.events = [];
  }

  // A sampler whose samples carry the corrected yaw (raw value in yawRaw).
  wrap(sample) {
    const f = async () => this.ingest(await sample());
    f.clock = sample.clock;
    return f;
  }

  ingest(raw) {
    if (!raw || raw.yaw == null || !Number.isFinite(Number(raw.yaw))) return raw;
    const y = Number(raw.yaw);
    const t = Number.isFinite(raw.t) ? raw.t : performance.now();
    if (!this.prev) {
      this.yawRaw = y;
      this.yawC = y;
    } else {
      const dy = normDeg(y - this.prev.yaw); // wrapped or unbounded yaw alike
      const dt = Math.max(0, (t - this.prev.t) / 1000);
      this.yawRaw += dy;
      const p = this.prev;
      // Standing needs a real pause: back-to-back samples of a slow spin (3 RPM
      // is 2 wheel degrees in 0.1 s) must not count, or turns get frozen.
      const standing = dt >= this.o.minStandS && raw.encL != null && raw.encR != null && p.encL != null && p.encR != null
        && Math.abs(raw.encL - p.encL) < this.o.stillEncDeg && Math.abs(raw.encR - p.encR) < this.o.stillEncDeg;
      if (standing) {
        // 1. the robot stands: any yaw change is bias; freeze and learn it
        this.frozenDeg += dy;
        const f = Math.exp(-dt / this.o.biasTauS);
        this.still = { dy: this.still.dy * f + dy, dt: this.still.dt * f + dt };
        if (this.still.dt >= this.o.minStillS) {
          const b = this.still.dy / this.still.dt;
          if (Math.abs(b) <= this.o.maxBiasDegPerS) this.biasDegPerS = b;
        }
      } else {
        this.yawC += dy - this.biasDegPerS * dt;
      }
    }
    this.prev = { yaw: y, t, encL: raw.encL, encR: raw.encR };
    this.t0 ??= t;
    this.lastT = t;
    if (raw.encL != null && raw.encR != null) this.enc = { l: raw.encL, r: raw.encR };
    return { ...raw, yaw: this.yawC, yawRaw: y };
  }

  // Bracket any motion (leg, turn, sweep) with mark() and since(): the
  // corrected gyro change and the encoder heading change between the two
  // (encDeg null when the samples in between carried no encoders), plus the
  // encoder distance for straight legs.
  mark() { return { yawC: this.yawC, enc: this.enc ? { ...this.enc } : null }; }

  since(m) {
    const gyroDeg = m?.yawC != null && this.yawC != null ? this.yawC - m.yawC : null;
    const e0 = m?.enc, e1 = this.enc;
    const ok = e0 && e1 && e1 !== e0;
    const dL = ok ? e1.l - e0.l : 0, dR = ok ? e1.r - e0.r : 0;
    return {
      gyroDeg,
      encDeg: ok ? encoderHeadingDeg(dL, dR) : null,
      cm: ok ? ((dL + dR) / 2) * CM_PER_DEG : null,
    };
  }

  // segment() for a bracketed motion: kind 'leg' learns the encoder curve.
  segmentSince(m, kind = 'leg') {
    const d = this.since(m);
    return this.segment({ gyroDeg: d.gyroDeg, encDeg: d.encDeg, kind, cm: d.cm });
  }

  yaw() { return this.yawC; }

  // 2. Fuse one leg or turn (gyroDeg from corrected samples, encDeg from the
  // encoders, cm the leg length). Records slip events. On straight legs the
  // encoders' systematic curve (unequal wheel diameters, Borenstein's type B
  // error) is learned from legs where both sources agree and taken out of
  // later legs: individual integer yaw readings are too coarse, their
  // average over legs is not.
  segment({ gyroDeg, encDeg, kind = 'leg', cm } = {}) {
    const straight = kind === 'leg' && Number.isFinite(cm) && cm > 5;
    const learned = this.legs >= this.o.encLearnMinLegs ? this.encDegPerCm : 0;
    const e = straight && Number.isFinite(encDeg) ? encDeg + learned * cm : encDeg;
    const r = fuseSegment({ gyroDeg, encDeg: e, tolDeg: this.o.segmentTolDeg, gyroWeight: this.o.gyroWeight });
    if (r.slip) this.events.push({ type: 'heading-slip', kind, ...r });
    else if (straight && Number.isFinite(gyroDeg) && Number.isFinite(encDeg)) {
      const k = (gyroDeg - encDeg) / cm;
      this.legs++;
      this.encDegPerCm += (k - this.encDegPerCm) * Math.max(this.o.encLearnRate, 1 / this.legs);
      this.encDegPerCm = Math.max(-this.o.encMaxDegPerCm, Math.min(this.o.encMaxDegPerCm, this.encDegPerCm));
    }
    return { kind, ...r, encCorrDeg: Number.isFinite(e) && Number.isFinite(encDeg) ? e - encDeg : 0 };
  }

  // 3. A sweep taken at scanHeadingDeg (pose heading at the sweep start, map
  // frame). The first sweep with enough agreeing walls sets the room axes;
  // later sweeps return a snap correction.
  // trusted: true for a sweep whose heading was just confirmed (a reference
  // or anchor match), false for one that must not set the axes. By default a
  // sweep may set them only within axisWindowS of the first sample, before
  // the heading had time to drift (field sim: axes set after 4 deg of drift
  // made every later snap hold that error).
  sweep(points, scanHeadingDeg, { trusted } = {}) {
    // The axes average the first axisSweeps sweeps that see agreeing walls
    // (one sweep is +-1 deg); no snaps until they are set.
    if (this.axisDeg == null) {
      const r = snapHeading(points, scanHeadingDeg, null, this.o);
      const early = this.t0 == null || (this.lastT - this.t0) / 1000 <= this.o.axisWindowS;
      const usable = trusted ?? early;
      if (!usable) {
        // too late to learn axes from this heading; settle for what we have
        if (this.axisSeen.length) this.axisDeg = wrap45(this.axisSeen.reduce((a, b) => a + b, 0) / this.axisSeen.length);
        const ev = { ...r, type: 'heading-axes', axisDeg: this.axisDeg, applied: false,
          reason: this.axisDeg != null ? 'room axes set' : 'heading not trusted for room axes' };
        this.events.push(ev);
        return ev;
      }
      if (r.axisDeg != null) {
        const ref = this.axisSeen[0] ?? r.axisDeg;
        this.axisSeen.push(ref + wrap45(r.axisDeg - ref));
      }
      const done = this.axisSeen.length >= this.o.axisSweeps;
      if (done) this.axisDeg = wrap45(this.axisSeen.reduce((a, b) => a + b, 0) / this.axisSeen.length);
      const ev = { ...r, type: 'heading-axes', axisDeg: this.axisDeg, applied: false,
        reason: done ? 'room axes set' : `room axes: ${this.axisSeen.length} of ${this.o.axisSweeps} sweeps` };
      this.events.push(ev);
      return ev;
    }
    const r = snapHeading(points, scanHeadingDeg, this.axisDeg, this.o);
    if (r.applied) r.correctionDeg = Math.round(r.correctionDeg * this.o.snapGain * 10) / 10;
    this.events.push(r);
    return r;
  }
}
