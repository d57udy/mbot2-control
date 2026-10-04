// Occupancy grid in the map frame (x right, y forward of the start, heading
// clockwise from +y). Square, centred on the origin.
//
// Evidence model (v0.6). Each cell keeps hit evidence H and miss evidence M
// separately, the number of distinct scans that hit it and the scan that last
// observed it. A scan contributes at most one hit and one miss per cell (the
// strongest), so the overlapping beams of a sweep or repeated readings cannot
// inflate the evidence; open a scan with beginScan()/endScan(), otherwise
// every integrateScan() call is one scan.
//
// Ultrasonic: the echo is the nearest surface anywhere in the cone.
//  - Hit: the cell lies on the arc at the reading, weighted toward the beam
//    centre (an off-centre echo is less likely to come from that cell) and
//    toward short range (a far arc is wide; on an oblique wall its centre
//    lies in front of the wall).
//  - Miss: the beam core (+-1/4 of the beam) passes through the cell and the
//    reading ends clearly beyond it (MISS_MARGIN_CM), weighted higher at short
//    range. A real obstacle in the core would have produced the reading, so it
//    cannot collect core misses; a phantom from a random reflection does.
//  - Cells without hit evidence also become free from the rest of the cone up
//    to the reading (weaker), and from the freeBeamDeg gap filler (weaker
//    still); that never erodes hit evidence.
// p = (H + 1/2) / (H + M + 1), a Beta posterior, so a well-confirmed wall
// needs proportionally many misses (specular ghosts) to fade, while a
// one-scan phantom goes after one or two. H + M is capped (EVIDENCE_CAP) so
// the map keeps adapting when something moves.
//
// States (kindOf): 'occupied' (confirmed: p >= P_OCC and hits from at least
// two scans, or one head-on hit closer than NEAR_CM), 'suspect' (hit evidence
// that is not confirmed, p >= 1/2), 'free' (p < P_FREE), 'unknown'.
// Consumers that read log-odds use L, a view derived from the evidence:
// confirmed > L_THRESH, suspect in (L_SUSPECT, L_THRESH], free < -L_THRESH.
// stateOf()/cell() keep the three legacy states (a suspect cell is
// 'unknown' there). Writes to L from outside (loading a saved map) are
// imported as evidence on the next touch().

export const L_THRESH = 0.5;   // |L| above this is known
export const L_SUSPECT = 0.15; // L above this (and up to L_THRESH) is a suspect cell
const P_OCC = 0.65;            // confirmed occupied needs at least this
const P_FREE = 0.4;            // free below this
const PRIOR = 0.5;             // Beta prior per side
const EVIDENCE_CAP = 12;       // H + M is scaled down to this
const HIT_EDGE = 0.3;          // hit weight at the beam edge (1 at the centre)
const HIT_COUNTS = 0.5;        // a hit at least this strong counts as a hit scan
const NEAR_CM = 60;            // a head-on core hit closer than this confirms at once
const MISS_MARGIN_CM = 8;      // misses on hit cells only this far short of the reading
const MISS_FULL_CM = 50;       // hit and miss weights are 1 up to this range ...
const MISS_FAR = 0.4;          // ... and fall to this at maxRange (far arcs are wide)
const CONE_MISS = 0.4;         // miss weight of the cone outside the core (cells without hits)
const WIDE_MISS = 0.12;        // gap filler outside the beam (cells without hits): free after 3 passes
const WIDE_REACH = 0.7;        // gap filler reach, as a fraction of the range
const CONTACT_CONFIRMS = 2;    // add(k, dl) with dl >= this is physical contact: confirmed

// 1D squared distance transform of f (0 at sites, large elsewhere) into d.
function edt1d(f, n, d, v, z) {
  let k = 0;
  v[0] = 0; z[0] = -Infinity; z[1] = Infinity;
  for (let q = 1; q < n; q++) {
    const at = (p) => ((f[q] + q * q) - (f[p] + p * p)) / (2 * q - 2 * p);
    let s = at(v[k]);
    while (s <= z[k]) s = at(v[--k]);   // z[0] = -Infinity ends the loop
    k++;
    v[k] = q; z[k] = s; z[k + 1] = Infinity;
  }
  k = 0;
  for (let q = 0; q < n; q++) {
    while (z[k + 1] < q) k++;
    const p = v[k];
    d[q] = (q - p) * (q - p) + f[p];
  }
}

const rad = (d) => (d * Math.PI) / 180;
const deg = (r) => (r * 180) / Math.PI;
const normDeg = (a) => { const r = ((((a + 180) % 360) + 360) % 360) - 180; return r === -180 ? 180 : r; };
const logit = (p) => Math.log(p / (1 - p));

export class GridMap {
  // decayAfterScans: evidence of cells not observed for this many scans fades
  // by decayRate per scan toward unknown (0 = off).
  constructor({ cellCm = 5, sizeCm = 800, decayAfterScans = 0, decayRate = 0.9 } = {}) {
    this.cellCm = cellCm;
    this.n = Math.ceil(sizeCm / cellCm);
    this.sizeCm = this.n * cellCm;
    this.half = this.sizeCm / 2;
    const N = this.n * this.n;
    this.L = new Float32Array(N);      // derived log-odds view (see above)
    this.Lw = new Float32Array(N);     // L as last written here, to detect outside writes
    this.H = new Float32Array(N);      // hit evidence
    this.M = new Float32Array(N);      // miss evidence
    this.S = new Uint16Array(N);       // scans with a hit of at least HIT_COUNTS
    this.near = new Uint8Array(N);     // 1 = head-on hit closer than NEAR_CM
    this.seen = new Int32Array(N);     // scan id of the last observation (0 = never)
    this.hitSid = new Int32Array(N);   // per-scan bookkeeping: strongest hit / miss so far
    this.hitW = new Float32Array(N);
    this.missSid = new Int32Array(N);
    this.missW = new Float32Array(N);
    this.scans = 0;                    // scan ids handed out
    this.openScan = 0;                 // id of the scan opened by beginScan(), or 0
    this.decayAfterScans = decayAfterScans;
    this.decayRate = decayRate;
    this.version = 0;
    this.cache = new Map();
  }

  clear() {
    for (const a of [this.L, this.Lw, this.H, this.M, this.S, this.near, this.seen, this.hitSid, this.hitW, this.missSid, this.missW]) a.fill(0);
    this.openScan = 0;
    this.touch();
  }

  // Takes over another map's cells and evidence (same cellCm and sizeCm).
  copyFrom(o) {
    if (o.n !== this.n || o.cellCm !== this.cellCm) throw new Error('map size differs');
    for (const f of ['L', 'Lw', 'H', 'M', 'S', 'near', 'seen']) this[f].set(o[f]);
    this.hitSid.fill(0); this.missSid.fill(0);
    this.scans = o.scans;
    this.openScan = 0;
    this.touch();
  }

  // Call after changing the map. Imports cells whose L was written from outside.
  touch() {
    const { L, Lw } = this;
    for (let k = 0; k < L.length; k++) if (L[k] !== Lw[k]) this.importL(k);
    this.version++;
    this.cache.clear();
  }

  // Evidence from a bare log-odds value (old saved maps, tests that set L).
  // L keeps the given value; the evidence matches its state, as if from a
  // few scans, until new observations of the cell rewrite it.
  importL(k) {
    const l = this.L[k];
    if (!Number.isFinite(l) || l === 0) {
      this.H[k] = 0; this.M[k] = 0; this.S[k] = 0; this.near[k] = 0;
      this.L[k] = 0;
    } else {
      let p = 1 / (1 + Math.exp(-Math.max(-8, Math.min(8, l))));
      if (l > L_THRESH) p = Math.max(p, P_OCC + 0.01);
      const n = 4;
      this.H[k] = Math.max(0, p * (n + 2 * PRIOR) - PRIOR);
      this.M[k] = Math.max(0, n - this.H[k]);
      this.S[k] = l > L_THRESH ? 2 : l > 0 ? 1 : 0;
      this.near[k] = 0;
    }
    this.Lw[k] = this.L[k];
  }

  // cell index helpers; i = column (x), j = row (y)
  col(x) { return Math.floor((x + this.half) / this.cellCm); }
  row(y) { return Math.floor((y + this.half) / this.cellCm); }
  inside(i, j) { return i >= 0 && j >= 0 && i < this.n && j < this.n; }
  index(x, y) { const i = this.col(x), j = this.row(y); return this.inside(i, j) ? j * this.n + i : -1; }
  centre(k) {
    const i = k % this.n, j = (k - i) / this.n;
    return { x: (i + 0.5) * this.cellCm - this.half, y: (j + 0.5) * this.cellCm - this.half };
  }

  // Occupancy probability from the evidence (0.5 = nothing known).
  prob(k) { return (this.H[k] + PRIOR) / (this.H[k] + this.M[k] + 2 * PRIOR); }

  confirmed(k) { return this.prob(k) >= P_OCC && (this.S[k] >= 2 || this.near[k] === 1); }

  // 'occupied' | 'suspect' | 'free' | 'unknown'
  kindOf(k) {
    if (k < 0) return 'unknown';
    if (this.H[k] + this.M[k] === 0) return 'unknown';
    const p = this.prob(k);
    if (this.H[k] > 0 && this.confirmed(k)) return 'occupied';
    if (this.H[k] > 0 && p >= 0.5) return 'suspect';
    return p < P_FREE ? 'free' : 'unknown';
  }

  kind(x, y) { return this.kindOf(this.index(x, y)); }

  // Legacy three states from the L view: suspect cells are 'unknown'.
  stateOf(k) {
    const l = this.L[k];
    return l > L_THRESH ? 'occupied' : l < -L_THRESH ? 'free' : 'unknown';
  }

  cell(x, y) {
    const k = this.index(x, y);
    return k < 0 ? 'unknown' : this.stateOf(k);
  }

  // Caps the evidence and rewrites the L view of cell k.
  update(k) {
    const t = this.H[k] + this.M[k];
    if (t > EVIDENCE_CAP) { const f = EVIDENCE_CAP / t; this.H[k] *= f; this.M[k] *= f; }
    let l = 0;
    if (this.H[k] + this.M[k] > 0) {
      const kind = this.kindOf(k), lp = logit(this.prob(k));
      if (kind === 'occupied') l = Math.max(L_THRESH + 0.01, lp);
      else if (kind === 'suspect') l = Math.min(L_THRESH, Math.max(L_SUSPECT + 0.05, lp));
      else if (kind === 'free') l = Math.min(-L_THRESH - 0.01, lp);
      else l = Math.min(L_SUSPECT, Math.max(-L_THRESH, lp)) || -1e-3; // observed, undecided: never exactly 0
    }
    this.L[k] = l;
    this.Lw[k] = this.L[k];
  }

  // One scan's hit / miss on cell k: only the strongest of each per scan counts.
  hitCell(k, w, sid, near = false) {
    if (this.hitSid[k] !== sid) { this.hitSid[k] = sid; this.hitW[k] = 0; }
    if (w > this.hitW[k]) {
      if (this.hitW[k] < HIT_COUNTS && w >= HIT_COUNTS) this.S[k] = Math.min(65535, this.S[k] + 1);
      this.H[k] += w - this.hitW[k];
      this.hitW[k] = w;
    }
    if (near) this.near[k] = 1;
    this.seen[k] = sid;
  }

  missCell(k, w, sid) {
    if (this.missSid[k] !== sid) { this.missSid[k] = sid; this.missW[k] = 0; }
    if (w > this.missW[k]) { this.M[k] += w - this.missW[k]; this.missW[k] = w; }
    this.seen[k] = sid;
  }

  // Groups the following integrateScan / markFree calls into one scan.
  beginScan() {
    this.openScan = ++this.scans;
    return this.openScan;
  }

  endScan() {
    if (!this.openScan) return;
    this.openScan = 0;
    this.age();
    this.touch();
  }

  // Raw evidence for compatibility: dl > 0 adds hits (dl >= CONTACT_CONFIRMS
  // is a physical contact and confirms the cell), dl < 0 adds misses.
  add(k, dl) {
    if (k < 0 || !dl) return;
    if (dl > 0) {
      this.H[k] += dl;
      if (dl >= CONTACT_CONFIRMS) this.S[k] = Math.max(this.S[k], 2);
      else if (dl >= HIT_COUNTS) this.S[k] = Math.max(this.S[k], 1);
    } else this.M[k] -= dl;
    this.seen[k] = Math.max(this.seen[k], this.scans);
    this.update(k);
  }

  // The robot's own footprint is free; hit cells under it get a weak miss
  // only (a pose error must not wipe a known obstacle in one go).
  markFree(x, y, rCm = 9) {
    const own = !this.openScan;
    const sid = this.openScan || ++this.scans;
    this.footprint(x, y, rCm, sid);
    if (own) this.age();
    this.touch();
  }

  footprint(x, y, rCm, sid) {
    const r = Math.ceil(rCm / this.cellCm);
    const ci = this.col(x), cj = this.row(y);
    for (let j = cj - r; j <= cj + r; j++) {
      for (let i = ci - r; i <= ci + r; i++) {
        if (!this.inside(i, j)) continue;
        const k = j * this.n + i, c = this.centre(k);
        if (Math.hypot(c.x - x, c.y - y) > rCm) continue;
        this.missCell(k, this.H[k] > 0 ? 0.5 : 1, sid);
        this.update(k);
      }
    }
  }

  // points: [{ angle, cm }] relative to pose.heading (as from scan()). The
  // sensor sits sensorOffsetCm along each beam's direction. Readings at or
  // beyond maxRangeCm mean nothing in range: misses up to maxRangeCm.
  // freeBeamDeg (default beamDeg) widens the free cone for coarse step scans.
  integrateScan(pose, points, { sensorOffsetCm = 6, beamDeg = 16, maxRangeCm = 250, freeBeamDeg = beamDeg, robotRadiusCm = 9, scanId } = {}) {
    const own = !scanId && !this.openScan;
    const sid = scanId || this.openScan || ++this.scans;
    const hb = beamDeg / 2, hc = beamDeg / 4, hf = Math.max(freeBeamDeg, beamDeg) / 2;
    const cell = this.cellCm, arc = cell * 0.71;
    const missW = (d) => (d <= MISS_FULL_CM ? 1
      : Math.max(MISS_FAR, 1 - ((1 - MISS_FAR) * (d - MISS_FULL_CM)) / Math.max(1, maxRangeCm - MISS_FULL_CM)));
    const touched = new Set();
    for (const p of points ?? []) {
      const cm = p?.cm == null ? NaN : Number(p.cm);
      if (!Number.isFinite(cm) || cm < 0) continue;
      const dir = rad((pose.heading ?? 0) + (Number(p.angle) || 0));
      const ux = Math.sin(dir), uy = Math.cos(dir);
      const sx = pose.x + sensorOffsetCm * ux, sy = pose.y + sensorOffsetCm * uy;
      const hit = cm < maxRangeCm;
      const R = hit ? cm : maxRangeCm;
      // bounding box of the sector
      let x0 = sx, x1 = sx, y0 = sy, y1 = sy;
      for (let a = -hf; a <= hf + 1e-9; a += Math.max(1, hf / 4)) {
        const t = dir + rad(a), ex = sx + (R + cell) * Math.sin(t), ey = sy + (R + cell) * Math.cos(t);
        x0 = Math.min(x0, ex); x1 = Math.max(x1, ex); y0 = Math.min(y0, ey); y1 = Math.max(y1, ey);
      }
      const i0 = Math.max(0, this.col(x0)), i1 = Math.min(this.n - 1, this.col(x1));
      const j0 = Math.max(0, this.row(y0)), j1 = Math.min(this.n - 1, this.row(y1));
      for (let j = j0; j <= j1; j++) {
        for (let i = i0; i <= i1; i++) {
          const k = j * this.n + i;
          const cx = (i + 0.5) * cell - this.half - sx, cy = (j + 0.5) * cell - this.half - sy;
          const d = Math.hypot(cx, cy);
          if (d > R + arc) continue;
          // angle off the beam axis, less the cell's own angular radius (a beam
          // through any part of a cell near the sensor counts)
          const off = Math.max(0, Math.abs(deg(Math.atan2(cx * uy - cy * ux, cx * ux + cy * uy))) - deg(Math.atan2(cell / 2, Math.max(d, 1e-6))));
          if (off > hf) continue;
          const inBeam = off <= hb;
          if (hit && inBeam && Math.abs(d - R) <= arc) {
            this.hitCell(k, (1 - (1 - HIT_EDGE) * (hb ? off / hb : 0)) * missW(d), sid, off <= hc && R < NEAR_CM);
            touched.add(k);
            continue;
          }
          const bare = this.H[k] === 0, end = hit ? R - cell : R;
          if (inBeam && off <= hc && d < (bare ? end : R - MISS_MARGIN_CM)) this.missCell(k, missW(d), sid);
          else if (bare && inBeam && d < end) this.missCell(k, CONE_MISS * missW(d), sid);
          else if (bare && !inBeam && d < R * WIDE_REACH) this.missCell(k, WIDE_MISS * missW(d), sid);
          else continue;
          touched.add(k);
        }
      }
    }
    for (const k of touched) this.update(k);
    if (robotRadiusCm > 0) this.footprint(pose.x, pose.y, robotRadiusCm, sid);
    if (own) this.age();
    this.touch();
  }

  // Fades evidence of cells not observed for decayAfterScans scans.
  age() {
    if (!(this.decayAfterScans > 0)) return;
    const r = this.decayRate, cut = this.scans - this.decayAfterScans;
    for (let k = 0; k < this.H.length; k++) {
      if (this.seen[k] >= cut || this.H[k] + this.M[k] === 0) continue;
      this.H[k] *= r; this.M[k] *= r;
      if (this.H[k] + this.M[k] < 0.05) { this.H[k] = 0; this.M[k] = 0; this.S[k] = 0; this.near[k] = 0; }
      this.update(k);
    }
  }

  // Removes hit evidence that one scan at most has seen and that is not
  // confirmed (random reflections). Returns the number of cells cleaned.
  cleanup() {
    let n = 0;
    for (let k = 0; k < this.H.length; k++) {
      if (this.H[k] === 0 || this.S[k] > 1 || this.confirmed(k)) continue;
      this.H[k] = 0; this.S[k] = 0; this.near[k] = 0;
      this.update(k);
      n++;
    }
    this.touch();
    return n;
  }

  // For a tap inspector: evidence of the cell at (x, y).
  cellInfo(x, y) {
    const k = this.index(x, y);
    if (k < 0) return null;
    const c = this.centre(k), r = (v) => Math.round(v * 100) / 100;
    return {
      x: c.x, y: c.y, state: this.kindOf(k), p: r(this.prob(k)),
      hits: r(this.H[k]), misses: r(this.M[k]), scans: this.S[k], near: this.near[k] === 1,
      lastSeen: this.seen[k], scansAgo: this.seen[k] ? this.scans - this.seen[k] : null,
    };
  }

  // Float32Array: distance in cm from each cell centre to the nearest cell
  // centre with log-odds above threshold (Infinity if there is none). Exact
  // Euclidean distance transform (Felzenszwalb), cached per map version.
  distanceField(threshold = L_THRESH) {
    const key = `df${threshold}`;
    let d = this.cache.get(key);
    if (d) return d;
    const n = this.n, INF = 1e20;
    const g = new Float64Array(n * n);
    for (let k = 0; k < g.length; k++) g[k] = this.L[k] > threshold ? 0 : INF;
    const f = new Float64Array(n), out = new Float64Array(n), v = new Int32Array(n), z = new Float64Array(n + 1);
    const pass = (get, set) => {
      for (let q = 0; q < n; q++) f[q] = get(q);
      edt1d(f, n, out, v, z);
      for (let q = 0; q < n; q++) set(q, out[q]);
    };
    for (let i = 0; i < n; i++) pass((q) => g[q * n + i], (q, x) => { g[q * n + i] = x; });
    for (let j = 0; j < n; j++) pass((q) => g[j * n + q], (q, x) => { g[j * n + q] = x; });
    d = new Float32Array(n * n);
    for (let k = 0; k < d.length; k++) d[k] = g[k] >= INF / 2 ? Infinity : Math.sqrt(g[k]) * this.cellCm;
    this.cache.set(key, d);
    return d;
  }

  // Distance in cm from (x, y) to the nearest occupied cell centre.
  clearance(x, y, threshold = L_THRESH) {
    const k = this.index(x, y);
    return k < 0 ? 0 : this.distanceField(threshold)[k];
  }

  // Uint8Array, 1 = an occupied cell lies within inflateCm (cached per version).
  inflated(inflateCm) {
    const key = `inf${inflateCm}`;
    let m = this.cache.get(key);
    if (m) return m;
    const d = this.distanceField();
    m = new Uint8Array(this.n * this.n);
    for (let k = 0; k < m.length; k++) if (d[k] <= inflateCm + 1e-6) m[k] = 1;
    this.cache.set(key, m);
    return m;
  }

  traversableAt(k, { inflateCm = 14, allowUnknown = false } = {}) {
    if (k < 0) return false;
    const s = this.stateOf(k);
    if (s === 'occupied' || (s === 'unknown' && !allowUnknown)) return false;
    return !this.inflated(inflateCm)[k];
  }

  isTraversable(x, y, opts = {}) { return this.traversableAt(this.index(x, y), opts); }

  // Free cells with an unknown (not suspect) 4-neighbour, clustered 8-connected; centroids, largest first.
  frontiers({ minCells = 3 } = {}) {
    const n = this.n;
    const isF = new Uint8Array(n * n);
    for (let j = 1; j < n - 1; j++) {
      for (let i = 1; i < n - 1; i++) {
        const k = j * n + i;
        if (this.L[k] >= -L_THRESH) continue;
        const unk = (q) => this.L[q] >= -L_THRESH && this.L[q] <= L_SUSPECT;   // suspect cells are evidence, not unknown
        if (unk(k - 1) || unk(k + 1) || unk(k - n) || unk(k + n)) isF[k] = 1;
      }
    }
    const out = [];
    for (let k0 = 0; k0 < isF.length; k0++) {
      if (isF[k0] !== 1) continue;
      isF[k0] = 2;
      const queue = [k0];
      let sx = 0, sy = 0;
      for (let q = 0; q < queue.length; q++) {
        const k = queue[q], c = this.centre(k);
        sx += c.x; sy += c.y;
        const i = k % n, j = (k - i) / n;
        for (let dj = -1; dj <= 1; dj++) {
          for (let di = -1; di <= 1; di++) {
            if (!this.inside(i + di, j + dj)) continue;
            const kk = (j + dj) * n + i + di;
            if (isF[kk] === 1) { isF[kk] = 2; queue.push(kk); }
          }
        }
      }
      if (queue.length >= minCells) {
        out.push({ x: Math.round(sx / queue.length), y: Math.round(sy / queue.length), size: queue.length });
      }
    }
    return out.sort((a, b) => b.size - a.size);
  }

  // Visits every observed cell with its centre, legacy state, probability and
  // kind ('occupied' | 'suspect' | 'free' | 'unknown').
  forEachCell(fn) {
    for (let k = 0; k < this.L.length; k++) {
      if (this.L[k] === 0) continue;
      const c = this.centre(k);
      fn(c.x, c.y, this.stateOf(k), this.prob(k), this.kindOf(k));
    }
  }

  // Extent of known cells, or null when nothing is known yet.
  get bounds() {
    let minX = Infinity, maxX = -Infinity, minY = Infinity, maxY = -Infinity;
    for (let k = 0; k < this.L.length; k++) {
      if (Math.abs(this.L[k]) <= L_THRESH) continue;
      const c = this.centre(k);
      if (c.x < minX) minX = c.x;
      if (c.x > maxX) maxX = c.x;
      if (c.y < minY) minY = c.y;
      if (c.y > maxY) maxY = c.y;
    }
    return minX > maxX ? null : { minX, maxX, minY, maxY };
  }

  stats() {
    let free = 0, occupied = 0, suspect = 0;
    for (let k = 0; k < this.L.length; k++) {
      if (this.L[k] < -L_THRESH) free++;
      else if (this.L[k] > L_THRESH) occupied++;
      else if (this.L[k] > L_SUSPECT) suspect++;
    }
    const a = (this.cellCm * this.cellCm) / 1e4;
    return { free, occupied, suspect, freeM2: free * a, knownM2: (free + occupied) * a };
  }

  // Compact text for the LLM. Directions are relative to the robot heading.
  describe(pose = { x: 0, y: 0, heading: 0 }) {
    const st = this.stats();
    if (!st.free && !st.occupied) return 'Map: empty (nothing scanned yet).';
    const sectors = ['ahead', 'ahead-right', 'right', 'back-right', 'behind', 'back-left', 'left', 'ahead-left'];
    const near = new Array(8).fill(Infinity);
    for (let k = 0; k < this.L.length; k++) {
      if (this.L[k] <= L_THRESH) continue;
      const c = this.centre(k);
      const d = Math.hypot(c.x - pose.x, c.y - pose.y);
      if (d > 300) continue;
      const b = normDeg(deg(Math.atan2(c.x - pose.x, c.y - pose.y)) - pose.heading);
      const s = ((Math.round(b / 45) % 8) + 8) % 8;
      near[s] = Math.min(near[s], d);
    }
    const obs = near.map((d, s) => (d < Infinity ? `${sectors[s]} ${Math.round(d)}` : null)).filter(Boolean);
    const fr = this.frontiers();
    let frText = `${fr.length} frontier${fr.length === 1 ? '' : 's'}`;
    if (fr.length) {
      const f = fr.reduce((a, b) => (Math.hypot(a.x - pose.x, a.y - pose.y) <= Math.hypot(b.x - pose.x, b.y - pose.y) ? a : b));
      const b = Math.round(normDeg(deg(Math.atan2(f.x - pose.x, f.y - pose.y)) - pose.heading));
      frText += `, nearest ${Math.round(Math.hypot(f.x - pose.x, f.y - pose.y))} cm at ${b}° (x ${f.x}, y ${f.y})`;
    }
    return `Map: ${st.knownM2.toFixed(1)} m² known (${st.freeM2.toFixed(1)} free). `
      + `Nearest obstacles (cm, relative to heading): ${obs.join(', ') || 'none within 3 m'}. ${frText}.`;
  }

  // Sparse persistence of the evidence: [k, H, M, S + 65536 * near, ...] for
  // observed cells. Version 1 (log-odds only) is still read.
  toJSON() {
    const cells = [];
    const r = (v) => Math.round(v * 100) / 100;
    for (let k = 0; k < this.L.length; k++) {
      if (this.H[k] + this.M[k] === 0) continue;
      cells.push(k, r(this.H[k]), r(this.M[k]), this.S[k] + 65536 * this.near[k]);
    }
    return { v: 2, cellCm: this.cellCm, sizeCm: this.sizeCm, scans: this.scans, cells };
  }

  static fromJSON(o) {
    const m = new GridMap({ cellCm: o?.cellCm ?? 5, sizeCm: o?.sizeCm ?? 800 });
    const c = o?.cells ?? [];
    if (o?.v === 2) {
      m.scans = Number(o.scans) || 0;
      for (let q = 0; q + 3 < c.length; q += 4) {
        const k = c[q];
        if (!(k >= 0 && k < m.L.length)) continue;
        m.H[k] = Math.max(0, Number(c[q + 1]) || 0);
        m.M[k] = Math.max(0, Number(c[q + 2]) || 0);
        m.S[k] = (c[q + 3] % 65536) || 0;
        m.near[k] = c[q + 3] >= 65536 ? 1 : 0;
        m.update(k);
      }
    } else {
      for (let q = 0; q + 1 < c.length; q += 2) if (c[q] >= 0 && c[q] < m.L.length) m.L[c[q]] = c[q + 1];
    }
    m.touch();
    return m;
  }
}
