// Log-odds occupancy grid in the map frame (x right, y forward of the start,
// heading clockwise from +y). Square, centred on the origin.
//
// Ultrasonic model: the echo is the nearest surface anywhere in the cone, so
// the whole cone up to the reading is free evidence, and the hit lies somewhere
// on the arc at the reading. The arc is marked occupied with a weight that
// falls off toward the beam edges: the centre becomes occupied after one scan,
// the edges need confirmation, and free cones from other poses erode the
// false part of the arc. Planning inflates obstacles on top of that.

const L_FREE = -0.7;
const L_OCC = 1.2;
const L_MIN = -4;
const L_MAX = 4;
const L_THRESH = 0.5;   // |log-odds| above this is known (p < 0.38 or p > 0.62)
const EDGE_OCC = 0.35;  // occupied weight at the beam edge relative to the centre
const WIDE_REACH = 0.7; // free reach of rays outside the real beam, as a fraction of the range

const rad = (d) => (d * Math.PI) / 180;
const deg = (r) => (r * 180) / Math.PI;
const normDeg = (a) => { const r = ((((a + 180) % 360) + 360) % 360) - 180; return r === -180 ? 180 : r; };

export class GridMap {
  constructor({ cellCm = 5, sizeCm = 800 } = {}) {
    this.cellCm = cellCm;
    this.n = Math.ceil(sizeCm / cellCm);
    this.sizeCm = this.n * cellCm;
    this.half = this.sizeCm / 2;
    this.L = new Float32Array(this.n * this.n);
    this.version = 0;
    this.cache = new Map();
  }

  clear() {
    this.L.fill(0);
    this.touch();
  }

  touch() { this.version++; this.cache.clear(); }

  // cell index helpers; i = column (x), j = row (y)
  col(x) { return Math.floor((x + this.half) / this.cellCm); }
  row(y) { return Math.floor((y + this.half) / this.cellCm); }
  inside(i, j) { return i >= 0 && j >= 0 && i < this.n && j < this.n; }
  index(x, y) { const i = this.col(x), j = this.row(y); return this.inside(i, j) ? j * this.n + i : -1; }
  centre(k) {
    const i = k % this.n, j = (k - i) / this.n;
    return { x: (i + 0.5) * this.cellCm - this.half, y: (j + 0.5) * this.cellCm - this.half };
  }

  stateOf(k) {
    const l = this.L[k];
    return l > L_THRESH ? 'occupied' : l < -L_THRESH ? 'free' : 'unknown';
  }

  cell(x, y) {
    const k = this.index(x, y);
    return k < 0 ? 'unknown' : this.stateOf(k);
  }

  prob(k) { return 1 / (1 + Math.exp(-this.L[k])); }

  add(k, dl) { this.L[k] = Math.min(L_MAX, Math.max(L_MIN, this.L[k] + dl)); }

  // Marks a disc free, e.g. the robot's own footprint.
  markFree(x, y, rCm = 9) {
    const r = Math.ceil(rCm / this.cellCm);
    const ci = this.col(x), cj = this.row(y);
    for (let j = cj - r; j <= cj + r; j++) {
      for (let i = ci - r; i <= ci + r; i++) {
        if (!this.inside(i, j)) continue;
        const k = j * this.n + i, c = this.centre(k);
        if (Math.hypot(c.x - x, c.y - y) <= rCm) this.add(k, 2 * L_FREE);
      }
    }
    this.touch();
  }

  // points: [{ angle, cm }] relative to pose.heading (as from scan()). The
  // robot turns in place while scanning, so the sensor sits sensorOffsetCm
  // along each beam's direction. freeBeamDeg (default beamDeg) widens only the
  // free cone, e.g. to cover the gaps between coarse scan steps. The widened
  // part says less: it counts 0.75 (still free after one pass) and reaches
  // only WIDE_REACH of the range, so a wall seen at a slant whose perpendicular
  // lies in the widened part is not cleared through.
  integrateScan(pose, points, { sensorOffsetCm = 6, beamDeg = 16, maxRangeCm = 250, freeBeamDeg = beamDeg, robotRadiusCm = 9 } = {}) {
    const halfOcc = beamDeg / 2, halfFree = Math.max(freeBeamDeg, beamDeg) / 2;
    for (const p of points ?? []) {
      const cm = p?.cm == null ? NaN : Number(p.cm);
      if (!Number.isFinite(cm) || cm < 0) continue;
      const dir = (pose.heading ?? 0) + (Number(p.angle) || 0);
      const sx = pose.x + sensorOffsetCm * Math.sin(rad(dir));
      const sy = pose.y + sensorOffsetCm * Math.cos(rad(dir));
      const hit = cm < maxRangeCm;
      const range = hit ? cm : maxRangeCm;
      const free = new Map(), occ = new Map();
      // enough rays that neighbours are at most one cell apart at full range
      const nRays = Math.max(3, Math.ceil(rad(2 * halfFree) * range / this.cellCm) + 1);
      for (let r = 0; r < nRays; r++) {
        const off = -halfFree + (2 * halfFree * r) / (nRays - 1);
        const a = rad(dir + off);
        const dx = Math.sin(a), dy = Math.cos(a);
        const inBeam = Math.abs(off) <= halfOcc + 1e-9;
        const wFree = inBeam ? 1 : 0.75;
        const freeTo = (hit ? range - this.cellCm : range) * (inBeam ? 1 : WIDE_REACH);
        for (let t = 0; t <= freeTo; t += this.cellCm / 2) {
          const k = this.index(sx + dx * t, sy + dy * t);
          if (k >= 0 && !(free.get(k) >= wFree)) free.set(k, wFree);
        }
        if (hit && inBeam) {
          const w = 1 - (1 - EDGE_OCC) * (halfOcc ? Math.abs(off) / halfOcc : 0);
          const k = this.index(sx + dx * range, sy + dy * range);
          if (k >= 0 && !(occ.get(k) >= w)) occ.set(k, w);
        }
      }
      for (const k of occ.keys()) free.delete(k);
      for (const [k, w] of free) this.add(k, L_FREE * w);
      for (const [k, w] of occ) this.add(k, L_OCC * w);
    }
    if (robotRadiusCm > 0) this.markFree(pose.x, pose.y, robotRadiusCm);
    else this.touch();
  }

  // Uint8Array, 1 = an occupied cell lies within inflateCm (cached per version).
  inflated(inflateCm) {
    const key = `inf${inflateCm}`;
    let m = this.cache.get(key);
    if (m) return m;
    const n = this.n, r = Math.ceil(inflateCm / this.cellCm);
    const offs = [];
    for (let dj = -r; dj <= r; dj++) {
      for (let di = -r; di <= r; di++) if (Math.hypot(di, dj) * this.cellCm <= inflateCm) offs.push([di, dj]);
    }
    m = new Uint8Array(n * n);
    for (let k = 0; k < this.L.length; k++) {
      if (this.L[k] <= L_THRESH) continue;
      const i = k % n, j = (k - i) / n;
      for (const [di, dj] of offs) if (this.inside(i + di, j + dj)) m[(j + dj) * n + i + di] = 1;
    }
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

  // Free cells with an unknown 4-neighbour, clustered 8-connected; centroids, largest first.
  frontiers({ minCells = 3 } = {}) {
    const n = this.n;
    const isF = new Uint8Array(n * n);
    for (let j = 1; j < n - 1; j++) {
      for (let i = 1; i < n - 1; i++) {
        const k = j * n + i;
        if (this.L[k] >= -L_THRESH) continue;
        const unk = (q) => Math.abs(this.L[q]) <= L_THRESH;
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

  // Visits every observed cell (log-odds != 0) with its centre, state and probability.
  forEachCell(fn) {
    for (let k = 0; k < this.L.length; k++) {
      if (this.L[k] === 0) continue;
      const c = this.centre(k);
      fn(c.x, c.y, this.stateOf(k), this.prob(k));
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
    let free = 0, occupied = 0;
    for (let k = 0; k < this.L.length; k++) {
      if (this.L[k] < -L_THRESH) free++;
      else if (this.L[k] > L_THRESH) occupied++;
    }
    const a = (this.cellCm * this.cellCm) / 1e4;
    return { free, occupied, freeM2: free * a, knownM2: (free + occupied) * a };
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

  // Sparse persistence: only observed cells.
  toJSON() {
    const cells = [];
    for (let k = 0; k < this.L.length; k++) if (this.L[k] !== 0) cells.push(k, Math.round(this.L[k] * 100) / 100);
    return { v: 1, cellCm: this.cellCm, sizeCm: this.sizeCm, cells };
  }

  static fromJSON(o) {
    const m = new GridMap({ cellCm: o?.cellCm ?? 5, sizeCm: o?.sizeCm ?? 800 });
    const c = o?.cells ?? [];
    for (let q = 0; q + 1 < c.length; q += 2) if (c[q] >= 0 && c[q] < m.L.length) m.L[c[q]] = c[q + 1];
    m.touch();
    return m;
  }
}
