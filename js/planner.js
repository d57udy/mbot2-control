// Path planning on a GridMap: A* (8-connected, octile heuristic) on cells,
// line-of-sight simplification and conversion to turn + straight moves.
//
// Costs: cells within inflateCm of a confirmed obstacle are blocked; cells
// near it (up to CLEAR_CM further) cost extra, so paths keep their distance
// where there is room and still fit through narrow gaps. Suspect cells (hit
// evidence that is not confirmed yet) and their inflation zone cost a lot,
// rising toward the cell, so they are crossed only as a last resort. Unknown
// cells cost UNKNOWN_COST times a free one.

import { L_SUSPECT } from './gridmap.js';

const SQRT2 = Math.SQRT2;
const UNKNOWN_COST = 3;   // unknown cells cost this much more than free ones
const ESCAPE_COST = 5;    // inflated cells next to the start (robot already sits there)
const SNAP_CM = 30;       // a blocked goal moves to the nearest traversable cell within this
const CLEAR_CM = 15;      // soft margin beyond the inflation
const CLEAR_COST = 2;     // extra cost at the inflation edge, falling to 0 over CLEAR_CM
const SUSPECT_COST = 4;   // extra cost within inflateCm of a suspect cell ...
const SUSPECT_CORE_COST = 20; // ... plus up to this much more toward the cell itself

// The real mBot2 is about 18 cm wide with its wheels. Inflation is measured
// between cell centres, so an obstacle surface can sit up to half a cell
// diagonal (3.5 cm) closer: 14 cm keeps at least ~10 cm from the centre to a
// seen surface, and the soft margin adds more where there is room. Larger
// values close gaps under about 40 cm (e.g. between the sim chair and wall).
export const ROBOT_RADIUS_CM = 9;
export const DEFAULT_INFLATE_CM = 14;

const normDeg = (a) => { const r = ((((a + 180) % 360) + 360) % 360) - 180; return r === -180 ? 180 : r; };

// Binary min-heap of [f, k].
class Heap {
  constructor() { this.a = []; }
  get size() { return this.a.length; }
  push(f, k) {
    const a = this.a;
    a.push([f, k]);
    for (let i = a.length - 1; i > 0;) {
      const p = (i - 1) >> 1;
      if (a[p][0] <= a[i][0]) break;
      [a[p], a[i]] = [a[i], a[p]];
      i = p;
    }
  }
  pop() {
    const a = this.a, top = a[0], last = a.pop();
    if (a.length) {
      a[0] = last;
      for (let i = 0; ;) {
        const l = 2 * i + 1, r = l + 1;
        let m = i;
        if (l < a.length && a[l][0] < a[m][0]) m = l;
        if (r < a.length && a[r][0] < a[m][0]) m = r;
        if (m === i) break;
        [a[m], a[i]] = [a[i], a[m]];
        i = m;
      }
    }
    return top;
  }
}

// Cost multiplier of entering cell k, or Infinity if blocked. Inflated
// cells within inflateCm of the start stay usable (at a high cost) only if
// they do not bring the robot closer to the obstacle than it already is, so
// it can back away from a spot next to an obstacle but not skim past it.
function cellCost(map, k, { inflateCm, allowUnknown }, start) {
  if (k < 0) return Infinity;
  const s = map.stateOf(k);
  if (s === 'occupied') return Infinity;
  if (s === 'unknown' && !allowUnknown) return Infinity;
  let cost = s === 'unknown' ? UNKNOWN_COST : 1;
  const d = map.distanceField()[k], ds = map.distanceField(L_SUSPECT)[k];
  if (ds < inflateCm + CLEAR_CM) cost += CLEAR_COST * Math.min(1, (inflateCm + CLEAR_CM - ds) / CLEAR_CM);
  if (ds <= inflateCm && d > inflateCm) cost += SUSPECT_COST + SUSPECT_CORE_COST * (1 - ds / Math.max(1, inflateCm));
  if (!map.inflated(inflateCm)[k]) return cost;
  if (start) {
    const c = map.centre(k);
    const near = Math.hypot(c.x - start.x, c.y - start.y) <= inflateCm + map.cellCm;
    if (near && d >= map.clearance(start.x, start.y) - 1e-6) return cost * ESCAPE_COST;
  }
  return Infinity;
}

function snapGoal(map, to, opts, start) {
  const k0 = map.index(to.x, to.y);
  if (k0 >= 0 && cellCost(map, k0, opts, start) < Infinity) return k0;
  const r = Math.ceil(SNAP_CM / map.cellCm), ci = map.col(to.x), cj = map.row(to.y);
  let best = -1, bestD = Infinity;
  for (let j = cj - r; j <= cj + r; j++) {
    for (let i = ci - r; i <= ci + r; i++) {
      if (!map.inside(i, j)) continue;
      const k = j * map.n + i, c = map.centre(k);
      const d = Math.hypot(c.x - to.x, c.y - to.y);
      // prefer known free cells over unknown ones at a similar distance
      const score = d + (map.stateOf(k) === 'unknown' ? map.cellCm : 0);
      if (d <= SNAP_CM && score < bestD && cellCost(map, k, opts, start) < Infinity) { best = k; bestD = score; }
    }
  }
  return best;
}

export function planPath(map, from, to, { inflateCm = DEFAULT_INFLATE_CM, allowUnknown = true } = {}) {
  const opts = { inflateCm, allowUnknown };
  const n = map.n;
  const sk = map.index(from.x, from.y);
  if (sk < 0) return null;
  const gk = snapGoal(map, to, opts, from);
  if (gk < 0) return null;
  const gi = gk % n, gj = (gk - gi) / n;
  const h = (k) => {
    const i = k % n, j = (k - i) / n;
    const dx = Math.abs(i - gi), dy = Math.abs(j - gj);
    return Math.max(dx, dy) + (SQRT2 - 1) * Math.min(dx, dy);
  };
  const g = new Float64Array(n * n).fill(Infinity);
  const prev = new Int32Array(n * n).fill(-1);
  const closed = new Uint8Array(n * n);
  const costs = new Float32Array(n * n).fill(-1); // memo of cellCost
  const cost = (k) => {
    if (k === sk) return 1;
    if (costs[k] < 0) costs[k] = cellCost(map, k, opts, from);
    return costs[k];
  };
  const heap = new Heap();
  g[sk] = 0;
  heap.push(h(sk), sk);
  while (heap.size) {
    const [, k] = heap.pop();
    if (closed[k]) continue;
    closed[k] = 1;
    if (k === gk) break;
    const i = k % n, j = (k - i) / n;
    for (let dj = -1; dj <= 1; dj++) {
      for (let di = -1; di <= 1; di++) {
        if (!di && !dj) continue;
        const ni = i + di, nj = j + dj;
        if (!map.inside(ni, nj)) continue;
        const nk = nj * n + ni;
        if (closed[nk]) continue;
        const c = cost(nk);
        if (c === Infinity) continue;
        // no corner cutting past a blocked cell
        if (di && dj && (cost(j * n + ni) === Infinity || cost(nj * n + i) === Infinity)) continue;
        const ng = g[k] + (di && dj ? SQRT2 : 1) * c;
        if (ng < g[nk]) { g[nk] = ng; prev[nk] = k; heap.push(ng + h(nk), nk); }
      }
    }
  }
  if (!closed[gk]) return null;
  const path = [];
  for (let k = gk; k >= 0; k = prev[k]) path.push(map.centre(k));
  path.reverse();
  path[0] = { x: from.x, y: from.y };
  return path;
}

// True if the straight segment a-b only crosses usable cells.
export function lineOfSight(map, a, b, { inflateCm = DEFAULT_INFLATE_CM, allowUnknown = true } = {}, start = a) {
  const opts = { inflateCm, allowUnknown };
  const d = Math.hypot(b.x - a.x, b.y - a.y);
  const steps = Math.max(1, Math.ceil(d / (map.cellCm / 2)));
  for (let s = 0; s <= steps; s++) {
    const x = a.x + ((b.x - a.x) * s) / steps, y = a.y + ((b.y - a.y) * s) / steps;
    if (cellCost(map, map.index(x, y), opts, start) === Infinity) return false;
  }
  return true;
}

// Greedy line-of-sight pruning: from each kept point jump to the farthest
// visible one whose shortcut keeps its distance from obstacles (confirmed or
// suspect): every point of the shortcut must be as clear as the planned path
// near the matching position (up to the soft margin), so simplification never
// cuts closer past an obstacle than A* went.
export function simplifyPath(path, map, opts = {}) {
  if (!path || path.length <= 2) return path ? [...path] : path;
  const want = (opts.inflateCm ?? DEFAULT_INFLATE_CM) + CLEAR_CM;
  const df = map.distanceField(L_SUSPECT);
  const at = (x, y) => { const k = map.index(x, y); return k < 0 ? 0 : df[k]; };
  const clear = path.map((p) => at(p.x, p.y));
  const W = 3;  // path points either side of the matching position
  const keeps = (ia, ib) => {
    const A = path[ia], B = path[ib];
    const L = Math.hypot(B.x - A.x, B.y - A.y), steps = Math.max(1, Math.ceil(L / (map.cellCm / 2)));
    for (let s = 0; s <= steps; s++) {
      const j = Math.round(ia + ((ib - ia) * s) / steps);
      let need = want;
      for (let i = Math.max(ia, j - W); i <= Math.min(ib, j + W); i++) need = Math.min(need, clear[i]);
      if (at(A.x + ((B.x - A.x) * s) / steps, A.y + ((B.y - A.y) * s) / steps) < need - map.cellCm / 2) return false;
    }
    return true;
  };
  const out = [path[0]];
  let a = 0;
  while (a < path.length - 1) {
    let b = path.length - 1;
    while (b > a + 1 && !(lineOfSight(map, path[a], path[b], opts, path[0]) && keeps(a, b))) b--;
    out.push(path[b]);
    a = b;
  }
  return out;
}

// Turn + straight moves from the pose along the path (path[0] is the start).
// Long segments are split into pieces of at most maxSegCm; only the first
// piece of a segment turns. Turns are whole degrees, -180..180.
export function pathToMoves(pose, path, { maxSegCm = 40 } = {}) {
  const moves = [];
  let x = pose.x, y = pose.y, h = pose.heading ?? 0;
  for (const p of (path ?? []).slice(1)) {
    const dist = Math.hypot(p.x - x, p.y - y);
    if (dist < 1) continue;
    const turnDeg = Math.round(normDeg((Math.atan2(p.x - x, p.y - y) * 180) / Math.PI - h));
    h = normDeg(h + turnDeg);
    const pieces = Math.ceil(dist / maxSegCm);
    let left = Math.round(dist);
    for (let q = 0; q < pieces; q++) {
      const cm = Math.round(left / (pieces - q));
      left -= cm;
      if (cm < 1) continue;
      x += cm * Math.sin((h * Math.PI) / 180);
      y += cm * Math.cos((h * Math.PI) / 180);
      moves.push({ turnDeg: q === 0 ? turnDeg : 0, cm, to: { x: Math.round(x * 10) / 10, y: Math.round(y * 10) / 10 } });
    }
  }
  return moves;
}
