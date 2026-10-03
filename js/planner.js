// Path planning on a GridMap: A* (8-connected, octile heuristic) on cells,
// line-of-sight simplification and conversion to turn + straight moves.

const SQRT2 = Math.SQRT2;
const UNKNOWN_COST = 3;   // unknown cells cost this much more than free ones
const ESCAPE_COST = 5;    // inflated cells next to the start (robot already sits there)
const SNAP_CM = 30;       // a blocked goal moves to the nearest traversable cell within this

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

// Cost multiplier of entering cell k, or Infinity if blocked. Cells within
// inflateCm of the start that are not occupied stay usable (at a high cost)
// so the robot can leave a spot next to an obstacle.
function cellCost(map, k, { inflateCm, allowUnknown }, start) {
  if (k < 0) return Infinity;
  const s = map.stateOf(k);
  if (s === 'occupied') return Infinity;
  if (s === 'unknown' && !allowUnknown) return Infinity;
  const base = s === 'unknown' ? UNKNOWN_COST : 1;
  if (!map.inflated(inflateCm)[k]) return base;
  if (start) {
    const c = map.centre(k);
    if (Math.hypot(c.x - start.x, c.y - start.y) <= inflateCm + map.cellCm) return base * ESCAPE_COST;
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

export function planPath(map, from, to, { inflateCm = 14, allowUnknown = true } = {}) {
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
export function lineOfSight(map, a, b, { inflateCm = 14, allowUnknown = true } = {}, start = a) {
  const opts = { inflateCm, allowUnknown };
  const d = Math.hypot(b.x - a.x, b.y - a.y);
  const steps = Math.max(1, Math.ceil(d / (map.cellCm / 2)));
  for (let s = 0; s <= steps; s++) {
    const x = a.x + ((b.x - a.x) * s) / steps, y = a.y + ((b.y - a.y) * s) / steps;
    if (cellCost(map, map.index(x, y), opts, start) === Infinity) return false;
  }
  return true;
}

// Greedy line-of-sight pruning: from each kept point jump to the farthest visible one.
export function simplifyPath(path, map, opts = {}) {
  if (!path || path.length <= 2) return path ? [...path] : path;
  const out = [path[0]];
  let a = 0;
  while (a < path.length - 1) {
    let b = path.length - 1;
    while (b > a + 1 && !lineOfSight(map, path[a], path[b], opts, path[0])) b--;
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
