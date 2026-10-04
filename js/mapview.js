// Top-down occupancy map: start at the origin, forward (+y) = up, x = right.
// Colours come from the page's CSS custom properties (light and dark mode).
// A view { cx, cy, cmPerPx, rot } is the world point at the canvas centre, the
// scale in canvas (backing store) pixels and the map's rotation on screen in
// degrees clockwise (0 = forward from the start points up; rot = -heading puts
// the robot's heading up). Client coordinates are mapped via
// getBoundingClientRect so CSS scaling and devicePixelRatio stay consistent.

import { NO_ECHO_CM } from './scan.js';

const SENSOR_CM = 6; // ultrasonic sensor ahead of the robot centre
const ROBOT_CM = 17; // robot length, for the marker

const FALLBACK = {
  '--bg': '#f4f5f7', '--surface': '#ffffff', '--border': '#d5d9e0', '--text': '#1b1f27',
  '--muted': '#667085', '--accent': '#2f6fde', '--stop': '#d92d20', '--ok': '#12805c', '--warn': '#b54708' };

function colours() {
  let css = null;
  try {
    if (typeof document !== 'undefined' && typeof getComputedStyle === 'function') css = getComputedStyle(document.documentElement);
  } catch { /* no DOM */ }
  const out = {};
  for (const [name, fb] of Object.entries(FALLBACK)) out[name.slice(2)] = css?.getPropertyValue(name).trim() || fb;
  return out;
}

const finite = (v) => typeof v === 'number' && Number.isFinite(v);

function rectOf(canvas) {
  const r = canvas.getBoundingClientRect?.();
  return r && r.width > 0 && r.height > 0 ? r : { left: 0, top: 0, width: canvas.width, height: canvas.height };
}

const rad = (d) => (d * Math.PI) / 180;
export const normDeg = (a) => { const r = ((((a + 180) % 360) + 360) % 360) - 180; return r === -180 ? 180 : r; };

// world offset from the view centre -> screen offset (y up), turned clockwise by rot
function turn(dx, dy, rot) {
  if (!rot) return { x: dx, y: dy };
  const c = Math.cos(rad(rot)), s = Math.sin(rad(rot));
  return { x: dx * c + dy * s, y: -dx * s + dy * c };
}

// world cm -> canvas pixels
function toPx(canvas, view, x, y) {
  const r = turn(x - view.cx, y - view.cy, view.rot);
  return { x: canvas.width / 2 + r.x / view.cmPerPx, y: canvas.height / 2 - r.y / view.cmPerPx };
}

// canvas pixels -> world cm
function fromPx(canvas, view, px, py) {
  const r = turn((px - canvas.width / 2) * view.cmPerPx, -(py - canvas.height / 2) * view.cmPerPx, -(view.rot || 0));
  return { x: view.cx + r.x, y: view.cy + r.y };
}

// The view at scale cmPerPx and rotation rot that shows the world point at the
// canvas pixel (px, py).
export function anchorPx(canvas, view, world, px, py, cmPerPx = view.cmPerPx, rot = view.rot ?? 0) {
  const r = turn((px - canvas.width / 2) * cmPerPx, -(py - canvas.height / 2) * cmPerPx, -rot);
  return { ...view, cmPerPx, rot, cx: world.x - r.x, cy: world.y - r.y };
}

export function clientToPx(canvas, clientX, clientY) {
  const r = rectOf(canvas);
  return { x: ((clientX - r.left) * canvas.width) / r.width, y: ((clientY - r.top) * canvas.height) / r.height };
}

export function worldToScreen(canvas, view, x, y) {
  const r = rectOf(canvas);
  const p = toPx(canvas, view, x, y);
  return { x: r.left + (p.x * r.width) / canvas.width, y: r.top + (p.y * r.height) / canvas.height };
}

export function screenToWorld(canvas, view, clientX, clientY) {
  const p = clientToPx(canvas, clientX, clientY);
  return fromPx(canvas, view, p.x, p.y);
}

// Compass in the top right corner, in canvas pixels; the lead resets the
// rotation when it is tapped (see compassHit).
export function compassOf(canvas) {
  const dpr = Math.max(1, canvas.width / rectOf(canvas).width);
  const r = 14 * dpr;
  return { x: canvas.width - r - 8 * dpr, y: r + 8 * dpr, r };
}

export function compassHit(canvas, clientX, clientY) {
  const c = compassOf(canvas), p = clientToPx(canvas, clientX, clientY);
  return Math.hypot(p.x - c.x, p.y - c.y) <= c.r * 1.4;
}

// Centre the known map, the start and the robot, with a margin, at rotation rot.
export function fitView(canvas, map, pose, { marginCm = 40, minSpanCm = 200, rot = 0 } = {}) {
  let minX = 0, maxX = 0, minY = 0, maxY = 0; // always include the start
  const b = map?.bounds;
  if (b && [b.minX, b.maxX, b.minY, b.maxY].every(finite) && b.minX <= b.maxX && b.minY <= b.maxY) {
    minX = Math.min(minX, b.minX); maxX = Math.max(maxX, b.maxX);
    minY = Math.min(minY, b.minY); maxY = Math.max(maxY, b.maxY);
  }
  if (pose && finite(pose.x) && finite(pose.y)) {
    minX = Math.min(minX, pose.x); maxX = Math.max(maxX, pose.x);
    minY = Math.min(minY, pose.y); maxY = Math.max(maxY, pose.y);
  }
  // extent of the box on screen when turned by rot
  const hx = (maxX - minX) / 2, hy = (maxY - minY) / 2;
  const c = Math.abs(Math.cos(rad(rot))), s = Math.abs(Math.sin(rad(rot)));
  const spanX = Math.max(minSpanCm, 2 * (hx * c + hy * s) + 2 * marginCm);
  const spanY = Math.max(minSpanCm, 2 * (hx * s + hy * c) + 2 * marginCm);
  const w = canvas.width || 1, h = canvas.height || 1;
  return { cx: (minX + maxX) / 2, cy: (minY + maxY) / 2, cmPerPx: Math.max(spanX / w, spanY / h), rot };
}

// Screen direction of a heading (clockwise from up), y down.
const dir = (deg) => { const a = (deg * Math.PI) / 180; return { x: Math.sin(a), y: -Math.cos(a) }; };

export function drawMap(canvas, map, pose, { path, goal, frontiers, trail, lastScan, view } = {}) {
  const ctx = canvas.getContext('2d');
  if (!ctx) return;
  const c = colours();
  const w = canvas.width, h = canvas.height;
  const v = view ?? fitView(canvas, map, pose);
  const P = (x, y) => toPx(canvas, v, x, y);
  const rot = v.rot || 0;
  const sdir = (deg) => dir(deg + rot); // screen direction of a map heading
  const dpr = Math.max(1, w / rectOf(canvas).width); // backing pixels per CSS pixel
  const font = Math.round(11 * dpr);

  ctx.save();
  ctx.fillStyle = c.bg;
  ctx.fillRect(0, 0, w, h);

  // cells: free in one path, occupied in a few alpha buckets
  const cellCm = map?.cellCm ?? 5;
  const cpx = cellCm / v.cmPerPx + 0.5; // slight overlap hides seams
  if (map?.forEachCell) {
    const free = [];
    const occ = [[], [], [], []];
    const suspect = []; // hit evidence not yet confirmed by a second scan (evidence model v2)
    map.forEachCell((x, y, state, p, kind) => {
      if (state === 'unknown' && kind !== 'suspect') return;
      const q = P(x, y);
      if (q.x < -cpx || q.y < -cpx || q.x > w + cpx || q.y > h + cpx) return;
      if (rot) { q.x -= w / 2; q.y -= h / 2; const u = turn(q.x, -q.y, -rot); q.x = u.x; q.y = -u.y; } // unturned, about the centre
      if (kind === 'suspect') suspect.push(q);
      else if (state === 'occupied') {
        const k = finite(p) && p >= 0 && p <= 1 ? Math.min(3, Math.max(0, Math.floor((p - 0.5) * 8))) : 3;
        occ[k].push(q);
      } else free.push(q);
    });
    // cells are drawn axis-aligned in a context turned about the canvas centre
    if (rot) { ctx.save(); ctx.translate(w / 2, h / 2); ctx.rotate(rad(rot)); }
    const fill = (list, style, alpha) => {
      if (!list.length) return;
      ctx.globalAlpha = alpha;
      ctx.fillStyle = style;
      ctx.beginPath();
      for (const q of list) ctx.rect(q.x - cpx / 2, q.y - cpx / 2, cpx, cpx);
      ctx.fill();
    };
    fill(free, c.surface, 1);
    fill(suspect, c.warn, 0.45);
    occ.forEach((list, k) => fill(list, c.text, 0.4 + k * 0.2));
    ctx.globalAlpha = 1;
    if (rot) ctx.restore();
  }

  // 1 m grid
  ctx.strokeStyle = c.border;
  ctx.lineWidth = dpr;
  ctx.beginPath();
  const corners = [[0, 0], [w, 0], [0, h], [w, h]].map(([px, py]) => fromPx(canvas, v, px, py));
  const x0 = Math.min(...corners.map((q) => q.x)), x1 = Math.max(...corners.map((q) => q.x));
  const y0 = Math.min(...corners.map((q) => q.y)), y1 = Math.max(...corners.map((q) => q.y));
  const line = (a, b) => { const p = P(a[0], a[1]), q = P(b[0], b[1]); ctx.moveTo(p.x, p.y); ctx.lineTo(q.x, q.y); };
  for (let x = Math.ceil(x0 / 100) * 100; x <= x1; x += 100) line([x, y0], [x, y1]);
  for (let y = Math.ceil(y0 / 100) * 100; y <= y1; y += 100) line([x0, y], [x1, y]);
  ctx.stroke();

  // compass, top right: the arrow shows "forward from the start"
  const cp = compassOf(canvas), f0 = sdir(0), n0 = { x: -f0.y, y: f0.x };
  ctx.globalAlpha = 0.85;
  ctx.fillStyle = c.surface;
  ctx.strokeStyle = c.border;
  ctx.lineWidth = dpr;
  ctx.beginPath(); ctx.arc(cp.x, cp.y, cp.r, 0, Math.PI * 2); ctx.fill(); ctx.stroke();
  ctx.globalAlpha = 1;
  ctx.fillStyle = rot ? c.accent : c.muted;
  ctx.beginPath();
  ctx.moveTo(cp.x + f0.x * cp.r * 0.8, cp.y + f0.y * cp.r * 0.8);
  ctx.lineTo(cp.x + n0.x * cp.r * 0.35, cp.y + n0.y * cp.r * 0.35);
  ctx.lineTo(cp.x - f0.x * cp.r * 0.5, cp.y - f0.y * cp.r * 0.5);
  ctx.lineTo(cp.x - n0.x * cp.r * 0.35, cp.y - n0.y * cp.r * 0.35);
  ctx.closePath();
  ctx.fill();

  // scale bar, bottom left
  const m = 100 / v.cmPerPx;
  ctx.strokeStyle = c.muted;
  ctx.fillStyle = c.muted;
  ctx.lineWidth = 2 * dpr;
  const sx = 8 * dpr, sy = h - 8 * dpr;
  ctx.beginPath(); ctx.moveTo(sx, sy - 4 * dpr); ctx.lineTo(sx, sy); ctx.lineTo(sx + m, sy); ctx.lineTo(sx + m, sy - 4 * dpr); ctx.stroke();
  ctx.font = `${font}px system-ui`;
  ctx.textAlign = 'left';
  ctx.textBaseline = 'bottom';
  ctx.fillText('1 m', sx + 4 * dpr, sy - 3 * dpr);

  // last scan rays from the pose they were taken at
  const sp = lastScan?.pose;
  if (sp && finite(sp.x) && finite(sp.y) && Array.isArray(lastScan.points)) {
    ctx.strokeStyle = c.accent;
    ctx.fillStyle = c.accent;
    ctx.lineWidth = dpr;
    const dot = Math.max(2 * dpr, 2 / v.cmPerPx);
    for (const p of lastScan.points) {
      if (!finite(p?.cm) || !finite(p?.angle)) continue;
      const d = sdir((sp.heading ?? 0) + p.angle);
      const o = P(sp.x, sp.y);
      const r0 = SENSOR_CM / v.cmPerPx, r1 = (SENSOR_CM + Math.min(p.cm, NO_ECHO_CM)) / v.cmPerPx;
      ctx.globalAlpha = 0.35;
      ctx.beginPath(); ctx.moveTo(o.x + d.x * r0, o.y + d.y * r0); ctx.lineTo(o.x + d.x * r1, o.y + d.y * r1); ctx.stroke();
      ctx.globalAlpha = 1;
      if (p.cm < NO_ECHO_CM) { ctx.beginPath(); ctx.arc(o.x + d.x * r1, o.y + d.y * r1, dot, 0, Math.PI * 2); ctx.fill(); }
    }
  }

  // frontiers
  if (frontiers?.length) {
    ctx.fillStyle = c.muted;
    ctx.globalAlpha = 0.7;
    ctx.beginPath();
    for (const f of frontiers) {
      if (!finite(f?.x) || !finite(f?.y)) continue;
      const q = P(f.x, f.y);
      ctx.moveTo(q.x + 3 * dpr, q.y);
      ctx.arc(q.x, q.y, 3 * dpr, 0, Math.PI * 2);
    }
    ctx.fill();
    ctx.globalAlpha = 1;
  }

  const polyline = (pts) => {
    ctx.beginPath();
    pts.filter((p) => finite(p?.x) && finite(p?.y)).forEach((p, i) => { const q = P(p.x, p.y); ctx[i ? 'lineTo' : 'moveTo'](q.x, q.y); });
    ctx.stroke();
  };

  // trail
  if (trail?.length > 1) {
    ctx.strokeStyle = c.muted;
    ctx.lineWidth = 1.5 * dpr;
    ctx.globalAlpha = 0.8;
    polyline(trail);
    ctx.globalAlpha = 1;
  }

  // planned path
  if (path?.length > 1) {
    ctx.strokeStyle = c.accent;
    ctx.lineWidth = 2.5 * dpr;
    ctx.setLineDash([6 * dpr, 5 * dpr]);
    polyline(path);
    ctx.setLineDash([]);
  }

  // start marker: a small house at the origin
  const o = P(0, 0), s = 6 * dpr;
  ctx.strokeStyle = c.ok;
  ctx.lineWidth = 1.5 * dpr;
  ctx.beginPath();
  ctx.moveTo(o.x - s, o.y + s); ctx.lineTo(o.x - s, o.y - s * 0.2); ctx.lineTo(o.x, o.y - s);
  ctx.lineTo(o.x + s, o.y - s * 0.2); ctx.lineTo(o.x + s, o.y + s); ctx.closePath();
  ctx.stroke();

  // goal: ring with a cross
  if (goal && finite(goal.x) && finite(goal.y)) {
    const g = P(goal.x, goal.y), r = 7 * dpr;
    ctx.strokeStyle = c.stop;
    ctx.lineWidth = 2 * dpr;
    ctx.beginPath(); ctx.arc(g.x, g.y, r, 0, Math.PI * 2); ctx.stroke();
    ctx.beginPath(); ctx.moveTo(g.x - r * 0.5, g.y); ctx.lineTo(g.x + r * 0.5, g.y); ctx.moveTo(g.x, g.y - r * 0.5); ctx.lineTo(g.x, g.y + r * 0.5); ctx.stroke();
  }

  // robot: arrow pointing along the heading, true size with a minimum
  if (pose && finite(pose.x) && finite(pose.y)) {
    const q = P(pose.x, pose.y);
    const len = Math.max(10 * dpr, ROBOT_CM / v.cmPerPx);
    const f = sdir(pose.heading ?? 0), side = { x: -f.y, y: f.x };
    ctx.fillStyle = c.text;
    ctx.strokeStyle = c.surface;
    ctx.lineWidth = dpr;
    ctx.beginPath();
    ctx.moveTo(q.x + f.x * len * 0.6, q.y + f.y * len * 0.6);
    ctx.lineTo(q.x - f.x * len * 0.4 + side.x * len * 0.4, q.y - f.y * len * 0.4 + side.y * len * 0.4);
    ctx.lineTo(q.x - f.x * len * 0.2, q.y - f.y * len * 0.2);
    ctx.lineTo(q.x - f.x * len * 0.4 - side.x * len * 0.4, q.y - f.y * len * 0.4 - side.y * len * 0.4);
    ctx.closePath();
    ctx.fill();
    ctx.stroke();
  }
  ctx.restore();
}
