// Zoom and pan for the map canvas: mouse wheel zooms around the cursor, two
// fingers pinch-zoom around their midpoint, one finger or the mouse drags, and
// a short press without movement is a tap. The math is in pure functions on
// the view { cx, cy, cmPerPx } so it can be tested without a DOM.
//
// Modes: "auto" refits the whole map on every redraw (the default until the
// user zooms or pans, and again after fit()); "follow" keeps the robot centred
// at the current zoom. Dragging or pinching ends both.

import { screenToWorld } from './mapview.js';

export const TAP_PX = 8;      // CSS pixels of movement that still count as a tap
export const TAP_MS = 400;
export const LIMITS = { minCmPerPx: 0.5, maxCmPerPx: 20 };

const clamp = (v, lo, hi) => Math.min(hi, Math.max(lo, v));

function rectOf(canvas) {
  const r = canvas.getBoundingClientRect?.();
  return r && r.width > 0 && r.height > 0 ? r : { left: 0, top: 0, width: canvas.width, height: canvas.height };
}

// A view at scale cmPerPx that shows the world point under the client point.
export function anchorView(canvas, view, world, clientX, clientY, cmPerPx) {
  const r = rectOf(canvas);
  const px = ((clientX - r.left) * canvas.width) / r.width;
  const py = ((clientY - r.top) * canvas.height) / r.height;
  return { ...view, cmPerPx, cx: world.x - (px - canvas.width / 2) * cmPerPx, cy: world.y + (py - canvas.height / 2) * cmPerPx };
}

// factor > 1 zooms out. The world point under the cursor stays put.
export function zoomAt(canvas, view, clientX, clientY, factor, limits = LIMITS) {
  const s = clamp(view.cmPerPx * factor, limits.minCmPerPx, limits.maxCmPerPx);
  return anchorView(canvas, view, screenToWorld(canvas, view, clientX, clientY), clientX, clientY, s);
}

// Moves the content with the pointer by (dx, dy) client pixels.
export function panBy(canvas, view, dx, dy) {
  const r = rectOf(canvas);
  return {
    ...view,
    cx: view.cx - ((dx * canvas.width) / r.width) * view.cmPerPx,
    cy: view.cy + ((dy * canvas.height) / r.height) * view.cmPerPx,
  };
}

// Two fingers moved from (a0, b0) to (a1, b1): the world point under the old
// midpoint follows the new midpoint, and the scale follows the finger spread.
export function pinchView(canvas, view, a0, b0, a1, b1, limits = LIMITS) {
  const d0 = Math.hypot(a0.x - b0.x, a0.y - b0.y), d1 = Math.hypot(a1.x - b1.x, a1.y - b1.y);
  const m0 = { x: (a0.x + b0.x) / 2, y: (a0.y + b0.y) / 2 }, m1 = { x: (a1.x + b1.x) / 2, y: (a1.y + b1.y) / 2 };
  const s = d0 > 1 && d1 > 1 ? clamp((view.cmPerPx * d0) / d1, limits.minCmPerPx, limits.maxCmPerPx) : view.cmPerPx;
  return anchorView(canvas, view, screenToWorld(canvas, view, m0.x, m0.y), m1.x, m1.y, s);
}

// Wheel delta in pixels (lines and pages converted) to a zoom factor.
export function wheelFactor(e) {
  const unit = e.deltaMode === 1 ? 16 : e.deltaMode === 2 ? 400 : 1;
  return Math.exp(clamp((Number(e.deltaY) || 0) * unit, -300, 300) * 0.0015);
}

// Pointer state machine. Feed it pointer-like objects
// { pointerId, clientX, clientY, timeStamp, button, pointerType }.
export function createMapGestures({ canvas, getView, setView, onTap, onChange, limits = LIMITS, now = () => performance.now() }) {
  const pts = new Map(); // pointerId -> { x, y }
  let start = null;      // single-pointer gesture: { id, x, y, t, view, dragging }
  let multi = false;     // a second finger was down during this gesture
  const state = { auto: true, follow: false };

  const apply = (v, { manual = true } = {}) => {
    if (manual) { state.auto = false; }
    setView(v);
    onChange?.(v);
  };
  const ts = (e) => (Number.isFinite(e.timeStamp) && e.timeStamp > 0 ? e.timeStamp : now());

  function down(e) {
    if (e.pointerType === 'mouse' && e.button !== 0) return false;
    pts.set(e.pointerId, { x: e.clientX, y: e.clientY });
    if (pts.size === 1) {
      multi = false;
      start = { id: e.pointerId, x: e.clientX, y: e.clientY, t: ts(e), view: getView(), dragging: false };
    } else {
      multi = true;
      start = null;
    }
    return true;
  }

  function move(e) {
    const prev = pts.get(e.pointerId);
    if (!prev) return false;
    const cur = { x: e.clientX, y: e.clientY };
    if (pts.size >= 2) {
      const [ida, idb] = [...pts.keys()];
      const a0 = pts.get(ida), b0 = pts.get(idb);
      pts.set(e.pointerId, cur);
      const view = getView();
      if (view) {
        state.follow = false;
        apply(pinchView(canvas, view, a0, b0, pts.get(ida), pts.get(idb), limits));
      }
      return true;
    }
    pts.set(e.pointerId, cur);
    if (!start || start.id !== e.pointerId || !start.view) return true;
    const dx = cur.x - start.x, dy = cur.y - start.y;
    if (!start.dragging && Math.hypot(dx, dy) < TAP_PX) return true;
    start.dragging = true;
    state.follow = false;
    apply(panBy(canvas, start.view, dx, dy));
    return true;
  }

  function up(e) {
    if (!pts.has(e.pointerId)) return false;
    pts.delete(e.pointerId);
    const s = start;
    if (pts.size === 1) {
      // one finger of a pinch lifted: the other one continues as a drag, never a tap
      const [id, p] = [...pts.entries()][0];
      start = { id, x: p.x, y: p.y, t: ts(e), view: getView(), dragging: true };
      return true;
    }
    start = null;
    if (pts.size || multi || !s || s.id !== e.pointerId) return true;
    const moved = Math.hypot(e.clientX - s.x, e.clientY - s.y);
    if (!s.dragging && moved < TAP_PX && ts(e) - s.t < TAP_MS) onTap?.(e.clientX, e.clientY);
    return true;
  }

  function cancel(e) {
    if (!pts.delete(e.pointerId)) return; // lostpointercapture also follows a normal pointerup
    start = null;
    if (pts.size) multi = true;
  }

  function wheel(e) {
    const view = getView();
    if (!view) return;
    const f = wheelFactor(e);
    // in follow mode zoom around the centre so the robot stays in the middle
    const r = rectOf(canvas);
    const x = state.follow ? r.left + r.width / 2 : e.clientX, y = state.follow ? r.top + r.height / 2 : e.clientY;
    apply(zoomAt(canvas, view, x, y, f, limits));
  }

  return { down, move, up, cancel, wheel, state };
}

// opts: { getView, setView, onTap, onChange, fit: () => view, limits }.
// Call viewFor(pose) on every redraw to get the view to draw with.
export function attachMapControls(canvas, opts) {
  const g = createMapGestures({ canvas, ...opts });
  const on = (type, fn, o) => { canvas.addEventListener(type, fn, o); return () => canvas.removeEventListener(type, fn, o); };
  const offs = [
    on('pointerdown', (e) => {
      if (g.down(e)) { try { canvas.setPointerCapture?.(e.pointerId); } catch { /* not capturable */ } }
    }),
    on('pointermove', (e) => { if (g.move(e)) e.preventDefault(); }),
    on('pointerup', (e) => g.up(e)),
    on('pointercancel', (e) => g.cancel(e)),
    on('lostpointercapture', (e) => g.cancel(e)),
    on('wheel', (e) => { e.preventDefault(); g.wheel(e); }, { passive: false }),
  ];

  const fit = () => {
    g.state.auto = true;
    const v = opts.fit?.();
    if (v) { opts.setView(v); opts.onChange?.(v); }
  };

  return {
    detach: () => offs.forEach((off) => off()),
    fit,
    setFollow(on) {
      g.state.follow = !!on;
      if (on) g.state.auto = false;
      opts.onChange?.(opts.getView());
    },
    get follow() { return g.state.follow; },
    get auto() { return g.state.auto; },
    // The view for the next redraw: refit in auto mode, recentre in follow mode.
    viewFor(pose) {
      let v = opts.getView();
      if (g.state.auto || !v) v = opts.fit?.() ?? v;
      else if (g.state.follow && pose && Number.isFinite(pose.x) && Number.isFinite(pose.y)) v = { ...v, cx: pose.x, cy: pose.y };
      if (v) opts.setView(v);
      return v;
    },
  };
}
