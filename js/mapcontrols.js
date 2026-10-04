// Zoom, pan and rotation for the map canvas: mouse wheel zooms around the
// cursor, Shift+wheel rotates in 5° steps, right-drag or Shift+drag rotates
// around the canvas centre, two fingers pinch-zoom, pan and twist around their
// midpoint, one finger or the mouse drags, and a short press without movement
// is a tap. The math is in pure functions on the view { cx, cy, cmPerPx, rot }
// so it can be tested without a DOM.
//
// Modes: "auto" refits the whole map on every redraw (the default until the
// user zooms or pans, and again after fit()); "follow" keeps the robot centred
// at the current zoom; "heading up" turns the map so the robot's heading points
// up. Dragging or pinching ends auto and follow; rotating by hand ends heading up.

import { screenToWorld, worldToScreen, anchorPx, clientToPx, compassHit, normDeg } from './mapview.js';

export const TAP_PX = 8;      // CSS pixels of movement that still count as a tap
export const TAP_MS = 400;
export const TWIST_DEG = 10;  // two-finger twist needed before a pinch also rotates
export const WHEEL_ROT_DEG = 5;
export const LIMITS = { minCmPerPx: 0.5, maxCmPerPx: 20 };

const clamp = (v, lo, hi) => Math.min(hi, Math.max(lo, v));
const finite = (v) => typeof v === 'number' && Number.isFinite(v);
const deg = (r) => (r * 180) / Math.PI;

function rectOf(canvas) {
  const r = canvas.getBoundingClientRect?.();
  return r && r.width > 0 && r.height > 0 ? r : { left: 0, top: 0, width: canvas.width, height: canvas.height };
}

const centreClient = (canvas) => { const r = rectOf(canvas); return { x: r.left + r.width / 2, y: r.top + r.height / 2 }; };

// A view at scale cmPerPx (and rotation rot) that shows the world point under the client point.
export function anchorView(canvas, view, world, clientX, clientY, cmPerPx, rot = view.rot ?? 0) {
  const p = clientToPx(canvas, clientX, clientY);
  return anchorPx(canvas, view, world, p.x, p.y, cmPerPx, normDeg(rot));
}

// factor > 1 zooms out. The world point under the cursor stays put.
export function zoomAt(canvas, view, clientX, clientY, factor, limits = LIMITS) {
  const s = clamp(view.cmPerPx * factor, limits.minCmPerPx, limits.maxCmPerPx);
  return anchorView(canvas, view, screenToWorld(canvas, view, clientX, clientY), clientX, clientY, s);
}

// Moves the content with the pointer by (dx, dy) client pixels.
export function panBy(canvas, view, dx, dy) {
  const c = centreClient(canvas);
  return anchorView(canvas, view, screenToWorld(canvas, view, c.x, c.y), c.x + dx, c.y + dy, view.cmPerPx);
}

// Turns the map clockwise on screen by d degrees around a client point
// (default: the canvas centre); the world point there stays put.
export function rotateBy(canvas, view, d, aroundClient = centreClient(canvas)) {
  const { x, y } = aroundClient;
  return anchorView(canvas, view, screenToWorld(canvas, view, x, y), x, y, view.cmPerPx, (view.rot ?? 0) + d);
}

// Screen angle (degrees, clockwise, y down) of the line from a to b.
const angleOf = (a, b) => deg(Math.atan2(b.y - a.y, b.x - a.x));

// Two fingers moved from (a0, b0) to (a1, b1): the world point under the old
// midpoint follows the new midpoint, the scale follows the finger spread and,
// with twist true, the rotation follows the angle between the fingers.
export function pinchView(canvas, view, a0, b0, a1, b1, limits = LIMITS, { twist = false } = {}) {
  const d0 = Math.hypot(a0.x - b0.x, a0.y - b0.y), d1 = Math.hypot(a1.x - b1.x, a1.y - b1.y);
  const m0 = { x: (a0.x + b0.x) / 2, y: (a0.y + b0.y) / 2 }, m1 = { x: (a1.x + b1.x) / 2, y: (a1.y + b1.y) / 2 };
  const ok = d0 > 1 && d1 > 1;
  const s = ok ? clamp((view.cmPerPx * d0) / d1, limits.minCmPerPx, limits.maxCmPerPx) : view.cmPerPx;
  const rot = (view.rot ?? 0) + (twist && ok ? normDeg(angleOf(a1, b1) - angleOf(a0, b0)) : 0);
  return anchorView(canvas, view, screenToWorld(canvas, view, m0.x, m0.y), m1.x, m1.y, s, rot);
}

// Wheel delta in pixels (lines and pages converted) to a zoom factor.
export function wheelFactor(e) {
  const unit = e.deltaMode === 1 ? 16 : e.deltaMode === 2 ? 400 : 1;
  return Math.exp(clamp((Number(e.deltaY) || 0) * unit, -300, 300) * 0.0015);
}

// Shift+wheel: browsers often report it as horizontal scrolling.
const wheelRotate = (e) => { const d = Number(e.deltaY) || Number(e.deltaX) || 0; return d ? Math.sign(d) * WHEEL_ROT_DEG : 0; };

// Pointer state machine. Feed it pointer-like objects
// { pointerId, clientX, clientY, timeStamp, button, pointerType, shiftKey }.
export function createMapGestures({ canvas, getView, setView, onTap, onChange, limits = LIMITS, now = () => performance.now() }) {
  const pts = new Map(); // pointerId -> { x, y }
  let start = null;      // single-pointer gesture: { id, x, y, t, view, dragging, rotate }
  let multi = false;     // a second finger was down during this gesture
  let twist = 0, twisting = false; // accumulated two-finger twist of this pinch
  const state = { auto: true, follow: false, headingUp: false };

  const apply = (v) => {
    state.auto = false;
    setView(v);
    onChange?.(v);
  };
  const ts = (e) => (Number.isFinite(e.timeStamp) && e.timeStamp > 0 ? e.timeStamp : now());

  function down(e) {
    const mouse = e.pointerType === 'mouse';
    if (mouse && e.button !== 0 && e.button !== 2) return false;
    pts.set(e.pointerId, { x: e.clientX, y: e.clientY });
    if (pts.size === 1) {
      multi = false;
      const rotate = (mouse && e.button === 2) || !!e.shiftKey;
      start = { id: e.pointerId, x: e.clientX, y: e.clientY, t: ts(e), view: getView(), dragging: false, rotate };
    } else {
      multi = true;
      start = null;
      twist = 0; twisting = false;
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
      const a1 = pts.get(ida), b1 = pts.get(idb);
      const view = getView();
      if (view) {
        state.follow = false;
        let v = pinchView(canvas, view, a0, b0, a1, b1, limits, { twist: twisting });
        // small twists while zooming are ignored until they add up to TWIST_DEG
        if (!twisting && Math.abs((twist += normDeg(angleOf(a1, b1) - angleOf(a0, b0)))) >= TWIST_DEG) {
          twisting = true;
          v = rotateBy(canvas, v, twist, { x: (a1.x + b1.x) / 2, y: (a1.y + b1.y) / 2 });
        }
        if (twisting) state.headingUp = false;
        apply(v);
      }
      return true;
    }
    pts.set(e.pointerId, cur);
    if (!start || start.id !== e.pointerId || !start.view) return true;
    const dx = cur.x - start.x, dy = cur.y - start.y;
    if (!start.dragging && Math.hypot(dx, dy) < TAP_PX) return true;
    start.dragging = true;
    if (start.rotate) {
      // turn by the pointer's angle change around the centre (where the robot is in follow mode)
      const c = centreClient(canvas);
      state.headingUp = false;
      apply(rotateBy(canvas, start.view, normDeg(angleOf(c, cur) - angleOf(c, start))));
      return true;
    }
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
      start = { id, x: p.x, y: p.y, t: ts(e), view: getView(), dragging: true, rotate: false };
      return true;
    }
    start = null;
    if (pts.size || multi || !s || s.id !== e.pointerId || s.rotate) return true;
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
    const c = centreClient(canvas);
    const at = state.follow ? c : { x: e.clientX, y: e.clientY }; // follow: keep the robot in the middle
    if (e.shiftKey) {
      const d = wheelRotate(e);
      if (!d) return;
      state.headingUp = false;
      apply(rotateBy(canvas, view, d, at));
      return;
    }
    apply(zoomAt(canvas, view, at.x, at.y, wheelFactor(e), limits));
  }

  return { down, move, up, cancel, wheel, state };
}

// opts: { getView, setView, onTap, onChange, onCompass, fit: (rot) => view, limits }.
// Call viewFor(pose) on every redraw to get the view to draw with. A tap on
// the compass (top right) resets the rotation and calls onCompass instead of onTap.
export function attachMapControls(canvas, opts) {
  const rotOf = () => opts.getView()?.rot ?? 0;
  const fitted = (rot) => { const v = opts.fit?.(rot); return v ? { ...v, rot } : v; };
  const changed = () => opts.onChange?.(opts.getView());
  let ctl = null;
  const onTap = (x, y) => {
    if (compassHit(canvas, x, y)) { ctl.resetRotation(); opts.onCompass?.(); return; }
    opts.onTap?.(x, y);
  };
  const g = createMapGestures({ canvas, ...opts, onTap });
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
    on('contextmenu', (e) => e.preventDefault()), // right-drag rotates
  ];

  // auto mode refits at the new angle; otherwise turn around the canvas centre
  const setRot = (rot) => {
    const v = opts.getView();
    if (!v) return;
    opts.setView(g.state.auto ? fitted(normDeg(rot)) ?? { ...v, rot: normDeg(rot) } : rotateBy(canvas, v, normDeg(rot - (v.rot ?? 0))));
  };

  ctl = {
    detach: () => offs.forEach((off) => off()),
    fit() {
      g.state.auto = true;
      const v = fitted(rotOf());
      if (v) { opts.setView(v); changed(); }
    },
    setFollow(on) {
      g.state.follow = !!on;
      if (on) g.state.auto = false;
      changed();
    },
    get follow() { return g.state.follow; },
    get auto() { return g.state.auto; },
    // The map turns so the robot's heading points up, on every redraw.
    setHeadingUp(on) { g.state.headingUp = !!on; changed(); },
    get headingUp() { return g.state.headingUp; },
    // By hand, around the canvas centre (the robot in follow mode); ends heading up.
    rotateBy(d) { g.state.headingUp = false; setRot(rotOf() + d); changed(); },
    resetRotation() { g.state.headingUp = false; setRot(0); changed(); },
    get rot() { return rotOf(); },
    // The view for the next redraw: refit in auto mode, recentre in follow
    // mode, and in heading-up mode turn around the robot's screen position.
    viewFor(pose) {
      const hasPose = pose && finite(pose.x) && finite(pose.y);
      let v = opts.getView();
      const rot = g.state.headingUp && hasPose && finite(pose.heading) ? normDeg(-pose.heading) : v?.rot ?? 0;
      if (g.state.auto || !v) v = fitted(rot) ?? v;
      else if (hasPose && (g.state.follow || rot !== (v.rot ?? 0))) {
        const s = g.state.follow ? centreClient(canvas) : worldToScreen(canvas, v, pose.x, pose.y);
        v = anchorView(canvas, v, pose, s.x, s.y, v.cmPerPx, rot);
      }
      if (v) opts.setView(v);
      return v;
    },
  };
  return ctl;
}
