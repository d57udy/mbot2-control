import { test } from 'node:test';
import assert from 'node:assert/strict';
import { drawMap, fitView, screenToWorld, worldToScreen } from '../js/mapview.js';
import { zoomAt, panBy, pinchView, wheelFactor, createMapGestures, attachMapControls, LIMITS } from '../js/mapcontrols.js';

// Records every call and property set; any method name works.
function fakeCtx() {
  const calls = [];
  const target = { calls };
  return new Proxy(target, {
    get(t, k) {
      if (k in t) return t[k];
      return (...args) => { calls.push([k, ...args]); };
    },
    set(t, k, v) { t[k] = v; calls.push(['set', k, v]); return true; },
  });
}

// 300 x 200 backing pixels shown at 150 x 100 CSS pixels (DPR 2), offset on the page
function fakeCanvas(ctx = fakeCtx()) {
  return {
    width: 300, height: 200, ctx,
    getContext: () => ctx,
    getBoundingClientRect: () => ({ left: 20, top: 50, width: 150, height: 100 }),
  };
}

function fakeMap(cells = [], bounds = null) {
  return {
    cellCm: 5,
    get bounds() { return bounds; },
    forEachCell(fn) { for (const c of cells) fn(c.x, c.y, c.state, c.p); },
  };
}

const close = (a, b, eps = 1e-9) => assert.ok(Math.abs(a - b) < eps, `${a} != ${b}`);

test('screen and world transforms round trip, up = +y, right = +x', () => {
  const canvas = fakeCanvas();
  const view = { cx: 30, cy: -40, cmPerPx: 2 };
  // centre of the element maps to the view centre
  const c = screenToWorld(canvas, view, 20 + 75, 50 + 50);
  close(c.x, 30); close(c.y, -40);
  // one CSS pixel = 2 backing pixels = 4 cm
  const right = screenToWorld(canvas, view, 96, 100);
  close(right.x, 34); close(right.y, -40);
  const up = screenToWorld(canvas, view, 95, 99);
  close(up.x, 30); close(up.y, -36);
  for (const [x, y] of [[0, 0], [123.5, -77], [-300, 250]]) {
    const s = worldToScreen(canvas, view, x, y);
    const w = screenToWorld(canvas, view, s.x, s.y);
    close(w.x, x, 1e-6); close(w.y, y, 1e-6);
  }
  assert.ok(worldToScreen(canvas, view, 0, 100).y < worldToScreen(canvas, view, 0, 0).y);
  assert.ok(worldToScreen(canvas, view, 100, 0).x > worldToScreen(canvas, view, 0, 0).x);
});

test('fitView covers bounds, start and robot', () => {
  const canvas = fakeCanvas();
  const empty = fitView(canvas, fakeMap(), null);
  assert.deepEqual([empty.cx, empty.cy], [0, 0]);
  close(empty.cmPerPx, 1); // 200 cm span over 200 px height
  const v = fitView(canvas, fakeMap([], { minX: -100, maxX: 300, minY: 0, maxY: 100 }), { x: 0, y: -200, heading: 0 });
  assert.deepEqual([v.cx, v.cy], [100, -50]);
  for (const [x, y] of [[-100, 100], [300, 0], [0, -200]]) {
    const s = worldToScreen(canvas, v, x, y);
    assert.ok(s.x >= 20 && s.x <= 170 && s.y >= 50 && s.y <= 150, `${x},${y} visible`);
  }
  // non-finite bounds of an empty map are ignored
  const inf = fitView(canvas, fakeMap([], { minX: Infinity, maxX: -Infinity, minY: Infinity, maxY: -Infinity }), { x: 50, y: 50 });
  assert.deepEqual([inf.cx, inf.cy], [25, 25]);
});

test('drawMap handles an empty map and missing extras', () => {
  const canvas = fakeCanvas();
  assert.doesNotThrow(() => drawMap(canvas, fakeMap(), { x: 0, y: 0, heading: 0 }));
  assert.doesNotThrow(() => drawMap(canvas, null, null));
  assert.doesNotThrow(() => drawMap(canvas, {}, { x: NaN, y: 1 }, { path: [], trail: [{ x: 0, y: 0 }], frontiers: [{}], lastScan: { points: [] } }));
  assert.ok(canvas.ctx.calls.some(([k]) => k === 'fillRect')); // background
});

test('drawMap draws cells, overlays and the robot', () => {
  const canvas = fakeCanvas();
  const cells = [
    { x: 0, y: 0, state: 'free', p: 0.1 }, { x: 5, y: 0, state: 'free', p: 0.2 },
    { x: 50, y: 50, state: 'occupied', p: 0.95 }, { x: 55, y: 50, state: 'occupied', p: 0.6 },
    { x: 10, y: 10, state: 'unknown', p: 0.5 }, { x: 9000, y: 0, state: 'occupied', p: 1 },
  ];
  const map = fakeMap(cells, { minX: 0, maxX: 55, minY: 0, maxY: 50 });
  drawMap(canvas, map, { x: 20, y: 30, heading: 90 }, {
    path: [{ x: 20, y: 30 }, { x: 40, y: 80 }], goal: { x: 40, y: 80 }, frontiers: [{ x: -20, y: 40, size: 4 }],
    trail: [{ x: 0, y: 0 }, { x: 20, y: 30 }],
    lastScan: { pose: { x: 0, y: 0, heading: 0 }, points: [{ angle: 0, cm: 40 }, { angle: 90, cm: 300 }, { angle: 180, cm: null }] },
  });
  const calls = canvas.ctx.calls;
  // 2 free + 2 occupied visible cells, the far one is culled, unknown skipped
  assert.equal(calls.filter(([k]) => k === 'rect').length, 4);
  assert.ok(calls.some(([k, v]) => k === 'setLineDash' && v.length)); // dashed path
  assert.ok(calls.some(([k, t]) => k === 'fillText' && t === '1 m'));
  assert.ok(calls.filter(([k]) => k === 'fill').length >= 4);
});

// --- zoom and pan (js/mapcontrols.js) ---

test('zoomAt keeps the world point under the cursor fixed and clamps the scale', () => {
  const canvas = fakeCanvas();
  const view = { cx: 30, cy: -40, cmPerPx: 2 };
  for (const [sx, sy] of [[40, 70], [95, 100], [160, 140]]) {
    const before = screenToWorld(canvas, view, sx, sy);
    const z = zoomAt(canvas, view, sx, sy, 0.5);
    close(z.cmPerPx, 1);
    const after = screenToWorld(canvas, z, sx, sy);
    close(after.x, before.x, 1e-9); close(after.y, before.y, 1e-9);
  }
  assert.equal(zoomAt(canvas, view, 50, 50, 1e-6).cmPerPx, LIMITS.minCmPerPx);
  assert.equal(zoomAt(canvas, view, 50, 50, 1e6).cmPerPx, LIMITS.maxCmPerPx);
  assert.ok(wheelFactor({ deltaY: 100 }) > 1 && wheelFactor({ deltaY: -100 }) < 1);
  close(wheelFactor({ deltaY: 3, deltaMode: 1 }), wheelFactor({ deltaY: 48 }));
});

test('panBy moves the content with the pointer', () => {
  const canvas = fakeCanvas();
  const view = { cx: 0, cy: 0, cmPerPx: 2 };
  const grab = screenToWorld(canvas, view, 60, 80);
  const v = panBy(canvas, view, 25, -10);
  const now = screenToWorld(canvas, v, 85, 70);
  close(now.x, grab.x); close(now.y, grab.y);
  close(v.cmPerPx, 2);
});

test('pinchView scales with the finger spread around the moving midpoint', () => {
  const canvas = fakeCanvas();
  const view = { cx: 10, cy: 10, cmPerPx: 4 };
  const a0 = { x: 60, y: 100 }, b0 = { x: 100, y: 100 };
  const mid0 = screenToWorld(canvas, view, 80, 100);
  const a1 = { x: 40, y: 110 }, b1 = { x: 120, y: 110 }; // twice the spread, midpoint moved down 10
  const v = pinchView(canvas, view, a0, b0, a1, b1);
  close(v.cmPerPx, 2);
  const mid1 = screenToWorld(canvas, v, 80, 110);
  close(mid1.x, mid0.x); close(mid1.y, mid0.y);
});

function gestures(extra = {}) {
  const canvas = fakeCanvas();
  let view = { cx: 0, cy: 0, cmPerPx: 2 };
  const taps = [], changes = [];
  let t = 0;
  const g = createMapGestures({
    canvas, getView: () => view, setView: (v) => { view = v; },
    onTap: (x, y) => taps.push([x, y]), onChange: (v) => changes.push(v), now: () => t, ...extra,
  });
  const ev = (id, x, y, at = t) => ({ pointerId: id, clientX: x, clientY: y, timeStamp: at, pointerType: 'touch', button: 0 });
  return { g, taps, changes, get view() { return view; }, ev, set t(v) { t = v; } };
}

test('gestures: a short still press is a tap, a drag pans and is not a tap', () => {
  const s = gestures();
  s.g.down(s.ev(1, 50, 60, 1000));
  s.g.move(s.ev(1, 53, 62, 1050)); // under 8 px: no pan yet
  s.g.up(s.ev(1, 53, 62, 1150));
  assert.deepEqual(s.taps, [[53, 62]]);
  assert.equal(s.changes.length, 0);
  assert.equal(s.g.state.auto, true);

  // too slow for a tap
  s.g.down(s.ev(2, 50, 60, 2000));
  s.g.up(s.ev(2, 50, 60, 2600));
  assert.equal(s.taps.length, 1);

  // drag: the content follows the pointer from the press position, follow and auto end
  s.g.state.follow = true;
  const grab = screenToWorld(fakeCanvas(), s.view, 50, 60);
  s.g.down(s.ev(3, 50, 60, 3000));
  s.g.move(s.ev(3, 70, 60, 3050));
  s.g.move(s.ev(3, 90, 75, 3100));
  s.g.up(s.ev(3, 90, 75, 3150));
  assert.equal(s.taps.length, 1);
  const at = screenToWorld(fakeCanvas(), s.view, 90, 75);
  close(at.x, grab.x); close(at.y, grab.y);
  assert.equal(s.g.state.follow, false);
  assert.equal(s.g.state.auto, false);
  assert.ok(s.changes.length >= 2);
});

test('gestures: pinch zooms, never taps, and the remaining finger keeps panning', () => {
  const s = gestures();
  s.g.down(s.ev(1, 60, 100, 0));
  s.g.down(s.ev(2, 100, 100, 10));
  s.g.move(s.ev(2, 140, 100, 20)); // spread 40 -> 80
  close(s.view.cmPerPx, 1);
  s.g.up(s.ev(2, 140, 100, 30));
  s.g.cancel(s.ev(2, 140, 100, 30)); // lostpointercapture after pointerup is ignored
  const before = { ...s.view };
  s.g.move(s.ev(1, 70, 100, 40)); // remaining finger pans immediately
  assert.notEqual(s.view.cx, before.cx);
  s.g.up(s.ev(1, 70, 100, 50));
  assert.equal(s.taps.length, 0);
  // mouse right button is ignored
  assert.equal(s.g.down({ ...s.ev(5, 1, 1), pointerType: 'mouse', button: 2 }), false);
});

test('gestures: wheel zooms around the cursor, or the centre in follow mode', () => {
  const s = gestures();
  const canvas = fakeCanvas();
  const w = screenToWorld(canvas, s.view, 40, 70);
  s.g.wheel({ clientX: 40, clientY: 70, deltaY: -200, deltaMode: 0 });
  assert.ok(s.view.cmPerPx < 2);
  const w2 = screenToWorld(canvas, s.view, 40, 70);
  close(w2.x, w.x, 1e-9); close(w2.y, w.y, 1e-9);
  s.g.state.follow = true;
  const c = { cx: s.view.cx, cy: s.view.cy };
  s.g.wheel({ clientX: 40, clientY: 70, deltaY: 200, deltaMode: 0 });
  close(s.view.cx, c.cx, 1e-9); close(s.view.cy, c.cy, 1e-9);
  assert.equal(s.g.state.follow, true);
});

test('attachMapControls: auto fit until the user zooms, then follow recentres on the robot', () => {
  const listeners = new Map();
  const canvas = { ...fakeCanvas(), addEventListener: (t, f) => listeners.set(t, f), removeEventListener: (t) => listeners.delete(t) };
  let view = null;
  const fitted = { cx: 5, cy: 5, cmPerPx: 3 };
  const c = attachMapControls(canvas, { getView: () => view, setView: (v) => { view = v; }, fit: () => fitted });
  assert.deepEqual(c.viewFor({ x: 50, y: 50 }), fitted);
  assert.equal(c.auto, true);
  listeners.get('wheel')({ clientX: 95, clientY: 100, deltaY: -100, deltaMode: 0, preventDefault() {} });
  assert.equal(c.auto, false);
  const zoomed = c.viewFor({ x: 50, y: 50 });
  assert.ok(zoomed.cmPerPx < 3);
  assert.equal(zoomed.cx, 5);
  c.setFollow(true);
  const followed = c.viewFor({ x: 50, y: -20 });
  assert.deepEqual([followed.cx, followed.cy, followed.cmPerPx], [50, -20, zoomed.cmPerPx]);
  c.fit();
  assert.equal(c.auto, true);
  assert.deepEqual(view, fitted);
  c.detach();
  assert.equal(listeners.size, 0);
});
