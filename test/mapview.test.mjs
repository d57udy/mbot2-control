import { test } from 'node:test';
import assert from 'node:assert/strict';
import { drawMap, fitView, screenToWorld, worldToScreen } from '../js/mapview.js';

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
