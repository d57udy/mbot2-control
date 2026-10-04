import { test } from 'node:test';
import assert from 'node:assert/strict';
import { GridMap } from '../js/gridmap.js';
import { saveMap, listMaps, loadMap, deleteMap, exportMap, importMap, parseMap, serializeMap, encodeGrid, decodeGrid, PREFIX } from '../js/mapstore.js';

function fakeStorage({ quota = Infinity } = {}) {
  const m = new Map();
  return {
    get length() { return m.size; },
    key: (i) => [...m.keys()][i] ?? null,
    getItem: (k) => (m.has(k) ? m.get(k) : null),
    setItem: (k, v) => {
      if (String(v).length > quota) { const e = new Error('exceeded the quota'); e.name = 'QuotaExceededError'; throw e; }
      m.set(k, String(v));
    },
    removeItem: (k) => { m.delete(k); },
    raw: m,
  };
}

function scannedMap() {
  const map = new GridMap({ cellCm: 5, sizeCm: 800 });
  const points = Array.from({ length: 12 }, (_, i) => ({ angle: i * 30, cm: 60 + (i % 4) * 30 }));
  map.integrateScan({ x: 0, y: 0, heading: 0 }, points, { freeBeamDeg: 30 });
  map.integrateScan({ x: 40, y: 20, heading: 90 }, points, { freeBeamDeg: 30 });
  // values right next to the known/unknown threshold must keep their state
  map.L[10] = 0.51; map.L[11] = -0.51; map.L[12] = 0.49; map.L[13] = 4; map.L[14] = -4;
  return map;
}

const states = (m) => Array.from(m.L, (_, k) => m.stateOf(k));

test('grid encoding round trip keeps every cell state and log-odds within 1/32', () => {
  const map = scannedMap();
  const L = decodeGrid(encodeGrid(map.L), map.L.length);
  for (let k = 0; k < L.length; k++) {
    if (k >= 10 && k <= 14) continue;
    assert.ok(Math.abs(L[k] - map.L[k]) <= 1 / 32 + 1e-6, `cell ${k}`);
  }
  const copy = new GridMap({ cellCm: 5, sizeCm: 800 });
  copy.L.set(L);
  assert.deepEqual(states(copy), states(map));
});

test('save, list, load and delete with a fake localStorage', () => {
  const storage = fakeStorage();
  const map = scannedMap();
  const r = saveMap('Wohnzimmer', map, { pose: { x: 40, y: 20, heading: 90 } }, { storage });
  assert.equal(r.ok, true);
  // includes the per-cell hit/miss evidence since GridMap v2 (about 30 kB for a scanned room)
  assert.ok(r.bytes < 60000, `compact: ${r.bytes} bytes`);
  storage.setItem('mbot.speed', '60'); // other keys are ignored
  storage.setItem(PREFIX + 'kaputt', '{nope');
  saveMap('Küche', new GridMap({ cellCm: 5, sizeCm: 800 }), {}, { storage });

  const list = listMaps({ storage });
  assert.deepEqual(list.map((e) => e.name).sort(), ['Küche', 'Wohnzimmer']);
  const wz = list.find((e) => e.name === 'Wohnzimmer');
  assert.equal(wz.cellCm, 5);
  assert.equal(wz.sizeCm, 800);
  assert.ok(wz.knownM2 > 0.5);
  assert.ok(Math.abs(wz.knownM2 - map.stats().knownM2) < 0.01);
  assert.ok(!Number.isNaN(Date.parse(wz.savedAt)));

  const { map: back, meta } = loadMap('Wohnzimmer', { storage });
  assert.ok(back instanceof GridMap);
  assert.deepEqual(states(back), states(map));
  assert.deepEqual(back.stats(), map.stats());
  assert.deepEqual(meta.pose, { x: 40, y: 20, heading: 90 });
  assert.deepEqual(meta.start, { x: 0, y: 0, heading: 0 });
  assert.equal(meta.name, 'Wohnzimmer');

  assert.equal(loadMap('fehlt', { storage }), null);
  assert.throws(() => loadMap('kaputt', { storage }), /JSON/);
  assert.equal(deleteMap('Küche', { storage }), true);
  assert.equal(deleteMap('Küche', { storage }), false);
  assert.deepEqual(listMaps({ storage }).map((e) => e.name), ['Wohnzimmer']);
});

test('storage failures are reported, not thrown', () => {
  const full = fakeStorage({ quota: 10 });
  const r = saveMap('x', scannedMap(), {}, { storage: full });
  assert.equal(r.ok, false);
  assert.match(r.error, /Speicher voll/);
  const broken = { get length() { throw new Error('denied'); }, getItem() { throw new Error('denied'); }, setItem() { throw new Error('denied'); } };
  assert.deepEqual(listMaps({ storage: broken }), []);
  assert.equal(loadMap('x', { storage: broken }), null);
  assert.equal(deleteMap('x', { storage: broken }), false);
  assert.equal(saveMap('x', scannedMap(), {}, { storage: broken }).ok, false);
  assert.equal(saveMap('  ', scannedMap(), {}, { storage: fakeStorage() }).ok, false);
});

test('export and import round trip, from text and from a Blob', async () => {
  const map = scannedMap();
  const blob = exportMap(map, { pose: { x: 1, y: 2, heading: 3 } });
  assert.equal(blob.type, 'application/json');
  const fromBlob = await importMap(blob);
  assert.deepEqual(states(fromBlob.map), states(map));
  assert.deepEqual(fromBlob.meta.pose, { x: 1, y: 2, heading: 3 });
  const fromText = await importMap(await blob.text());
  assert.deepEqual(states(fromText.map), states(map));
  // a different grid size survives too
  const small = new GridMap({ cellCm: 10, sizeCm: 400 });
  small.markFree(0, 0, 30);
  const s = parseMap(serializeMap(small));
  assert.equal(s.map.cellCm, 10);
  assert.equal(s.map.n, small.n);
  assert.deepEqual(states(s.map), states(small));
});

test('import rejects malformed and other-version input with a clear error', async () => {
  const good = serializeMap(scannedMap());
  await assert.rejects(importMap('not json'), /kein JSON/);
  await assert.rejects(importMap('{"a":1}'), /Keine mBot2-Kartendatei/);
  await assert.rejects(importMap(JSON.stringify({ ...good, v: 2 })), /Version 2/);
  await assert.rejects(importMap(JSON.stringify({ ...good, n: 7 })), /beschädigt/);
  await assert.rejects(importMap(JSON.stringify({ ...good, cellCm: -1 })), /beschädigt/);
  await assert.rejects(importMap(JSON.stringify({ ...good, grid: good.grid.slice(0, 40) })), /beschädigt/);
  await assert.rejects(importMap(null), /Keine Kartendatei/);
});

test('saved maps keep hit/miss evidence (GridMap v2)', async () => {
  const { GridMap } = await import('../js/gridmap.js');
  const { serializeMap, parseMap } = await import('../js/mapstore.js');
  const m = new GridMap({ cellCm: 5, sizeCm: 800 });
  m.integrateScan({ x: 0, y: 0, heading: 0 }, [{ angle: 0, cm: 90 }], { maxRangeCm: 150, beamDeg: 25 });
  m.integrateScan({ x: 0, y: 0, heading: 0 }, [{ angle: 0, cm: 90 }], { maxRangeCm: 150, beamDeg: 25 });
  const a = m.cellInfo(0, 96);
  const { map } = parseMap(JSON.stringify(serializeMap(m)));
  const b = map.cellInfo(0, 96);
  assert.equal(b.state, a.state);
  assert.equal(b.scans, a.scans);
  assert.ok(Math.abs(b.hits - a.hits) < 0.05);
});
