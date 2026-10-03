// Saved maps: localStorage (key prefix 'mbot.map.') plus JSON export/import.
// The grid is stored dense: log-odds quantised to Int8 (1/32 steps, the grid
// clamps to +-4), runs of unknown cells run-length coded, then base64.
// A 160 x 160 map with a few rooms seen is typically a few kB.
//
// Meta: { name, savedAt, start: { x: 0, y: 0, heading: 0 } (the map frame's
// origin, i.e. where the start was), pose (robot pose when saved, optional),
// knownM2, plus anything the caller adds }.

import { GridMap } from './gridmap.js';

export const PREFIX = 'mbot.map.';
export const FORMAT = 'mbot2-map';
export const VERSION = 1;
const SCALE = 32;     // Int8 step = 1/32 log-odds
const RUN = -128;     // marker: RUN, lo, hi = that many zero cells

const THRESH = 0.5;   // GridMap's known/unknown threshold; quantising must not cross it

const store = (o) => o?.storage ?? globalThis.localStorage;

function quantise(l) {
  let q = Math.max(-127, Math.min(127, Math.round(l * SCALE)));
  if (Math.abs(l) > THRESH && Math.abs(q) <= THRESH * SCALE) q = Math.sign(l) * (THRESH * SCALE + 1);
  return q;
}

function toBase64(bytes) {
  let s = '';
  for (let i = 0; i < bytes.length; i += 0x8000) s += String.fromCharCode(...bytes.subarray(i, i + 0x8000));
  return btoa(s);
}

function fromBase64(b64) {
  const s = atob(b64);
  const out = new Uint8Array(s.length);
  for (let i = 0; i < s.length; i++) out[i] = s.charCodeAt(i);
  return out;
}

export function encodeGrid(L) {
  const out = new Int8Array(L.length + 16);
  let o = 0;
  const push = (v) => { if (o >= out.length - 3) throw new Error('encode overflow'); out[o++] = v; };
  for (let k = 0; k < L.length;) {
    const q = quantise(L[k]);
    if (q === 0) {
      let n = 1;
      while (k + n < L.length && n < 0xffff && quantise(L[k + n]) === 0) n++;
      if (n >= 3) { push(RUN); push(n & 0xff); push(n >> 8); k += n; continue; }
    }
    push(q);
    k++;
  }
  return toBase64(new Uint8Array(out.buffer, 0, o));
}

export function decodeGrid(b64, length) {
  const bytes = new Int8Array(fromBase64(b64).buffer);
  const L = new Float32Array(length);
  let k = 0;
  for (let i = 0; i < bytes.length; i++) {
    if (bytes[i] === RUN) {
      if (i + 2 >= bytes.length) throw new Error('map data truncated');
      k += (bytes[i + 1] & 0xff) | ((bytes[i + 2] & 0xff) << 8);
      i += 2;
    } else {
      if (k >= length) throw new Error('map data longer than the grid');
      L[k++] = bytes[i] / SCALE;
    }
  }
  if (k !== length) throw new Error(`map data has ${k} cells, expected ${length}`);
  return L;
}

// map + meta -> plain object (what gets stored and exported).
export function serializeMap(map, meta = {}) {
  if (!map?.L || !map.n) throw new Error('not a GridMap');
  const st = map.stats?.() ?? {};
  return {
    format: FORMAT, v: VERSION,
    cellCm: map.cellCm, sizeCm: map.sizeCm, n: map.n,
    grid: encodeGrid(map.L),
    meta: {
      start: { x: 0, y: 0, heading: 0 },
      savedAt: new Date().toISOString(),
      knownM2: Math.round((st.knownM2 ?? 0) * 100) / 100,
      ...meta,
    },
  };
}

// Plain object or JSON text -> { map, meta }. Throws a readable Error on bad input.
export function parseMap(input) {
  let o = input;
  if (typeof input === 'string') {
    try { o = JSON.parse(input); } catch { throw new Error('Keine gültige Kartendatei (kein JSON).'); }
  }
  if (!o || typeof o !== 'object' || o.format !== FORMAT) throw new Error('Keine mBot2-Kartendatei.');
  if (o.v !== VERSION) throw new Error(`Kartenformat Version ${o.v} wird nicht unterstützt (erwartet ${VERSION}).`);
  const { cellCm, sizeCm, n } = o;
  if (![cellCm, sizeCm, n].every((v) => Number.isFinite(v) && v > 0) || n > 2000 || typeof o.grid !== 'string') {
    throw new Error('Kartendatei beschädigt (Größe).');
  }
  const map = new GridMap({ cellCm, sizeCm });
  if (map.n !== n) throw new Error('Kartendatei beschädigt (Rastergröße passt nicht).');
  try {
    map.L.set(decodeGrid(o.grid, n * n));
  } catch (e) {
    throw new Error(`Kartendatei beschädigt (${e.message}).`);
  }
  map.touch();
  return { map, meta: o.meta && typeof o.meta === 'object' ? o.meta : {} };
}

const keyOf = (name) => {
  const s = String(name ?? '').trim();
  if (!s || s.length > 60) throw new Error('Kartenname: 1 bis 60 Zeichen.');
  return PREFIX + s;
};

// -> { ok: true, bytes } or { ok: false, error }
export function saveMap(name, map, meta = {}, opts) {
  try {
    const key = keyOf(name);
    const text = JSON.stringify(serializeMap(map, { ...meta, name: key.slice(PREFIX.length) }));
    store(opts).setItem(key, text);
    return { ok: true, bytes: text.length };
  } catch (e) {
    const quota = e?.name === 'QuotaExceededError' || /quota/i.test(e?.message ?? '');
    return { ok: false, error: quota ? 'Speicher voll: alte Karten löschen.' : e?.message ?? String(e) };
  }
}

// Newest first. Unreadable entries are skipped.
export function listMaps(opts) {
  const out = [];
  try {
    const s = store(opts);
    for (let i = 0; i < s.length; i++) {
      const key = s.key(i);
      if (!key?.startsWith(PREFIX)) continue;
      try {
        const o = JSON.parse(s.getItem(key));
        if (o?.format !== FORMAT) continue;
        out.push({ name: key.slice(PREFIX.length), savedAt: o.meta?.savedAt ?? null, cellCm: o.cellCm, sizeCm: o.sizeCm, knownM2: o.meta?.knownM2 ?? null });
      } catch { /* skip */ }
    }
  } catch { return []; }
  return out.sort((a, b) => String(b.savedAt).localeCompare(String(a.savedAt)));
}

// -> { map, meta } or null when there is no such map. Throws on a corrupt entry.
export function loadMap(name, opts) {
  let text = null;
  try { text = store(opts).getItem(keyOf(name)); } catch { return null; }
  return text == null ? null : parseMap(text);
}

export function deleteMap(name, opts) {
  try {
    const s = store(opts), key = keyOf(name);
    if (s.getItem(key) == null) return false;
    s.removeItem(key);
    return true;
  } catch { return false; }
}

export function exportMap(map, meta = {}) {
  return new Blob([JSON.stringify(serializeMap(map, meta))], { type: 'application/json' });
}

// File, Blob or JSON text -> Promise<{ map, meta }>.
export async function importMap(fileOrText) {
  const text = typeof fileOrText === 'string' ? fileOrText : await fileOrText?.text?.();
  if (typeof text !== 'string') throw new Error('Keine Kartendatei.');
  return parseMap(text);
}
