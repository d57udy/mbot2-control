// Tool definitions for the conversation agent and the executor that maps
// tool calls onto bus commands. Every robot action goes through bus.submit,
// so the bus safety layer (obstacle guard, clamps, whitelists) still applies.

import { EYE_EFFECTS } from './bus.js';

const SAFE_CM = 20; // never plan to end closer than this to an obstacle

const obj = (properties = {}, required = []) => ({ type: 'object', properties, required, additionalProperties: false });

export const EMOTIONS = {
  happy: { eyes: 'happy', led: [255, 180, 0], tone: [880, 0.15] },
  excited: { eyes: 'new_happy', led: [0, 255, 80], tone: [1200, 0.12] },
  sad: { eyes: 'aggrieved', led: [0, 0, 160], tone: [330, 0.4] },
  surprised: { eyes: 'raises_brow', led: [255, 255, 255], tone: [1400, 0.08] },
  thinking: { eyes: 'thinking', led: [120, 0, 255] },
  curious: { eyes: 'look_right', led: [0, 200, 255] },
  dizzy: { eyes: 'dizzy', led: [255, 100, 0] },
  wink: { eyes: 'wink', led: [255, 60, 160] },
  naughty: { eyes: 'naughty', led: [255, 0, 200] },
  angry: { eyes: 'aggrieved', led: [255, 0, 0], tone: [200, 0.3] },
  sleepy: { eyes: 'standby', led: [10, 10, 60] },
  neutral: { eyes: 'standby', led: [40, 40, 40] },
};

export const COLORS = {
  red: [255, 0, 0], rot: [255, 0, 0], green: [0, 255, 0], grün: [0, 255, 0], blue: [0, 0, 255], blau: [0, 0, 255],
  yellow: [255, 200, 0], gelb: [255, 200, 0], orange: [255, 100, 0], purple: [160, 0, 255], lila: [160, 0, 255],
  pink: [255, 60, 160], rosa: [255, 60, 160], cyan: [0, 200, 255], türkis: [0, 200, 255],
  white: [255, 255, 255], weiß: [255, 255, 255], off: [0, 0, 0], aus: [0, 0, 0], black: [0, 0, 0], schwarz: [0, 0, 0],
};

export const SOUNDS = {
  beep: [[700, 0.2]],
  happy: [[660, 0.1], [880, 0.1], [1100, 0.15]],
  sad: [[500, 0.2], [350, 0.35]],
  alarm: [[1000, 0.15], [600, 0.15], [1000, 0.15], [600, 0.15]],
  question: [[600, 0.1], [900, 0.15]],
};

const COLOR_PROP = { type: 'string', description: 'Colour name (red, green, blue, yellow, orange, purple, pink, cyan, white, off) or hex #rrggbb.' };

export const TOOLS = [
  {
    name: 'move_straight',
    description: 'Drive straight. cm: -100..100, positive forward, negative backward. Blocks until done. Forward moves are shortened to stay 20 cm from obstacles and refused if blocked.',
    input_schema: obj({ cm: { type: 'number', minimum: -100, maximum: 100 } }, ['cm']),
  },
  {
    name: 'turn',
    description: 'Turn in place with the gyro. degrees: -360..360, positive = right (clockwise), negative = left. Blocks until done.',
    input_schema: obj({ degrees: { type: 'number', minimum: -360, maximum: 360 } }, ['degrees']),
  },
  { name: 'stop', description: 'Stop all motors immediately.', input_schema: obj() },
  {
    name: 'scan_surroundings',
    description: 'Rotate in place in steps, measure distance at each heading, return to the start heading. Returns points [angle deg, cm] (angle clockwise from current front, 0 = ahead) and open directions. Takes several seconds.',
    input_schema: obj({ steps: { type: 'integer', enum: [8, 12, 16], description: 'Headings to sample, default 12.' } }),
  },
  {
    name: 'drive_toward',
    description: 'Turn to an angle from the latest scan, then drive up to cm forward safely. Requires a scan since the last movement. angle: -180..180 clockwise positive; cm: 1..100.',
    input_schema: obj({ angle: { type: 'number', minimum: -180, maximum: 180 }, cm: { type: 'number', minimum: 1, maximum: 100 } }, ['angle', 'cm']),
  },
  { name: 'read_distance', description: 'Ultrasonic distance straight ahead in cm (about 3..300).', input_schema: obj() },
  { name: 'read_battery', description: 'Battery level in percent.', input_schema: obj() },
  {
    name: 'read_floor',
    description: 'Read the 4-channel floor sensor under the robot (line bits, gray values; with colors=true also colour names and RGB).',
    input_schema: obj({ colors: { type: 'boolean' } }),
  },
  {
    name: 'express_emotion',
    description: 'Show an emotion with eye LED animation, back LED colour and a short sound.',
    input_schema: obj({ emotion: { type: 'string', enum: Object.keys(EMOTIONS) } }, ['emotion']),
  },
  {
    name: 'set_back_leds',
    description: 'Set the 5 back LEDs: either color for all, or colors with exactly 5 entries (left to right).',
    input_schema: obj({ color: COLOR_PROP, colors: { type: 'array', items: COLOR_PROP, minItems: 5, maxItems: 5 } }),
  },
  {
    name: 'play_sound',
    description: 'Play a short built-in sound.',
    input_schema: obj({ kind: { type: 'string', enum: Object.keys(SOUNDS) } }, ['kind']),
  },
  {
    name: 'play_tone',
    description: 'Play one tone. freq: 100..4000 Hz, secs: 0.05..1.',
    input_schema: obj({ freq: { type: 'number', minimum: 100, maximum: 4000 }, secs: { type: 'number', minimum: 0.05, maximum: 1 } }, ['freq']),
  },
  {
    name: 'show_text',
    description: 'Show short text (max 40 characters) on the CyberPi display.',
    input_schema: obj({ text: { type: 'string', maxLength: 40 } }, ['text']),
  },
];

const num = (v) => (typeof v === 'number' ? v : typeof v === 'string' && v.trim() !== '' ? Number(v) : NaN);
const clamp = (v, lo, hi) => Math.min(hi, Math.max(lo, v));
const finite = (v) => typeof v === 'number' && Number.isFinite(v);

export function parseColor(c) {
  if (Array.isArray(c) && c.length === 3 && c.every((x) => finite(num(x)))) return c.map((x) => clamp(Math.round(num(x)), 0, 255));
  if (typeof c !== 'string') return null;
  const s = c.trim().toLowerCase();
  if (s in COLORS) return COLORS[s];
  const m = s.match(/^#?([0-9a-f]{6})$/);
  if (m) return [0, 2, 4].map((i) => parseInt(m[1].slice(i, i + 2), 16));
  return null;
}

// Printable text only; quotes and backslashes are dropped as defence in depth.
const cleanText = (t) => String(t ?? '').replace(/[\u0000-\u001f\u007f"'\\`]/g, ' ').replace(/\s+/g, ' ').trim().slice(0, 40);

// Normalise an angle to -180..180.
const wrap = (a) => ((((a + 180) % 360) + 360) % 360) - 180;

const fail = (error) => ({ ok: false, error });

export function createToolExecutor({ bus, makeCommand, scan, findOpenings, describeScan, driveToward, onEmotion, onScan }) {
  let lastScan = null; // { points, openings }, cleared by any movement

  const submit = (cmd, args = {}, timeout_ms) => bus.submit(makeCommand(cmd, args, 'agent', timeout_ms));

  async function readDistance() {
    const r = await submit('read', { sensor: 'distance' });
    return r.ok ? Number(r.value) : null;
  }

  async function tones(list) {
    for (const [freq, secs] of list) {
      const r = await submit('beep', { freq, secs });
      if (!r.ok) return r;
    }
    return { ok: true };
  }

  const handlers = {
    async move_straight({ cm }) {
      let n = num(cm);
      if (!finite(n)) return fail('cm must be a number');
      n = Math.round(clamp(n, -100, 100));
      if (n === 0) return { ok: true, moved_cm: 0 };
      let note;
      if (n > 0) {
        const d = await readDistance();
        if (finite(d) && d > 0) {
          const max = Math.floor(d - SAFE_CM);
          if (max <= 0) return fail(`obstacle ${Math.round(d)} cm ahead, not moving`);
          if (n > max) { note = `shortened from ${n} cm, obstacle at ${Math.round(d)} cm`; n = max; }
        }
      }
      lastScan = null;
      const r = await submit('straight', { cm: n, wait: true }, 15000);
      return r.ok ? { ok: true, moved_cm: n, ...(note && { note }) } : fail(r.error);
    },

    async turn({ degrees }) {
      const d = num(degrees);
      if (!finite(d)) return fail('degrees must be a number');
      const deg = Math.round(clamp(d, -360, 360));
      lastScan = null;
      const r = await submit('turn', { deg, wait: true }, 15000);
      return r.ok ? { ok: true, turned_deg: deg } : fail(r.error);
    },

    async stop() {
      const r = await submit('stop');
      return r.ok ? { ok: true } : fail(r.error);
    },

    async scan_surroundings({ steps } = {}, { signal } = {}) {
      if (!scan) return fail('scan not available');
      const n = [8, 12, 16].includes(num(steps)) ? num(steps) : 12;
      lastScan = null;
      const res = await scan(bus, { steps: n, signal, makeCommand });
      const points = (res?.points ?? []).map((p) => ({ angle: Math.round(p.angle), cm: finite(p.cm) ? Math.round(p.cm) : null }));
      const openings = findOpenings ? findOpenings(points, { minCm: 50 }) : [];
      lastScan = { points, openings };
      onScan?.(points, openings);
      return {
        ok: true,
        summary: describeScan ? describeScan(points, openings) : undefined,
        points: points.map((p) => [p.angle, p.cm]),
        openings: openings.slice(0, 3).map((o) => ({ angle: Math.round(o.angle), widthDeg: Math.round(o.widthDeg), cm: Math.round(o.cm) })),
      };
    },

    async drive_toward({ angle, cm }) {
      const a = num(angle);
      const c = num(cm);
      if (!finite(a) || !finite(c)) return fail('angle and cm must be numbers');
      if (!lastScan?.points.length) return fail('no current scan, call scan_surroundings first');
      const target = wrap(a);
      const near = lastScan.points.reduce((best, p) =>
        Math.abs(wrap(p.angle - target)) < Math.abs(wrap(best.angle - target)) ? p : best);
      const free = near.cm;
      let dist = Math.round(clamp(c, 1, 100));
      if (finite(free)) {
        const max = Math.floor(free - SAFE_CM);
        if (max <= 0) return fail(`obstacle at ${free} cm in direction ${near.angle}, choose another direction`);
        dist = Math.min(dist, max);
      }
      lastScan = null;
      if (driveToward) {
        const res = await driveToward(bus, { angle: target, cm: dist, makeCommand });
        if (res && res.ok === false) return fail(res.error ?? 'drive failed');
        if (res && 'droveCm' in res) return { ok: true, angle: Math.round(target), cm: res.droveCm, ...(res.note && { note: res.note }) };
      } else {
        const t = await submit('turn', { deg: Math.round(target), wait: true }, 15000);
        if (!t.ok) return fail(t.error);
        const s = await submit('straight', { cm: dist, wait: true }, 15000);
        if (!s.ok) return fail(s.error);
      }
      return { ok: true, angle: Math.round(target), cm: dist };
    },

    async read_distance() {
      const r = await submit('read', { sensor: 'distance' });
      return r.ok ? { ok: true, cm: Number(r.value) } : fail(r.error);
    },

    async read_battery() {
      const r = await submit('read', { sensor: 'battery' });
      return r.ok ? { ok: true, percent: Number(r.value) } : fail(r.error);
    },

    async read_floor({ colors } = {}) {
      const r = await submit('read', { sensor: 'floor', colors: colors === true });
      return r.ok ? { ok: true, floor: r.value } : fail(r.error);
    },

    async express_emotion({ emotion }) {
      const e = EMOTIONS[emotion];
      if (!e) return fail(`unknown emotion, use one of ${Object.keys(EMOTIONS).join(', ')}`);
      onEmotion?.(emotion);
      // best effort: each part may fail on its own
      const parts = {};
      const led = await submit('led', { r: e.led[0], g: e.led[1], b: e.led[2] });
      parts.leds = led.ok || led.error;
      if (e.tone) {
        const t = await submit('beep', { freq: e.tone[0], secs: e.tone[1] });
        parts.sound = t.ok || t.error;
      }
      if (EYE_EFFECTS.includes(e.eyes)) {
        const y = await submit('eyes_effect', { name: e.eyes }, 5000);
        parts.eyes = y.ok || y.error;
      } else {
        parts.eyes = 'effect not supported';
      }
      const ok = Object.values(parts).some((v) => v === true);
      return ok ? { ok: true, emotion, parts } : { ok: false, error: 'no effect worked', parts };
    },

    async set_back_leds({ color, colors }) {
      if (Array.isArray(colors)) {
        if (colors.length !== 5) return fail('colors needs exactly 5 entries');
        const rgb = colors.map(parseColor);
        if (rgb.some((x) => !x)) return fail('unknown colour in colors');
        const r = await submit('leds', { colors: rgb });
        return r.ok ? { ok: true } : fail(r.error);
      }
      const rgb = parseColor(color);
      if (!rgb) return fail('unknown colour');
      const r = await submit('led', { r: rgb[0], g: rgb[1], b: rgb[2] });
      return r.ok ? { ok: true } : fail(r.error);
    },

    async play_sound({ kind }) {
      const s = SOUNDS[kind];
      if (!s) return fail(`unknown sound, use one of ${Object.keys(SOUNDS).join(', ')}`);
      const r = await tones(s);
      return r.ok ? { ok: true } : fail(r.error);
    },

    async play_tone({ freq, secs }) {
      const f = num(freq);
      if (!finite(f)) return fail('freq must be a number');
      const s = finite(num(secs)) ? clamp(num(secs), 0.05, 1) : 0.2;
      const r = await submit('beep', { freq: Math.round(clamp(f, 100, 4000)), secs: s });
      return r.ok ? { ok: true } : fail(r.error);
    },

    async show_text({ text }) {
      const t = cleanText(text);
      const r = await submit('display', { text: t });
      return r.ok ? { ok: true, shown: t } : fail(r.error);
    },
  };

  return {
    get lastScan() { return lastScan; },
    async execute(name, input, { signal } = {}) {
      if (signal?.aborted) return fail('aborted');
      if (!Object.hasOwn(handlers, name)) return fail(`unknown tool ${name}`);
      const args = input && typeof input === 'object' && !Array.isArray(input) ? input : {};
      try {
        return await handlers[name](args, { signal });
      } catch (e) {
        return fail(e?.name === 'AbortError' ? 'aborted' : String(e?.message ?? e));
      }
    },
  };
}
