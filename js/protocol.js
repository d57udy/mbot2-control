// Makeblock "f3/f4" (HalocodeProtocol) framing used by CyberPi Live Mode.
// Ported from DrorSh/mbot_python (MIT). See research/01-ble-protocol.md.

export const UUID = {
  notify: '0000ffe2-0000-1000-8000-00805f9b34fb',
  write: '0000ffe3-0000-1000-8000-00805f9b34fb',
  // ffe1 is the best guess; the others are fallbacks until confirmed on hardware.
  serviceCandidates: [
    '0000ffe1-0000-1000-8000-00805f9b34fb',
    '0000ffe0-0000-1000-8000-00805f9b34fb',
    '0000ffe5-0000-1000-8000-00805f9b34fb',
  ],
};

// Fixed frame that switches the CyberPi into online (Live) mode.
export const ONLINE_FRAME = Uint8Array.from([0xf3, 0xf6, 0x03, 0x00, 0x0d, 0x00, 0x01, 0x0e, 0xf4]);

export const MODE_NO_REPLY = 0;
export const MODE_REPLY = 1;

export function buildScriptFrame(script, idx, mode) {
  const sb = new TextEncoder().encode(script);
  const data = new Uint8Array(2 + sb.length);
  data[0] = sb.length & 0xff;
  data[1] = (sb.length >> 8) & 0xff;
  data.set(sb, 2);
  const datalen = data.length + 4;
  const lenLo = datalen & 0xff, lenHi = (datalen >> 8) & 0xff;
  const body = [0x28, mode, idx & 0xff, (idx >> 8) & 0xff, ...data];
  const cksum = body.reduce((a, b) => a + b, 0) & 0xff;
  return Uint8Array.from([0xf3, (0xf3 + lenLo + lenHi) & 0xff, lenLo, lenHi, ...body, cksum, 0xf4]);
}

// MicroPython may answer with Python literals instead of strict JSON.
function parseRet(text) {
  try { return JSON.parse(text).ret; } catch { /* fall through */ }
  try {
    const fixed = text.replace(/'/g, '"').replace(/\bTrue\b/g, 'true')
      .replace(/\bFalse\b/g, 'false').replace(/\bNone\b/g, 'null');
    return JSON.parse(fixed).ret;
  } catch { return text; }
}

// Reassembles reply frames that arrive split across several notifications.
export class F3Parser {
  constructor() { this.buf = []; this.rx = false; this.len = 0; }

  feed(bytes) {
    const out = [];
    for (const b of bytes) {
      const r = this.byte(b);
      if (r) out.push(r);
    }
    return out;
  }

  byte(b) {
    this.buf.push(b);
    const n = this.buf.length;
    if (n > 3) {
      const [h, c, lo, hi] = this.buf.slice(n - 4);
      if (h === 0xf3 && ((h + lo + hi) & 0xff) === c) {
        this.buf = [h, c, lo, hi];
        this.len = lo | (hi << 8);
        this.rx = true;
      }
    }
    if (this.rx) {
      if (this.buf.length === this.len + 6) {
        const f = this.buf;
        this.buf = []; this.rx = false;
        if (f[4] !== 0x28 || f.length < 10) return null;
        const idx = f[6] | (f[7] << 8);
        const raw = new TextDecoder().decode(Uint8Array.from(f.slice(10, f.length - 2)));
        return { idx, value: parseRet(raw), raw };
      }
      if (this.buf.length > 4096) { this.buf = []; this.rx = false; }
    } else if (this.buf.length > 64) {
      this.buf = this.buf.slice(-4);
    }
    return null;
  }
}

export const toHex = (bytes) => Array.from(bytes, (b) => b.toString(16).padStart(2, '0')).join(' ');
