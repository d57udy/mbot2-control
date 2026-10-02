// Voice: Web Speech recognition plus a small command parser.
// The parser returns commands, not actions, so an LLM can replace it later.

const STOP = /\b(stopp?|halt|anhalten|stehen ?bleiben|aus|stop it|freeze)\b/i;

const NUM_WORDS = {
  ein: 1, eine: 1, eins: 1, one: 1, zwei: 2, two: 2, drei: 3, three: 3, vier: 4, four: 4,
  fünf: 5, five: 5, sechs: 6, six: 6, sieben: 7, seven: 7, acht: 8, eight: 8, neun: 9, nine: 9,
  zehn: 10, ten: 10, zwanzig: 20, dreißig: 30, thirty: 30, fünfundvierzig: 45, neunzig: 90,
  ninety: 90, hundertachtzig: 180, halbe: 0.5, half: 0.5,
};

const COLORS = {
  rot: [255, 0, 0], red: [255, 0, 0], grün: [0, 255, 0], green: [0, 255, 0],
  blau: [0, 0, 255], blue: [0, 0, 255], gelb: [255, 200, 0], yellow: [255, 200, 0],
  weiß: [255, 255, 255], white: [255, 255, 255], lila: [160, 0, 255], purple: [160, 0, 255],
  aus: [0, 0, 0], off: [0, 0, 0],
};

function number(text) {
  const m = text.match(/(\d+(?:[.,]\d+)?)/);
  if (m) return parseFloat(m[1].replace(',', '.'));
  for (const w of text.split(/\s+/)) if (w in NUM_WORDS) return NUM_WORDS[w];
  return null;
}

export const isStop = (text) => STOP.test(text);

// Returns { cmd, args } (plus { drive: secs } for long moves) or null.
export function parseUtterance(raw) {
  const t = raw.toLowerCase().replace(/[.!?]/g, ' ').trim();
  if (!t) return null;
  if (isStop(t) && !/licht|light/.test(t)) return { cmd: 'stop', args: {} };

  const n = number(t);
  const secs = /sekunde|second/.test(t) ? n : null;
  const deg = /grad|degree/.test(t) ? n : null;
  const small = /bisschen|etwas|wenig|little|bit/.test(t);

  if (/licht|light|led|farbe|colou?r/.test(t)) {
    const words = t.split(/\s+/);
    for (const [name, rgb] of Object.entries(COLORS)) {
      if (words.includes(name)) return { cmd: 'led', args: { r: rgb[0], g: rgb[1], b: rgb[2] } };
    }
  }
  if (/umdrehen|wenden|dreh dich um|turn around/.test(t)) return { cmd: 'turn', args: { deg: 180 } };
  if (/\b(links|left)\b/.test(t)) return { cmd: 'turn', args: { deg: -(deg ?? (small ? 30 : 90)) } };
  if (/\b(rechts|right)\b/.test(t)) return { cmd: 'turn', args: { deg: deg ?? (small ? 30 : 90) } };
  if (/zurück|rückwärts|back|reverse/.test(t)) {
    return { cmd: 'move', args: { dir: 'backward' }, drive: secs ?? (small ? 0.4 : 1) };
  }
  if (/vorwärts|\bvor\b|geradeaus|\blos\b|\bfahr|forward|ahead|\bgo\b/.test(t)) {
    return { cmd: 'move', args: { dir: 'forward' }, drive: secs ?? (small ? 0.4 : 1) };
  }
  if (/schneller|faster/.test(t)) return { cmd: 'speed', args: { delta: 15 } };
  if (/langsamer|slower/.test(t)) return { cmd: 'speed', args: { delta: -15 } };
  if (/hup|piep|beep|honk/.test(t)) return { cmd: 'beep', args: {} };
  if (/abstand|entfernung|distance/.test(t)) return { cmd: 'read', args: { sensor: 'distance' } };
  if (/akku|batterie|battery/.test(t)) return { cmd: 'read', args: { sensor: 'battery' } };
  return null;
}

// Wraps webkitSpeechRecognition for Chrome on Android, whose continuous mode
// reports every partial guess as final. Stop words fire immediately on any
// result; everything else waits until the transcript has settled.
export class VoiceListener {
  constructor({ lang = 'de-DE', onStop, onCommand, onTranscript, onState, settleMs = 500 }) {
    const SR = window.SpeechRecognition || window.webkitSpeechRecognition;
    this.supported = !!SR;
    this.SR = SR;
    Object.assign(this, { lang, onStop, onCommand, onTranscript, onState, settleMs });
    this.active = false;
    this.rec = null;
    this.settleTimer = null;
    this.pendingText = '';
    this.stopFiredFor = '';
    this.backoff = 200;
  }

  start() {
    if (!this.supported || this.active) return;
    this.active = true;
    this.spawn();
  }

  stop() {
    this.active = false;
    clearTimeout(this.settleTimer);
    try { this.rec?.abort(); } catch { /* ignore */ }
    this.onState?.('off');
  }

  spawn() {
    const rec = new this.SR();
    rec.lang = this.lang;
    rec.continuous = true;
    rec.interimResults = true;
    rec.maxAlternatives = 1;
    rec.onstart = () => { this.backoff = 200; this.onState?.('listening'); };
    rec.onresult = (e) => this.handle(e);
    rec.onerror = (e) => {
      this.onState?.(`error: ${e.error}`);
      if (e.error === 'not-allowed' || e.error === 'service-not-allowed') this.active = false;
      else this.backoff = Math.min(this.backoff * 2, 5000);
    };
    rec.onend = () => {
      if (!this.active) return;
      this.onState?.('restarting');
      setTimeout(() => this.active && this.spawn(), this.backoff);
    };
    this.rec = rec;
    try { rec.start(); } catch { setTimeout(() => this.active && this.spawn(), 500); }
  }

  handle(e) {
    const last = e.results[e.results.length - 1];
    const text = last[0].transcript.trim();
    this.onTranscript?.(text, last.isFinal);

    if (isStop(text) && !/licht|light/i.test(text)) {
      if (this.stopFiredFor !== text) { this.stopFiredFor = text; this.onStop?.(text); }
      clearTimeout(this.settleTimer);
      this.pendingText = '';
      return;
    }
    if (!last.isFinal) return;
    this.pendingText = text;
    clearTimeout(this.settleTimer);
    this.settleTimer = setTimeout(() => {
      const t = this.pendingText;
      this.pendingText = '';
      if (t && t !== this.lastExecuted) {
        this.lastExecuted = t;
        setTimeout(() => { if (this.lastExecuted === t) this.lastExecuted = ''; }, 2000);
        this.onCommand?.(t);
      }
    }, this.settleMs);
  }
}
