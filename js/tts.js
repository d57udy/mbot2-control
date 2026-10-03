// Text to speech via window.speechSynthesis. Importable in Node (no window).
// Chrome workarounds: long text is spoken sentence by sentence, and a
// pause/resume nudge keeps desktop Chrome from going silent after ~15 s.

const synth = () => (typeof window !== 'undefined' ? window.speechSynthesis : null);
const Utterance = () => (typeof window !== 'undefined' ? window.SpeechSynthesisUtterance : null);

let generation = 0;
let active = false;

// Splits into sentences, then hard-wraps anything longer than max chars.
export function splitSentences(text, max = 200) {
  const parts = String(text ?? '').replace(/\s+/g, ' ').trim().match(/[^.!?…]+[.!?…]*["“”»«)]*\s*/g) ?? [];
  const out = [];
  for (let p of parts.map((s) => s.trim()).filter(Boolean)) {
    while (p.length > max) {
      let cut = p.lastIndexOf(' ', max);
      if (cut < max / 2) cut = max;
      out.push(p.slice(0, cut).trim());
      p = p.slice(cut).trim();
    }
    if (p) out.push(p);
  }
  return out;
}

// Exact lang match first, then same language; local voices preferred.
export function pickVoice(voices, lang) {
  const want = String(lang || '').toLowerCase().replace('_', '-');
  const base = want.split('-')[0];
  const norm = (v) => String(v.lang || '').toLowerCase().replace('_', '-');
  const score = (v) => (norm(v) === want ? 4 : norm(v).split('-')[0] === base ? 2 : 0) + (v.localService ? 1 : 0);
  let best = null;
  for (const v of voices ?? []) if (score(v) >= 2 && (!best || score(v) > score(best))) best = v;
  return best;
}

function speakOne(s, U, text, lang, voice) {
  return new Promise((resolve) => {
    const u = new U(text);
    u.lang = lang;
    if (voice) u.voice = voice;
    // Chrome on desktop stops long utterances after ~15 s unless nudged;
    // Android ignores pause() so the nudge is harmless there.
    const nudge = setInterval(() => { if (s.speaking && !s.paused) { s.pause(); s.resume(); } }, 10000);
    // some engines never fire end/error after cancel(); do not hang the caller
    const guard = setTimeout(() => done(), 5000 + text.length * 150);
    const done = () => { clearInterval(nudge); clearTimeout(guard); resolve(); };
    u.onend = done;
    u.onerror = done;
    s.speak(u);
  });
}

export async function speak(text, lang = 'de-DE') {
  const s = synth();
  const U = Utterance();
  if (!s || !U) return;
  cancel();
  const gen = ++generation;
  const voice = pickVoice(s.getVoices?.() ?? [], lang);
  active = true;
  try {
    for (const part of splitSentences(text)) {
      if (gen !== generation) return;
      await speakOne(s, U, part, lang, voice);
    }
  } finally {
    if (gen === generation) active = false;
  }
}

export function cancel() {
  generation++;
  active = false;
  try { synth()?.cancel(); } catch { /* ignore */ }
}

export const tts = {
  speak,
  cancel,
  get speaking() { return active || !!synth()?.speaking; },
};

export default tts;
