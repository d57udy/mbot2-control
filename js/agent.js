// Conversation agent: a manual tool-use loop over the Claude Messages API,
// called directly from the browser with a user-supplied key.

export const API_URL = 'https://api.anthropic.com/v1/messages';
export const API_VERSION = '2023-06-01';

// Haiku is the fast default for voice; the others trade latency for smarts.
export const MODELS = [
  { id: 'claude-haiku-4-5', label: 'Claude Haiku 4.5 (schnell)' },
  { id: 'claude-sonnet-5-5', label: 'Claude Sonnet 5.5' },
  { id: 'claude-opus-5-5', label: 'Claude Opus 5.5' },
];
export const DEFAULT_MODEL = MODELS[0].id;

// Models with adaptive thinking on by default; they take effort and server-side fallbacks.
const isThinkingModel = (m) => !/haiku/.test(m);

export function buildSystemPrompt(lang = 'de-DE') {
  const de = lang.startsWith('de');
  return [
    'You are a small, friendly Makeblock mBot2 robot living in a family home. Children may talk to you.',
    `Speak ${de ? 'German' : 'English'} by default and always answer in the language the user speaks.`,
    'Your replies are spoken aloud: answer in 1 or 2 short sentences, plain text only, no markdown, no lists, no emojis.',
    'React emotionally with express_emotion when something is funny, sad, surprising or exciting, but not on every turn.',
    'Move only when asked to or when a request clearly needs it. Prefer small moves.',
    'Before navigating in unknown space, call scan_surroundings, then use drive_toward with an open direction.',
    'Safety first: never drive toward anything closer than 20 cm; if a move is refused or shortened, say so and do not force it.',
    'Conventions: distances in cm, forward positive; angles in degrees, positive = right (clockwise), 0 = straight ahead.',
    'Motion tools block until the robot is done. If someone says stop, stop at once.',
    'Briefly say what you are doing. Be kind and playful; if a request could hurt someone or the robot, decline gently.',
  ].join('\n');
}

export class AgentError extends Error {
  constructor(message, status) { super(message); this.name = 'AgentError'; this.status = status; }
}

const sleep = (ms, signal) => new Promise((resolve, reject) => {
  if (signal?.aborted) { reject(abortError()); return; }
  const t = setTimeout(() => { signal?.removeEventListener('abort', on); resolve(); }, ms);
  const on = () => { clearTimeout(t); reject(abortError()); };
  signal?.addEventListener('abort', on, { once: true });
});

function abortError() {
  const e = new Error('aborted');
  e.name = 'AbortError';
  return e;
}

const hasToolResult = (m) => m.role === 'user' && Array.isArray(m.content) && m.content.some((b) => b.type === 'tool_result');
const textOf = (content) => content.filter((b) => b.type === 'text').map((b) => b.text).join(' ').trim();

export class ConversationAgent {
  constructor({ apiKey, model = DEFAULT_MODEL, tools = [], executor, lang = 'de-DE', systemPrompt, onEvent,
    fetchImpl = (...a) => fetch(...a), maxIterations = 8, maxTokens = 2048, maxHistory = 20, effort = 'low',
    retryDelayMs = 1000, requestTimeoutMs = 30000 }) {
    Object.assign(this, { apiKey, model, tools, executor, lang, onEvent, fetchImpl, maxIterations, maxTokens, maxHistory, effort, retryDelayMs, requestTimeoutMs });
    this.systemPrompt = systemPrompt ?? buildSystemPrompt(lang);
    this.history = [];
    this.busy = false;
  }

  reset() { this.history = []; }

  emit(type, data = {}) {
    try { this.onEvent?.({ type, ...data }); } catch { /* UI errors must not break the loop */ }
  }

  // Runs one user turn. Returns { text, toolCalls, aborted? }. Throws AgentError on API failure.
  async send(userText, { signal } = {}) {
    if (this.busy) throw new AgentError('agent is busy');
    this.busy = true;
    const toolCalls = [];
    const texts = [];
    try {
      this.trim();
      this.pushUser([{ type: 'text', text: String(userText) }]);
      for (let i = 0; i < this.maxIterations; i++) {
        if (signal?.aborted) return this.aborted(texts, toolCalls);
        let msg;
        try {
          msg = await this.request(signal);
        } catch (e) {
          if (e.name === 'AbortError' || signal?.aborted) return this.aborted(texts, toolCalls);
          throw e;
        }
        if (msg.stop_reason === 'refusal') {
          // not appended: keeps history free of the declined turn
          const text = this.lang.startsWith('de') ? 'Dabei kann ich leider nicht helfen.' : "Sorry, I can't help with that.";
          this.emit('text', { text });
          this.emit('done', { text, toolCalls });
          return { text, toolCalls };
        }
        let content = Array.isArray(msg.content) ? msg.content : [];
        // a truncated turn may hold a tool_use we will never answer
        if (msg.stop_reason !== 'tool_use') content = content.filter((b) => b.type !== 'tool_use');
        if (content.length) this.history.push({ role: 'assistant', content });
        const t = textOf(content);
        if (t) { texts.push(t); this.emit('text', { text: t }); }
        if (msg.stop_reason !== 'tool_use') break;

        const uses = content.filter((b) => b.type === 'tool_use');
        const results = [];
        const last = i === this.maxIterations - 1;
        for (const u of uses) {
          if (signal?.aborted || last) {
            results.push({ type: 'tool_result', tool_use_id: u.id, is_error: true, content: last ? 'skipped: step limit reached' : 'aborted by user' });
            continue;
          }
          this.emit('tool_call', { name: u.name, input: u.input, id: u.id });
          let r;
          try {
            r = await this.executor.execute(u.name, u.input, { signal });
          } catch (e) {
            r = { ok: false, error: String(e?.message ?? e) };
          }
          if (r == null || typeof r !== 'object') r = { ok: true, value: r };
          toolCalls.push({ name: u.name, input: u.input, result: r });
          this.emit('tool_result', { name: u.name, id: u.id, result: r });
          results.push({ type: 'tool_result', tool_use_id: u.id, content: JSON.stringify(r), ...(r.ok === false && { is_error: true }) });
        }
        this.history.push({ role: 'user', content: results });
        if (signal?.aborted) return this.aborted(texts, toolCalls);
        if (last) this.emit('error', { message: 'step limit reached' });
      }
      const text = texts.join(' ');
      this.emit('done', { text, toolCalls });
      return { text, toolCalls };
    } finally {
      this.busy = false;
    }
  }

  aborted(texts, toolCalls) {
    const text = texts.join(' ');
    this.emit('done', { text, toolCalls, aborted: true });
    return { text, toolCalls, aborted: true };
  }

  // After an abort the history may end on a user message; merge instead of
  // sending two user turns in a row. tool_result blocks stay first.
  pushUser(blocks) {
    const last = this.history[this.history.length - 1];
    if (last?.role === 'user') last.content = [...last.content, ...blocks];
    else this.history.push({ role: 'user', content: blocks });
  }

  // Drops whole turns from the front: the new first message is always a plain
  // user message, so no tool_result loses its tool_use. Thinking blocks are
  // stripped after a trim because they are bound to the exact earlier prefix.
  trim() {
    const h = this.history;
    if (h.length <= this.maxHistory) return;
    let cut = -1;
    for (let i = h.length - this.maxHistory; i < h.length; i++) {
      if (h[i].role === 'user' && !hasToolResult(h[i])) { cut = i; break; }
    }
    if (cut <= 0) {
      // no clean boundary in the window: keep only the last complete turn start
      for (let i = h.length - 1; i > 0; i--) if (h[i].role === 'user' && !hasToolResult(h[i])) { cut = i; break; }
    }
    if (cut <= 0) return;
    this.history = h.slice(cut).map((m) => {
      if (m.role !== 'assistant') return m;
      const content = m.content.filter((b) => b.type !== 'thinking' && b.type !== 'redacted_thinking');
      return { ...m, content: content.length ? content : [{ type: 'text', text: '...' }] };
    });
  }

  body() {
    const b = {
      model: this.model,
      max_tokens: this.maxTokens,
      // explicit breakpoint on the static prefix (tools + system); top-level
      // automatic caching covers the growing conversation
      system: [{ type: 'text', text: this.systemPrompt, cache_control: { type: 'ephemeral' } }],
      cache_control: { type: 'ephemeral' },
      tools: this.tools,
      messages: this.history,
    };
    if (isThinkingModel(this.model)) {
      b.output_config = { effort: this.effort };
      b.fallbacks = 'default';
    }
    return b;
  }

  headers() {
    const h = {
      'content-type': 'application/json',
      'x-api-key': this.apiKey,
      'anthropic-version': API_VERSION,
      'anthropic-dangerous-direct-browser-access': 'true',
    };
    if (isThinkingModel(this.model)) h['anthropic-beta'] = 'server-side-fallback-2026-07-01';
    return h;
  }

  async request(signal) {
    if (!this.apiKey) throw this.fail(new AgentError('Kein API-Schlüssel gesetzt', 0));
    const payload = JSON.stringify(this.body());
    for (let attempt = 0; ; attempt++) {
      this.emit('request', { model: this.model, messages: this.history.length, attempt });
      let res;
      // A stalled request must not hang the conversation: time out per attempt.
      const timeout = new AbortController();
      const timer = setTimeout(() => timeout.abort(), this.requestTimeoutMs);
      const onAbort = () => timeout.abort();
      signal?.addEventListener('abort', onAbort);
      try {
        res = await this.fetchImpl(API_URL, { method: 'POST', headers: this.headers(), body: payload, signal: timeout.signal });
      } catch (e) {
        if (signal?.aborted) throw abortError();
        if (timeout.signal.aborted) throw this.fail(new AgentError('Zeitüberschreitung bei der Anfrage an Claude', 0));
        throw this.fail(new AgentError(`Netzwerkfehler: ${e?.message ?? e}`, 0));
      } finally {
        clearTimeout(timer);
        signal?.removeEventListener('abort', onAbort);
      }
      if (res.ok) return res.json();
      let detail = '';
      try { detail = (await res.json())?.error?.message ?? ''; } catch { /* not json */ }
      const s = res.status;
      if ((s === 408 || s === 429 || s >= 500) && attempt === 0) {
        const ra = Number(res.headers?.get?.('retry-after'));
        await sleep(Number.isFinite(ra) && ra > 0 ? Math.min(ra * 1000, 5000) : this.retryDelayMs, signal);
        continue;
      }
      if (s === 401 || s === 403) throw this.fail(new AgentError(`API-Schlüssel ungültig oder ohne Berechtigung (${s})`, s));
      if (s === 429) throw this.fail(new AgentError(`Zu viele Anfragen, bitte kurz warten (429) ${detail}`.trim(), s));
      if (s === 529 || s >= 500) throw this.fail(new AgentError(`Claude ist gerade überlastet (${s})`, s));
      throw this.fail(new AgentError(`API-Fehler ${s}: ${detail}`.trim(), s));
    }
  }

  fail(err) {
    this.emit('error', { message: err.message, status: err.status });
    return err;
  }
}
