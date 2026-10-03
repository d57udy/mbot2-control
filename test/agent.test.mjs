import { test } from 'node:test';
import assert from 'node:assert/strict';
import { ConversationAgent, buildSystemPrompt, API_URL, DEFAULT_MODEL } from '../js/agent.js';
import { TOOLS } from '../js/tools.js';
import { splitSentences, pickVoice, speak, cancel, tts } from '../js/tts.js';

const KEY = 'test-key-not-real';

const msg = (content, stop_reason = 'end_turn') => ({ id: 'msg_x', type: 'message', role: 'assistant', content, stop_reason });
const text = (t) => ({ type: 'text', text: t });
const use = (id, name, input = {}) => ({ type: 'tool_use', id, name, input });

function jsonRes(body, status = 200, headers = {}) {
  return { ok: status >= 200 && status < 300, status, headers: { get: (k) => headers[k.toLowerCase()] ?? null }, json: async () => body };
}

// Scripted fetch: each entry is a response object, a function(req) -> response, or an Error to throw.
function scripted(...steps) {
  const requests = [];
  const fetchImpl = async (url, init) => {
    const req = { url, init, headers: init.headers, body: JSON.parse(init.body) };
    requests.push(req);
    const s = steps.shift();
    if (!s) throw new Error('no more scripted responses');
    if (s instanceof Error) throw s;
    const r = typeof s === 'function' ? await s(req) : s;
    return r.status ? r : jsonRes(r);
  };
  return { fetchImpl, requests };
}

function recorder(results = {}) {
  const calls = [];
  return {
    calls,
    async execute(name, input) {
      calls.push({ name, input });
      return results[name] ?? { ok: true };
    },
  };
}

function agent(fetchImpl, opts = {}) {
  const events = [];
  const a = new ConversationAgent({ apiKey: KEY, tools: TOOLS, executor: opts.executor ?? recorder(), fetchImpl, onEvent: (e) => events.push(e), retryDelayMs: 1, ...opts });
  return { a, events };
}

test('plain text answer and request shape', async () => {
  const { fetchImpl, requests } = scripted(msg([text('Hallo! Ich bin dein Roboter.')]));
  const { a, events } = agent(fetchImpl);
  const r = await a.send('Hallo');
  assert.equal(r.text, 'Hallo! Ich bin dein Roboter.');
  assert.deepEqual(r.toolCalls, []);
  const req = requests[0];
  assert.equal(req.url, API_URL);
  assert.equal(req.init.method, 'POST');
  assert.equal(req.headers['x-api-key'], KEY);
  assert.equal(req.headers['anthropic-version'], '2023-06-01');
  assert.equal(req.headers['anthropic-dangerous-direct-browser-access'], 'true');
  assert.equal(req.body.model, DEFAULT_MODEL);
  assert.equal(req.body.model, 'claude-haiku-4-5');
  assert.equal(req.body.tools.length, TOOLS.length);
  assert.equal(req.body.system[0].type, 'text');
  assert.deepEqual(req.body.system[0].cache_control, { type: 'ephemeral' });
  assert.match(req.body.system[0].text, /German/);
  assert.equal(req.body.output_config, undefined); // haiku 4.5 takes no effort
  assert.deepEqual(req.body.messages, [{ role: 'user', content: [text('Hallo')] }]);
  assert.deepEqual(events.map((e) => e.type), ['request', 'text', 'done']);
  assert.equal(a.history.length, 2);
});

test('sonnet requests carry effort and server-side fallbacks', async () => {
  const { fetchImpl, requests } = scripted(msg([text('ok')]));
  const { a } = agent(fetchImpl, { model: 'claude-sonnet-5-5' });
  await a.send('hi');
  assert.deepEqual(requests[0].body.output_config, { effort: 'low' });
  assert.equal(requests[0].body.fallbacks, 'default');
  assert.equal(requests[0].headers['anthropic-beta'], 'server-side-fallback-2026-07-01');
});

test('tool_use round trip', async () => {
  const { fetchImpl, requests } = scripted(
    msg([text('Ich fahre vor.'), use('tu_1', 'move_straight', { cm: 20 })], 'tool_use'),
    msg([text('Fertig!')]),
  );
  const ex = recorder({ move_straight: { ok: true, moved_cm: 20 } });
  const { a, events } = agent(fetchImpl, { executor: ex });
  const r = await a.send('Fahr ein Stück vor');
  assert.equal(r.text, 'Ich fahre vor. Fertig!');
  assert.deepEqual(ex.calls, [{ name: 'move_straight', input: { cm: 20 } }]);
  const last = requests[1].body.messages.at(-1);
  assert.equal(last.role, 'user');
  assert.equal(last.content[0].type, 'tool_result');
  assert.equal(last.content[0].tool_use_id, 'tu_1');
  assert.deepEqual(JSON.parse(last.content[0].content), { ok: true, moved_cm: 20 });
  assert.equal(last.content[0].is_error, undefined);
  assert.deepEqual(events.map((e) => e.type), ['request', 'text', 'tool_call', 'tool_result', 'request', 'text', 'done']);
  assert.equal(r.toolCalls.length, 1);
});

test('multiple tool_use blocks run sequentially, results in one message', async () => {
  let running = 0;
  let maxRunning = 0;
  const ex = {
    order: [],
    async execute(name) {
      running++; maxRunning = Math.max(maxRunning, running);
      await new Promise((r) => setTimeout(r, 5));
      this.order.push(name);
      running--;
      return { ok: true };
    },
  };
  const { fetchImpl, requests } = scripted(
    msg([use('a', 'express_emotion', { emotion: 'happy' }), use('b', 'turn', { degrees: 90 }), use('c', 'read_distance')], 'tool_use'),
    msg([text('Erledigt.')]),
  );
  const { a } = agent(fetchImpl, { executor: ex });
  await a.send('Freu dich und dreh dich');
  assert.equal(maxRunning, 1);
  assert.deepEqual(ex.order, ['express_emotion', 'turn', 'read_distance']);
  const results = requests[1].body.messages.at(-1).content;
  assert.deepEqual(results.map((b) => b.tool_use_id), ['a', 'b', 'c']);
});

test('tool error sets is_error', async () => {
  const { fetchImpl, requests } = scripted(
    msg([use('t1', 'move_straight', { cm: 50 })], 'tool_use'),
    msg([text('Da ist ein Hindernis.')]),
  );
  const { a } = agent(fetchImpl, { executor: recorder({ move_straight: { ok: false, error: 'obstacle 10 cm ahead' } }) });
  await a.send('fahr');
  const tr = requests[1].body.messages.at(-1).content[0];
  assert.equal(tr.is_error, true);
  assert.match(tr.content, /obstacle/);
});

test('maxIterations caps the loop and leaves no orphan tool_use', async () => {
  const loop = () => msg([use(`t${Math.random()}`, 'read_distance')], 'tool_use');
  const { fetchImpl, requests } = scripted(loop, loop, loop, loop);
  const ex = recorder();
  const { a, events } = agent(fetchImpl, { executor: ex, maxIterations: 3 });
  await a.send('miss immer weiter');
  assert.equal(requests.length, 3);
  assert.equal(ex.calls.length, 2);
  const last = a.history.at(-1);
  assert.equal(last.content[0].is_error, true);
  assert.match(last.content[0].content, /step limit/);
  assert.ok(events.some((e) => e.type === 'error'));
  assertPaired(a.history);
});

test('abort mid-loop stops tools and keeps history consistent', async () => {
  const ac = new AbortController();
  const ex = {
    calls: [],
    async execute(name) { this.calls.push(name); if (name === 'turn') ac.abort(); return { ok: true }; },
  };
  const { fetchImpl, requests } = scripted(
    msg([use('a', 'turn', { degrees: 90 }), use('b', 'move_straight', { cm: 30 })], 'tool_use'),
    msg([text('weiter')]),
    msg([text('Okay, ich habe angehalten.')]),
  );
  const { a } = agent(fetchImpl, { executor: ex });
  const r = await a.send('dreh dich und fahr', { signal: ac.signal });
  assert.equal(r.aborted, true);
  assert.deepEqual(ex.calls, ['turn']);
  assert.equal(requests.length, 1);
  const results = a.history.at(-1).content;
  assert.equal(results[1].tool_use_id, 'b');
  assert.equal(results[1].is_error, true);
  assertPaired(a.history);
  // next turn merges into the trailing user message instead of two user turns
  await a.send('Was ist passiert?');
  const sent = requests[1].body.messages;
  for (let i = 1; i < sent.length; i++) assert.notEqual(sent[i].role, sent[i - 1].role);
  assert.equal(sent.at(-1).content.at(-1).text, 'Was ist passiert?');
});

test('abort during fetch returns aborted', async () => {
  const ac = new AbortController();
  const fetchImpl = (url, init) => new Promise((_, reject) => {
    init.signal.addEventListener('abort', () => { const e = new Error('aborted'); e.name = 'AbortError'; reject(e); });
  });
  const { a } = agent(fetchImpl);
  const p = a.send('hallo', { signal: ac.signal });
  ac.abort();
  const r = await p;
  assert.equal(r.aborted, true);
  assert.equal(a.busy, false);
});

test('401 gives a clear error without leaking the key', async () => {
  const { fetchImpl } = scripted(jsonRes({ type: 'error', error: { type: 'authentication_error', message: 'invalid x-api-key' } }, 401));
  const { a, events } = agent(fetchImpl);
  await assert.rejects(a.send('hi'), (e) => e.status === 401 && /API-Schlüssel/.test(e.message) && !e.message.includes(KEY));
  assert.ok(events.some((e) => e.type === 'error' && e.status === 401));
  assert.ok(!JSON.stringify(events).includes(KEY));
});

test('529 is retried once', async () => {
  const { fetchImpl, requests } = scripted(jsonRes({ error: { message: 'Overloaded' } }, 529), msg([text('Da bin ich.')]));
  const { a } = agent(fetchImpl);
  const r = await a.send('hi');
  assert.equal(r.text, 'Da bin ich.');
  assert.equal(requests.length, 2);
});

test('second 529 and network errors surface as errors', async () => {
  let { fetchImpl } = scripted(jsonRes({}, 529), jsonRes({}, 529));
  let { a } = agent(fetchImpl);
  await assert.rejects(a.send('hi'), (e) => e.status === 529);
  ({ fetchImpl } = scripted(new TypeError('Failed to fetch')));
  const b = agent(fetchImpl);
  await assert.rejects(b.a.send('hi'), /Netzwerkfehler/);
  assert.ok(b.events.some((e) => e.type === 'error'));
});

test('missing key fails fast', async () => {
  const { fetchImpl, requests } = scripted();
  const { a } = agent(fetchImpl, { apiKey: '' });
  await assert.rejects(a.send('hi'), /API-Schlüssel/);
  assert.equal(requests.length, 0);
});

test('refusal is not added to history', async () => {
  const { fetchImpl } = scripted(msg([], 'refusal'));
  const { a } = agent(fetchImpl);
  const r = await a.send('...');
  assert.ok(r.text.length > 0);
  assert.equal(a.history.length, 1);
});

function assertPaired(history) {
  assert.equal(history[0].role, 'user');
  assert.ok(!history[0].content.some?.((b) => b.type === 'tool_result'), 'history starts with a tool_result');
  for (let i = 0; i < history.length; i++) {
    const m = history[i];
    if (m.role !== 'user' || !Array.isArray(m.content)) continue;
    for (const b of m.content.filter((x) => x.type === 'tool_result')) {
      const prev = history[i - 1];
      assert.ok(prev?.role === 'assistant' && prev.content.some((x) => x.type === 'tool_use' && x.id === b.tool_use_id), `orphan ${b.tool_use_id}`);
    }
  }
}

test('history trimming never orphans tool_result and strips thinking', async () => {
  let n = 0;
  const step = (req) => {
    const last = req.body.messages.at(-1);
    const isResult = last.content.some((b) => b.type === 'tool_result');
    n++;
    return isResult
      ? msg([{ type: 'thinking', thinking: '', signature: 's' }, text(`ok ${n}`)])
      : msg([{ type: 'thinking', thinking: '', signature: 's' }, use(`t${n}`, 'read_distance')], 'tool_use');
  };
  const steps = Array(40).fill(step);
  const { fetchImpl, requests } = scripted(...steps);
  const { a } = agent(fetchImpl, { maxHistory: 7 });
  for (let i = 0; i < 10; i++) await a.send(`frage ${i}`);
  assert.ok(a.history.length <= 7 + 4);
  for (const r of requests) assertPaired(r.body.messages);
  assertPaired(a.history);
  assert.equal(a.history.at(-1).content.at(-1).text, `ok ${n}`);
  assert.equal(requests.at(-1).body.messages[0].content[0].text.startsWith('frage'), true);
  assert.notEqual(requests.at(-1).body.messages[0].content[0].text, 'frage 0'); // trimmed
  assert.ok(!requests.at(-1).body.messages[1].content.some((b) => b.type === 'thinking'));
  a.reset();
  assert.equal(a.history.length, 0);
});

test('system prompt follows language and states conventions', () => {
  const de = buildSystemPrompt('de-DE');
  const en = buildSystemPrompt('en-US');
  assert.match(de, /German/);
  assert.match(en, /English/);
  for (const p of [de, en]) {
    assert.match(p, /clockwise/);
    assert.match(p, /20 cm/);
    assert.match(p, /no markdown/);
    assert.doesNotMatch(p, /\u2014/);
  }
});

test('tts helpers', async () => {
  assert.deepEqual(splitSentences('Hallo! Wie geht es dir? Gut.'), ['Hallo!', 'Wie geht es dir?', 'Gut.']);
  const long = splitSentences('wort '.repeat(100), 50);
  assert.ok(long.every((s) => s.length <= 50));
  const voices = [
    { lang: 'en-US', localService: true, name: 'en' },
    { lang: 'de-AT', localService: true, name: 'at' },
    { lang: 'de-DE', localService: false, name: 'net' },
    { lang: 'de_DE', localService: true, name: 'local' },
  ];
  assert.equal(pickVoice(voices, 'de-DE').name, 'local');
  assert.equal(pickVoice(voices, 'fr-FR'), null);
  await speak('no window in node'); // must not throw
  assert.equal(tts.speaking, false);

  const spoken = [];
  globalThis.window = {
    SpeechSynthesisUtterance: class { constructor(t) { this.text = t; } },
    speechSynthesis: {
      speaking: false, paused: false,
      getVoices: () => voices,
      speak(u) { spoken.push(u); setTimeout(() => u.onend?.(), 1); },
      cancel() {}, pause() {}, resume() {},
    },
  };
  try {
    await speak('Erster Satz. Zweiter Satz!', 'de-DE');
    assert.deepEqual(spoken.map((u) => u.text), ['Erster Satz.', 'Zweiter Satz!']);
    assert.equal(spoken[0].voice.name, 'local');
    assert.equal(spoken[0].lang, 'de-DE');
    const p = speak('Eins. Zwei. Drei.');
    cancel();
    await p;
    assert.equal(tts.speaking, false);
  } finally {
    delete globalThis.window;
  }
});
