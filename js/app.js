import { BleRobot } from './robot-ble.js';
import { SimRobot, SIM_ROOM, SIM_OBSTACLES } from './robot-sim.js';
import { drawSim } from './sim-view.js';
import { scan, findOpenings, describeScan, driveToward, explore } from './scan.js';
import { drawRadar } from './radar.js';
import { GridMap } from './gridmap.js';
import { PoseTracker } from './pose.js';
import { Navigator } from './navigate.js';
import { drawMap, fitView, screenToWorld } from './mapview.js';
import { CommandBus, makeCommand, LIMITS, LED_EFFECTS, EYE_EFFECTS } from './bus.js';
import { VoiceListener, parseUtterance, isStop, isStrictStop } from './voice.js';
import { toolsFor, createToolExecutor } from './tools.js';
import { ConversationAgent, MODELS, DEFAULT_MODEL } from './agent.js';
import tts from './tts.js';
import { Joystick } from './joystick.js';
import { DriveStream, mixArcade } from './drive.js';

const $ = (id) => document.getElementById(id);
const sleep = (ms) => new Promise((r) => setTimeout(r, ms));
const store = {
  get(k, d) { try { return localStorage.getItem(k) ?? d; } catch { return d; } },
  set(k, v) { try { localStorage.setItem(k, v); } catch { /* private mode */ } },
};

// --- log ---------------------------------------------------------------

const logEl = $('log');
function log(msg, detail) {
  const t = new Date().toLocaleTimeString('de-DE');
  console.log(`[mbot] ${t} ${msg}${detail ? ` | ${detail}` : ''}`); // readable by browser automation
  logEl.textContent = `${t} ${msg}${detail ? `\n      ${detail}` : ''}\n${logEl.textContent}`.slice(0, 20000);
}

// --- state -------------------------------------------------------------

const STATUS_TEXT = {
  idle: 'nicht verbunden', connecting: 'verbinde…', handshake: 'Live-Modus…',
  connected: 'verbunden', disconnected: 'getrennt',
};

let robot = null;
let ble = null; // kept so "reconnect" can reuse the same BluetoothDevice
let wakeLock = null;

const bus = new CommandBus({ log, onSensor: showSensor });
const stream = new DriveStream({
  bus,
  maxRpm: () => Number($('speed').value),
  onWheels: (l, r) => { $('w-left').textContent = l; $('w-right').textContent = r; },
});
window.mbot = { bus, makeCommand, LIMITS, stream }; // console / future bridge access

function setStatus(state) {
  const el = $('status');
  el.dataset.state = state;
  el.textContent = `${STATUS_TEXT[state] ?? state}${robot?.kind === 'sim' && state === 'connected' ? ' (Sim)' : ''}`;
  const live = state === 'connected';
  $('btn-disconnect').hidden = !live;
  $('btn-reconnect').hidden = !(state === 'disconnected' && ble?.device);
  $('btn-connect').hidden = live;
  $('btn-sim').hidden = live;
  const wd = $('watchdog');
  wd.hidden = !live;
  if (live) {
    wd.dataset.ok = String(!!robot.watchdog);
    wd.textContent = robot.watchdog ? 'Watchdog an' : 'kein Watchdog';
    wd.hidden = robot.kind === 'sim';
    wd.title = robot.watchdog
      ? 'Der Roboter stoppt 0,4 s nach dem letzten Fahrbefehl von selbst.'
      : 'Bei Verbindungsabbruch während Joystick-Fahrt kann der Roboter weiterfahren.';
  }
  if (state === 'disconnected') {
    stream.halt();
    joystick.reset();
    releaseWakeLock();
    log('Verbindung getrennt.');
  }
  if (live) { applyWheelSettings(); acquireWakeLock(); pollSensors(); }
}

// --- connect -----------------------------------------------------------

function checkSupport() {
  const msgs = [];
  if (!navigator.bluetooth) msgs.push('Web Bluetooth fehlt: Chrome auf Android oder Desktop nötig (nicht iOS, nicht Firefox).');
  if (!window.isSecureContext) msgs.push('Seite muss über HTTPS laufen.');
  if (!(window.SpeechRecognition || window.webkitSpeechRecognition)) msgs.push('Spracherkennung wird von diesem Browser nicht unterstützt.');
  if (msgs.length) { $('support-warning').textContent = msgs.join(' '); $('support-warning').hidden = false; }
  $('btn-connect').disabled = !navigator.bluetooth;
}

async function connectBle(reuse) {
  try {
    if (!reuse || !ble) {
      ble = new BleRobot({
        log,
        onStatus: setStatus,
        chunkSize: Number($('opt-chunk').value),
        chunkDelayMs: Number($('opt-delay').value),
        helpers: $('opt-helpers').checked,
        debugAllDevices: $('opt-all').checked,
      });
    }
    robot = ble;
    bus.setRobot(robot);
    applyWheelSettings();
    await robot.connect();
    resetMap();
    bus.submit(makeCommand('read', { sensor: 'battery' }));
    probeEyes();
  } catch (e) {
    log(`Verbindung fehlgeschlagen: ${e.message}`);
    setStatus(ble?.device ? 'disconnected' : 'idle');
  }
}

async function connectSim() {
  $('sim').hidden = false;
  robot = new SimRobot({ log, onStatus: setStatus, onChange: redrawSim });
  bus.setRobot(robot);
  await robot.connect();
  resetMap();
  redrawSim();
  bus.submit(makeCommand('read', { sensor: 'battery' }));
}

$('btn-connect').onclick = () => connectBle(false);
$('btn-reconnect').onclick = () => connectBle(true);
$('btn-sim').onclick = connectSim;
$('btn-disconnect').onclick = async () => {
  stream.halt();
  await robot?.disconnect();
  if (robot?.kind === 'sim') $('sim').hidden = true;
};

function applyWheelSettings() {
  if (robot?.wheels) {
    robot.wheels.mirrored = $('opt-mirrored').checked;
    robot.wheels.swap = $('opt-swap').checked;
  }
}
$('opt-mirrored').onchange = applyWheelSettings;
$('opt-swap').onchange = applyWheelSettings;
$('opt-guard').onchange = () => { bus.guard = $('opt-guard').checked; };

// --- sensors -----------------------------------------------------------

function showSensor(name, v) {
  if (name === 'battery') $('r-battery').textContent = `${Math.round(v)} %`;
  if (name === 'distance') {
    const el = $('r-distance');
    el.textContent = v >= 300 ? 'frei' : `${Math.round(v)} cm`;
    el.classList.toggle('alert', v < LIMITS.obstacleCm);
  }
  if (name === 'floor') showFloor(v);
}

const CSS_COLORS = {
  red: '#e5484d', yellow: '#f5d90a', green: '#30a46c', cyan: '#05a2c2',
  blue: '#3e63dd', purple: '#8e4ec6', white: '#ffffff', black: '#111111',
};
const COLOR_DE = {
  red: 'rot', yellow: 'gelb', green: 'grün', cyan: 'cyan', blue: 'blau', purple: 'lila', white: 'weiß', black: 'schwarz',
};

function showFloor({ line, offset, gray, names, rgb }) {
  document.querySelectorAll('.probe').forEach((el, i) => {
    el.querySelector('.gray').textContent = gray?.[i] ?? '–';
    // line bits: bit 3 = L2 ... bit 0 = R2
    el.classList.toggle('on', Number.isInteger(line) && ((line >> (3 - i)) & 1) === 1);
    const sw = el.querySelector('.swatch');
    if (rgb?.[i]) sw.style.background = `rgb(${rgb[i].join(',')})`;
    else if (names?.[i]) sw.style.background = CSS_COLORS[names[i]] ?? '';
    else sw.style.background = '';
    el.querySelector('.cname').textContent = names?.[i] ? (COLOR_DE[names[i]] ?? names[i]) : '–';
  });
  $('line-sta').textContent = Number.isInteger(line) ? line.toString(2).padStart(4, '0') : '–';
  const off = Number(offset);
  $('offset-val').textContent = Number.isFinite(off) ? off : '–';
  if (Number.isFinite(off)) $('offset-dot').style.left = `${50 + off / 2}%`;
}

// One loop for all polling so BLE traffic stays predictable.
let polling = false;
async function pollSensors() {
  if (polling) return;
  polling = true;
  let n = 0;
  while (robot?.connected) {
    const tick = performance.now();
    if (!document.hidden) {
      const driving = stream.timer || mode === 'joystick';
      if ($('opt-guard').checked && (driving || n % 4 === 0) && !bus.sensorsQuiet()) {
        await bus.submit(makeCommand('read', { sensor: 'distance' }));
      }
      if ($('floor-live').checked && $('panel-floor').open && n % 2 === 0) {
        await bus.submit(makeCommand('read', { sensor: 'floor', colors: $('floor-colors').checked }));
      }
      if (n % 120 === 0) await bus.submit(makeCommand('read', { sensor: 'battery' }));
    }
    n++;
    await sleep(Math.max(50, 250 - (performance.now() - tick)));
  }
  polling = false;
}

// --- driving -----------------------------------------------------------

const speedEl = $('speed');
// One speed for everything: joystick/buttons (max RPM), and turns, straight
// moves, scans and AI moves (bus.settings.speed, capped at LIMITS.maxSpeed).
function setSpeed(v) {
  const s = Math.max(10, Math.min(LIMITS.maxDriveRpm, Math.round(v)));
  speedEl.value = s;
  bus.settings.speed = Math.min(s, LIMITS.maxSpeed);
  $('r-speed').textContent = s;
  document.querySelectorAll('[data-speed]').forEach((b) => b.classList.toggle('on', Number(b.dataset.speed) === s));
  store.set('speed', s);
}
speedEl.oninput = () => setSpeed(Number(speedEl.value));
document.querySelectorAll('[data-speed]').forEach((b) => { b.onclick = () => setSpeed(Number(b.dataset.speed)); });

function emergencyStop(src = 'ui') {
  stream.halt();
  scanAbort?.abort();
  chatAbort?.abort();
  tts.cancel();
  joystick.reset();
  document.querySelectorAll('.pad .held').forEach((b) => b.classList.remove('held'));
  return bus.stop(src);
}
document.querySelectorAll('.btn-stop').forEach((b) => b.addEventListener('pointerdown', () => emergencyStop()));

// Button targets as fractions of max speed (forward-positive wheels).
const BUTTON_TARGETS = {
  forward: [0.8, 0.8], backward: [-0.6, -0.6], left: [-0.45, 0.45], right: [0.45, -0.45],
};

document.querySelectorAll('.pad .dir').forEach((btn) => {
  let held = false;
  btn.addEventListener('pointerdown', (e) => {
    e.preventDefault();
    try { btn.setPointerCapture(e.pointerId); } catch { /* synthetic events */ }
    held = true;
    takeOver();
    btn.classList.add('held');
    stream.set(...BUTTON_TARGETS[btn.dataset.drive]);
  });
  const end = () => {
    if (!held) return;
    held = false;
    btn.classList.remove('held');
    stream.release();
  };
  btn.addEventListener('pointerup', end);
  btn.addEventListener('pointercancel', end);
  btn.addEventListener('lostpointercapture', end);
  btn.addEventListener('contextmenu', (e) => e.preventDefault());
});

// Touching the manual controls cancels a running scan or AI action.
function takeOver() {
  if (scanAbort || chatAbort) log('Manuelle Steuerung übernimmt.');
  scanAbort?.abort();
  chatAbort?.abort();
}

const joystick = new Joystick($('joystick'), {
  onStart: takeOver,
  onMove: ({ x, y }) => stream.set(...mixArcade(x, y)),
  onRelease: () => stream.release(),
});

let mode = 'buttons';
function setMode(m) {
  mode = m === 'joystick' ? 'joystick' : 'buttons';
  document.querySelectorAll('.tabs [data-mode]').forEach((t) => t.setAttribute('aria-selected', String(t.dataset.mode === mode)));
  $('mode-buttons').hidden = mode !== 'buttons';
  $('mode-joystick').hidden = mode !== 'joystick';
  store.set('mode', mode);
}
document.querySelectorAll('.tabs [data-mode]').forEach((t) => { t.onclick = () => { stream.release(); setMode(t.dataset.mode); }; });

// --- voice -------------------------------------------------------------

const transcriptEl = $('transcript');
let voice = null;

function makeVoice() {
  return new VoiceListener({
    lang: $('lang').value,
    onState: (s) => { $('voice-state').textContent = s === 'off' ? '' : `Mikrofon: ${s}`; },
    onTranscript: (text, final) => { transcriptEl.textContent = final ? text : `${text} …`; },
    onStop: (text) => { log(`🎤 „${text}“ → stop`); emergencyStop('voice'); },
    onCommand: (text) => (voiceMode === 'chat' ? chat(text) : runVoice(text)),
    stopTest: voiceMode === 'chat' ? isStrictStop : isStop,
  });
}

function runVoice(text) {
  const p = parseUtterance(text);
  if (!p) { log(`🎤 „${text}“ → nicht verstanden`); return; }
  log(`🎤 „${text}“ → ${p.cmd} ${JSON.stringify(p.args)}${p.drive ? ` ${p.drive}s` : ''}`);
  if (p.cmd === 'speed') { setSpeed(Number(speedEl.value) + p.args.delta); return; }
  if (p.cmd === 'stop') { emergencyStop('voice'); return; }
  if (p.drive) {
    const t = BUTTON_TARGETS[p.args.dir];
    stream.set(...t, Math.min(p.drive, 10) * 1000);
    return;
  }
  stream.halt();
  bus.submit(makeCommand(p.cmd, p.args, 'voice'));
}
window.mbot.say = runVoice; // inject an utterance as if spoken

function setMic(on) {
  if (on) {
    voice = makeVoice();
    if (!voice.supported) { log('Spracherkennung nicht verfügbar'); return; }
    voice.start();
  } else {
    voice?.stop();
  }
  $('btn-mic').setAttribute('aria-pressed', String(on));
  $('btn-mic').textContent = on ? '🎤 Sprache aus' : '🎤 Sprache an';
}
$('btn-mic').onclick = () => setMic($('btn-mic').getAttribute('aria-pressed') !== 'true');
$('lang').onchange = () => { if (voice?.active) { setMic(false); setMic(true); } };

// --- lights ------------------------------------------------------------

const hexToRgb = (h) => [1, 3, 5].map((i) => parseInt(h.slice(i, i + 2), 16));
const send = (cmd, args) => bus.submit(makeCommand(cmd, args, 'ui'));

// Sliders fire continuously; only the latest value is sent, at most every 150 ms.
function throttled(fn, ms = 150) {
  let t = null, last = null;
  return (...a) => {
    last = a;
    if (t) return;
    t = setTimeout(() => { t = null; fn(...last); }, ms);
  };
}

document.querySelectorAll('.led-color').forEach((inp) => {
  inp.addEventListener('input', throttled(() => {
    const [r, g, b] = hexToRgb(inp.value);
    send('led', { r, g, b, id: Number(inp.dataset.id) });
  }));
});
$('btn-leds-apply').onclick = () => send('leds', {
  colors: [...document.querySelectorAll('.led-color')].map((i) => hexToRgb(i.value)),
});
$('btn-leds-all').onclick = () => {
  const [r, g, b] = hexToRgb($('led-all').value);
  document.querySelectorAll('.led-color').forEach((i) => { i.value = $('led-all').value; });
  send('led', { r, g, b });
};
$('btn-leds-off').onclick = () => send('led_off', {});
$('led-bri').addEventListener('input', throttled(() => send('led_brightness', { value: Number($('led-bri').value) })));

const effectSel = $('led-effect');
for (const name of LED_EFFECTS) effectSel.add(new Option(name.replace('_', ' '), name));
$('btn-led-effect').onclick = () => send('led_effect', { name: effectSel.value });

const sendEyes = throttled(() => send('eyes', { left: Number($('eye-left').value), right: Number($('eye-right').value) }));
$('eye-left').addEventListener('input', sendEyes);
$('eye-right').addEventListener('input', sendEyes);

const EYE_LABELS = {
  happy: 'fröhlich', new_happy: 'fröhlich 2', wink: 'zwinkern', naughty: 'frech', aggrieved: 'beleidigt',
  raises_brow: 'Braue hoch', look_left: 'schaut links', look_right: 'schaut rechts', eye_left: 'Auge links',
  eye_right: 'Auge rechts', thinking: 'nachdenken', dizzy: 'schwindelig', standby: 'standby',
};
for (const name of EYE_EFFECTS) {
  const b = document.createElement('button');
  b.textContent = EYE_LABELS[name] ?? name;
  b.dataset.effect = name;
  b.onclick = async () => {
    // drive frames sent during a blocking effect pile up on the robot
    if (stream.timer) { log('Erst anhalten, dann Emotion abspielen.'); return; }
    const r = await send('eyes_effect', { name });
    if (r.ok) log(`Augen ${name}: ${r.value?.running ? 'läuft noch' : `${r.value?.ms ?? '?'} ms`}`);
  };
  $('eye-effects').append(b);
}
for (let id = 1; id <= 8; id++) {
  const l = document.createElement('label');
  l.innerHTML = `LED ${id}<input type="range" min="0" max="100" value="0">`;
  l.querySelector('input').addEventListener('input', throttled((e) => send('eye_led', { id, bri: Number(e.target.value) })));
  $('eye-leds').append(l);
}

// Hide effect buttons this firmware does not have; show all if probing fails.
async function probeEyes() {
  const buttons = document.querySelectorAll('#eye-effects [data-effect]');
  buttons.forEach((b) => { b.hidden = false; });
  if (!robot?.probeEyeEffects) return;
  try {
    const ok = await robot.probeEyeEffects(EYE_EFFECTS);
    if (Array.isArray(ok) && ok.length) {
      buttons.forEach((b) => { b.hidden = !ok.includes(b.dataset.effect); });
      log(`Augen-Effekte verfügbar: ${ok.join(', ')}`);
    }
  } catch (e) { log(`Augen-Effekte nicht geprüft: ${e.message}`); }
}

$('floor-light').onchange = () => send('floor_light', { color: $('floor-light').value });

// --- conversation (LLM with tool calling) ------------------------------------

let voiceMode = store.get('voiceMode', 'commands');
let agent = null;
let chatAbort = null;
let chatRunning = null; // promise of the current agent.send
let aiKey = store.get('aiKey', '');

function setVoiceMode(m) {
  voiceMode = m === 'chat' ? 'chat' : 'commands';
  document.querySelectorAll('[data-voice]').forEach((t) => t.setAttribute('aria-selected', String(t.dataset.voice === voiceMode)));
  store.set('voiceMode', voiceMode);
  if (voiceMode === 'chat') $('panel-chat').open = true;
  if (!voice?.active) {
    transcriptEl.textContent = voiceMode === 'chat'
      ? 'Sprich ganz normal mit dem Roboter, z.B. „Wie geht es dir?“ oder „Schau dich mal um und fahr dahin, wo Platz ist.“ „Stopp“ hält alles an.'
      : 'Sag z.B. „vorwärts“, „links“, „rechts 45 Grad“, „zurück 2 Sekunden“, „Licht blau“, „stopp“.';
  }
  if (voice?.active) { setMic(false); setMic(true); } // new stop-word rules
}
document.querySelectorAll('[data-voice]').forEach((t) => { t.onclick = () => setVoiceMode(t.dataset.voice); });

for (const m of MODELS) $('ai-model').add(new Option(m.label, m.id));
$('ai-model').value = store.get('aiModel', DEFAULT_MODEL);
$('ai-key').value = aiKey;
$('ai-remember').checked = !!aiKey;
$('ai-speak').checked = store.get('aiSpeak', '1') === '1';

function updateChatHint() { $('chat-hint').hidden = !!aiKey; }
function dropAgent() { chatAbort?.abort(); agent = null; }
$('ai-key').onchange = () => {
  aiKey = $('ai-key').value.trim();
  store.set('aiKey', $('ai-remember').checked ? aiKey : '');
  updateChatHint();
  dropAgent();
};
$('ai-remember').onchange = () => store.set('aiKey', $('ai-remember').checked ? aiKey : '');
$('ai-model').onchange = () => { store.set('aiModel', $('ai-model').value); dropAgent(); };
$('ai-speak').onchange = () => store.set('aiSpeak', $('ai-speak').checked ? '1' : '0');
updateChatHint();

const EMOTION_DE = {
  happy: 'fröhlich', excited: 'aufgeregt', sad: 'traurig', surprised: 'überrascht', thinking: 'nachdenklich',
  curious: 'neugierig', dizzy: 'schwindelig', wink: 'zwinkert', naughty: 'frech', angry: 'verärgert',
  sleepy: 'müde', neutral: 'neutral',
};

function chatLine(kind, text) {
  const li = document.createElement('li');
  li.className = kind;
  li.textContent = text;
  $('chat-log').append(li);
  li.scrollIntoView({ block: 'nearest' });
  return li;
}

function getAgent() {
  if (!aiKey) throw new Error('Kein API-Schlüssel (Einstellungen → KI).');
  if (agent && agent.lang === $('lang').value) return agent;
  const executor = createToolExecutor({
    navigator: nav,
    bus, makeCommand, scan, findOpenings, describeScan, driveToward,
    onEmotion: (e) => chatLine('tool', `😶 ${EMOTION_DE[e] ?? e}`),
    onScan: (points) => {
      if (robot?.kind === 'sim') { lastScan = { x: robot.state.x, y: robot.state.y, heading: robot.state.heading, points }; }
      $('panel-scan').open = true;
      showScan(points);
      redrawSim();
    },
  });
  agent = new ConversationAgent({
    apiKey: aiKey,
    model: $('ai-model').value,
    tools: toolsFor({ navigation: true }),
    executor,
    lang: $('lang').value,
    onEvent: (e) => {
      if (e.type === 'tool_call') stream.halt();
      if (e.type === 'tool_call') chatLine('tool', `⚙ ${e.name} ${JSON.stringify(e.input ?? {})}`);
      if (e.type === 'tool_result' && e.result && e.result.ok === false) chatLine('tool', `⚠ ${e.name}: ${e.result.error}`);
      if (e.type === 'error') log(`! KI: ${e.message ?? e.error ?? ''}`);
    },
  });
  agent.lang = $('lang').value;
  return agent;
}

async function chat(text) {
  text = text.trim();
  if (!text) return;
  chatLine('user', text);
  let a;
  try { a = getAgent(); } catch (e) { chatLine('error', e.message); return; }
  chatAbort?.abort();
  tts.cancel();
  const ctl = new AbortController();
  chatAbort = ctl;
  // barge-in: the agent rejects a second send until the aborted one unwinds
  if (chatRunning) await chatRunning.catch(() => {});
  if (ctl.signal.aborted) { if (chatAbort === ctl) chatAbort = null; return; }
  $('chat-state').textContent = 'denkt nach…';
  try {
    chatRunning = a.send(text, { signal: ctl.signal });
    const r = await chatRunning;
    if (r.aborted || ctl.signal.aborted) { $('chat-state').textContent = 'abgebrochen'; return; }
    if (r.text) {
      chatLine('bot', r.text);
      if ($('ai-speak').checked) await say(r.text);
    }
    $('chat-state').textContent = '';
  } catch (e) {
    if (ctl.signal.aborted) { $('chat-state').textContent = 'abgebrochen'; return; }
    chatLine('error', e.message);
    $('chat-state').textContent = '';
  } finally {
    if (chatAbort === ctl) { chatAbort = null; chatRunning = null; }
  }
}
window.mbot.chat = chat;

// The mic is paused while the robot speaks so it does not hear itself.
async function say(text) {
  const micWasOn = !!voice?.active;
  if (micWasOn) voice.stop();
  const micPressed = () => $('btn-mic').getAttribute('aria-pressed') === 'true';
  $('chat-state').textContent = 'spricht…';
  try { await tts.speak(text, $('lang').value); } finally {
    if (micWasOn && micPressed() && !document.hidden && !voice?.active) setMic(true);
  }
}

$('btn-chat-send').onclick = () => { const t = $('chat-text').value; $('chat-text').value = ''; chat(t); };
$('chat-text').addEventListener('keydown', (e) => { if (e.key === 'Enter') $('btn-chat-send').click(); });
$('btn-chat-reset').onclick = () => { dropAgent(); tts.cancel(); $('chat-log').textContent = ''; $('chat-state').textContent = ''; };

// --- manual command box --------------------------------------------------

$('btn-send').onclick = async () => {
  const text = $('cmd-input').value.trim();
  if (!text) return;
  try {
    if (text.startsWith('py:')) {
      if (!robot?.connected) throw new Error('nicht verbunden');
      const v = await robot.raw(text.slice(3).trim());
      log(`= ${JSON.stringify(v)}`);
      return;
    }
    const c = JSON.parse(text);
    const r = await bus.submit(makeCommand(c.cmd, c.args, 'ui'));
    log(`= ${JSON.stringify(r)}`);
  } catch (e) {
    log(`! ${e.message}`);
  }
};

// Link settings apply on the next connect; remembered per device.
$('opt-delay').value = store.get('chunkDelay', '8');
$('opt-helpers').checked = store.get('helpers', '0') === '1';
$('opt-delay').onchange = () => { store.set('chunkDelay', $('opt-delay').value); if (ble) ble.chunkDelayMs = Number($('opt-delay').value); };
$('opt-helpers').onchange = () => { store.set('helpers', $('opt-helpers').checked ? '1' : '0'); log('Roboter-Helfer: wirkt beim nächsten Verbinden (Roboter vorher aus- und einschalten).'); };

$('btn-sensortest').onclick = async () => {
  if (!robot?.sensorTest) { log('! Sensor-Test nur mit echtem Roboter'); return; }
  $('btn-sensortest').disabled = true;
  try { await robot.sensorTest((line) => log(line)); } finally { $('btn-sensortest').disabled = false; }
};

// Copies the log (oldest first) for pasting into a chat; falls back to the share sheet.
$('btn-log-copy').onclick = async () => {
  const text = logEl.textContent.split('\n').reverse().join('\n').trim();
  try {
    await navigator.clipboard.writeText(text);
    log('Log in die Zwischenablage kopiert.');
  } catch {
    try { await navigator.share({ title: 'mBot2 Log', text }); } catch { log('! Kopieren nicht möglich'); }
  }
};

$('btn-linktest').onclick = async () => {
  if (!robot?.connectionTest) { log('! Verbindungstest nur mit echtem Roboter'); return; }
  $('btn-linktest').disabled = true;
  try { await robot.connectionTest((line) => log(line)); } finally { $('btn-linktest').disabled = false; }
};

$('btn-diagnose').onclick = async () => {
  if (!robot?.connected) { log('! nicht verbunden'); return; }
  try {
    const [fw, led, us, quad, mb] = await robot.diagnose();
    log(`Firmware ${fw}\ncyberpi.led: ${led}\ncyberpi.ultrasonic2: ${us}\ncyberpi.quad_rgb_sensor: ${quad}\nmbot2: ${mb}`);
  } catch (e) { log(`! Diagnose: ${e.message}`); }
};

// --- page lifecycle safety ---------------------------------------------

async function acquireWakeLock() {
  try {
    if ('wakeLock' in navigator && !wakeLock && !document.hidden) {
      wakeLock = await navigator.wakeLock.request('screen');
      wakeLock.addEventListener('release', () => { wakeLock = null; });
    }
  } catch (e) { log(`Wake Lock: ${e.message}`); }
}
function releaseWakeLock() { wakeLock?.release(); wakeLock = null; }

document.addEventListener('visibilitychange', () => {
  if (document.hidden) {
    if (robot?.connected) emergencyStop('ui');
    if (voice?.active || $('btn-mic').getAttribute('aria-pressed') === 'true') setMic(false);
  } else if (robot?.connected) {
    acquireWakeLock();
  }
});
window.addEventListener('pagehide', () => { if (robot?.connected) emergencyStop('ui'); });
window.addEventListener('blur', () => { if (stream.timer) emergencyStop('ui'); });

// --- simulator drawing -------------------------------------------------

let lastScan = null; // { x, y, heading, points } for drawing rays in the sim

function redrawSim() {
  if (robot?.kind !== 'sim') return;
  drawSim($('sim'), robot.state, SIM_ROOM, SIM_OBSTACLES, { rays: lastScan });
}

// --- environment scan ------------------------------------------------------

let scanAbort = null;

function showScan(points) {
  const openings = findOpenings(points);
  drawRadar($('radar'), points, { highlight: openings });
  $('scan-text').textContent = describeScan(points, openings);
  const chips = $('scan-openings');
  chips.textContent = '';
  for (const o of openings.slice(0, 4)) {
    const b = document.createElement('button');
    const dir = o.angle === 0 ? 'geradeaus' : o.angle > 0 ? `${o.angle}° rechts` : `${-o.angle}° links`;
    b.textContent = `→ ${dir} (${o.cm >= 300 ? 'frei' : `${Math.round(o.cm)} cm`})`;
    b.onclick = () => runScanTask(async () => {
      const r = await driveToward(bus, { angle: o.angle, cm: 40, makeCommand: bus.stamped() });
      log(`Fahrt Richtung ${o.angle}°: ${r.ok ? `${r.droveCm} cm` : r.error ?? r.note}`);
      lastScan = null;
      redrawSim();
    });
    chips.append(b);
  }
  return openings;
}

// Scans and explores share one AbortController so STOPP cancels them.
async function runScanTask(fn) {
  if (!robot?.connected) { log('! nicht verbunden'); return null; }
  if (scanAbort) { log('! Scan läuft bereits'); return null; }
  stream.halt();
  scanAbort = new AbortController();
  $('btn-scan').disabled = $('btn-explore').disabled = $('btn-home').disabled = true;
  try {
    return await fn(scanAbort.signal);
  } catch (e) {
    log(e.name === 'AbortError' ? 'Scan abgebrochen' : `! Scan: ${e.message}`);
    return null;
  } finally {
    scanAbort = null;
    $('btn-scan').disabled = $('btn-explore').disabled = $('btn-home').disabled = false;
  }
}

async function doScan(steps, signal) {
  const simPose = robot.kind === 'sim' ? { x: robot.state.x, y: robot.state.y, heading: robot.state.heading } : null;
  nav.steps = steps;
  const result = await nav.scanHere({ signal });
  if (!result?.points) { log(`! Scan: ${result?.note ?? 'fehlgeschlagen'}`); return result; }
  if (simPose) { lastScan = { ...simPose, points: result.points }; redrawSim(); }
  showScan(result.points);
  log(`Scan: ${describeScan(result.points)}`);
  return result;
}

// --- map and navigation --------------------------------------------------------

const map = new GridMap({ cellCm: 5, sizeCm: 800 });
const tracker = new PoseTracker();
tracker.attach(bus); // turn/straight from any source (buttons, AI, navigator) update the pose
let mapView = null;
let mapScan = null; // { pose, points } of the last scan, drawn on the map

const nav = new Navigator({
  bus, map, pose: tracker, scan,
  useYaw: false,
  onEvent: (e) => {
    if (e.type === 'scan') mapScan = { pose: e.pose, points: e.points };
    if (e.type === 'leg') log(`Navigation: ${e.turnDeg ?? 0}° drehen, ${e.cm ?? '?'} cm fahren`);
    if (e.type === 'blocked') log(`Navigation: Hindernis${e.note ? ` (${e.note})` : ''}, neuer Plan`);
    if (e.type === 'arrived') log('Navigation: angekommen');
    if (e.type === 'error') log(`! Navigation: ${e.note ?? e.message ?? ''}`);
    redrawMap();
    redrawSim();
  },
});
window.mbot.map = map;
window.mbot.tracker = tracker;
window.mbot.nav = nav;

// Joystick and button driving stream wheel speeds; integrate them into the pose.
let lastDrive = null; // { l, r, at }
bus.onCommand((c, r) => {
  if (!r.ok) return;
  const now = performance.now();
  if (lastDrive) tracker.applyDrive(lastDrive.l, lastDrive.r, Math.min(0.3, (now - lastDrive.at) / 1000));
  lastDrive = c.cmd === 'drive' && (c.args.left || c.args.right) ? { l: c.args.left, r: c.args.right, at: now } : null;
  if (c.cmd === 'drive' || c.cmd === 'turn' || c.cmd === 'straight' || c.cmd === 'stop') scheduleMapDraw();
});

let mapDrawPending = false;
function scheduleMapDraw() {
  if (mapDrawPending) return;
  mapDrawPending = true;
  requestAnimationFrame(() => { mapDrawPending = false; redrawMap(); });
}

function redrawMap() {
  const canvas = $('map');
  if (!canvas || !$('panel-scan').open) return;
  const pose = tracker.pose;
  mapView = fitView(canvas, map, pose);
  drawMap(canvas, map, pose, {
    view: mapView,
    trail: tracker.trail,
    path: nav.lastPath,
    goal: nav.goal,
    frontiers: map.frontiers({ minCells: 4 }),
    lastScan: mapScan,
  });
  $('map-text').textContent = map.bounds ? map.describe(pose) : 'Karte: noch leer. Scannen füllt sie; auf die Karte tippen fährt dorthin.';
}
$('panel-scan').addEventListener('toggle', redrawMap);

function resetMap() {
  map.clear();
  tracker.reset();
  nav.lastPath = null;
  nav.goal = null;
  mapScan = null;
  lastDrive = null;
  redrawMap();
}
$('btn-map-clear').onclick = () => { resetMap(); log('Karte gelöscht; die aktuelle Position ist der neue Start.'); };
$('opt-yaw').onchange = () => { nav.useYaw = $('opt-yaw').checked; };

function navResult(label, r) {
  if (!r) return;
  log(`${label}: ${r.ok ? (r.reached === false ? 'nicht ganz erreicht' : 'fertig') : `abgebrochen${r.note ? ` (${r.note})` : ''}`}`);
  redrawMap();
  redrawSim();
}

$('map').addEventListener('pointerup', (e) => {
  if (!mapView) return;
  const goal = screenToWorld($('map'), mapView, e.clientX, e.clientY);
  goal.x = Math.round(goal.x); goal.y = Math.round(goal.y);
  log(`Ziel: ${goal.x} cm rechts, ${goal.y} cm vorne (vom Start)`);
  runScanTask(async (signal) => navResult('Fahrt zum Ziel', await nav.goTo(goal, { signal })));
});

$('btn-scan').onclick = () => runScanTask((signal) => doScan(Number($('scan-steps').value), signal));
$('btn-explore').onclick = () => runScanTask(async (signal) => {
  nav.steps = Number($('scan-steps').value);
  const r = await nav.explore({ signal, maxMoves: 6 });
  navResult(`Erkunden (${r?.moves ?? 0} Fahrten, ${r?.frontiersLeft ?? '?'} offene Bereiche)`, r);
});
$('btn-home').onclick = () => runScanTask(async (signal) => navResult('Nach Hause', await nav.goHome({ signal })));

checkSupport();
setSpeed(Number(store.get('speed', 60)));
setMode(store.get('mode', 'buttons'));
setVoiceMode(voiceMode);
log('Bereit. Roboter einschalten, Startbildschirm, kein Programm aktiv, nicht mit mBlock verbunden.');
