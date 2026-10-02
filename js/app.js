import { BleRobot } from './robot-ble.js';
import { SimRobot, SIM_ROOM } from './robot-sim.js';
import { CommandBus, makeCommand, LIMITS, LED_EFFECTS, EYE_EFFECTS } from './bus.js';
import { VoiceListener, parseUtterance } from './voice.js';
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
        debugAllDevices: $('opt-all').checked,
      });
    }
    robot = ble;
    bus.setRobot(robot);
    applyWheelSettings();
    await robot.connect();
    bus.submit(makeCommand('read', { sensor: 'battery' }));
  } catch (e) {
    log(`Verbindung fehlgeschlagen: ${e.message}`);
    setStatus(ble?.device ? 'disconnected' : 'idle');
  }
}

async function connectSim() {
  $('sim').hidden = false;
  robot = new SimRobot({ log, onStatus: setStatus, onChange: drawSim });
  bus.setRobot(robot);
  await robot.connect();
  drawSim(robot.state);
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
      if ($('opt-guard').checked && (driving || n % 4 === 0)) {
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
function setSpeed(v) {
  const s = Math.max(20, Math.min(LIMITS.maxDriveRpm, v));
  speedEl.value = s;
  bus.settings.speed = Math.min(s, LIMITS.maxSpeed);
  $('r-speed').textContent = s;
  store.set('speed', s);
}
speedEl.oninput = () => setSpeed(Number(speedEl.value));

function emergencyStop(src = 'ui') {
  stream.halt();
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

const joystick = new Joystick($('joystick'), {
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
    onCommand: (text) => runVoice(text),
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
  b.onclick = () => send('eyes_effect', { name });
  $('eye-effects').append(b);
}
for (let id = 1; id <= 8; id++) {
  const l = document.createElement('label');
  l.innerHTML = `LED ${id}<input type="range" min="0" max="100" value="0">`;
  l.querySelector('input').addEventListener('input', throttled((e) => send('eye_led', { id, bri: Number(e.target.value) })));
  $('eye-leds').append(l);
}

$('floor-light').onchange = () => send('floor_light', { color: $('floor-light').value });

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
    if (voice?.active) setMic(false);
  } else if (robot?.connected) {
    acquireWakeLock();
  }
});
window.addEventListener('pagehide', () => { if (robot?.connected) emergencyStop('ui'); });
window.addEventListener('blur', () => { if (stream.timer) emergencyStop('ui'); });

// --- simulator drawing -------------------------------------------------

function drawSim(s) {
  const c = $('sim');
  const ctx = c.getContext('2d');
  const k = c.width / SIM_ROOM.w;
  const css = getComputedStyle(document.documentElement);
  ctx.clearRect(0, 0, c.width, c.height);
  ctx.strokeStyle = css.getPropertyValue('--border');
  ctx.lineWidth = 4;
  ctx.strokeRect(2, 2, c.width - 4, c.height - 4);
  ctx.save();
  ctx.translate(s.x * k, s.y * k);
  ctx.rotate((s.heading * Math.PI) / 180);
  ctx.fillStyle = css.getPropertyValue('--accent');
  ctx.fillRect(-24, -18, 48, 36);
  // five back LEDs along the rear edge
  s.leds.forEach((rgb, i) => {
    ctx.fillStyle = `rgb(${rgb.join(',')})`;
    ctx.beginPath(); ctx.arc(-24, -14 + i * 7, 3, 0, Math.PI * 2); ctx.fill();
  });
  // eyes at the front
  s.eyes.forEach((bri, i) => {
    ctx.fillStyle = `rgba(80,160,255,${0.15 + (bri / 100) * 0.85})`;
    ctx.beginPath(); ctx.arc(26, i === 0 ? -8 : 8, 5, 0, Math.PI * 2); ctx.fill();
  });
  ctx.restore();
  if (s.label) {
    ctx.fillStyle = css.getPropertyValue('--text');
    ctx.font = '24px system-ui';
    ctx.fillText(s.label, 12, 32);
  }
}

checkSupport();
setSpeed(Number(store.get('speed', 80)));
setMode(store.get('mode', 'buttons'));
log('Bereit. Roboter einschalten, Startbildschirm, kein Programm aktiv, nicht mit mBlock verbunden.');
