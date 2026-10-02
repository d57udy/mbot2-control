# mBot2 / CyberPi BLE "Live Mode" protocol (for a Web Bluetooth app)

Research date: 2026-10-02. Goal: drive an mBot2 from a single-page JS app (GitHub Pages, Chrome on Android) over BLE, without uploading code, by speaking the same "Live Mode" protocol mBlock uses.

Legend: every claim carries a source. **UNVERIFIED** = inferred, or claimed by a source without hardware evidence I could inspect.

## Sources examined

| Tag | Source | What it is |
|---|---|---|
| **[DS]** | `github.com/DrorSh/mbot_python` @ `cb91440` (cloned locally, `repos/mbot_python`) | Python + bleak, MIT. Primary source. Docs say verified on a real mBot2, firmware 44.01.013 |
| **[RV]** | `github.com/Hulupeep/mbot_ruvector` @ `c90bb7f` (cloned, `repos/mbot_ruvector`) | Rust (btleplug) + many Python probe tools. Source DS credits. Licence `MIT OR Apache-2.0` (Cargo.toml:12) |
| **[MB]** | PyPI `makeblock` 0.1.8 (Makeblock's own package, downloaded from PyPI) | Origin of the f3/f4 "HalocodeProtocol". Talks serial/mLink, not BLE |
| **[BW]** | `github.com/binomed/mbot-webbluetooth` (2016, MIT) | Web Bluetooth app for the *classic* mBot's Makeblock BLE module |
| **[PC]** | primalcortex.wordpress.com/2018/07/05/makeblock-mbot-nodejs-ble/ | Classic mBot BLE module GATT notes |
| **[DC]** | github.com/dachrisch/mbot2-course-setup (README) | States CyberPi BT is BLE-only and mBlock web reaches it via Web Bluetooth |
| **[TG]** | github.com/tatiang/mbot-vr PR #6 | Web Bluetooth for classic mBot BLE modules; documents the "filters vs optionalServices" gotcha. Not tested on hardware |
| **[MK]** | support.makeblock.com articles (Program CyberPi with the mBlock App; Bluetooth troubleshooting) | Official: the mBlock phone app connects to CyberPi's built-in BLE directly; Live mode requires staying connected |

I also cross-checked byte-for-byte: a JS port of the frame builder (below) produces output identical to DS `build_script_frame`, and the JS parser correctly decodes a real captured reply split across two notifications (RV protocol.rs test vectors).

---

## 1. Advertised name, GATT service and characteristics

| Item | Value | Source |
|---|---|---|
| Advertised name | Starts with **`Makeblock_LE`** (RV logs `BLE connected to Makeblock_LE...`) | RV README.md:142; RV tools/cyberpi_ble_discover.py:8 ("The CyberPi advertises as "Makeblock_LE..." via CyberPiOS") |
| Name-match heuristics used by working code | substrings `makeblock`, `cyberpi`, `mbot`, `bluefi` (case-insensitive) | DS mbot2/connection.py:16, :35-39; RV transport.rs:507 (`Makeblock` / `CyberPi` / `mBot`) |
| Notify characteristic (robot to app) | **`0000ffe2-0000-1000-8000-00805f9b34fb`** | DS connection.py:15; RV transport.rs:64; RV tools/*.py |
| Write characteristic (app to robot) | **`0000ffe3-0000-1000-8000-00805f9b34fb`** | DS connection.py:14; RV transport.rs:65 |
| Service | **`0000ffe1-0000-1000-8000-00805f9b34fb`** (best evidence) | RV transport.rs:63 and RV docs/contracts/feature_ble_transport.yml ("Makeblock devices use ffe1 service, ffe2 notify, ffe3 write"); matches classic Makeblock BLE module [PC], [BW] scripts/mbot/mbot.js:12 |

**Is the service ffe1?** Probably yes, but **UNVERIFIED for CyberPi specifically**. Neither DS nor RV ever uses the service UUID: both look characteristics up directly (bleak / btleplug search all services). RV declares `SERVICE = ffe1` but never references it in the connect path (transport.rs:519-533). No GATT dump from a real CyberPi is committed in either repo (DS `tools/explore_ble.py` and RV `tools/cyberpi_ble_discover.py` print one, but the output was not saved). [DC] calls `0000ffe1` a "characteristic, HM-10-style", which conflicts (HM-10 uses service `ffe0` + char `ffe1`). Treat the service UUID as the #1 thing to confirm on day one.

This matters for Web Bluetooth because Chrome only exposes services listed in `filters[].services` or `optionalServices`. Recommended defensive request:

```js
const SVC_CANDIDATES = [
  '0000ffe1-0000-1000-8000-00805f9b34fb', // Makeblock (expected)
  '0000ffe0-0000-1000-8000-00805f9b34fb', // HM-10 style fallback
  '0000ffe5-0000-1000-8000-00805f9b34fb', // seen on some Makeblock modules (UNVERIFIED)
];
const device = await navigator.bluetooth.requestDevice({
  filters: [{ namePrefix: 'Makeblock' }],   // do NOT filter by services: Makeblock modules
                                            // may not advertise the service UUID ([TG])
  optionalServices: SVC_CANDIDATES,
});
const server = await device.gatt.connect();
// find whichever candidate service holds ffe2 + ffe3
let writeChar, notifyChar;
for (const uuid of SVC_CANDIDATES) {
  try {
    const svc = await server.getPrimaryService(uuid);
    writeChar  = await svc.getCharacteristic('0000ffe3-0000-1000-8000-00805f9b34fb');
    notifyChar = await svc.getCharacteristic('0000ffe2-0000-1000-8000-00805f9b34fb');
    break;
  } catch (_) {}
}
```

For a first hardware session, a debug build with `acceptAllDevices: true` plus the candidate list, and `server.getPrimaryServices()` to log what is actually visible, will settle this in minutes. (`namePrefix: 'Makeblock'` is the guess; `cyberpi.get_name()` / `set_name()` exist on the robot per DS docs/commands.md:203, so the name may be user-changeable. **UNVERIFIED** whether renaming changes the advertised name.)

---

## 2. Frame format ("f3/f4", Makeblock HalocodeProtocol)

Defined in DS mbot2/protocol.py:8-14 and 34-44; originates from MB `protocols/PackData.py` (`HalocodePackData.to_buffer`, lines ~197-222 of that file); same in RV protocol.rs:7-16, 33-78.

```
offset  field          value
0       header         0xF3
1       hdr_check      (0xF3 + datalen_lo + datalen_hi) & 0xFF
2       datalen_lo     datalen = len(data) + 4      (4 = type + mode + idx_lo + idx_hi)
3       datalen_hi
4       type           0x28 = SCRIPT   (0x29 = SUBSCRIBE push, 0x0D = ONLINE/mode switch)
5       mode           0x00 = run, no response; 0x01 = run, with response
                       (MB also defines 0x02 RESET, 0x03 "RUN_IMDT_WITH_RESPONSE")
6       idx_lo         16-bit request id, little-endian, echoed in the reply
7       idx_hi
8..     data           for SCRIPT: [script_len_lo, script_len_hi, <UTF-8 script bytes>]
n-2     checksum       (type + mode + idx_lo + idx_hi + sum(data)) & 0xFF
n-1     footer         0xF4
```

Total frame length = datalen + 6. Constants: DS protocol.py:23-28; MB PackData.py `HalocodePackData` class constants.

idx: DS starts at 1 and wraps `(idx % 0xFFFE) + 1` (connection.py:65-68); MB/RV wrap at 0xFFFF back to 1. idx 0 is avoided (MB `request()` treats idx==0 as "assign one").

### Worked example: `mbot2.forward(50,1)` (drive forward at speed 50 for 1 s, fire-and-forget, idx=1)

script = `mbot2.forward(50,1)` = 19 bytes (0x13). data = `13 00` + script = 21 bytes. datalen = 25 = 0x19. hdr_check = (F3+19+00)&FF = 0x0C. checksum = (28+00+01+00 + 13+00 + sum(script)) & FF = 0x56.

```
f3 0c 19 00 28 00 01 00 13 00 6d 62 6f 74 32 2e 66 6f 72 77 61 72 64 28 35 30 2c 31 29 56 f4
                                 m  b  o  t  2  .  f  o  r  w  a  r  d  (  5  0  ,  1  )
```
31 bytes. Sent as 20-byte chunks: `f3 0c 19 00 28 00 01 00 13 00 6d 62 6f 74 32 2e 66 6f 72 77` then `61 72 64 28 35 30 2c 31 29 56 f4`. (Generated with DS `build_script_frame`, reproduced identically by the JS below.)

Request with response, `cyberpi.get_battery()`, idx=2, mode=1:
```
f3 0e 1b 00 28 01 02 00 15 00 63 79 62 65 72 70 69 2e 67 65 74 5f 62 61 74 74 65 72 79 28 29 47 f4
```

### JS frame builder (tested in Node 22, byte-identical to DS)

```js
export function buildScriptFrame(script, idx, mode /* 0 = no reply, 1 = reply */) {
  const sb = new TextEncoder().encode(script);
  const data = new Uint8Array(2 + sb.length);
  data[0] = sb.length & 0xff; data[1] = (sb.length >> 8) & 0xff; data.set(sb, 2);
  const datalen = data.length + 4;
  const lenLo = datalen & 0xff, lenHi = (datalen >> 8) & 0xff;
  const body = [0x28, mode, idx & 0xff, (idx >> 8) & 0xff, ...data];
  const cksum = body.reduce((a, b) => a + b, 0) & 0xff;
  return Uint8Array.from([0xf3, (0xf3 + lenLo + lenHi) & 0xff, lenLo, lenHi, ...body, cksum, 0xf4]);
}
```

---

## 3. Handshake / mode switch

Sequence used by all three implementations (DS connection.py:41-54, 82-97; RV transport.rs:557-622; MB api_cyberpi_api.py `autoconnect()`):

1. Connect, enable notifications on ffe2 **before** writing anything.
2. Write the **"go online" (Live Mode) frame**, a fixed 9-byte frame:
   `f3 f6 03 00 0d 00 01 0e f4` (DS protocol.py:31; MB `HalocodePackData.broadcast()` and `goto_online_mode()`; RV protocol.rs:80-82).
   Decoding: datalen 3, type 0x0D, then `00 01`, checksum 0x0E. MB's `goto_offline_mode()` sends `... 0d 00 00 0e f4`, so the last data byte is the mode (1 = online/live, 0 = offline/upload). Note MB's offline frame keeps checksum 0x0E although the arithmetic gives 0x0D; **UNVERIFIED** whether the firmware validates checksums at all.
3. Wait ~500 ms (DS connection.py:84; RV transport.rs:578).
4. "Ping" until the robot answers: send `cyberpi.get_bri()` with mode=1, wait up to 500 ms for any parsed reply; DS retries 12x (connection.py:85-95), RV 10x, MB loops every 0.3 s until `protocol.ready`. On success reset idx to 1.
5. DS proceeds even without an ack: "No ack, but the robot often still accepts commands" (connection.py:96).

Robot state requirements:
- Powered on, **on the home screen, no uploaded program running**; if a program runs, press Home first (RV README.md:121-122, :148). **UNVERIFIED** what happens if a user program is running (likely the live commands compete with it or are ignored).
- **Not connected to a phone or mBlock** (DS README.md:26, connection.py:47-49). CyberPi BLE accepts one central at a time (**UNVERIFIED**, inferred from these warnings).
- No pairing/bonding required: bleak and btleplug connect without pairing in both repos; no pairing step documented anywhere.

Other frames seen in RV probing (not needed): `f3 f5 02 00 08 c0 c8 f4` labelled "f5 handshake" (RV tools/cyberpi_ble_handshake.py:52; origin undocumented). Plain-text REPL pokes (Ctrl-C, `print("hello")`) over ffe3 do not get a REPL (DS tools/probe_ble.py purpose; RV textmode probes). The pipe is binary-framed only.

---

## 4. Payload: Python evaluated on the robot (MicroPython, CyberPiOS)

The script string is evaluated by the robot's MicroPython; with mode=1 the result is returned as JSON `{"ret": <value>}` (DS protocol.py:4-6, 14; MB modules.py `common_request_response_cb`). Both top-level `mbot2` and `cyberpi` objects are in scope (DS uses bare `mbot2.forward`, verified on 44.01.013; MB's generated API uses `cyberpi.mbot2.forward`, table_cyberpi_api.py:354). Stick to **single expressions**; for statements/loops wrap them in `exec("...")` as RV does (protocol.rs:225-238). **UNVERIFIED** whether bare statements (`a=1`) are accepted.

Commands (DS = verified by DS on firmware 44.01.013 unless marked):

| Purpose | Script | Source |
|---|---|---|
| Forward / backward | `mbot2.forward(speed[, secs])`, `mbot2.backward(speed[, secs])` (speed in RPM; with secs it auto-stops on the robot) | DS robot.py:92-100, docs/commands.md:30-31,108 |
| Spin | `mbot2.turn_left(speed[, secs])`, `mbot2.turn_right(speed[, secs])` | DS robot.py:99-100 |
| Gyro turn | `mbot2.turn(degrees)` (+ right, - left); MB signature `turn(angle, speed=50)` | DS robot.py:102-105; MB api_cyberpi_api.py `mbot2_c.turn` |
| Exact distance | `mbot2.straight(cm)` (MB: `straight(distance, speed=50)`) | DS commands.md:87,110 |
| Per-wheel | `mbot2.drive_speed(l, r)` (RPM), `mbot2.drive_power(l, r)` | DS robot.py:107-109. **Conflict:** RV protocol.rs:150-152 says drive_speed "has inverted semantics (same sign = spin)", i.e. EM1/EM2 are mirror-mounted; DS's line-follow example assumes same sign = forward. Test on hardware; expect to negate one side |
| Stop | `mbot2.EM_stop()` (MB default `port="all"`) | DS robot.py:111-112 |
| Battery % | `cyberpi.get_battery()` | DS robot.py:155-157; RV protocol.rs:109-111 |
| Ultrasonic cm | `cyberpi.ultrasonic2.get(1)` (reads 300 when clear; 13.4 measured at ~13 cm) | DS robot.py:151-153, docs/sensors.md:49-51. RV uses `mbot2.ultrasonic2.get(1)`, which DS commit a2077db says is the wrong namespace |
| Line follower | `cyberpi.quad_rgb_sensor.get_line_sta()` (0..15 bitmask), `.get_offset_track()` (-100..100), `.is_line(i)`, `.get_gray(i)` for i in 1..4 | DS docs/sensors.md:180-195 (stated verified) |
| RGB LEDs | `cyberpi.led.on(r,g,b)`, `cyberpi.led.on(r,g,b,id=1..5)`, `cyberpi.led.off()`, `cyberpi.led.play(name='rainbow')` | DS robot.py:115-128 |
| Ultrasonic "eye" LEDs | `cyberpi.ultrasonic2.happy_effect()` etc.; `set_both_led_bri(a,b)` then `led_show()` | DS docs/sensors.md:93-125 |
| Display | `cyberpi.display.show_label('text',24,'center')`; `cyberpi.console.println('text')`; `cyberpi.display.clear()` | DS robot.py:146-148; RV protocol.rs:198-199; DS commands.md:152-155 |
| Speaker | `cyberpi.audio.play_tone(freq, secs)`, `cyberpi.audio.play_music(note=60, beat=0.5)`, `cyberpi.audio.set_vol(0..100)`, `cyberpi.audio.play('name')` | DS robot.py:131-143, commands.md:139-147 |
| Other sensors | `cyberpi.get_bri()`, `cyberpi.get_loudness('maximum')`, `cyberpi.get_roll()/get_pitch()/get_yaw()`, `cyberpi.get_acc('z')`, `cyberpi.controller.is_press('a')` | DS robot.py:159-173 |
| Housekeeping | `cyberpi.get_firmware_version()`, `dir(mbot2)` for introspection | DS tools/introspect.py; RV test vector shows `"44.01.009"` |

Escape text you interpolate (quotes, backslashes) since it becomes Python source; DS does not (robot.py:148), RV escapes `'` (protocol.rs:199).

Sensor streaming alternative: MB registers server-side subscriptions with `subscribe.add_item(key, func, paras)` and receives pushed TYPE 0x29 frames (MB modules.py `create_subscribe_str`, Protocols.py `HalocodeProtocol.on_subscribe_response`). **UNVERIFIED** on mBot2 over BLE; polling with mode=1 is the proven path.

---

## 5. Responses on ffe2

- Replies arrive as BLE notifications on ffe2, in the same f3/f4 frame layout, type 0x28, mode byte 0x01, **idx echoed from the request**, data = `[len_lo, len_hi, JSON bytes]`, checksum computed the same way as requests (verified: recomputing the checksum of RV's captured frame gives 0x61 as stored).
- Real capture (RV protocol.rs:608-614, "brightness = 75"):
  `f3 03 10 00 28 01 01 00 0a 00 7b 22 72 65 74 22 3a 37 35 7d 61 f4` = idx 1, `{"ret":75}`.
  Others: `{"ret":-9.6}` (accel), `{"ret":"44.01.009"}` (firmware) (RV protocol.rs:630-655).
- **Reassembly**: frames span notifications; parse as a byte stream. The algorithm (DS protocol.py:79-121, identical to MB Protocols.py `HalocodeProtocol.on_parse` and RV protocol.rs:325-370): append each byte; whenever the last 4 bytes are `F3, c, lo, hi` with `(F3+lo+hi)&FF == c`, restart the buffer at that header and set expected length `lo|hi<<8`; when buffer length == datalen+6, emit the frame. Cap buffer at 4096, trim to 4 bytes while hunting.
- **Matching**: by idx (DS connection.py:70-75 keeps `idx -> Future`; MB `check_response` compares idx; RV loops until `resp.idx == idx`).
- **JSON**: parse `data[2:]` as JSON and take `.ret`; DS falls back to `ast.literal_eval` because MicroPython may emit Python-style literals (e.g. single quotes, `True`) (DS protocol.py:60-76; MB uses Python `eval`). In JS: try `JSON.parse`, then a lenient fallback (replace `'` with `"`, `True/False/None` with `true/false/null`). `dir(...)` returns a list (DS introspect.py:25).
- Fire-and-forget (mode 0) produces no reply (DS uses no future for `run`). **UNVERIFIED** whether errors in a mode-0 script are reported anywhere.

JS parser (tested on a reply split at byte 20):

```js
export class F3Parser {
  constructor() { this.buf = []; this.rx = false; this.len = 0; }
  feed(bytes) { const out = []; for (const b of bytes) { const r = this.byte(b); if (r) out.push(r); } return out; }
  byte(b) {
    this.buf.push(b);
    const n = this.buf.length;
    if (n > 3) {
      const [h, c, lo, hi] = this.buf.slice(n - 4);
      if (h === 0xf3 && ((h + lo + hi) & 0xff) === c) { this.buf = [h, c, lo, hi]; this.len = lo | (hi << 8); this.rx = true; }
    }
    if (this.rx) {
      if (this.buf.length === this.len + 6) {
        const f = this.buf; this.buf = []; this.rx = false;
        if (f[4] !== 0x28 || f.length < 10) return null;
        const idx = f[6] | (f[7] << 8);
        const json = new TextDecoder().decode(Uint8Array.from(f.slice(10, f.length - 2)));
        let value; try { value = JSON.parse(json).ret; } catch { value = json; }
        return { idx, value, raw: json };
      }
      if (this.buf.length > 4096) { this.buf = []; this.rx = false; }
    } else if (this.buf.length > 64) this.buf = this.buf.slice(-4);
    return null;
  }
}
// notifyChar.addEventListener('characteristicvaluechanged', e =>
//   parser.feed(new Uint8Array(e.target.value.buffer)).forEach(resolvePendingByIdx));
```

Caveat: header detection can false-trigger on payload bytes; harmless for ASCII JSON (all bytes < 0x80) but non-ASCII strings could in theory confuse it.

---

## 6. Write constraints

| Topic | Finding | Source |
|---|---|---|
| Chunk size | DS writes in **20-byte** chunks ("ffe-module size") | DS connection.py:18, 77-80 |
| Pacing | **8 ms** sleep between chunks | DS connection.py:19, 80 |
| Write type | **Write Without Response** | DS connection.py:79 (`response=False`); RV transport.rs:573, 594, 642 (`WriteType::WithoutResponse`) |
| Larger writes | RV writes whole frames unchunked (btleplug on Linux, which negotiates a larger MTU), and it worked for them | RV transport.rs:593-596, 642-643, 699-700. Suggests the robot supports MTU > 23 (**UNVERIFIED**) |
| Between queries | RV waits 30 ms between sensor queries, 500-1000 ms reply timeout | RV transport.rs:404-405 (serial), 717-718 (BLE) (INTER_QUERY_MS / TIMEOUT_MS) |
| makeblock pkg | 1 ms delay before each request | MB modules.py `GET_VALUE_DELAY_TIME`/`REQUEST_DELAY_TIME` |

Web Bluetooth specifics (general Chrome behaviour, not mBot-specific): use `characteristic.writeValueWithoutResponse(chunk)`; serialize all GATT operations through one promise queue (Chrome rejects concurrent ops with "GATT operation already in progress"); Chrome does not expose the negotiated MTU, so keep 20-byte chunks to be safe. A queue of `for chunk of 20B: await writeValueWithoutResponse(chunk); await sleep(8)` mirrors DS exactly.

Throughput / latency: **no measured numbers in any source.** Back-of-envelope with DS pacing: a typical 30-40 byte command = 2 chunks ≈ 10-20 ms to send; reply round trip budgets are 500 ms (handshake) to 3 s (DS `eval` default timeout, connection.py:104). **UNVERIFIED** real latency; measure it.

Sending too fast: **no source documents it.** Risks (UNVERIFIED): BLE write-without-response packets dropped by the Android stack under load, and blocking robot calls (`mbot2.forward(50,1)`, `mbot2.turn(90)`, `play_tone`) probably occupy the robot's executor, so later commands may queue or be dropped. DS works around blocking calls by sleeping client-side for the move duration (robot.py:94-95, 105, 134). For joystick-style driving, send non-blocking `mbot2.forward(speed)` / `drive_speed` at a modest rate (e.g. on change, max ~10-20 Hz) rather than flooding.

---

## 7. Failure modes and risks

- **Firmware**: DS verified on **44.01.013** (docs/commands.md:9, sensors.md:49); RV captured **44.01.009** (protocol.rs:648-655). Protocol is Makeblock-private and undocumented; a CyberPiOS update could change it (**UNVERIFIED**, no evidence of breakage so far; MB package itself dates from the Halocode era, so the protocol has been stable across products).
- **API namespace drift**: ultrasonic moved/misdocumented (`mbot2.ultrasonic2` in RV vs `cyberpi.ultrasonic2` in DS, DS commit a2077db). Use `dir()` introspection to confirm on the target robot.
- **Built-in BLE vs dongle**: DS and RV both use the **CyberPi's built-in BLE** ("no dongle", DS README.md:4). The Makeblock USB Bluetooth adaptor is a separate path for desktop mBlock that pairs to the robot itself [MK]; if the dongle is plugged in and paired, the robot's BLE link is likely taken (**UNVERIFIED**). The official mBlock phone app connects directly to CyberPi BLE [MK], which is the same path Chrome on Android will use.
- **Exclusive connection**: must not be connected to phone/mBlock (DS README.md:26). Close the mBlock app on the phone; Android may also auto-connect from system Bluetooth settings (**UNVERIFIED**).
- **Handshake may never ack**: DS proceeds anyway (connection.py:96); RV warns and proceeds (transport.rs:620-622).
- **Disconnect safety**: **no source states that motors stop on BLE disconnect.** DS explicitly designs around it: "pass `secs` ... the robot stops **itself** after that time, safe even if a Bluetooth packet drops. Leave `secs` off and it keeps going until `stop()`" (README.md:60-61; robot.py:90-91). Treat as: continuous `forward(speed)` keeps running if the link dies. Mitigations: prefer short timed moves re-issued while a button is held (DS keyboard_drive.py uses 0.25 s bursts, :49-50, "can't run away if you let go"), send `mbot2.EM_stop()` on `visibilitychange`/`pagehide`/`gattserverdisconnected` (the last one cannot reach the robot, so the timed-burst pattern is the real safety net). **UNVERIFIED** whether CyberPiOS has its own live-mode watchdog; test by killing the connection mid-drive.
- **Pairing**: none required by any implementation.
- **drive_speed sign convention** conflict (section 4).
- **Service UUID** not directly confirmed (section 1).

## 8. Licences

- DrorSh/mbot_python: **MIT**, "Copyright (c) 2026 DrorSh" (LICENSE:1-3; pyproject.toml:13).
- Hulupeep/mbot_ruvector: **MIT OR Apache-2.0** (Cargo.toml:12; no LICENSE file in repo root).
- binomed/mbot-webbluetooth: MIT (LICENSE).
- makeblock PyPI package: licence not checked; the protocol facts (not code) are what we reuse.

The JS snippets above are a fresh port of a tiny algorithm; keeping an MIT attribution to DrorSh/mbot_python in the app is the simple, safe choice.

## Open questions to settle on first hardware session

1. Actual primary service UUID holding ffe2/ffe3 (log `getPrimaryServices()`), and exact advertised name.
2. Does Chrome on Android need 20-byte chunks, or does a full frame in one `writeValueWithoutResponse` work?
3. Round-trip latency of `cyberpi.get_battery()` and max sustainable command rate.
4. Do motors keep running after disconnect? Does a blocking `mbot2.forward(50,2)` delay subsequent commands?
5. `drive_speed(l, r)` sign convention.
