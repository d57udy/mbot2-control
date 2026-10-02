# Smooth joystick driving with a robot-side dead-man watchdog

Research date: 2026-10-02. Question: why the current hold-to-drive burst loop (`mbot2.forward(50,0.4)` every ~320 ms) is janky, and how to build an RC-car style virtual joystick (throttle + steering, 10-20 Hz) that still stops the robot on its own when the BLE link drops or the page is hidden.

Legend: **UNVERIFIED** = inferred or not backed by a source I could inspect. "Hardware test" items are the cheapest way to settle them.

## Sources

| Tag | Source | Notes |
|---|---|---|
| **[FW]** | `PerfecXX/mBot2` repo (cloned locally, `repos/perfecxx`, commit `18111ba`), file `firmware/CyberPi_firmware_44_01_011-ht2.bin`, a full CyberPiOS **44.01.011** image (sha256 prefix `150b5805b13f8380`). I ran `strings -n 6` on it and also located byte offsets with `grep -boa`. It contains the MicroPython qstr table (names of every module, function and constant compiled into the firmware) and plain-text factory demo scripts written by Makeblock | Same 44.01.x line as the target robot. Qstrs prove that a name exists, not how it behaves |
| **[PX]** | `PerfecXX/mBot2` example code (MIT, Teeraphat Kullanankanjana), same clone | Community, MicroPython running on the robot |
| **[DS]** | `DrorSh/mbot_python` @ `cb91440` | See 01-ble-protocol.md |
| **[RV]** | `Hulupeep/mbot_ruvector` @ `c90bb7f` | See 01-ble-protocol.md |
| **[MB]** | PyPI `makeblock` 0.1.8 (local copy `mkpkg/makeblock-0.1.8`) | Makeblock's own host-side library |
| **[BB]** | Barry Butler, "MakeBlock mBot2 / CyberPi Python Code" v2.2 (Feb 2022), hosted on Makeblock's CDN: `d4iqe7beda780.cloudfront.net/resources/static/main/pdf/mb10132_python_instruction_manual.pdf` | Teacher-written, distributed by Makeblock |
| **[API]** | Makeblock "Python API Documentation for CyberPi" (yuque.com/makeblock-help-center-en/mcode/cyberpi-api, PDF snapshot at lbotics.at) | Code samples are images; only prose survived text extraction |
| **[MOT]** | makeblock.com product page "180 Optical Encoder Motor for mBot2" (via search snippet) | Motor specs |

---

## 1. Why the burst approach is janky

Three effects stack up. The first two are well supported by evidence; the third follows from them.

### 1a. Each timed call ramps up and ramps down

The frozen `mbot2.py` module in [FW] contains the qstrs `car_spd_mode_forward`, `run_time`, `accel_time`, `decel_time`, `car_spd_mode_backward`, `car_spd_mode_turn_left`, `car_spd_mode_turn_right`, `car_spd_mode_apiece`, `encoder_motor_set_speed`, `EM1_speed`, `EM2_speed`, `__DEFAULT_SPEED`, `__DEFAULT_RUN_TIMS` (strings around byte offset 1227676, `car_spd_mode_forward`; `accel_time` at 1227712). The shield-side "car speed mode" for forward/backward/turn therefore takes an acceleration time and a deceleration time as well as the run time. **Inference (UNVERIFIED, hardware test):** every `forward(50,0.4)` accelerates from standstill, cruises briefly, then decelerates to zero at the end of `t`. A 0.4 s burst is short enough that a large part of it is ramp, which feels like pulsing.

Does the end of `t` brake hard? The `decel_time` parameter suggests a ramped stop rather than a hard brake. What `EM_stop()` does (brake vs coast) is **UNVERIFIED**; the firmware calls it `encoder_motor_stop` ([FW] qstr list next to `mbot2.py`).

### 1b. Timed calls block the robot's single Live Mode executor

Evidence that Live Mode scripts run one at a time from a queue:

- [FW] `common_protocol/common_online.py` qstrs (offsets ~1192500-1192700): `script_list`, `SCRIPT_LIST_MAX_LEN`, `script_list_ope_lock`, `exec_sync_sema`, `script_exec_process`, `script_exec_run`, `ONLINE_EXECUTE_THREAD_STACK_SIZE`, `ONLINE_EXECUTE_THREAD_PRIORITY`, plus `EVAL_INDEX`, `EXEC_INDEX`, **`EVAL_IMMEDIATLY_INDEX`**, **`EXEC_IMMEDIATLY_INDEX`**, `EXEC_BY_EVAL`, `EXEC_BY_EXEC`, `ONLINE_SERVICE_EXEC_NO_RESPOND`, `ONLINE_SERVICE_EXEC_WITH_RESPOND`, `ONLINE_SERVICE_RESET`. That reads as: incoming scripts go into a bounded list, a dedicated executor thread runs them in order, and there is an "immediately" path that bypasses the list.
- [MB] uses the immediate path for exactly one call: `cyberpi.audio.stop()` is sent with `w_mode=3` (`modules/cyberpi/api_cyberpi_api.py:219-221`), where mode 3 is `TYPE_RUN_IMDT_WITE_RESPONSE` (`protocols/PackData.py:157`). Every other call, including all `mbot2.*` motion, uses mode 0/1 and waits up to 30 s for completion (`api_cyberpi_api.py:1146-1170`, `modules.py:167-200`). The only reason to need an "immediate" mode for `audio.stop` is that a blocking `play_*` call is occupying the normal executor.
- [API] states for several calls "This API blocks the thread until the playing ends" (api.txt lines 160, 314, 550, 859).
- [DS] sleeps client-side for the duration of every timed move (`mbot2/robot.py:90-95`), and its docs say the call "waits that long".

**Conclusion (strong inference, UNVERIFIED on hardware):** `mbot2.forward(50,0.4)` occupies the executor for at least 0.4 s (plus any decel), and later frames wait in `script_list`.

### 1c. The current loop overruns the robot and builds a backlog

`app.js:139-153` sends a 0.4 s burst, then sleeps `secs * 800` ms = 320 ms. The `await bus.submit(...)` resolves when the BLE write finishes, not when the robot finishes (mode 0 has no reply). So the page issues 0.4 s of work every ~0.33 s (320 ms + ~16 ms of chunk pacing). Each second of holding adds roughly 0.2 s of queued motion on the robot (more if decel time is not included in `t`). Consequences:

- Between bursts the robot decelerates to 0 and re-accelerates (1a), so motion pulses.
- After release the robot keeps moving through the backlog.
- `mbot2.EM_stop()` is sent with mode 0 (`robot-ble.js:186`, `run()` uses `MODE_NO_REPLY`), so it queues **behind** the backlog on the robot. The client-side `urgent` flag only drops frames not yet written; it cannot reach frames already in `script_list`. The stop button can therefore lag by up to the backlog length. **UNVERIFIED**, but this matches the "janky" report and is cheap to test (hold forward 10 s, release, time how long it keeps going).
- If `SCRIPT_LIST_MAX_LEN` is hit, frames are presumably dropped (which ones is **UNVERIFIED**).

**Immediate fix regardless of the joystick:** send `EM_stop` with mode byte `0x03` (immediate, with response) as Makeblock does for `audio.stop`, and keep mode 0 as a second copy in case mode 3 is not handled for scripts on this firmware.

---

## 2. Non-blocking differential drive calls

### 2a. Signatures (authoritative: [MB] `api_cyberpi_api.py:1145-1197`, names confirmed in [FW] qstrs)

| Call | Meaning | Units / range | Blocking? |
|---|---|---|---|
| `mbot2.forward(speed=50, t="null")`, `backward`, `turn_left`, `turn_right` | Whole-chassis speed mode with accel/decel ramps | RPM. [PX] labels `forward(60,1)` "60 RMP" (`example/micropython/mBot2/01-mBot2 Chassis/01-basic_movement.py`); [BB] says 0..100, negative reverses (`forward(speed=-50)`), manual.txt:273-289 | With `t`: blocks for `t` (section 1b). Without `t`: returns at once and runs until stopped ([BB] "forever", manual.txt:277-284) |
| `mbot2.drive_speed(EM1_speed, EM2_speed)` | Closed-loop speed per encoder motor | RPM. [PX] comments "target RPMs ... The mBot2's built-in encoder motors will work to maintain these speeds" (`project/01-mBot2_bluetooth_control/01-basic_movement.py:36-39`) | Non-blocking, runs until changed ([DS] `robot.py:107-109`, [PX] `04-encoder_speed.py` uses `drive_speed` then `sleep(1)`) |
| `mbot2.drive_power(EM1_power, EM2_power)` | Open-loop PWM per motor | -100..100 ([BB] manual.txt:273-274; motor_set "power is -100 to 100" manual.txt:716) | Non-blocking ([BB] manual.txt:307-309 uses `drive_power` + `time.sleep(2)` + `EM_stop`) |
| `mbot2.EM_set_speed(speed, port)` | Closed-loop speed, one motor | RPM, port `"EM1"`/`"EM2"`/`"all"` (port strings **UNVERIFIED**; `EM_stop("ALL")` appears in [FW] factory script, offset 5600584) | Non-blocking (**UNVERIFIED**) |
| `mbot2.EM_set_power(power, port)` | Open-loop, one motor | -100..100 | Non-blocking (**UNVERIFIED**) |
| `mbot2.EM_stop(port="all")` | Stop motors | | Non-blocking |

Speed limits: the mBot2 motor is rated ~178 RPM under load, 350 RPM no-load, and the chassis API accepts 1-200 RPM [MOT] (**UNVERIFIED**, search snippet only). [BB] notes "Movement still occurs at speeds close to zero" (manual.txt:274).

`forward(speed)` without `t` is non-blocking but only drives straight, and [RV] had to approximate arcs with `turn_left/turn_right` (RV `protocol.rs:153-178`), which is not true proportional steering. For a joystick, use a per-wheel call.

### 2b. Sign convention verdict: **the motors are mirrored; forward is `(+v, -v)`**

| Evidence | Says |
|---|---|
| **Makeblock's own factory line-follower, embedded in firmware 44.01.011** ([FW] offset 5743799): `left_power = (30 - 0.6 * offset)`; `right_power = -1 * ((30 + 0.6 * offset))`; `mbot2.drive_power(left_power, right_power)` | Forward = EM1 positive, EM2 negative |
| [BB] manual.txt:584 `drive_power(50, -50)  #forward`, :590 `drive_power(20, -20)  #straight ahead`, :307 `drive_power(60, -40)  #left +, right -` (gradual turn) | Same |
| [PX] joystick drive: `v_left = Ly + Rx` (line 24), `v_right = Rx - Ly` (line 34), `mbot2.drive_speed(v_left, v_right)` (line 39) | Same for `drive_speed`: pure throttle gives `(+Ly, -Ly)` |
| [PX] `05-encoder_power.py:9` `drive_power(100, -100)` as the demo move | Same |
| [RV] `protocol.rs:150-152` "drive_speed() has inverted semantics (same sign = spin)"; RV `README.md:765-766` "Robot goes in a circle instead of straight ... fixed by switching from drive_speed() to mbot2.forward()" | Same, from a hardware bug report |
| [RV] `transport.rs:1088` labels `drive_speed(30, 30)` "forward" in a diagnostics routine | Contradicts, but the same repo's README documents that this produced circles; treat as a stale label |
| [DS] `docs/sensors.md:205-213` line-follow example uses `drive(25+k, 25-k)` (same sign = forward) | Contradicts; this is a doc example, not marked as hardware-verified (DS only claims verification for sensor reads) |

Verdict: `drive_speed(l, r)` and `drive_power(l, r)` are **raw per-motor** commands with no internal negation, and the two motors are mounted mirrored. Forward is `(+v, -v)`, spin in place is same sign. The DS example is wrong.

Still open: **which port is the left wheel.** [PX] comments imply EM1 = left (pushing the right stick right adds to `v_left`, turning right), and the Makeblock line follower is consistent with that but depends on the sign of `get_offset_track`. **UNVERIFIED**; one hardware test settles it: `mbot2.drive_speed(30,0)` for 0.5 s and see which wheel turns, and whether it turns forward. Put a `swapSides` and `invertDir` toggle in settings.

### 2c. Which call is smoothest

**`drive_speed`** (closed-loop RPM). Reasons: the encoder loop holds speed against battery sag and load, so the stick position maps to the same ground speed every time and both wheels track equally (straight lines without trim); it is non-blocking; and it does not go through the forward/turn ramp profile, so the client controls the feel. `drive_power` is the fallback if `drive_speed`'s PID turns out to hunt or lag at low RPM (**UNVERIFIED** either way; compare both on the floor). Do not use timed `forward/turn_*` for continuous control.

Whether `drive_speed` applies its own acceleration ramp is **UNVERIFIED** (`car_spd_mode_apiece` exists and may be what it uses; no evidence of an accel parameter for it). Client-side slew limiting (section 5) makes this irrelevant.

---

## 3. Robot-side watchdog (dead-man switch)

### 3a. What the firmware offers ([FW] qstrs, 44.01.011, MicroPython banner "MicroPython 44.01.008-13-g6cc07873-dirty on 2022-09-21; ESP32 module with ESP32")

| Facility | Evidence | Status |
|---|---|---|
| `_thread` with `start_new_thread`, `allocate_lock`, `get_ident`, `stack_size` | qstrs `_thread`, `start_new_thread` (offset 1096093), `allocate_lock`, `get_ident`, `stack_size`, plus `mp_thread` | Present in firmware. Behaviour from Live Mode **UNVERIFIED** |
| `time.ticks_ms`, `ticks_diff`, `ticks_add`, `sleep_ms` | qstrs (offset 1097735 for `ticks_ms`) | Present |
| `machine` / `umachine`, `machine.Timer` with `ONE_SHOT`, `PERIODIC` | qstrs `machine`, `umachine`, `ONE_SHOT`, `PERIODIC`, repr string `Timer(%p; ` | Present. Which hardware timer ids are free is **UNVERIFIED** (CyberPiOS may use some) |
| `micropython.schedule` | qstr `schedule` | Present |
| CyberPi script manager: `add_thread`, `remove_thread_by_id`, `stop_thread`, `stop_other_script`, `stop_this_script`, `stop_all_script`, `cyberpi.stop_other()` | qstrs; `cyberpi.stop_other()` used in the factory script ([FW] offset 5600554) | Firmware tracks its own threads; whether it kills a raw `_thread` thread on Live Mode reset is **UNVERIFIED** |
| CyberPi event API with a timer comparison (`greater_than(threshold, 'timer')`), `get_timer`/`reset_timer` | [API] events section (api.txt ~2560-2590 lists "timer" as a `greater_than` source); qstrs `greater_than`, `get_timer`, `reset_timer` | Exact Python spelling **UNVERIFIED** (code samples were images); check with `dir(cyberpi)` and `dir(cyberpi.event)` |
| Firmware link supervision: `online_connect_check`, `GET_ONLINE_CONNECT_STA_PROTOCOL_ID`, `ONLINE_CONNECT_STA_REPLY_WAIT_TIME`, `online_to_menu_check`, `stop_all`, `online_restart` | qstrs (offset 1194980) | CyberPiOS has *some* connection check in Live Mode. Whether it stops motors on link loss is **UNVERIFIED**; test first (drive with `drive_speed(40,-40)`, power off the phone's Bluetooth, time it) |
| Multi-statement code via `exec("...")` | [RV] `protocol.rs:226-238` sends `exec("for c in '...': cyberpi.audio.play_tone(...) ...")` and it plays; [FW] `EXEC_BY_EXEC` | Single-line `exec` proven by RV; multi-line string with `\n` escapes **UNVERIFIED** but is plain Python |

No source (DS, RV, PX, forums) shows a thread being started over Live Mode. Feasibility is therefore **plausible but UNVERIFIED**; the script below is designed to be checked in three queries.

### 3b. Candidate install script

Design choices:
- State is stored as attributes on the `mbot2` **module** (`mbot2.py` is a frozen Python module per [FW] qstr `mbot2.py`, so its namespace is writable). This survives even if each Live Mode frame is evaluated in a fresh globals dict (**UNVERIFIED** whether it is).
- The drive function itself refreshes the heartbeat, so every drive frame is also a keepalive; no extra traffic.
- The watchdog only acts while moving (`_mv`), so an idle robot never spams `EM_stop`.
- Left/right are forward-positive at the API boundary; the mirror flip happens on the robot.
- Idempotent: re-sending it after a reconnect does not start a second thread; it only refreshes the functions and timeout.
- Optional robot-side obstacle clamp (reads the ultrasonic sensor in the watchdog thread at ~10 Hz), so the guard does not depend on BLE polling.

Readable form:

```python
import _thread, time, mbot2 as M, cyberpi as C
M._wto = 400                 # ms without a drive frame before stopping
M._stop_cm = 0               # 0 = obstacle clamp off; e.g. 15 to enable
M._t = time.ticks_ms(); M._mv = 0; M._fwd = 0; M._cm = 300
def _d(l, r):                # l, r: forward-positive RPM, left and right wheel
    M._t = time.ticks_ms()
    M._fwd = l + r > 0
    if M._stop_cm and M._fwd and M._cm < M._stop_cm:
        l = r = 0
    M._mv = 1 if (l or r) else 0
    M.drive_speed(l, -r)     # EM1 = left assumed; swap/negate here if the hardware test says otherwise
def _wd():
    n = 0
    while M._wr:
        try:
            if M._mv and time.ticks_diff(time.ticks_ms(), M._t) > M._wto:
                M._mv = 0
                M.EM_stop()
            if M._stop_cm:
                n += 1
                if n % 2 == 0:
                    M._cm = C.ultrasonic2.get(1)
                    if M._mv and M._fwd and M._cm < M._stop_cm:
                        M._mv = 0
                        M.EM_stop()
        except Exception:
            pass
        time.sleep_ms(50)
M._d = _d
if not getattr(M, '_wr', 0):
    M._wr = 1
    _thread.start_new_thread(_wd, ())
```

Sent as one Live Mode frame (mode 1 so we can see an error reply; the JS `py()` helper, `JSON.stringify`, produces a valid Python string literal):

```js
const WD_SRC = [
  "import _thread,time,mbot2 as M,cyberpi as C",
  "M._wto=400",
  "M._stop_cm=0",
  "M._t=time.ticks_ms();M._mv=0;M._fwd=0;M._cm=300",
  "def _d(l,r):",
  " M._t=time.ticks_ms()",
  " M._fwd=l+r>0",
  " if M._stop_cm and M._fwd and M._cm<M._stop_cm:",
  "  l=r=0",
  " M._mv=1 if (l or r) else 0",
  " M.drive_speed(l,-r)",
  "def _wd():",
  " n=0",
  " while M._wr:",
  "  try:",
  "   if M._mv and time.ticks_diff(time.ticks_ms(),M._t)>M._wto:",
  "    M._mv=0;M.EM_stop()",
  "   if M._stop_cm:",
  "    n+=1",
  "    if n%2==0:",
  "     M._cm=C.ultrasonic2.get(1)",
  "     if M._mv and M._fwd and M._cm<M._stop_cm:",
  "      M._mv=0;M.EM_stop()",
  "  except Exception:",
  "   pass",
  "  time.sleep_ms(50)",
  "M._d=_d",
  "if not getattr(M,'_wr',0):",
  " M._wr=1",
  " _thread.start_new_thread(_wd,())",
].join("\n");
await robot.query(`exec(${JSON.stringify(WD_SRC)})`, 3000);   // ~700 bytes, ~36 chunks, ~0.3 s
```

If a ~700-byte script is rejected (max script length is **UNVERIFIED**), send it as 3-4 smaller `exec(...)` frames (state lines, `def _d`, `def _wd`, start), since all state lives on `M`.

Verification queries (mode 1), in order:
1. `__import__('_thread').get_ident()` returns an int: `_thread` usable from Live Mode.
2. After install: `mbot2._wr` returns 1 and `hasattr(mbot2,'_d')` returns True. If `mbot2` in Live Mode scope is not the module, fall back to `__import__('mbot2')._d(l,r)` as the drive expression (longer frames, same behaviour).
3. Watchdog test, wheels off the ground: send `mbot2._d(40,40)` once and nothing else; the wheels must stop within ~0.45 s. Then repeat while killing the link (toggle phone Bluetooth) to confirm the thread survives a disconnect and a Live Mode re-handshake (`f3 f6 03 00 0d 00 01 0e f4`) does not kill it.

Heartbeat / drive expression: **`mbot2._d(L,R)`** with integer RPM, e.g. `mbot2._d(62,48)`. Script length 15-17 bytes, frame 27-29 bytes = 2 BLE chunks. Release: `mbot2._d(0,0)`. Tuning at runtime: `mbot2._stop_cm=15`, `mbot2._wto=400` (fire and forget).

Concurrency notes (**UNVERIFIED**): `EM_stop` from the watchdog thread can race with a `drive_speed` in the executor only at the instant a late frame arrives after a timeout, which is harmless (the next frame re-arms). Reading the ultrasonic sensor from the thread while the page also queries it could collide on the mBuild bus; when the robot-side clamp is on, the page should read `mbot2._cm` instead of calling `cyberpi.ultrasonic2.get(1)` itself.

### 3c. Fallbacks if `_thread` is unavailable or killed

1. **`machine.Timer` callback** (same logic, no thread):
   ```python
   from machine import Timer
   M._tm = Timer(3)   # try ids 3,2,1; CyberPiOS may already use some (UNVERIFIED)
   M._tm.init(period=100, mode=Timer.PERIODIC,
              callback=lambda t: (M._mv and time.ticks_diff(time.ticks_ms(), M._t) > M._wto) and (setattr(M, '_mv', 0) or M.EM_stop()))
   ```
   On ESP32 the callback is scheduled into the VM, so it runs only when Python is running somewhere; **UNVERIFIED** whether that holds in the Live Mode idle state.
2. **CyberPi timer event:** register `@cyberpi.event.greater_than(0.4, 'timer')` -> `mbot2.EM_stop()` via `exec`, and make the drive expression `(cyberpi.timer.reset(), mbot2.drive_speed(l,-r))`. Events fire on the rising edge, so it fires once per stall. Method names and whether events are dispatched in Live Mode are **UNVERIFIED** (check `dir(cyberpi)`, `dir(cyberpi.event)`).
3. **Client-only (no robot code):** stream `mbot2.drive_speed(l,-r)` at 10 Hz; on release send `drive_speed(0,0)`; on `visibilitychange`(hidden), `pagehide`, `blur`, `pointercancel`, stop-button and any write error send `mbot2.EM_stop()` with mode 3, then mode 0. This covers page hidden and user error, but **not** a silent link loss, unless CyberPiOS's own `online_connect_check` stops the motors (test in section 3a). If neither the watchdog nor the firmware covers link loss, keep a "safe mode" toggle that reverts to timed moves, fixed as follows: use `t` of 0.25-0.3 s, send the next burst only after the previous one should have finished (sleep `t*1000 + 30` ms, never less), never more than one outstanding, so there is no backlog. It will still pulse (ramps), which is the price of having no watchdog.

---

## 4. Command rate

Evidence:
- No source measures throughput or latency over BLE. DS paces 20-byte chunks 8 ms apart (`connection.py:18-19, 77-80`); RV waits 30 ms between queries (`transport.rs` `INTER_QUERY_MS`); MB adds only 1 ms before each request (`modules.py:65-66`) but waits for each reply.
- [FW] shows a bounded robot-side queue (`script_list`, `SCRIPT_LIST_MAX_LEN`) feeding one executor thread. Max length and overflow policy are **UNVERIFIED**.
- [PX] runs `drive_speed` in a tight loop on the robot itself (`01-basic_movement.py:20-39`, no sleep), so `drive_speed` itself is cheap; the cost per Live Mode frame is parse/compile of the expression plus a shield write, likely a few ms (**UNVERIFIED**).

Budget: a `mbot2._d(62,48)` frame is 2 chunks, about 16-20 ms of client-side pacing with the current 8 ms delay. At 20 Hz that is ~40% of the write queue; at 10 Hz ~20%, which leaves room for battery/distance queries.

Recommended policy:
- **Control loop at 20 Hz (50 ms)** on the client for reading the stick and slewing.
- **Send on change**: when either wheel target changed by >= 3 RPM since the last sent value, at most one frame per 50 ms.
- **Keepalive**: if nothing was sent for 120 ms while moving, resend the current value (about 8 Hz). With a 400 ms watchdog this tolerates two lost frames.
- **Latest-wins mailbox, not a FIFO**: hold at most one pending drive frame; a newer value overwrites it. Never let drive frames pile up behind a slow query (cap query timeouts, and skip distance polling while a drive frame is pending).
- Stop frames bypass the mailbox and use mode 3 (then mode 0 as backup).
- Measure on day one: round trip of `cyberpi.get_battery()` while streaming at 20 Hz; if it grows over time, the robot is backing up, so drop to 10 Hz.

---

## 5. Recommended joystick design

### 5a. Input shaping

Stick: `x` (steer, right positive), `y` (throttle, up positive), each -1..1, pointer-captured, clamped to the unit circle.

```js
const DEAD = 0.08;        // radial deadzone (fraction of radius)
const EXPO_T = 0.35;      // throttle expo (0 = linear, 1 = pure cubic)
const EXPO_S = 0.5;       // steering expo
const expo = (v, e) => Math.sign(v) * ((1 - e) * Math.abs(v) + e * Math.abs(v) ** 3);

function shape(x, y) {
  const m = Math.hypot(x, y);
  if (m < DEAD) return { t: 0, s: 0 };
  const k = Math.min(1, (m - DEAD) / (1 - DEAD)) / m;   // rescale so output starts at 0
  return { t: expo(y * k, EXPO_T), s: expo(x * k, EXPO_S) };
}
```

### 5b. Arcade mixing with RC-car feel

```js
const VMAX = settings.speed;          // RPM, slider; default 80, hard cap 150 (API allows 200 [MOT])
const SPIN_MAX = 0.6;                 // in-place spin limited to 60% of VMAX
const STEER_AT_SPEED = 0.5;           // steering authority shrinks to 50% at full throttle

function mix(t, s) {
  const steer = s * (1 - (1 - STEER_AT_SPEED) * Math.abs(t)) * (t === 0 ? SPIN_MAX : 1);
  let l = t + steer, r = t - steer;
  const m = Math.max(1, Math.abs(l), Math.abs(r));      // normalise, keeps the curve radius
  return { l: (l / m) * VMAX, r: (r / m) * VMAX };       // forward-positive, left/right
}
```

- Reversing: keep `l = t + steer`, `r = t - steer` (stick right while reversing swings the tail left, like a car). If the owner prefers "stick right = nose right" when reversing, swap steer sign when `t < 0`. Make it a toggle.
- Minimum speed: if `|l|` and `|r|` are both < 6 RPM, send 0 (motor still creeps near zero per [BB] manual.txt:274). Round to integers.

### 5c. Slew-rate limiting (client side)

Per 50 ms tick, move each wheel's commanded RPM toward its target:
- **Accel 300 RPM/s** (0 -> 80 RPM in ~0.27 s): 15 RPM per tick.
- **Decel 600 RPM/s**: 30 RPM per tick. Use the decel limit whenever `|target| < |current|` or the sign flips (a sign flip always passes through 0).
- Stick release: target 0 with the decel limit, then once both wheels are at 0 send `mbot2._d(0,0)` (active zero-RPM hold) and, 300 ms later, one `mbot2.EM_stop()` to release the motors. Stop is reached in at most ~150 ms from 80 RPM.
- Emergency (stop button, page hidden, `pagehide`, `blur`, `pointercancel`, BLE error, guard trip): skip the ramp, send `mbot2.EM_stop()` with mode 3 immediately, then mode 0, and clear the mailbox.

### 5d. What to send on release

| Situation | Send | Why |
|---|---|---|
| Normal release | ramp down, then `mbot2._d(0,0)` (= `drive_speed(0,0)`), then `EM_stop()` after 300 ms | Smooth stop under client control; final `EM_stop` puts motors in a defined idle state |
| Emergency | `EM_stop()` mode 3, then mode 0 | Must not wait in the executor queue (section 1c) |
| Link lost | nothing can be sent | Robot-side watchdog stops within 400 ms (or firmware behaviour, to be tested) |

Brake vs coast for `EM_stop` vs `drive_speed(0,0)` is **UNVERIFIED**; observe on the floor.

### 5e. Obstacle guard

Preferred: robot-side clamp in the watchdog (`mbot2._stop_cm=15`): forward commands are zeroed on the robot when the last reading is under 15 cm, independent of BLE latency. The page reads `mbot2._cm` at 2-5 Hz for display.

Client-side shaping on top (works with either variant):

```js
const STOP_CM = 15, SLOW_CM = 40, STALE_MS = 600;
function guard(t, dist, age) {
  if (t <= 0) return t;                                  // reversing always allowed
  if (dist == null || age > STALE_MS) return Math.min(t, 0.3);   // unknown -> creep only
  if (dist <= STOP_CM) return 0;
  return Math.min(t, (dist - STOP_CM) / (SLOW_CM - STOP_CM));    // linear slow-down 40 -> 15 cm
}
```

Apply `guard` to `t` before mixing, so steering still works in front of a wall (spin away). Note the current `checkObstacle()` treats a stale reading as "no obstacle" (`bus.js:147-153`) and polls at 1 Hz (`app.js:110-120`); for continuous driving that is ~1 s of blind travel, so poll distance at 5 Hz while `t > 0` (or use the robot-side clamp) and treat stale as "creep".

### 5f. Parameter summary

| Parameter | Value |
|---|---|
| Drive call | `mbot2._d(L,R)` -> `drive_speed(L,-R)` on the robot (forward-positive L/R, EM1 = left assumed) |
| Client loop | 20 Hz |
| Send policy | on change >= 3 RPM, max 20 Hz; keepalive every 120 ms while moving; latest-wins mailbox |
| Watchdog timeout | 400 ms, checked every 50 ms on the robot |
| Deadzone | 8% radial, rescaled |
| Expo | throttle 0.35, steering 0.5 |
| Max speed | slider, default 80 RPM, cap 150; spin cap 60% |
| Steering at full throttle | 50% |
| Slew | accel 300 RPM/s, decel 600 RPM/s |
| Min speed | < 6 RPM -> 0 |
| Release | ramp, `_d(0,0)`, `EM_stop()` +300 ms |
| Emergency | `EM_stop()` mode 3 then mode 0 |
| Obstacle | stop 15 cm, slow from 40 cm, stale (> 600 ms) -> 30% throttle cap |

---

## Hardware test checklist (in order)

1. `dir(mbot2)`, `dir(cyberpi)`: confirm `drive_speed`, `EM_set_speed`, timer and event names.
2. `mbot2.drive_speed(30,0)` 0.5 s, then `EM_stop()`: which wheel, which direction. Then `drive_speed(30,-30)`: must drive forward.
3. Backlog test with the current build: hold forward 10 s, release, time the overrun. Repeat with `EM_stop` in mode 3.
4. Firmware link-loss behaviour: `drive_speed(40,-40)` (wheels up), turn phone Bluetooth off, time it.
5. `__import__('_thread').get_ident()`; install watchdog; `mbot2._d(40,40)` once, wheels must stop in ~0.45 s; repeat across a disconnect/reconnect.
6. Stream at 20 Hz for 60 s while querying `cyberpi.get_battery()` every second; round-trip time must stay flat.
