# Ultrasonic Sensor 2 eye LEDs: why some emotions fail

Research date: 2026-10-03. Robot not available; findings come from the CyberPiOS firmware image and the existing sources in 04-leds-and-rgb-sensor.md. Owner report: "Some of the buttons for the different emotions don't work. The LED control of the distance sensor is not properly working yet."

## Sources

| Tag | Source |
|---|---|
| **[FW]** | `repos/perfecxx/firmware/CyberPi_firmware_44_01_011-ht2.bin` (CyberPiOS 44.01.011, same image as research/05). Analysed beyond `strings`: I located the frozen MicroPython qstr pools (static pool header at file offset `0x10c9b4`, 3607 entries; frozen pool header at `0x1ec318`, 4314 entries, DROM mapping file offset + `0x3f301000`), then decoded the bytecode preludes (n_pos_args, n_def_pos_args), the const tables (argument names) and the module body of `mbuild_modules/led_ultrasonic_sensor.py`. Scripts (local, not committed): `decode.py`, `sig.py` |
| **[DS]** | DrorSh/mbot_python docs/sensors.md:86-125, tested on firmware **44.01.013** |
| **[DOC]** | Makeblock "Input modules" Ultrasonic Sensor 2 API (see research/04) |

## Findings from the firmware ([FW], 44.01.011)

1. `cyberpi.ultrasonic2` is not an object with methods but the **module** `mbuild_modules.led_ultrasonic_sensor`: both `cyberpi.py` and `mbuild.py` do `import mbuild_modules.led_ultrasonic_sensor` and bind it as `ultrasonic2` (IMPORT_NAME / STORE_NAME sequence at file offsets `0x1b7d72` and `0x1e28e5`). So all calls are plain functions without `self`.
2. All 13 names in `EYE_EFFECTS` exist as `<name>_effect` functions. The list is not the problem.
3. **Signatures differ.** Decoded from the bytecode preludes and const tables:

| Function | Signature in 44.01.011 | `name_effect()` with no args |
|---|---|---|
| `happy_effect`, `wink_effect`, `naughty_effect`, `aggrieved_effect`, `look_left_effect`, `look_right_effect`, `eye_left_effect`, `eye_right_effect` | `(led_bri, index=1)` | **TypeError** (missing required argument) |
| `new_happy_effect`, `raises_brow_effect`, `standby_effect`, `dizzy_effect`, `thinking_effect` | `(index=1)` | works |
| `set_both_led_bri` | `(bri_1, bri_2=0, ..., bri_8=0, index=1)`; if `bri_1` is a list it is extended to 8 values | |
| `set_single_led_bri` | `(led_bri, led_index, index=1)` | |
| `change_led_bri` | `(led_bri, led_index, index=1)` | |
| `get_led_bri` | `(led_index, index=1)` | |
| `get_distance` | `(index=1)` | |
| `show_led_emotion` | `(led_bri, name='happy', index=1)` | |
| `show_animation` | `(emotion='happy', index=1)` | |

4. Aliases at the end of the module body: `get = get_distance`, `set_bri = set_single_led_bri`, `add_bri = change_led_bri`, `get_bri = get_led_bri`, **`led_show = set_both_led_bri`**, **`play = show_animation`**.
5. `play(emotion)` maps names to effects: `happy` to `new_happy_effect()`, `wink` to `raises_brow_effect()`, `sleepy` to `standby_effect()`, `dizzy`, `thinking`, and `happy_old`, `wink_old`, `naughty`, `aggrieved`, `look_left`, `look_right`, `eye_left`, `eye_right` to the brightness-taking effects with a global `led_bri` that the module never assigns (likely a NameError, so `play` is no better for those eight).
6. `set_both_led_bri` (and therefore `led_show`), `play` and `show_led_emotion` first call `__wait_blocking_out`, which sets a `clear_flag` that running effects poll in `_wait_for_runing`. The effect functions sleep in loops (`time.sleep`), so they block the Live Mode executor for their whole run.

Consequences for the app before this change:
- `cyberpi.ultrasonic2.happy_effect()` and seven others raise TypeError on 44.01.011. They were sent with mode 0, so nothing was reported. That matches "some buttons don't work": on this firmware exactly happy, wink, naughty, aggrieved, look_left, look_right, eye_left and eye_right fail; new_happy, raises_brow, standby, dizzy and thinking work.
- The eye sliders sent `[set_both_led_bri(L,R), led_show()]`. On 44.01.011 this sets LED 1 = L, LED 2 = R, LEDs 3 to 8 = 0, then `led_show()` raises TypeError. Result: two LEDs of one eye respond, the other eye stays dark. That matches "LED control of the distance sensor is not properly working".
- [DS] reports on 44.01.013 that `happy_effect()` and `led_show()` with no arguments work and that `set_both_led_bri(L,R)` switches whole eyes. So the signatures changed between 011 and 013, and the owner's firmware version decides which form works. The fix therefore tries both at runtime.

## Root causes, ranked

| # | Cause | Confidence | Evidence |
|---|---|---|---|
| 1 | Effect signature mismatch: 8 of 13 effects need a brightness argument on some firmware; called without it they raise TypeError | High for 44.01.011 ([FW] bytecode), unknown for the owner's version | Table above; exactly the "some buttons" pattern |
| 2 | Errors invisible: effects and eye writes were mode 0, Live Mode reports nothing for them | High | robot-ble.js used `run()`; research/01 line 182 |
| 3 | Eye sliders used `led_show()` with no args and `set_both_led_bri(L,R)`, which on 011 only drives LEDs 1 and 2 | High for 011, conflicts with [DS] on 013 | [FW] signature and default tuple `(0,0,0,0,0,0,0,1)` |
| 4 | Effects block the single executor for seconds; distance polls (4 Hz, or every tick while driving) and 20 Hz drive frames pile up behind them, time out in the browser (1.5 s), and can overflow the robot's bounded `script_list` | Medium | research/05 §1b; effect bodies sleep in loops |
| 5 | A later `led_show`/`set_both_led_bri` cancels a running effect through `clear_flag` | Low to medium (only matters if two scripts run concurrently, which the queue suggests they do not) | `__wait_blocking_out` call at the top of `set_both_led_bri` |
| 6 | Wrong effect names | Ruled out for 011 | All 13 present ([FW]) |

## What changed

`js/robot-ble.js`
- `EYE_HELPER_SRC`: installed once per connection with `exec` (like the drive watchdog), defines `mbot2._fx(name)` and `mbot2._eyes(l, r)`. `_fx` calls `<name>_effect()`, retries with `(100)` on TypeError, and returns the run time in ms, `'missing'`, or `'error: ...'`. `_eyes` tries `led_show([l,l,l,l,r,r,r,r],1)` (documented and matches 011) and falls back to `set_both_led_bri(l,r)` + `led_show()` (the [DS] form).
- `ensureEyeHelper()`: lazy install, re-done after a reconnect (keyed on the write characteristic).
- `queryUntilDone(script, timeoutMs, label)`: reply-mode send for long animations; resolves `{done:true, value, ms}` or `{done:false}` on timeout; a late reply is logged as "finished late" instead of "unmatched reply".
- `eyesEffect(name)`: uses `_fx`, throws on `missing`/`error`, returns `{name, ms}` or `{name, running:true}`; learns the duration. Without the helper it calls the effect directly with `(100)` for the eight brightness effects (`EYE_EFFECT_NEEDS_BRI`).
- `eyes(l, r)`: via `_eyes`; the first call per connection is reply-mode and logs which form works; later calls are fire and forget.
- `eyeLed('all', bri)`: now `led_show([bri x8],1)`; single LEDs keep `set_bri(bri,id,1)` (= `set_single_led_bri`).
- `ledEffect(name)`: reply-mode with `queryUntilDone`, since `led.play` blocks too.
- `probeEyeEffects(names)`: one `hasattr` list comprehension, returns the names that exist.
- `effectEstimateMs(name)`: measured duration + 300 ms, default 6 s.

`js/bus.js`
- `sensorQuietUntil`, `sensorsQuiet()`, `runEffect()`: `eyes_effect` and `led_effect` set a quiet period from `robot.effectEstimateMs` (3 s if the driver lacks it), end it when the robot replies, keep it if the effect is still running after the timeout, and refuse a second effect while one runs ("busy").
- `read` during the quiet period returns the last value (distance: `lastDistance.value`, whose timestamp is not refreshed so the obstacle guard ignores it once stale) without touching the robot.

`test/eyes.test.mjs`: 12 tests (frames decoded from a fake characteristic; helper and no-helper paths; error surfacing; late replies; whitelist; quiet period with a fake robot; simulator compatibility).

## Hardware test procedure

Type these in "Befehl senden" with the `py:` prefix (sent in reply mode, 3 s timeout; a timeout on an effect only means it took longer than 3 s).

1. Firmware: `py: cyberpi.get_firmware_version()`. Record it.
2. Confirm cause 1: `py: cyberpi.ultrasonic2.happy_effect()` then `py: cyberpi.ultrasonic2.happy_effect(100)`. On 011-like firmware the first does nothing (and returns nothing or an error), the second animates. Repeat with `dizzy_effect()` (should work without arguments).
3. Confirm helper: after one emotion button click, the log should show `> mbot2._fx("...")` and the result. `py: [hasattr(mbot2,'_fx'),hasattr(mbot2,'_eyes')]` should give `[true,true]`.
4. Per-effect check: `py: mbot2._fx("happy")` for each name; expect a number (ms). Note any `'error: ...'` strings and the durations.
5. Eye slider form: `py: cyberpi.ultrasonic2.led_show([100,100,100,100,0,0,0,0],1)` (one whole eye on?), then `py: cyberpi.ultrasonic2.led_show([0,0,0,0,100,100,100,100],1)`. Then the [DS] form: `py: cyberpi.ultrasonic2.set_both_led_bri(100,0)` and `py: cyberpi.ultrasonic2.led_show()`. Record which one lights whole eyes and which eye is "left".
6. Signature probe without side effects: `py: [n for n in ['happy','wink','dizzy'] if hasattr(cyberpi.ultrasonic2,n+'_effect')]`.
7. Cause 4: start joystick mode, keep the stick centred, click "schwindelig" (dizzy). The log should show no distance timeouts during the animation and the effect result afterwards. Repeat with the old build to compare if needed.
8. Cause 5: start `py: mbot2._fx("thinking")`, immediately move an eye slider. Note whether the animation stops early.
