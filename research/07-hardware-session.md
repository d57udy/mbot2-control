# Hardware session 2026-10-03 (Mac Chrome, real mBot2)

Measured with the page's connection test and step-by-step Live Mode queries from Chrome automation.

| Finding | Evidence |
|---|---|
| Advertised name `Makeblock_LE345F45DD03BA`, service `0000ffe1-...`, characteristics ffe2 (notify) / ffe3 (write) | log "Using service 0000ffe1" |
| Firmware **44.01.013** | `cyberpi.get_firmware_version()` |
| Round trip of a short query about **90 ms** (20-byte chunks) | connection test |
| Scripts up to **200 bytes** work; **300 bytes are silently dropped** (no reply, robot stays responsive) | connection test |
| Chunk pause of 0 ms works on macOS Chrome; with 8 ms pauses a background tab stretches each pause to about 1 s (timer throttling) | `chunkDelayMs = 0` vs 8 in a hidden tab |
| `exec("b=2")`, assembling text in `mbot2._s` and `exec(mbot2._s)` with plain assignments work | step test |
| **Robot reboots** (back to the start screen) on: `exec("import time")`, `exec("T=__import__('time')")`, top-level `setattr(mbot2,'_T',__import__('time'))`, and a `sorted(__import__('sys').modules)` listing | five reboots, each reproducible on the first try |
| `__import__('_thread').get_ident()` and `__import__('sys').modules.get('mbot2') is mbot2` work; `mbot2` and `cyberpi` are plain modules | step test |
| All 13 ultrasonic eye effect methods exist (`hasattr`) | eye probe |
| **Motors keep running after Bluetooth disconnects**: `mbot2.drive_speed(20,-20)`, then `gatt.disconnect()` without a stop; wheels kept turning until power off | owner observation |

Consequences in the code:
- Robot-side helpers (watchdog, eye helper) are off by default; they need imports and cannot be installed safely on this firmware.
- Scripts over 200 bytes are refused by the driver; long code and long replies are split (`execLong`, `queryLong`).
- Joystick driving stays continuous (`drive_speed`); the page stops the robot on hide, blur, page close and errors, but a silent link loss mid-drive is not covered. A timed-burst "safe mode" is the fallback if needed.
- Eye effects use direct calls with a brightness argument where the firmware needs one.

Open: whether the eye effects now all play, the line-sensor polarity, wheel direction (mirrored), and the remaining UAT items.
