# mBot2 Control

**Open the app: https://d57udy.github.io/mbot2-control/** (Chrome on Android or desktop)

Drive a Makeblock mBot2 from a phone browser over Web Bluetooth, with German or English voice commands. Static site, intended for GitHub Pages. No code is uploaded to the robot; the page uses CyberPi's Live Mode protocol.

Status: **v0.8.3**, tested on a real mBot2 (firmware 44.01.013). In the owner's room the robot navigates to several goals, across a parquet/carpet transition, and returns to the start within about 2 cm after its home check. Hardware findings: `research/07-hardware-session.md`; localization research: `reports/Sonar localization for mBot2.md`.

### What the robot does

- Drive with buttons, joystick (smooth, gyro-aware) or voice; talk to it with Claude (tool calling).
- Continuous 360° ultrasonic sweeps (about 70 to 85 readings, timing-corrected) build an occupancy map with hit/miss evidence: obstacles get darker with every confirming scan and fade when later scans see through them; single glitches and crash contacts are temporary.
- Localization: wheel encoders (calibrated 6.7 cm wheel) plus gyro with bias estimation and gyro/odometry cross-check, wall-direction heading snap, scan matching against the map with an uncertainty-sized search window, and a home check against the first scan.
- Route planning (A*) with short legs, obstacle and crash detection (stall, jolt, twist), automatic back-off and replanning.
- Calibration and diagnostics in the app: automatic direction calibration, Scan-Labor (sensor latency, beam width), Strecken-Test (wheel vs ultrasonic), run recording export.

### Calibration found on the test robot

| Setting | Value | How it was found |
|---|---|---|
| Gyro direction | counterclockwise positive (Gyro umgekehrt) | joystick right = right, sweep yaw falling |
| `mbot2.turn` | as documented (+ = clockwise) | Dreh-Test with corrected gyro |
| Wheel mapping | EM1 left, EM2 mirrored | Automatisch erkennen |
| Sensor latency | 120 ms | Scan-Labor, cw/ccw bottle test |
| Beam width | about 25° | Scan-Labor |
| Ultrasonic range | 150 cm (no echo reads about 190 cm) | field scans |
| Wheel diameter | 6.7 cm | Strecken-Test |
| Script length limit | 200 bytes per Bluetooth frame | Verbindungstest |

## Use

1. Open the site in **Chrome on Android** (or desktop Chrome). iOS and Firefox have no Web Bluetooth.
2. Robot on, home screen, no program running, not connected to mBlock.
3. Tap **Verbinden**, pick the `Makeblock_LE...` device.
4. Drive with **Tasten** (hold the arrows) or **Joystick** (RC-car style: up/down is throttle, left/right steers, curves blend both). The slider sets the maximum wheel speed.
5. Or tap **Sprache an** and speak: "vorwärts", "zurück 2 Sekunden", "links", "rechts 45 Grad", "dreh dich um", "Licht blau", "schneller", "stopp".
6. **Lichter**: colours for the five CyberPi LEDs on the back, brightness, animations, and the ultrasonic sensor's blue "eye" LEDs (per eye, emotion presets, experimental per-LED).
7. **Bodensensor**: live readout of the Quad RGB sensor (four probes L2 L1 R1 R2: gray value, detected colour, line status, line offset) and its fill light.
8. **Umgebung scannen**: the robot turns in 8/12/16 steps and measures the distance in each direction. Scans build a **map** (5 cm grid, free/obstacle/unknown) that persists while you drive; the robot's position is tracked from its turns and moves. **Tap the map** to drive there around obstacles (route planning with short legs, rescans and replanning), **Erkunden** explores unknown areas, **Nach Hause** returns to the start. **Karte löschen** makes the current position the new start.
9. **Gespräch (KI)**: paste an Anthropic API key under Einstellungen → KI, switch the voice mode to "Gespräch (KI)" and talk normally. Claude answers aloud, shows emotions on the eyes and LEDs, and can move, turn, scan and read sensors through the same safety layer as the buttons. Saying "stopp" stops the robot, the speech and the AI immediately.

**Simulator** runs everything without a robot.

**Map controls**: drag to pan, pinch or mouse wheel to zoom, tap to set a goal. Rotate the map to match the room: twist with two fingers, right-drag or Shift+drag with the mouse, Shift+wheel (5° steps), or the ⟲ ⟳ buttons. "Roboter-Richtung oben" keeps the robot's heading pointing up; tapping the compass arrow (top right, points to "forward from the start") resets the rotation. Maps can be saved on the device, exported as a JSON file and imported again.

## Install on Android

1. Open https://d57udy.github.io/mbot2-control/ in Chrome.
2. Chrome menu (three dots) → **Install app** (or **Add to Home screen**).
3. Start mBot2 from the home screen icon. It opens full screen without the browser bar, and Bluetooth works the same as in the browser tab. The app files are cached, so it starts without network; the AI conversation still needs internet.

Updates arrive when the app is opened online after a new version is published.

## Layout

| Path | Purpose |
|---|---|
| `index.html`, `style.css` | UI |
| `js/protocol.js` | f3/f4 frame builder and reply parser |
| `js/robot-ble.js` | Web Bluetooth driver (GATT queue, handshake, primitives) |
| `js/robot-sim.js` | Simulator with the same interface |
| `js/bus.js` | JSON command schema v1 and safety layer |
| `js/drive.js` | Continuous drive: arcade mixing, expo, ramping, obstacle slow-down, 20 Hz stream |
| `js/joystick.js` | Touch joystick component |
| `js/voice.js` | Speech recognition wrapper and command parser |
| `js/scan.js`, `js/radar.js` | Environment scan, open-direction finder, radar plot |
| `js/pose.js`, `js/gridmap.js`, `js/planner.js`, `js/navigate.js`, `js/mapview.js` | Position tracking, occupancy map, A* route planning, navigator (go to, explore, go home), map drawing |
| `js/mapcontrols.js` | Map zoom, pan, rotation, pinch and twist, tap, fit, follow and heading-up |
| `js/mapstore.js` | Save, load, export and import maps (localStorage, JSON) |
| `manifest.webmanifest`, `sw.js`, `icons/` | Installable app (PWA): manifest, offline cache, icons |
| `js/tools.js`, `js/agent.js`, `js/tts.js` | LLM tool definitions and executor, Claude tool-use loop, speech output |
| `js/sim-view.js` | Simulator drawing (room, obstacles, scan rays) |
| `js/app.js` | Wiring, hold-to-drive, page lifecycle safety |
| `docs/ASSESSMENT.md` | Blockers, AI integration architecture, roadmap |
| `research/` | Source research with citations |

## Develop

```sh
python3 -m http.server 8765    # then open http://localhost:8765 (localhost counts as secure)
npm test                        # unit + simulator integration tests (Node 20+)
```

The service worker (offline cache) is not registered on localhost, so local edits show up on reload. Bump `VERSION` in `sw.js` with every deploy.

From the browser console: `mbot.bus.submit(mbot.makeCommand('turn', {deg: 90}))` or `mbot.say('vorwärts')`.

## Safety

Driving streams `mbot2.drive_speed` at up to 20 Hz. On connect the page installs a small watchdog thread on the robot (via Live Mode `exec`) that stops the motors 0.4 s after the last drive command, so the robot halts if Bluetooth drops. The header shows whether the watchdog is active. Stop is sent in the robot's immediate mode so it skips anything still queued. The page also stops the robot when it is hidden or the screen locks, and slows down from 40 cm and stops at 15 cm in front of obstacles.

## AI conversation and privacy

The page calls `https://api.anthropic.com` directly from the browser with your own key (header `anthropic-dangerous-direct-browser-access`). Use a key with a spending limit. The key stays in memory unless you tick "merken", which stores it in this browser's localStorage. The model only gets tool access through the command bus (speed, distance and obstacle limits apply); it never sends raw Python to the robot.

## Credits

Protocol details from [DrorSh/mbot_python](https://github.com/DrorSh/mbot_python) (MIT) and [Hulupeep/mbot_ruvector](https://github.com/Hulupeep/mbot_ruvector) (MIT OR Apache-2.0).
