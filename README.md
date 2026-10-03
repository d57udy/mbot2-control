# mBot2 Control

Drive a Makeblock mBot2 from a phone browser over Web Bluetooth, with German or English voice commands. Static site, intended for GitHub Pages. No code is uploaded to the robot; the page uses CyberPi's Live Mode protocol.

Status: v0.3. Basic Bluetooth driving confirmed on a real mBot2. Joystick, watchdog, lights, floor sensor, environment scan and AI conversation are tested with unit and simulator integration tests; hardware acceptance tests are in `docs/UAT-v0.3.md`.

## Use

1. Open the site in **Chrome on Android** (or desktop Chrome). iOS and Firefox have no Web Bluetooth.
2. Robot on, home screen, no program running, not connected to mBlock.
3. Tap **Verbinden**, pick the `Makeblock_LE...` device.
4. Drive with **Tasten** (hold the arrows) or **Joystick** (RC-car style: up/down is throttle, left/right steers, curves blend both). The slider sets the maximum wheel speed.
5. Or tap **Sprache an** and speak: "vorwärts", "zurück 2 Sekunden", "links", "rechts 45 Grad", "dreh dich um", "Licht blau", "schneller", "stopp".
6. **Lichter**: colours for the five CyberPi LEDs on the back, brightness, animations, and the ultrasonic sensor's blue "eye" LEDs (per eye, emotion presets, experimental per-LED).
7. **Bodensensor**: live readout of the Quad RGB sensor (four probes L2 L1 R1 R2: gray value, detected colour, line status, line offset) and its fill light.
8. **Umgebung scannen**: the robot turns in 8/12/16 steps and measures the distance in each direction. A radar plot shows the result; tap an open direction to drive there, or **Erkunden** to scan and move up to three times.
9. **Gespräch (KI)**: paste an Anthropic API key under Einstellungen → KI, switch the voice mode to "Gespräch (KI)" and talk normally. Claude answers aloud, shows emotions on the eyes and LEDs, and can move, turn, scan and read sensors through the same safety layer as the buttons. Saying "stopp" stops the robot, the speech and the AI immediately.

**Simulator** runs everything without a robot.

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
| `js/scan.js`, `js/radar.js` | Environment scan, open-direction finder, navigation, radar plot |
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

From the browser console: `mbot.bus.submit(mbot.makeCommand('turn', {deg: 90}))` or `mbot.say('vorwärts')`.

## Safety

Driving streams `mbot2.drive_speed` at up to 20 Hz. On connect the page installs a small watchdog thread on the robot (via Live Mode `exec`) that stops the motors 0.4 s after the last drive command, so the robot halts if Bluetooth drops. The header shows whether the watchdog is active. Stop is sent in the robot's immediate mode so it skips anything still queued. The page also stops the robot when it is hidden or the screen locks, and slows down from 40 cm and stops at 15 cm in front of obstacles.

## AI conversation and privacy

The page calls `https://api.anthropic.com` directly from the browser with your own key (header `anthropic-dangerous-direct-browser-access`). Use a key with a spending limit. The key stays in memory unless you tick "merken", which stores it in this browser's localStorage. The model only gets tool access through the command bus (speed, distance and obstacle limits apply); it never sends raw Python to the robot.

## Credits

Protocol details from [DrorSh/mbot_python](https://github.com/DrorSh/mbot_python) (MIT) and [Hulupeep/mbot_ruvector](https://github.com/Hulupeep/mbot_ruvector) (MIT OR Apache-2.0).
