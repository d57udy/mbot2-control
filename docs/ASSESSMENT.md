# mBot2 web control: assessment, blockers and roadmap

Date: 2026-10-02. Synthesis of `research/01-ble-protocol.md`, `research/02-web-bluetooth-and-speech.md` and `research/03-ai-integration.md`. Sources and verification status are in those files. Nothing here has been tested on a real robot yet.

## How it works

```
Phone (Chrome, Android)                                     mBot2 (CyberPi)
┌───────────────────────────────────────────┐   BLE GATT    ┌──────────────────────┐
│ GitHub Pages site (HTTPS, static)         │  write ffe3   │ CyberPiOS Live Mode  │
│  buttons ─┐                               │ ────────────▶ │ evaluates MicroPython│
│  voice ───┼─▶ command bus ─▶ safety ─▶ BLE│               │ e.g. mbot2.forward() │
│  (later)  │   (JSON v1)     layer   driver│ ◀──────────── │ replies {"ret": ...} │
│  relay/AI ┘                               │  notify ffe2  │                      │
└───────────────────────────────────────────┘               └──────────────────────┘
```

- No code is uploaded to the robot. The page speaks the same private "f3/f4" Live Mode protocol mBlock uses: each frame wraps one MicroPython expression that the robot runs, optionally returning JSON.
- Web Bluetooth works from an HTTPS page, so GitHub Pages hosting is fine. Chrome's Web Speech API also needs HTTPS, so voice works on the same page.
- Every producer (buttons, voice, later a relay or an LLM) emits the same JSON command. Only the bus talks to the robot, through a safety layer.

## Blockers, ranked by how hard they are

| # | Blocker | Severity | Why it is hard | What v1 does about it |
|---|---|---|---|---|
| 1 | **Unofficial protocol** | High, unknowable | Reverse-engineered by two hobby projects (firmware 44.01.009 and 44.01.013). Makeblock can change it with any CyberPiOS update. | Single driver module, so a fix is local. Pin the firmware once it works (do not update from mBlock). |
| 2 | **The page must stay in the foreground** | High, structural | Screen off or tab switch: speech is aborted, timers are throttled, the robot cannot be supervised. No background mode exists for Web Bluetooth. This limits "AI drives the robot while the phone sits in a drawer". | Screen Wake Lock while connected; stop on `visibilitychange`/`pagehide`. For unattended use, plan a Raspberry Pi bridge (see roadmap). |
| 3 | **No guaranteed stop on disconnect** | High, safety | No source says CyberPiOS halts motors when BLE drops. A `forward(speed)` without a duration keeps going. | Only timed moves exist in the API: buttons and voice send 0.3 to 0.6 s bursts repeated while active, so losing the link stops the robot within one burst. |
| 4 | **Service UUID not confirmed on CyberPi** | Medium, first-hour fix | Chrome only exposes services named up front. Characteristics ffe2/ffe3 are well sourced; service ffe1 is a best guess. | Tries ffe1, ffe0, ffe5; logs visible services; "show all devices" diagnostic switch. |
| 5 | **Speech recognition on Android is quirky** | Medium | Needs network (no on-device mode on Android Chrome). `continuous` reports every partial guess as final, sessions end on silence, restarts may beep. | Stop words act on interim results immediately; other commands wait 0.5 s for the transcript to settle; restart loop with backoff. Fallback options: Vosk-browser grammar, Moonshine. |
| 6 | **Unknown command timing** | Medium | No measured latency or throughput. Blocking robot calls (timed moves, gyro turns) may queue later commands, including stop. | Short bursts keep the worst case to one burst. Measure on hardware; tune burst length and chunk size in settings. |
| 7 | **Reconnect needs a tap after reload** | Low, annoying | Persistent Web Bluetooth permissions (`getDevices()`) are still behind a flag. | Keep the page open; "reconnect" reuses the same device object without the chooser. |
| 8 | **Platform coverage** | Low for you | No Web Bluetooth on iOS or Firefox. Your phones are Android, so fine; guests with iPhones cannot drive. | Support warning on load. |
| 9 | **Exclusive connection** | Low | The robot accepts one central. mBlock or another phone connected blocks the page. | Documented in the start checklist. |

Not a blocker, but a recurring trap: the `drive_speed(l, r)` sign convention is contradictory between sources, so v1 uses only `forward/backward/turn_left/turn_right/turn`.

## AI integration: what changes and what does not

The phone is the only BLE link, so any AI system has to reach the robot through the page (or through a separate BLE device such as a Raspberry Pi).

Recommended path:

1. **LLM inside the page** (fastest to try). The page calls Claude with tool definitions generated from the command schema. Anthropic allows direct browser calls with the `anthropic-dangerous-direct-browser-access` header, but the key would sit in the browser. Use a spend-capped key pasted at runtime, never committed. Note: localStorage on `<user>.github.io` is shared by all of that user's project pages.
2. **Relay for external agents.** The page opens an outbound `wss://` connection to a small authenticated relay (Cloudflare Worker with a Durable Object fits the free tier). The same Worker can host a remote MCP server so Claude Desktop or Claude Code get tools like `move`, `turn`, `stop`, `read_distance`. Home Assistant or a hosted MQTT broker are alternatives. Prior art with the same shape but no auth: kumavulp/mcp-ble-bridge.
3. **Raspberry Pi bridge** if unattended operation matters more than the phone camera. Python (bleak) can speak the same protocol (DrorSh/mbot_python does) and run an MCP server locally.

Hard limits to design around:
- LLM decisions take seconds, so agents must issue bounded, timed primitives, never "drive until I say stop". The robot-side and page-side safety layers stay authoritative.
- Vision (phone camera to a vision model) gives one decision every few seconds. Fast closed-loop steering has to run in the page, with the LLM setting goals.
- A relay without strong auth means strangers can drive the robot. Separate credentials for phone and agent, and a "remote armed" switch on the phone.

## What v1 already does to keep this open

- **Versioned JSON command schema** (`js/bus.js`). The same definitions can become LLM tool schemas and MCP tools.
- **Transport-agnostic bus**: `window.mbot.bus.submit(command)` is the only entry point. A relay client is just another producer with `src: 'remote'`.
- **Safety layer**: speed and duration clamps, expiry of stale commands, stop jumps the queue and cancels pending writes, obstacle guard from ultrasonic polling, stop on page hidden or disconnect.
- **Voice parser returns commands, not actions**, so it can be swapped for an LLM.
- **Simulator driver** with the same interface, so agents can be developed without the robot.

## Roadmap

| Stage | Scope | Exit criterion |
|---|---|---|
| v0.1 (this commit) | Web Bluetooth driver, buttons, German/English voice, simulator, safety layer | Hardware checklist (`docs/HARDWARE_TEST.md`) passes |
| v0.2 | Fixes from hardware test; publish on GitHub Pages; measure latency | Drive a lap by voice on the phone |
| v1 | LLM-in-page mode with user-supplied key, tool calling over the command schema | "Fahr zum Sofa und dreh dich um" works in the simulator, then on hardware |
| v2 | Cloudflare relay + remote MCP server, authenticated, remote-armed switch | Claude Desktop drives the robot through the phone |
| v3 | Camera goals (approach target), or Raspberry Pi bridge for unattended use | Defined after v2 |
