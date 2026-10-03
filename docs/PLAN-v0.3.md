# v0.3 plan: eye LED fixes, environment scan, AI conversation

Date: 2026-10-03. Owner is away; work runs autonomously with parallel agents. The physical robot is not available, so everything is verified with unit tests, simulator integration tests and a mocked LLM. Hardware acceptance tests are prepared in `docs/UAT-v0.3.md`.

## Goals

1. **Eye LEDs**: find out why some emotion buttons do nothing and fix what can be fixed without hardware; make failures visible.
2. **Environment scan and navigation**: the robot spins in steps, measures distance at each heading, builds a polar map, finds open directions, and drives toward them.
3. **Conversation**: speak to the robot; a tool-calling LLM answers by voice, expresses emotions (eyes, LEDs, sounds) and can call robot functions (move, turn, scan, sensors).

## Architecture

```
mic ─▶ VoiceListener ─▶ (mode: commands) ─▶ parseUtterance ─▶ bus
                      └▶ (mode: conversation) ─▶ ConversationAgent ─▶ Claude Messages API
                                                    │  tool_use            (direct browser access,
                                                    ▼                       user-supplied key)
                                              ToolExecutor ─▶ bus / scan.js ─▶ robot (BLE or sim)
                                                    │
                                              reply text ─▶ tts.speak() (mic paused while speaking)
```

- All robot actions still go through `CommandBus.submit` and its safety layer. The LLM never gets raw Python.
- "stopp" stays a local, immediate safety path: it aborts the agent loop and stops the robot.

## Interface contracts (fixed before parallel work)

Already in place (`js/bus.js`, `js/robot-*.js`):
- `turn { deg, wait }` and new `straight { cm, wait }` (max 100 cm, clamped to the last fresh distance minus 15 cm). `wait: true` resolves when the robot finished (BLE: reply-mode query, robot replies after the blocking call; UNVERIFIED on hardware).
- Every robot driver implements `turn(deg, {wait})` and `straight(cm, {wait})`.

`js/scan.js` (agent: scan):
- `scan(bus, { steps = 12, onPoint, signal }) -> { points: [{ angle, cm }], startedAt }`. Angles relative to the start heading, clockwise positive, 0 = straight ahead. Ends facing the start heading.
- `findOpenings(points, { minCm = 50 }) -> [{ angle, widthDeg, cm }]` sorted best first.
- `describeScan(points) -> string` compact text for the LLM.
- `js/radar.js`: `drawRadar(canvas, points, { highlight })`.
- Simulator gains obstacles so scans are meaningful; drawing moves to `js/sim-view.js` exporting `drawSim(canvas, state, room)`.

`js/tools.js`, `js/agent.js`, `js/tts.js` (agent: conversation):
- `TOOLS`: Anthropic tool definitions. `createToolExecutor({ bus, makeCommand, scan, findOpenings, describeScan, onEmotion }) -> { execute(name, input) -> Promise<object> }`.
- `ConversationAgent({ apiKey, model, tools, executor, lang, onEvent, fetchImpl })` with `send(text, { signal }) -> Promise<{ text }>`, `reset()`.
- `tts.speak(text, lang) -> Promise`, `tts.cancel()`, `tts.speaking`.

## Work split and file ownership

| Stream | Owner | Files |
|---|---|---|
| A. Eye LED investigation and fix | agent "eyes" | `js/robot-ble.js` (eye/LED methods only), `js/bus.js` (eye cases, effect list), `research/06-eye-leds.md`, `test/eyes.test.mjs` |
| B. Scan, navigation, sim obstacles, radar | agent "scan" | `js/scan.js`, `js/radar.js`, `js/robot-sim.js`, `js/sim-view.js`, `test/scan.test.mjs` |
| C. LLM conversation | agent "conversation" | `js/tools.js`, `js/agent.js`, `js/tts.js`, `test/agent.test.mjs`, `test/tools.test.mjs` |
| D. Integration, UI, docs, release | lead | `index.html`, `style.css`, `js/app.js`, CSP, `docs/*`, browser integration tests, commit and deploy |

## Test strategy

| Level | What | How |
|---|---|---|
| Unit | Frame builder, voice parser, drive stream, scan math and openings, tool schemas and executor, agent loop (tool_use, errors, abort, iteration cap), TTS wrapper | `npm test` (Node test runner), mocked `fetch`, fake clocks |
| Integration | Full page in Chrome with the simulator: scan from UI, radar renders, conversation turn with a mocked LLM that calls tools and the sim moves, stop aborts the agent | Chrome automation against a local server, `window.fetch` stub for the Anthropic endpoint |
| UAT | Real robot and phone | `docs/UAT-v0.3.md` checklist with pass criteria |

## Risks

- LLM key in the browser: user pastes their own spend-capped key; stored in `localStorage` only if they tick "remember". CSP is widened only for `https://api.anthropic.com`.
- Blocking robot calls (`turn`, `straight`, eye effects) occupy the robot's executor; scans are sequential by design.
- TTS output may be picked up by the microphone: the mic is paused while speaking.
