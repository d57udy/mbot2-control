# 03: AI Integration Architectures for the mBot2 Web Bluetooth Controller

Status: research notes, October 2026.
Context: static page on GitHub Pages (HTTPS), Chrome on Android, Web Bluetooth link to an mBot2 (CyberPi). The page already does manual driving and voice commands. Goal: let AI systems (Claude, LLM agents, Home Assistant, vision) control the robot later.

Confidence markers: **[verified]** = checked against a cited source during this research. **[reported]** = third-party report, not officially documented. **[estimate]** = my engineering estimate, measure before relying on it.

---

## TL;DR

* **Recommended end state:** the phone page stays the only BLE endpoint and acts as a *robot driver*. It opens an outbound `wss://` connection to a small authenticated relay (Cloudflare Worker + Durable Object is the best fit; Home Assistant is a good second option since it already runs at home). A remote MCP server (can live in the same Worker) exposes bounded tools (`move`, `turn`, `stop`, `read_sensors`) to Claude Desktop / Claude Code / claude.ai connectors.
* **Simplest AI step first:** an in-page LLM agent with a user-pasted API key (Anthropic supports browser CORS via the `anthropic-dangerous-direct-browser-access` header). Fine for a single owner; never ship a key in the repo.
* **Safety lives on the robot side of the link, not in the LLM:** every motion command carries a duration and the robot auto-stops (the mbot_python `secs` pattern), plus a page-side watchdog, rate limits, and an E-stop that preempts everything.
* **The v1 decision that matters most:** a single JSON command schema and a transport-agnostic command bus in the page. UI buttons, voice, in-page LLM, and the remote relay all become producers of the same command objects.

---

## 1. LLM directly in the page

### 1.1 Pattern

Browser holds the conversation loop: user speaks or types, page sends messages plus tool definitions to an LLM API, model returns tool calls (`move`, `turn`, `stop`, `read_distance`), page executes them over BLE, returns results as tool results, loops until the model replies with text.

```
[mic / text] -> page agent loop -> LLM API (HTTPS, tool calling)
                      |   ^
                tool call  tool result
                      v   |
                 command bus -> BLE -> mBot2
```

### 1.2 Can the browser call the APIs directly (CORS)?

| Provider | Browser CORS | Notes |
|---|---|---|
| Anthropic (Claude) | **Yes, opt-in [verified]** | Add request header `anthropic-dangerous-direct-browser-access: true`. Introduced Aug 2024. Anthropic's position: OK for internal tools or "bring your own key" apps, dangerous if you embed your own key. Sources: [Simon Willison, 2024-08-23](https://simonwillison.net/2024/Aug/23/anthropic-dangerous-direct-browser-access/), [DEV: streaming Claude client in browser](https://dev.to/ferhatatagun/building-a-streaming-claude-client-in-the-browser-without-the-sdk-5f80). The JS SDK has an equivalent `dangerouslyAllowBrowser` option. |
| OpenAI (Chat/Responses) | **Unreliable [reported]** | SDK requires `dangerouslyAllowBrowser: true` ([OpenAI TS SDK](https://developers.openai.com/api/reference/typescript)). Reports conflict: a 2026 post says the OPTIONS preflight is rejected upstream (403) even though responses carry `access-control-allow-origin: *` ([DEV, May 2026](https://dev.to/tracepilot_2841f1db6718a1/that-openai-call-from-your-browser-is-failing-heres-why-3p3c)); an Oct 2025 outage broke browser calls and OpenAI called it a bug ([forum](https://community.openai.com/t/chat-completions-api-endpoint-down-blocked-any-web-browser-request/1362527)). Treat as "may work, not supported". For voice, the supported browser path is Realtime with ephemeral keys (below). |
| Google Gemini | **Partial [reported]** | Native endpoint generally usable from browser with an API key; the OpenAI-compatible endpoint and some SDK headers fail preflight ([Google AI forum](https://discuss.ai.google.dev/t/gemini-api-cors-error-with-openai-compatability/58619), [obsidian-gemini issue](https://github.com/allenhutchison/obsidian-gemini/issues/1023)). Google recommends keeping keys server-side. For Live API, ephemeral tokens are the supported browser path. |

Takeaway: Anthropic is the cleanest choice for a no-backend, in-page agent. Do a 5-minute `fetch` test from the GitHub Pages origin for any provider before committing.

### 1.3 The API key problem on a public GitHub Pages site

Options, from weakest to strongest:

1. **Key in source or build output.** Never. GitHub Pages is public; secret scanners and bots harvest keys within minutes.
2. **User pastes key, stored in `localStorage` (BYOK).** Acceptable for a single-owner hobby page. Risks: any XSS or compromised third-party script on the origin can read it; `username.github.io` is a shared origin across all your Pages project sites (`/repo1`, `/repo2` share `localStorage`), so a bug in another repo's page can read the key. Mitigations:
   * Use a dedicated Anthropic workspace/key with a low monthly spend limit.
   * Load no third-party scripts, or pin with Subresource Integrity; add a strict CSP via `<meta http-equiv="Content-Security-Policy">` with `connect-src` limited to the API and relay hosts.
   * Consider a custom domain or a separate GitHub account/org for origin isolation.
   * Offer "remember key" as opt-in; default to `sessionStorage`.
3. **Thin proxy (Cloudflare Worker) holding the key.** The page calls the Worker, which injects the key. The proxy itself then needs auth (otherwise it is an open LLM relay paid by you): a shared passphrase, Cloudflare Access, or a signed token. Also lets you enforce model allowlist, max tokens, and rate limits. Natural fit if you build the v2 relay on Cloudflare anyway.
4. **Ephemeral tokens** (OpenAI Realtime, Gemini Live). Backend mints a short-lived token; browser never sees the real key. Still needs a backend, so same as option 3 in effort.

Recommendation: BYOK with a spend-capped key for v1/v3-lite; move to the Worker proxy when the relay exists.

### 1.4 Latency per turn

* One tool-calling round trip to a hosted LLM is typically about 0.5 to 3 s depending on model size, prompt length, and network **[estimate]**. A "drive forward, check distance, turn" task with three sequential tool calls is three round trips: several seconds total.
* Use a small/fast model tier for the control loop, stream responses, keep the system prompt and tool list short, and enable prompt caching for the static prefix.
* Let the model batch: a tool like `run_sequence([...])` (bounded list of primitives) cuts round trips. Combined with on-robot timing, this is much better than one LLM call per 20 cm.
* Mobile network adds latency variance; measure on the phone, not the desktop.

### 1.5 Voice-to-voice realtime from the browser

* **OpenAI Realtime over WebRTC [verified].** Backend calls `POST /v1/realtime/client_secrets` to mint an ephemeral `ek_...` key; browser does WebRTC SDP exchange against `/v1/realtime/calls` with that key. Supports function calling inside the voice session. Docs: [WebRTC guide](https://developers.openai.com/api/docs/guides/realtime-webrtc), [Realtime overview](https://developers.openai.com/api/docs/guides/realtime), [Agents SDK voice quickstart](https://openai.github.io/openai-agents-js/guides/voice-agents/quickstart/).
* **Gemini Live API [verified].** WebSocket from the browser using ephemeral tokens minted by your backend. Defaults: 1 minute to start a session with the token, 30 minutes session lifetime. Supports audio/video input and tool calls. Docs: [ephemeral tokens](https://ai.google.dev/gemini-api/docs/live-api/ephemeral-tokens), example repo [sanjayojha/gemini-live-ephemeral-websocket](https://github.com/sanjayojha/gemini-live-ephemeral-websocket). Security note: tokens minted without `live_connect_constraints` let the client reconfigure the session, including tools ([report](https://cybersecuritynews.com/gemini-live-voice-session-flaw/)). Lock model, config, and tools when minting.
* **Claude:** no first-party browser realtime voice API that I could verify. Pattern is Web Speech API (already used for voice commands) for STT, Claude Messages API for reasoning/tool calls, `speechSynthesis` for TTS. Higher latency than native speech-to-speech but simple and works today.
* Both realtime options need a token-minting backend, so they belong in v2/v3 once a Worker exists. Realtime voice is attractive for "talk to the robot" UX but does not change the control-loop safety story.

---

## 2. Phone page as a bridge (relay architecture)

### 2.1 Why

The phone is the only BLE endpoint. Any external agent (Claude Desktop, Claude Code, Home Assistant automations, a cron job) must reach it. Phones have no stable inbound address, so the page must dial **out**.

### 2.2 Hard browser constraints

* **Mixed content:** an HTTPS page may only open `wss://` (TLS) WebSockets. `ws://192.168.x.x` to a LAN box is blocked. So every relay option needs a valid TLS cert on a public or tunnel hostname. (Chrome's Private Network Access rules add further friction for public-to-private requests.)
* **Web Bluetooth needs a secure context and a user gesture for `requestDevice()` [verified]** ([Chrome docs](https://developer.chrome.com/docs/capabilities/bluetooth)). After a page reload, reconnect needs either a new tap or `navigator.bluetooth.getDevices()` (permission-persistent reconnect; check current Chrome Android support).
* **Background / screen off:** the page must stay in the foreground with the screen on. Screen Wake Lock works on Android Chrome but is released when the tab is hidden [verified] ([Chrome wake lock docs](https://developer.chrome.com/docs/capabilities/web-apis/wake-lock)). Background tabs get timer throttling and may be frozen; WebSocket and GATT behavior in the background is not something to rely on **[estimate]**. Design for: phone on a stand/robot, page visible, wake lock held, and auto-stop if `visibilitychange` goes hidden.
* GATT `gattserverdisconnected` event must trigger "robot offline" status upstream ([sample](https://googlechrome.github.io/samples/web-bluetooth/automatic-reconnect.html)).

### 2.3 Relay options

| Option | How it works | Pros | Cons | Fit |
|---|---|---|---|---|
| **Cloudflare Worker + Durable Object** | One DO instance per robot holds the phone's WebSocket; agents POST commands or connect their own WS; DO forwards and returns results. Use the WebSocket Hibernation API. | Free plan includes DOs (100k requests/day, 13,000 GB-s/day); hibernating idle sockets are not billed for duration; no charge for outgoing WS messages [verified] ([DO pricing](https://developers.cloudflare.com/durable-objects/platform/pricing), [Workers pricing](https://developers.cloudflare.com/workers/platform/pricing/)). Same Worker can host the remote MCP server and the LLM key proxy. Global TLS for free. | You write ~200 lines of code. Auth is your job. | **Best overall** |
| **Hosted MQTT over WSS** (HiveMQ Cloud Serverless, EMQX Cloud) | Page uses mqtt.js on `wss://<cluster>:8884/mqtt`; topics `robot/<id>/cmd`, `robot/<id>/state`. | HiveMQ free tier: 100 connections, 10 GB/month, TLS-only, WSS on 8884 [verified] ([HiveMQ WS support](https://www.hivemq.com/blog/websocket-support-for-hivemq-cloud-basic/), [product](https://www.hivemq.com/products/mqtt-cloud-broker/)). Pub/sub, retained state, last-will for "phone offline". Home Assistant speaks MQTT natively. | Broker credentials must be in the page (scope with per-client ACLs: phone can only sub `cmd`/pub `state`). Request/response over MQTT is clunky. MCP server still needed separately. | Good, especially with HA |
| **Home Assistant WebSocket API** | Page connects to `wss://<ha-host>/api/websocket`, authenticates, and either fires/listens to events or acts as a client for a custom integration. | Already running. HA auth exists (long-lived tokens valid 10 years, or OAuth-style refresh flow which is preferred in browsers) [verified] ([HA auth API](https://developers.home-assistant.io/docs/auth_api/), [home-assistant-js-websocket](https://github.com/home-assistant/home-assistant-js-websocket)). HA has an official **MCP Server integration** exposing Assist-exposed entities to MCP clients [verified] ([HA MCP server](https://www.home-assistant.io/integrations/mcp_server/)), plus community [ha-mcp](https://github.com/homeassistant-ai/ha-mcp). | Needs HA reachable over public TLS (Nabu Casa, Cloudflare Tunnel, Tailscale Funnel). A long-lived admin-ish token in a phone browser is high-value. Modeling "robot" as HA entities (e.g. `script.robot_move`, `sensor.robot_distance`) via events or MQTT takes design work. CORS: HA needs the GitHub Pages origin in `http.cors_allowed_origins` for REST; WS is generally fine. | Good if HA is the hub you want |
| **ngrok / Tailscale Funnel to a home server** | Home box runs a WS relay; tunnel gives it a public HTTPS hostname. | Tailscale Funnel is free for personal use, fixed `*.ts.net` hostname, ports 443/8443/10000 only [reported]; ngrok free accounts get a persistent dev domain since Jan 2026 [reported] ([comparison](https://localxpose.io/blog/ngrok-vs-tailscale), [ngrok vs Tailscale](https://ngrok.com/compare/tailscale)). | Home box must be up; you still write the relay. Funnel endpoints are public, so app-level auth is still required. | OK if a Pi already runs 24/7 |

Plain Tailscale (non-Funnel) does not help the browser page directly: Chrome on Android can reach tailnet hosts only if the Tailscale app is running, and you would still need a valid cert (Tailscale can issue `*.ts.net` certs, so this is workable for a private-only setup).

### 2.4 MCP server in front of the relay

```
Claude Desktop / Claude Code / claude.ai connector
        | MCP (Streamable HTTP, OAuth or bearer token)
        v
Remote MCP server (Cloudflare Worker)  --same DO-->  phone page WS  --BLE-->  mBot2
```

* Cloudflare documents building **remote MCP servers on Workers with OAuth** [verified] ([guide](https://developers.cloudflare.com/agents/model-context-protocol/guides/remote-mcp-server/), [blog](https://blog.cloudflare.com/model-context-protocol/), example [kentcdodds/cloudflare-remote-mcp-server](https://github.com/kentcdodds/cloudflare-remote-mcp-server)). A remote MCP server can be added as a custom connector in Claude apps, so the robot becomes usable from claude.ai on any device, not only a desktop.
* Alternative: a local stdio MCP server (Python FastMCP or TS SDK) on the laptop that connects to the relay. Simpler auth (secret in a local config), but only works where that config lives.
* Tools should map 1:1 to the page's JSON command schema (section 6), with the same bounds enforced again on the page.

### 2.5 Existing projects with this pattern

* **[kumavulp/mcp-ble-bridge](https://github.com/kumavulp/mcp-ble-bridge)** [verified]: almost exactly this design. Cloud Node.js MCP server (Streamable HTTP/SSE) to WebSocket to a `bridge.html` page on the phone to Web Bluetooth to BLE toy. Device logic in the page, server device-agnostic. Has long-press E-stop, auto-stop after duration, wake lock. **Explicitly has no authentication** ("Anyone who knows the URL can send commands"); OAuth endpoints are stubs. Good reference, do not copy the auth model.
* **[iOS BLE-MCP Bridge](https://glama.ai/mcp/servers/zhy1369800/ble-mcp-bridge)**: same idea via a Safari extension on iPhone.
* **[WebMCP local relay](https://docs.mcp-b.ai/packages/webmcp-local-relay/reference)**: exposes tools defined in a browser tab to desktop MCP clients via a localhost WebSocket relay. Interesting if Claude Desktop and the page ran on the same machine, which is not this case (phone), but shows the "page declares tools" idea.
* Browser-control bridges ([robhicks/browser-mcp-bridge](https://github.com/robhicks/browser-mcp-bridge), [claude-browser-bridge](https://github.com/softwaresoftware-dev/claude-browser-bridge)) use the same outbound-WS-to-MCP shape.

---

## 3. Alternative: bypass the phone (PC / Raspberry Pi with BLE)

* **[DrorSh/mbot_python](https://github.com/DrorSh/mbot_python)** [verified]: Python + `bleak`, speaks Makeblock's "f3/f4" Live Mode framing; subscribe notify `ffe2`, write `ffe3`; each frame wraps a Python snippet the CyberPi evaluates and can return JSON. API: `forward/backward/turn/drive/stop`, `distance()`, `battery()`, IMU, LEDs, sound. Motion calls with `secs` auto-stop on the robot, "safe even if a Bluetooth packet drops". Protocol lineage: Hulupeep/mbot_ruvector and Makeblock's `makeblock` pip package.
* Architecture: Pi (or laptop) near the robot runs `bleak` + a FastMCP server. Local stdio MCP for Claude Desktop/Code on the same box, or Streamable HTTP over Tailscale for remote clients. HA integration is easy (Pi publishes MQTT).

| Criterion | Phone bridge | Pi/PC direct |
|---|---|---|
| Hardware | none extra | Pi with BLE, near the robot (BLE range ~10 m indoors) |
| Uptime | needs page open, screen on, foreground | headless, systemd, survives reboots |
| Reconnect | user gesture or `getDevices()` | fully automatic |
| Camera | phone camera rides on robot (if mounted) | needs Pi camera or none |
| Network | outbound `wss` only, works anywhere | home LAN; remote via Tailscale |
| Code reuse with v1 page | high (same command bus) | protocol must be reimplemented in Python (already done by mbot_python) |
| Auth surface | public relay | can stay entirely private on tailnet |
| Latency | LLM -> cloud -> phone -> BLE (+50-200 ms relay) **[estimate]** | LLM -> Pi -> BLE (lower) |

Note: the robot accepts one BLE central at a time **[estimate, typical for CyberPi]**, so phone and Pi cannot both be connected; pick one per session.

Verdict: the Pi route is more robust for unattended "AI agent drives the robot" use. The phone route wins for portability, camera-on-robot, and zero extra hardware. Keeping the JSON command schema identical in both lets you switch later; an MCP server can target either backend.

---

## 4. Camera and vision

### 4.1 Mechanics

* `getUserMedia({video: {facingMode: "environment"}})` on the phone, draw to canvas, JPEG at 320 to 640 px, send to a vision-capable LLM as an image block. Phone mounted on the robot gives a first-person view.
* Per decision: capture + encode ~50 ms, upload of a 30 to 80 KB JPEG over mobile ~100 to 500 ms, model inference with an image ~1 to 4 s **[estimate]**. Realistic loop: **one decision every 2 to 5 s**.
* Gemini Live and OpenAI Realtime accept streaming video/frames, which lowers per-frame overhead, but the model's reaction time is still in the hundreds of ms to seconds.

### 4.2 Closed-loop limits

* At mBot2 speeds (tens of cm/s), a 3 s decision latency means 50 cm or more of travel per decision if moving continuously. Hence the pattern must be **stop, look, decide, short bounded move, stop**.
* Research on VLM-driven control confirms even ~300 to 400 ms end-to-end latency on a local RTX 4090 is too slow for tight feedback control, and the field uses fast/slow splits ("think at 5 Hz, act at 20 Hz") ([arXiv 2607.15621](https://arxiv.org/pdf/2607.15621), [arXiv 2609.22925](https://arxiv.org/html/2609.22925v1), [VLCP code replanning](https://arxiv.org/pdf/2608.16978)).
* Practical hobby design for "drive to the red ball":
  * **Slow loop (LLM, ~0.3 Hz):** decides goal and strategy, picks target ("red ball, left of center"), handles failure.
  * **Fast loop (in-page, 10 to 30 Hz):** local detector tracks the target and steers. Options: simple HSV color threshold on a canvas (enough for a red ball), or MediaPipe Tasks Vision object detector in the browser. Ultrasonic distance gates the final approach and stops at a threshold.
  * Expose this as a single tool: `approach_target({color|label, stop_distance_cm, timeout_s})`, executed locally and reporting a result. The LLM orchestrates; it does not steer frame by frame.
* Cost: images are several hundred to ~1.5k input tokens each **[estimate]**; a 2-minute session at one frame per 3 s is ~40 images. Fine for hobby use, but cap it.

---

## 5. Safety and control loop

The core mismatch: LLM decisions take seconds; the robot moves continuously. Assume any message can be late, duplicated, or lost, and any agent can be wrong.

### 5.1 Command design

* **Bounded, timed primitives only.** `move(distance_cm ≤ 50, speed ≤ 60%)`, `turn(deg ≤ 180)`, `drive(left, right, duration_ms ≤ 2000)`. No unbounded "go forward". Enforce limits in the page (and again in the MCP server).
* **Auto-stop on the robot side.** Use the timed form (as mbot_python's `secs` does) so a dropped follow-up never leaves motors running.
* **Obstacle guard in the page:** poll ultrasonic during forward motion; abort if distance < threshold. This overrides the LLM.
* **Idempotency:** every command has an `id`; the page drops duplicates and stale commands (`ts` older than e.g. 2 s, or `expires_at` passed).

### 5.2 Watchdogs

* **Stop-on-silence (relay):** the relay sends heartbeats; if the page misses N heartbeats or the WS closes, stop motors immediately.
* **Stop-on-hidden:** `visibilitychange` to hidden, wake lock lost, or `pagehide` triggers stop.
* **BLE disconnect:** report offline upstream; on reconnect, robot starts in stopped state; never replay queued commands after reconnect.
* **Agent session timeout:** remote control lease (e.g. 60 s, renewable). No lease, no motion.

### 5.3 Priority and E-stop

* Command bus has priority lanes: `stop` and E-stop bypass the queue, cancel the current command, and flush pending ones.
* Local always wins: on-screen E-stop (big button, also the volume key if feasible), voice "stop" handled locally without the LLM, and local UI input preempts the remote agent (remote is a "lease" the local user can revoke).
* Consider a "remote armed" toggle on the page that must be on for any remote command to execute.

### 5.4 Rate limits

* Page: max N motion commands per second (e.g. 5), max total distance per minute.
* Relay/MCP: per-token request limits; cap LLM calls per session to bound cost and runaway agent loops.

### 5.5 Authentication on the relay

* Phone to relay: per-robot secret, entered once and stored on the phone, exchanged for a short-lived token. Never put it in the repo or in a URL query string that gets logged.
* Agent to relay: separate credentials from the phone's (an agent should not be able to impersonate the robot). For remote MCP, use OAuth (Cloudflare's Workers OAuth provider) or at minimum a bearer token; restrict by Cloudflare Access if only you use it.
* Unique unguessable robot IDs are not auth. The mcp-ble-bridge "no auth" model is the anti-pattern.
* Optional: sign commands (HMAC with the shared secret plus timestamp) so the page verifies origin even if the relay is compromised.
* MQTT: per-client ACLs (phone may only subscribe to its `cmd` topic and publish `state`).
* Prompt injection: if the agent reads untrusted content (web pages, camera text), bounds and the local E-stop are what keep this safe. Do not give the agent tools beyond the robot.

---

## 6. Staged roadmap

### v1: manual + voice (current)

Ship the controller, but build it as if remote control already existed.

Design decisions that keep v2/v3 cheap:

1. **One JSON command schema**, versioned, used by every input source:
   ```json
   {"v":1,"id":"c-0193","ts":1759420000123,"src":"ui|voice|llm|remote",
    "cmd":"move","args":{"distance_cm":20,"speed":40},"timeout_ms":3000}
   ```
   Commands: `move`, `turn`, `drive` (timed), `stop`, `estop`, `led`, `beep`, `read_sensors`. Responses:
   ```json
   {"v":1,"id":"c-0193","ok":true,"state":"done|aborted|rejected","reason":null,
    "telemetry":{"distance_cm":42.1,"battery_pct":78}}
   ```
   Keep a JSON Schema file (`commands.schema.json`); it later doubles as the MCP/LLM tool definitions.
2. **Transport-agnostic command bus** in the page: `bus.submit(cmd) -> Promise<result>`; producers (buttons, voice parser, later LLM loop and WS relay) only call `submit`; one consumer (robot driver) owns BLE. Telemetry published as events (`bus.on('telemetry')`).
3. **Robot driver interface** (`connect`, `execute(cmd)`, `stop()`, `telemetry$`) with the mBot2 BLE implementation behind it, plus a **simulator driver** for testing agents without the robot.
4. **Safety layer between bus and driver** from day one: limits, timed primitives, obstacle guard, dedupe/expiry, priority stop, stop-on-hidden. Same code protects voice, LLM and remote.
5. **Voice parser outputs commands, not actions.** Then an LLM can replace or augment the parser (free-form speech to tool calls) without touching the driver.
6. **Status model:** explicit states (`disconnected`, `connected_idle`, `executing`, `estopped`, `low_battery`) emitted on the bus; this becomes the relay's presence/state topic.
7. **Settings slot for credentials** (API key, relay URL, robot secret) with opt-in persistence, strict CSP, no third-party scripts.
8. **Wake lock + reconnect UX** (`getDevices()` where available).

### v2: bridge

* Cloudflare Worker + Durable Object relay (or HiveMQ/HA if preferred). Page adds a `RemoteTransport` that subscribes to commands and forwards them into `bus.submit`, and publishes results/telemetry.
* Auth (section 5.5), heartbeats, lease, "remote armed" toggle.
* Test with a CLI client (`websocat` or a small script) before any AI.
* Optional: HA integration via MQTT topics or HA events so automations can trigger "patrol", "come to kitchen".

### v3: AI agent

* **v3a in-page agent:** Claude Messages API with BYOK (or via the Worker proxy), tools generated from `commands.schema.json`, Web Speech in/out. Add `run_sequence` and a short "robot rules" system prompt.
* **v3b remote MCP server** in the same Worker, exposing the same tools to Claude Desktop/Code/claude.ai. Tools return telemetry after each action.
* **v3c vision:** camera frames on demand (`look()` tool returns a JPEG), then the local fast-loop `approach_target` tool.
* **v3d realtime voice** (OpenAI Realtime or Gemini Live via ephemeral tokens from the Worker) if the UX is worth the extra moving parts.
* Parallel option: Pi + mbot_python + FastMCP for unattended operation, reusing the same schema.

---

## 7. Top risks

1. **Phone page lifecycle:** screen off, tab switch, OS battery optimization, or a reload drops BLE and the relay. Mitigate with wake lock, stop-on-hidden, clear offline status, and consider the Pi route for unattended use.
2. **Unauthenticated or weakly authenticated relay:** strangers driving the robot. Auth both legs, separate credentials, leases.
3. **API key leakage** from `localStorage` on a shared `github.io` origin. Spend caps, CSP, separate origin, or proxy.
4. **Latency-induced overshoot / collisions:** solved only by bounded primitives, robot-side auto-stop, and local obstacle guard; not by a better prompt.
5. **Protocol fragility:** the f3/f4 Live Mode protocol is unofficial and reverse-engineered; firmware updates could change it. Pin the CyberPi firmware once it works.
6. **Provider CORS changes** (OpenAI/Gemini browser access is not officially supported and has changed before).

---

## Sources

* Anthropic CORS header: https://simonwillison.net/2024/Aug/23/anthropic-dangerous-direct-browser-access/
* Browser Claude client without SDK: https://dev.to/ferhatatagun/building-a-streaming-claude-client-in-the-browser-without-the-sdk-5f80
* OpenAI TS SDK (`dangerouslyAllowBrowser`): https://developers.openai.com/api/reference/typescript
* OpenAI browser CORS reports: https://dev.to/tracepilot_2841f1db6718a1/that-openai-call-from-your-browser-is-failing-heres-why-3p3c , https://community.openai.com/t/chat-completions-api-endpoint-down-blocked-any-web-browser-request/1362527
* OpenAI Realtime WebRTC: https://developers.openai.com/api/docs/guides/realtime-webrtc , https://developers.openai.com/api/docs/guides/realtime
* Gemini CORS reports: https://discuss.ai.google.dev/t/gemini-api-cors-error-with-openai-compatability/58619
* Gemini Live ephemeral tokens: https://ai.google.dev/gemini-api/docs/live-api/ephemeral-tokens ; constraint flaw: https://cybersecuritynews.com/gemini-live-voice-session-flaw/
* Web Bluetooth (Chrome): https://developer.chrome.com/docs/capabilities/bluetooth ; reconnect sample: https://googlechrome.github.io/samples/web-bluetooth/automatic-reconnect.html
* Screen Wake Lock: https://developer.chrome.com/docs/capabilities/web-apis/wake-lock
* Cloudflare DO pricing: https://developers.cloudflare.com/durable-objects/platform/pricing ; Workers pricing: https://developers.cloudflare.com/workers/platform/pricing/
* Cloudflare remote MCP: https://developers.cloudflare.com/agents/model-context-protocol/guides/remote-mcp-server/ , https://blog.cloudflare.com/model-context-protocol/ , https://github.com/kentcdodds/cloudflare-remote-mcp-server
* HiveMQ Cloud WebSockets: https://www.hivemq.com/blog/websocket-support-for-hivemq-cloud-basic/ , https://www.hivemq.com/products/mqtt-cloud-broker/
* Home Assistant auth/WebSocket: https://developers.home-assistant.io/docs/auth_api/ , https://github.com/home-assistant/home-assistant-js-websocket
* Home Assistant MCP Server: https://www.home-assistant.io/integrations/mcp_server/ , https://github.com/homeassistant-ai/ha-mcp
* Tunnels: https://localxpose.io/blog/ngrok-vs-tailscale , https://ngrok.com/compare/tailscale
* MCP to Web Bluetooth bridge: https://github.com/kumavulp/mcp-ble-bridge ; iOS variant: https://glama.ai/mcp/servers/zhy1369800/ble-mcp-bridge ; WebMCP relay: https://docs.mcp-b.ai/packages/webmcp-local-relay/reference
* mBot2 Python BLE: https://github.com/DrorSh/mbot_python
* VLM control latency: https://arxiv.org/pdf/2607.15621 , https://arxiv.org/html/2609.22925v1 , https://arxiv.org/pdf/2608.16978
