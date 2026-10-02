# 02: Web Bluetooth and Speech Recognition on Android Chrome

Research date: 2026-10-02. Target: static SPA on GitHub Pages (HTTPS), Chrome on Android (Pixel 7 Pro, Galaxy S20 class), mBot2 over BLE GATT, German (and English) voice commands.

Method: official docs (developer.chrome.com, MDN, W3C/WICG explainers, blink-dev intents) plus **Chromium source code on `main`** (fetched 2026-10-02 from the GitHub mirror `chromium/chromium`). Source code is the strongest evidence for current behaviour, so it is cited where docs are vague. Anything I could not confirm is marked **UNVERIFIED**.

---

## TL;DR: hard constraints and blockers

| # | Constraint / blocker | Severity |
|---|---|---|
| 1 | `requestDevice()` needs a user gesture (tap) and a secure context. GitHub Pages HTTPS is fine. | Hard rule, easy |
| 2 | **No persistent Bluetooth permission on Chrome stable.** `getDevices()` and the "new permissions backend" are still flag-only in Chromium `main` (Oct 2026). After every page reload you need a new chooser tap. Reconnect without chooser only works with the same in-memory `BluetoothDevice` object. | Major UX limit |
| 3 | Service-UUID filters only match **advertised** UUIDs. If the mBot2 does not advertise `0xffe1`, filter by name prefix (or `acceptAllDevices`) and list `0xffe1` in `optionalServices`. | Must design for |
| 4 | **One GATT operation at a time per characteristic** in Chrome Android; overlapping writes reject with "GATT operation already in progress". You must serialize writes yourself. | Must design for |
| 5 | Chrome Android **requests MTU 517 on every connect** (source verified), but the effective MTU depends on the robot. Web Bluetooth exposes no MTU getter. Hard cap: 512 bytes per `writeValue*`. | Design for 20-byte safe default |
| 6 | **Speech recognition is aborted whenever the page becomes hidden on Android** (source verified). Screen off or app switch = recognition stops. Use Screen Wake Lock to keep the screen on. | Major, mitigable |
| 7 | **On-device Web Speech (`processLocally`), `quality` and contextual biasing (`phrases`) do NOT work on Android.** Android Chrome routes recognition to Google's Android `SpeechRecognizer` (Speech Services by Google) instead. | Major for offline/accuracy |
| 8 | Android `continuous = true` is quirky: interim guesses arrive as **final** results (source verified), sessions end on silence/periodically, each restart may play a **system beep**. | Major UX issue, mitigable |
| 9 | Hidden tabs get timer throttling and may be frozen; your JS command loop and any "AI bridge" WebSocket cannot be relied on in background. Keep the page foreground with wake lock. | Must design for |
| 10 | iOS: no Web Bluetooth in Safari or any WebKit browser; only third-party Bluefy-style browsers. Firefox: never. | Platform limit |

---

## A. Web Bluetooth on Android Chrome

### A1. Requirements

**Secure context.** Web Bluetooth is "made available only to secure contexts". https://developer.chrome.com/docs/capabilities/bluetooth

**User gesture.** "Discovering Bluetooth devices with `navigator.bluetooth.requestDevice` must be triggered by a user gesture such as a touch or a mouse click." (same URL). In practice: call `requestDevice()` synchronously inside the click handler, before any other `await`.

**Android version.** Chrome docs list "Chrome for Android 6.0+" (i.e. Android Marshmallow+). https://developer.chrome.com/docs/capabilities/bluetooth . Web Bluetooth is `stable` for Android in Blink's `runtime_enabled_features.json5` (Chromium `main`, `name: "WebBluetooth"`, `"Android": "stable"`). https://github.com/chromium/chromium/blob/main/third_party/blink/renderer/platform/runtime_enabled_features.json5

**OS permissions (verified in Chromium source).**
- Chrome's manifest declares `BLUETOOTH_SCAN` with `usesPermissionFlags="neverForLocation"` and `BLUETOOTH_CONNECT`. https://github.com/chromium/chromium/blob/main/chrome/android/java/AndroidManifest.xml
- `PermissionUtil.needsNearbyDevicesPermissionForBluetooth()`: on Android 12+ (SDK S) Chrome needs the **Nearby devices** runtime permission (`BLUETOOTH_SCAN` + `BLUETOOTH_CONNECT`).
- `PermissionUtil.needsLocationServicesForBluetooth()`: **Location services are only required below Android 12**. Comment in source: "Location services are not required on Android S+ to use Bluetooth if the application has Nearby Devices permission and has set the neverForLocation flag". https://github.com/chromium/chromium/blob/main/components/permissions/android/java/src/org/chromium/components/permissions/PermissionUtil.java
- The chooser dialog itself shows inline links to grant the missing permission or turn on location (`BluetoothChooserDialog.checkLocationServicesAndPermission`). https://github.com/chromium/chromium/blob/main/components/permissions/android/java/src/org/chromium/components/permissions/BluetoothChooserDialog.java
- Pixel 7 Pro (Android 13+) and Galaxy S20 (shipped Android 10, upgradable to Android 13) both qualify for the Android 12+ path if updated. If an S20 is still on Android 10/11, Location permission for Chrome **and** system Location must be on.

**Bluetooth on.** The chooser reacts to adapter power state (`WebBluetoothServiceImpl::AdapterPoweredChanged`). https://github.com/chromium/chromium/blob/main/content/browser/bluetooth/web_bluetooth_service_impl.cc . `navigator.bluetooth.getAvailability()` reports whether an adapter exists (MDN, see Permissions-Policy link below).

**Filters vs `acceptAllDevices`.**
- `filters` can match `services`, `name`, `namePrefix`, `manufacturerData`; `acceptAllDevices: true` shows everything and is discouraged. https://developer.chrome.com/docs/capabilities/bluetooth
- With name/manufacturer filters or `acceptAllDevices` "you will also need to define the `optionalServices` key to be able to access any services not included in a service filter." (same URL). So **`0xffe1` must be in `optionalServices`** unless it is in a `services` filter.
- Real-world mBot gotcha: a project that added Web Bluetooth for Makeblock BLE modules found that `filters: [{services: [uuid]}]` "only matches a peripheral's advertising payload, not its GATT table" and had to switch to `acceptAllDevices`. https://github.com/tatiang/mbot-vr/pull/6 . Whether the mBot2/CyberPi advertises `0xffe1` is **UNVERIFIED**; test with nRF Connect. Recommended: `filters: [{namePrefix: '<observed name prefix>'}], optionalServices: [0xffe1]`, fallback `acceptAllDevices`.
- `0xffe1` is **not** on the Web Bluetooth GATT blocklist (checked the registry file). https://github.com/WebBluetoothCG/registries/blob/master/gatt_blocklist.txt
- Same mbot-vr PR reports the Makeblock profile as service `ffe1`, notify `ffe2`, write `ffe3`, and Makeblock's "f3/f4" Live Mode framing for CyberPi. Treat as **UNVERIFIED** until sniffed on your unit (the author themselves flags uncertainty).

### A2. GitHub Pages, Permissions-Policy, iframes

- GitHub Pages serves HTTPS, which satisfies the secure-context requirement. No special headers are needed: the `bluetooth` Permissions-Policy default allowlist is `self`, so the top-level page and same-origin iframes are allowed. https://developer.mozilla.org/en-US/docs/Web/HTTP/Reference/Headers/Permissions-Policy/bluetooth
- **Cross-origin iframes are blocked by default**; the embedder must use `<iframe allow="bluetooth; microphone">`. If disallowed, `requestDevice()` rejects with `SecurityError`. (same MDN URL). GitHub Pages cannot set custom response headers, but you don't need any for a top-level page.
- Note: all repos under `<user>.github.io` share one origin (`github.io` is on the Public Suffix List, so each user subdomain is its own site). Bluetooth and microphone grants, localStorage etc. are per origin, so other project pages on the same account share them. https://publicsuffix.org/list/public_suffix_list.dat (search for `github.io`).

### A3. Writes: types, size, throughput, serialization

**API.** `writeValueWithResponse()` and `writeValueWithoutResponse()` are the explicit variants; `writeValue()` is deprecated and lets the UA choose. In Chromium Android the two map directly to Android `WRITE_TYPE_DEFAULT` and `WRITE_TYPE_NO_RESPONSE` (`BluetoothRemoteGattCharacteristicAndroid::WriteRemoteCharacteristic`). https://github.com/chromium/chromium/blob/main/device/bluetooth/bluetooth_remote_gatt_characteristic_android.cc . Check `characteristic.properties.writeWithoutResponse` / `.write` to see what `ffe3` supports (**UNVERIFIED** for mBot2).

**Max bytes.**
- Hard cap 512 bytes per write: the browser process kills the renderer for `value.size() > 512` (`web_bluetooth_service_impl.cc`, both characteristic and descriptor writes). https://github.com/chromium/chromium/blob/main/content/browser/bluetooth/web_bluetooth_service_impl.cc
- **MTU: Chrome Android requests ATT MTU 517 immediately after connecting** (`ChromeBluetoothDevice.java`: "Try requesting for a larger ATT MTU ... `bluetoothGatt.requestMtu(517)`", then service discovery). https://github.com/chromium/chromium/blob/main/device/bluetooth/android/java/src/org/chromium/device/bluetooth/ChromeBluetoothDevice.java . (Older community statements that Web Bluetooth "never negotiates MTU" are outdated for Android.)
- Android 14+ also forces 517 on the first `requestMtu` call. https://developer.android.com/about/versions/14/behavior-changes-all
- The **result** depends on the peripheral (the mBot2 BLE stack may accept far less). Web Bluetooth still has no API to read the negotiated MTU (open since 2018). https://github.com/WebBluetoothCG/web-bluetooth/issues/383 , MTU-change request https://github.com/WebBluetoothCG/web-bluetooth/issues/284
- Practical rule: default ATT MTU 23 gives 20 payload bytes. Chunk to **20 bytes** unless you've measured a bigger MTU with the robot; this is what the mbot-vr project does ("20-byte write chunking (conservative default ATT MTU)"). https://github.com/tatiang/mbot-vr/pull/6 . Behaviour of Android when a write-without-response exceeds MTU-3 (truncation vs error) is **UNVERIFIED**; avoid it.

**Throughput.** Drive commands are a few bytes at 10 to 20 Hz, far below any limit. Rough native BLE numbers for orientation: MTU 23 ~2.5 KB/s, MTU 517 ~45 KB/s with write-without-response (native apps, not Web Bluetooth). https://uynguyen.github.io/2026/04/12/Reliable-BLE-Data-Transfer-MTU-Throughput-Chunking/ . Web Bluetooth adds IPC hops; no authoritative Web Bluetooth Android throughput benchmark found (**UNVERIFIED**).

**"GATT operation already in progress".**
- Chromium Android rejects a read or write on a characteristic while another read/write on it is pending (`if (read_pending_ || write_pending_) ... GattErrorCode::kInProgress`). Same file as above. Note that even write-without-response holds `write_pending_` until Android's `onCharacteristicWrite` callback.
- Chromium team position: Chrome "leaves the work to web applications to handle this case and queue". https://issues.chromium.org/issues/40446211 . Earlier Chromium group thread: on Android, parallel reads/writes fail; `getPrimaryService*`/`getCharacteristic*` can run in parallel. https://groups.google.com/a/chromium.org/g/web-bluetooth/c/QQP6ExsKHRI
- Android's own stack is single-operation per connection (`mDeviceBusy`), so also serialize across characteristics (e.g. a `startNotifications()` on `ffe2` concurrent with a write on `ffe3`). https://dev.to/ble_advertiser/solving-the-android-ble-gatt-race-condition-reliable-sequential-operations-with-kotlin-coroutines-k04
- Pattern (promise-chain queue with coalescing so a backlog never builds up):

```js
let chain = Promise.resolve();
let pendingDrive = null;            // latest drive command wins
function enqueue(fn) { chain = chain.then(fn, fn); return chain; }
function sendDrive(bytes) {          // high-rate, droppable
  const first = pendingDrive === null;
  pendingDrive = bytes;
  if (first) enqueue(async () => { const b = pendingDrive; pendingDrive = null; await ch.writeValueWithoutResponse(b); });
}
function sendStop(bytes) {           // never dropped, jumps the coalescer
  pendingDrive = null;
  return enqueue(() => ch.writeValueWithResponse(bytes));
}
```

### A4. Notifications

- `await ch.startNotifications(); ch.addEventListener('characteristicvaluechanged', e => e.target.value /* DataView */)`. https://developer.chrome.com/docs/capabilities/bluetooth
- Call `startNotifications()` before starting the write queue, or route it through the same queue (it writes the CCCD descriptor, which is a GATT op). Reassemble frames across notifications yourself (mbot-vr does "notification reassembly"). https://github.com/tatiang/mbot-vr/pull/6
- Known spec issue: `startNotifications` can error or tie up GATT operations. https://lists.w3.org/Archives/Public/public-web-bluetooth-log/2024Apr/thread.html (issue #466). More evidence to serialize everything.

### A5. Lifecycle: background, screen off, lock, reconnect

**What Chrome does on hide (source verified).** `WebBluetoothServiceImpl::OnVisibilityChanged`: when HIDDEN or OCCLUDED it calls only `ClearAdvertisementClients()`; `OnWebContentsLostFocus` likewise. **The GATT connection is not explicitly dropped** by Chrome when the tab is hidden. https://github.com/chromium/chromium/blob/main/content/browser/bluetooth/web_bluetooth_service_impl.cc

**But in practice on Android:**
- While a site is connected, Chrome Android shows an ongoing notification ("connected to a Bluetooth device", tap to bring tab to front) via `BluetoothNotificationService`. It is a plain started service with `setOngoing(true)`, not a declared foreground service, so it does not protect Chrome from being killed. https://github.com/chromium/chromium/blob/main/chrome/browser/bluetooth/android/java/src/org/chromium/chrome/browser/bluetooth/BluetoothNotificationManager.java , manifest entry in https://github.com/chromium/chromium/blob/main/chrome/android/java/AndroidManifest.xml
- Hidden pages: timers aligned to 1 s, and after 5 min "intensive throttling" to once per minute; pages can be **frozen** (timers and fetch callbacks don't run). https://developer.chrome.com/blog/timer-throttling-in-chrome-88 , https://developer.chrome.com/docs/web-platform/page-lifecycle-api
- If Android kills Chrome's process (battery optimization, memory), the connection is gone. https://developer.android.com/develop/connectivity/bluetooth/ble/background
- Exact time-to-disconnect after screen off / lock on Pixel and Samsung is **UNVERIFIED**; expect "connection may survive briefly, but your JS can't drive the robot". Treat screen-off as "robot must stop".

**Safety implication.** Implement a **dead-man's switch**: the page sends drive commands with a short validity (e.g. robot stops if no command for 300 to 500 ms), or send explicit stop on `visibilitychange` → hidden and on `pagehide`. Whether mBot2 firmware supports a timeout natively is **UNVERIFIED**; a CyberPi-side script could implement it.

**Screen Wake Lock.** Supported in Chrome Android since 84. Requires secure context and a visible page; the lock is released automatically when the tab is hidden, so re-acquire on `visibilitychange`. Requests can be rejected (power-save mode, low battery). https://developer.chrome.com/docs/capabilities/web-apis/wake-lock , https://developer.mozilla.org/en-US/docs/Web/API/Screen_Wake_Lock_API

**Reconnect without a new chooser.**
- Within the same page lifetime: keep the `BluetoothDevice` object, listen for `gattserverdisconnected`, call `device.gatt.connect()` again with exponential backoff. No prompt. Official samples: https://googlechrome.github.io/samples/web-bluetooth/automatic-reconnect-async-await.html
- After reload / new visit: **a new chooser tap is required on stable Chrome**. In Chromium `main` (Oct 2026): `WebBluetoothGetDevices` and `WebBluetoothWatchAdvertisements` are `status: "experimental"` (runtime_enabled_features.json5), and `kWebBluetoothNewPermissionsBackend` is `FEATURE_DISABLED_BY_DEFAULT` (content_features.cc). https://github.com/chromium/chromium/blob/main/content/public/common/content_features.cc . Implementation-status doc agrees (flags `#enable-experimental-web-platform-features` + `#enable-web-bluetooth-new-permissions-backend`). https://github.com/WebBluetoothCG/web-bluetooth/blob/main/implementation-status.md . The 2020 Intent to Ship never completed. https://groups.google.com/a/chromium.org/g/blink-dev/c/lqCQ63CTKEQ
- For a personal/dev phone you can enable both flags in `chrome://flags` to get `navigator.bluetooth.getDevices()` + `device.watchAdvertisements()` auto-reconnect. Not viable for other users.
- Implication for the app: make the page a **single long-lived page** (no navigation, SPA routing via hash/state), so the `BluetoothDevice` survives.

### A6. OS pairing and multiple centrals

- **OS-level pairing is not needed** for an unencrypted custom GATT service; Web Bluetooth connects directly from the chooser. Chromium only triggers pairing when a characteristic requires authentication (Android stack handles it). Whether pre-bonding in Android settings causes trouble with mBot2 is **UNVERIFIED**; recommended: do not pair in Android settings, and if previously paired and connection fails, "Forget" the device. Makeblock's own troubleshooting page does not cover this. https://support.makeblock.com/hc/en-us/articles/15891560491671-2-mBot2-Won-t-Connect-to-the-Mobile-Device-via-Bluetooth
- **One central at a time**: typical BLE peripherals stop advertising when connected, so a second central (mBlock app, PC with mBlock, another phone) can't find or connect. This is very likely for mBot2 but **UNVERIFIED**. Plan: disconnect mBlock/app first; show a hint in the UI if the robot doesn't appear in the chooser.
- Also: mBlock 5 web itself uses browser "direct connection" (Web Bluetooth/Web Serial) for CyberPi, which confirms the approach is feasible on Chrome. https://support.makeblock.com/hc/en-us/articles/19412317319191-Introduction-to-Direct-Connection-of-mBlock-5-on-the-web

### A7. Browser support matrix

| Browser | Web Bluetooth GATT | Source |
|---|---|---|
| Chrome Android | Yes (shipped, no flag) | https://github.com/WebBluetoothCG/web-bluetooth/blob/main/implementation-status.md |
| Samsung Internet (Android) | Yes, since 6.4 | same; https://caniuse.com/mdn-api_bluetooth_requestdevice |
| Edge / Opera Android | Chromium-based; Opera Android "since 46". Edge Android **UNVERIFIED** | implementation-status.md |
| Firefox (all) | No, "no plan to support" | implementation-status.md |
| Safari / all iOS browsers | No (WebKit engine rule) | implementation-status.md |
| iOS workaround | Bluefy (free, last update Jan 2026, v3.9.3) | https://www.appbrain.com/appstore/bluefy-web-ble-browser/ios-1492822055 |

Note for Samsung Internet: it is a separate Chromium fork with its own speech stack; Web Speech support/quality there is **UNVERIFIED**. Recommend Chrome.

---

## B. Speech recognition on Android

### B1. Web Speech API status on Chrome Android (2025 to 2026)

**How Android Chrome actually does it (source verified).** Chrome Android does **not** use Chrome's desktop cloud/on-device (SODA) pipeline. `SpeechRecognitionImpl.java` wraps Android's `SpeechRecognizer`:
- Provider on Android 12+: package `com.google.android.tts` (**Speech Services by Google**); below Android 12: the Google app (`com.google.android.googlequicksearchbox`, min version). If no Google provider exists, Web Speech is unavailable. https://github.com/chromium/chromium/blob/main/content/public/android/java/src/org/chromium/content/browser/SpeechRecognitionImpl.java
- `lang` → `EXTRA_LANGUAGE`, `interimResults` → `EXTRA_PARTIAL_RESULTS`, `continuous` → `android.speech.extra.DICTATION_MODE`. Chrome does not set `EXTRA_PREFER_OFFLINE`. Whether audio goes to Google's cloud or is handled by the on-device Google model therefore depends on the Speech Services app and installed offline language packs (**UNVERIFIED** per device; assume network needed).
- Galaxy S20: Samsung's default voice input is irrelevant; Chrome requires Speech Services by Google to be installed and enabled (normally present on GMS devices, **UNVERIFIED** on a given unit).

**API surface.** Unprefixed `SpeechRecognition` is `stable` in Blink (alias `webkitSpeechRecognition`). Use `window.SpeechRecognition || window.webkitSpeechRecognition`. https://github.com/chromium/chromium/blob/main/third_party/blink/renderer/modules/speech/speech_recognition.idl

**`lang = 'de-DE'`.** Passed straight to Android; German is supported by Google's recognizer (well established, no single doc located for the Android service language list: **UNVERIFIED** formally, but low risk). English: set `en-US`. One recognizer instance has one `lang`; you cannot recognise both at once. Practical approach: run `de-DE` and add English command words to your fuzzy matcher (Google usually transcribes "stop", "go" fine inside German).

**On-device (`processLocally`, `available()`, `install()`, `quality`, `phrases`).**
- Shipped on desktop in Chrome 139 (Aug 2025). https://developer.chrome.com/blog/new-in-chrome-139 , API: https://github.com/WebAudio/web-speech-api/blob/main/explainers/on-device-speech-recognition.md , MDN: https://developer.mozilla.org/en-US/docs/Web/API/Web_Speech_API/Using_the_Web_Speech_API
- Intent to Ship: "Initially supported on Windows, Mac, and Linux with ChromeOS support to follow"; language packs ~60 MB. https://groups.google.com/a/chromium.org/g/blink-dev/c/VNOok2dbmHM/m/gwbtzV-lAQAJ
- **May 2026, Chrome team (Evan Liu): "Android & ChromeOS currently do not support on-device Web Speech."** https://groups.google.com/a/chromium.org/g/blink-dev/c/P8P-x7AnC6I
- Contextual biasing (`phrases`, Chrome 142): Blink flag `WebSpeechRecognitionContext` is stable only on Win/Mac/Linux, experimental on ChromeOS, **off on Android**; and the code says "Only on device speech recognition supports contextual biasing" (throws `phrases-not-supported` otherwise). https://github.com/chromium/chromium/blob/main/third_party/blink/renderer/modules/speech/speech_recognition.cc , https://developer.chrome.com/blog/chrome-142-beta
- Conclusion: on Android, **do not rely on `processLocally` or `phrases`**. Feature-detect and ignore.
- `start(MediaStreamTrack)` (feed your own mic track) is stable only on desktop, experimental elsewhere (flag `MediaStreamTrackWebSpeech`). runtime_enabled_features.json5 link above.

**Continuous, interim, duration, silence.**
- In continuous mode Chrome Android ignores `onEndOfSpeech` and, critically, "In continuous mode, Android's recognizer sends final results as provisional", so Chrome **marks them final** (`provisional = false`). Every partial guess shows up as a new `isFinal` result. SpeechRecognitionImpl.java (above).
- Real-world reports: Chrome Android "reports every in-progress guess ... as a final result (usually with confidence 0)"; mobile browsers re-deliver the same final, and fire `onend` even with `continuous=true`. https://github.com/jaypetez/ideaforge/pull/49 , https://github.com/Markvs923/voicenote/pull/1
- Chrome ends sessions on its own after silence and periodically ("roughly every minute"). https://www.assemblyai.com/blog/speech-recognition-javascript-web-speech-api . Long-standing Chromium bug "Continuous speech recognition is broken on Android". https://issues.chromium.org/issues/40324711 (login-gated, could not read current status: **UNVERIFIED**). Exact no-speech timeout on Android (commonly reported 3 to 8 s) is **UNVERIFIED**.
- **Page hidden ⇒ abort (Android only, source verified):** `SpeechRecognition::PageVisibilityChanged() { #if BUILDFLAG(IS_ANDROID) if (!GetPage()->IsPageVisible()) abort(); }`. Screen off, app switch, tab switch all kill listening.

**Beeps.** Restarting the Android recognizer plays the start/stop earcon each time ("beeping after each sentence"). https://www.assemblyai.com/blog/speech-recognition-javascript-web-speech-api (summary of community reports), https://webreflection.medium.com/taming-the-web-speech-api-ef64f5a245e1 . No web API to suppress it. Muting notification/system volume is a reported user workaround (**UNVERIFIED** which stream on Android 13/14).

**Latency.** Not documented for Android. Typical observation: first partial in a few hundred ms, final after end-of-speech detection (~0.5 to 1.5 s) (**UNVERIFIED**). Use interim results + your own keyword matching to act early, especially for "stopp".

**Network.** MDN: in Chrome, recognition is server-based and "will not work offline". https://developer.mozilla.org/en-US/docs/Web/API/SpeechRecognition . On Android it depends on the Google service (see above).

**Permissions / HTTPS.** Microphone permission prompt from Chrome (site-level) plus Chrome needs the Android RECORD_AUDIO permission. Mic access requires a secure context (getUserMedia rule). Whether Chrome Android refuses Web Speech on plain HTTP is **UNVERIFIED** but irrelevant on GitHub Pages. Errors to handle: `not-allowed`, `no-speech`, `network`, `audio-capture`, `aborted` (mapping in SpeechRecognitionImpl.java: e.g. `ERROR_NETWORK` → network, `ERROR_SPEECH_TIMEOUT` → no-speech).

### B2. Android quirks and the recommended loop

1. Treat `continuous=true` on Android as unreliable. Two workable patterns:
   - **Short sessions**: `continuous=false, interimResults=true`, restart in `onend` while "listening mode" is on. Each restart may beep.
   - **continuous=true** with dedup: rebuild transcript from `event.results` each event, drop consecutive identical finals, ignore replay of last committed phrase after restart (pattern from the PRs above). For a command app you only care about the *newest* text anyway: match commands on `event.results[event.results.length-1][0].transcript` and debounce identical commands within ~1 s.
2. Restart with a small delay (100 to 300 ms) and a backoff on `network`/`not-allowed` errors to avoid tight loops (a tight `start()` after `onend` can throw `InvalidStateError` "recognition has already started" per speech_recognition.cc).
3. Re-arm on `visibilitychange` → visible (because Android aborts on hide).
4. **Mic contention**: Web Speech on Android runs the mic in the Google service. Using `getUserMedia` at the same time (e.g. for a VU meter, Vosk, Porcupine) is likely to conflict or starve one side (**UNVERIFIED**; avoid running both).

### B3. Web Bluetooth + speech recognition together

No documented conflict: BLE GATT runs in Chrome's browser process over the Bluetooth LE controller, speech runs via the Google speech service on the mic. Both require the page to be visible to be useful, which aligns. Caveat: a Bluetooth **audio** headset could change mic routing (SCO) and affect recognition (**UNVERIFIED**). Also both trigger permission UI; request Bluetooth first (needs gesture), then mic, in separate taps. Overall: **no known blocker, UNVERIFIED by an explicit source**; must be tested on the Pixel 7 Pro and S20.

### B4. Alternatives if Web Speech is poor

| Option | German | Size / footprint | Latency (rough) | Licence / cost | Notes |
|---|---|---|---|---|---|
| **Push-to-talk + cloud STT** (e.g. OpenAI `gpt-4o-mini-transcribe`) | Yes | none on device | ~0.5 to 2 s round-trip (**UNVERIFIED**) | ~$0.003/min (https://developers.openai.com/api/docs/models/gpt-4o-mini-transcribe ; price via https://costgoat.com/pricing/openai-transcription ) | Needs API key, which **cannot be stored in a static GitHub Pages site**; needs a small proxy (Cloudflare Worker etc.). Good accuracy, no beep, uses `getUserMedia` + `MediaRecorder`. |
| **Vosk (vosk-browser, WASM)** | Yes, `vosk-model-small-de-0.15` ~45 MB, WER 13.75 | 45 MB download, runs in a Web Worker | Streaming partials, low hundreds of ms (**UNVERIFIED** on phone) | Apache-2.0 (lib), models Apache-2.0 (**UNVERIFIED** per model) | **Supports a grammar** (`new model.KaldiRecognizer(sampleRate, grammar)`) = restricted vocabulary, ideal for commands. Repo last push Dec 2025, lib v0.0.8. https://github.com/ccoreilly/vosk-browser , model info https://alphacephei.com/vosk/models |
| **Moonshine Voice** (JS/WASM) | Yes: German Tiny Streaming 34M params (WER 12.0%), Small Streaming 123M (7.5%) | tens of MB (**UNVERIFIED** exact) | Built for streaming/low latency | **MIT** (streaming models, all languages) | https://github.com/moonshine-ai/moonshine , model table https://github.com/moonshine-ai/moonshine/blob/main/docs/models/available-models.md . Newer and promising; Android Chrome performance **UNVERIFIED**. |
| **Whisper via transformers.js / whisper.cpp WASM** | Yes (multilingual) | tiny ~40 MB, base ~200 MB | Batch, not streaming; seconds on phone CPU; WebGPU helps | MIT | WebGPU on Chrome Android 121+, Android 12+, Qualcomm/ARM GPUs. https://developer.chrome.com/blog/new-in-webgpu-121 . transformers.js latest 4.3.0 (Sep 2026). https://github.com/huggingface/transformers.js . Overkill for 6 commands. |
| **Picovoice Porcupine (wake word) + Rhino (speech-to-intent), Web SDK** | Yes (`rhino_params_de.pv` exists) | small (~1 to few MB, **UNVERIFIED**) | very low, on-device | Apache-2.0 SDK, but needs AccessKey; free tier for small personal use, paid tiers expensive (secondary sources: https://www.hackster.io/news/picovoice-launches-completely-free-usage-tier-for-offline-voice-recognition-for-up-to-three-users-e1eafbc97bb0 ; current terms **UNVERIFIED**, pricing page did not render) | Rhino is exactly "small command grammar → intent". https://github.com/Picovoice/rhino |
| **TensorFlow.js speech-commands** | No German base vocab (English 18 words: up/down/left/right/go/stop...) | small (few MB, **UNVERIFIED**) | very low | Apache-2.0 | Transfer learning in browser lets you train "vorwärts/zurück/links/rechts/stopp" from your own samples. https://github.com/tensorflow/tfjs-models/tree/master/speech-commands |

Recommendation: start with Web Speech (`de-DE`) for v1 (zero download). Keep a **fallback path**: Vosk-browser with a grammar, or Moonshine German Tiny, for offline/no-beep. Push-to-talk cloud STT only once there's a backend (also relevant for the AI bridge).

### B5. Command grammar and safety design

- **Small fixed vocabulary**, each command phonetically distinct: `vor/vorwärts/los/geradeaus`, `zurück/rückwärts`, `links`, `rechts`, `stopp/stop/halt/anhalten`, optional `schneller/langsamer`, `drehen`. English aliases: `forward/go`, `back`, `left`, `right`, `stop`.
- **Normalize** transcripts: lowercase, strip punctuation, map umlauts (`ü→ue`, `ä→ae`, `ö→oe`, `ß→ss`) and also match the non-umlaut spellings recognizers sometimes emit ("zuruck", "vorwarts").
- **Fuzzy match** per token: Levenshtein distance ≤1 for words ≥4 letters, or a phonetic code (Kölner Phonetik is designed for German). Known mis-hearings to add as aliases: "rechts"/"recht"/"rex", "links"/"link", "stopp"/"stop"/"top", "halt"/"hallt"/"alt", "zurück"/"zu rück". (Aliases list is heuristic, **UNVERIFIED**; tune with logs.)
- Scan **interim** results token by token and act on the **last** command word in the utterance, with a ~1 s debounce for repeats (needed because Android re-delivers finals).
- **Stop has absolute priority**: if any stop alias appears anywhere in any interim or final result, send stop immediately, bypassing the coalescing queue and clearing pending drive commands (see A3 code). Never require confidence for stop (Android often reports 0 confidence).
- **Fail-safe**: robot stops when (a) page hidden/pagehide, (b) BLE disconnect, (c) recognition error/end while moving if you use "move until stop" semantics, (d) watchdog timeout. Prefer **timed moves** ("vorwärts" = drive 1 s) over latched motion for voice control, so a missed "stopp" is not dangerous.
- Big on-screen STOP button always visible; volume key or tap anywhere = stop is a cheap extra.

---

## Implications for the later "AI bridge"

Not researched in depth here, but constraints from above: the page can only make outbound connections (WebSocket/WebRTC/fetch to a relay), must stay **visible** (Android aborts speech and throttles timers when hidden; WebRTC data channels are exempt from intensive throttling per https://developer.chrome.com/blog/timer-throttling-in-chrome-88), and must hold the wake lock. Remote commands go through the same serialized write queue and the same safety layer (stop priority, watchdog).

---

## Source list

- Chrome Web Bluetooth guide: https://developer.chrome.com/docs/capabilities/bluetooth
- Implementation status: https://github.com/WebBluetoothCG/web-bluetooth/blob/main/implementation-status.md
- Chromium `web_bluetooth_service_impl.cc`: https://github.com/chromium/chromium/blob/main/content/browser/bluetooth/web_bluetooth_service_impl.cc
- Chromium `ChromeBluetoothDevice.java` (MTU 517): https://github.com/chromium/chromium/blob/main/device/bluetooth/android/java/src/org/chromium/device/bluetooth/ChromeBluetoothDevice.java
- Chromium `bluetooth_remote_gatt_characteristic_android.cc`: https://github.com/chromium/chromium/blob/main/device/bluetooth/bluetooth_remote_gatt_characteristic_android.cc
- Chromium `PermissionUtil.java`, `BluetoothChooserDialog.java`: https://github.com/chromium/chromium/tree/main/components/permissions/android/java/src/org/chromium/components/permissions
- Chromium `content_features.cc` (permissions backend disabled): https://github.com/chromium/chromium/blob/main/content/public/common/content_features.cc
- Blink runtime features: https://github.com/chromium/chromium/blob/main/third_party/blink/renderer/platform/runtime_enabled_features.json5
- Chromium `SpeechRecognitionImpl.java`: https://github.com/chromium/chromium/blob/main/content/public/android/java/src/org/chromium/content/browser/SpeechRecognitionImpl.java
- Blink `speech_recognition.cc` / `.idl`: https://github.com/chromium/chromium/tree/main/third_party/blink/renderer/modules/speech
- GATT in progress bug: https://issues.chromium.org/issues/40446211 ; parallel ops thread: https://groups.google.com/a/chromium.org/g/web-bluetooth/c/QQP6ExsKHRI
- MTU issues: https://github.com/WebBluetoothCG/web-bluetooth/issues/383 , https://github.com/WebBluetoothCG/web-bluetooth/issues/284
- Android 14 MTU: https://developer.android.com/about/versions/14/behavior-changes-all
- Permissions-Policy bluetooth: https://developer.mozilla.org/en-US/docs/Web/HTTP/Reference/Headers/Permissions-Policy/bluetooth
- GATT blocklist: https://github.com/WebBluetoothCG/registries/blob/master/gatt_blocklist.txt
- Reconnect samples: https://googlechrome.github.io/samples/web-bluetooth/automatic-reconnect-async-await.html
- Wake Lock: https://developer.chrome.com/docs/capabilities/web-apis/wake-lock
- Page lifecycle / throttling: https://developer.chrome.com/docs/web-platform/page-lifecycle-api , https://developer.chrome.com/blog/timer-throttling-in-chrome-88
- mBot BLE in browser: https://github.com/tatiang/mbot-vr/pull/6 ; mBlock direct connection: https://support.makeblock.com/hc/en-us/articles/19412317319191-Introduction-to-Direct-Connection-of-mBlock-5-on-the-web
- On-device Web Speech: https://groups.google.com/a/chromium.org/g/blink-dev/c/VNOok2dbmHM/m/gwbtzV-lAQAJ , https://groups.google.com/a/chromium.org/g/blink-dev/c/P8P-x7AnC6I , https://github.com/WebAudio/web-speech-api/blob/main/explainers/on-device-speech-recognition.md , https://github.com/WebAudio/web-speech-api/blob/main/explainers/contextual-biasing.md
- MDN SpeechRecognition: https://developer.mozilla.org/en-US/docs/Web/API/SpeechRecognition , guide: https://developer.mozilla.org/en-US/docs/Web/API/Web_Speech_API/Using_the_Web_Speech_API
- Android quirks: https://issues.chromium.org/issues/40324711 , https://webreflection.medium.com/taming-the-web-speech-api-ef64f5a245e1 , https://www.assemblyai.com/blog/speech-recognition-javascript-web-speech-api , https://github.com/jaypetez/ideaforge/pull/49
- Alternatives: https://github.com/ccoreilly/vosk-browser , https://alphacephei.com/vosk/models , https://github.com/moonshine-ai/moonshine , https://github.com/huggingface/transformers.js , https://github.com/Picovoice/rhino , https://github.com/tensorflow/tfjs-models/tree/master/speech-commands , https://developer.chrome.com/blog/new-in-webgpu-121
