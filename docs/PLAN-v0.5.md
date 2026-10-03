# v0.5 plan: robust navigation, localization, map storage, installable app

Date: 2026-10-03. Owner feedback after the first real-robot navigation test:
1. Map cannot zoom or pan.
2. Going home ignores obstacles.
3. Crashes are not detected, and a crash ruins the position on the map.
4. Position comes only from counted moves; fuse scan matching, gyro and odometry.
5. Save and load maps; after loading, a scan should relocate the robot on the map.
6. README link to the page near the top.
7. Installable Android app (PWA).

Hardware constraints: `research/07-hardware-session.md` (200-byte script limit, imports reboot the robot, ~90 ms per query, motors keep running after a Bluetooth disconnect, blocking `turn`/`straight` occupy the robot until done). Frames: `docs/PLAN-v0.4.md` (x right, y forward from the start, heading clockwise from +y, degrees).

## Work split and file ownership

| Stream | Owner | Files |
|---|---|---|
| A. Localization, map quality, home bug | agent "localize" | `js/localize.js` (new), `js/gridmap.js`, `js/planner.js`, `test/localize.test.mjs`, `test/gridmap.test.mjs`, `test/planner.test.mjs` |
| B. Motion with crash detection | agent "motion" | `js/motion.js` (new), `js/navigate.js`, `js/robot-sim.js` (sensor emulation only), `research/08-motion-sensors.md`, `test/motion.test.mjs`, `test/navigate.test.mjs` |
| C. Map UI, storage, PWA, README | agent "appshell" | `js/mapview.js`, `js/mapcontrols.js` (new), `js/mapstore.js` (new), `manifest.webmanifest`, `sw.js`, `icons/*`, `README.md`, `test/mapview.test.mjs`, `test/mapstore.test.mjs` |
| D. Integration, hardware API discovery, docs | lead | `index.html`, `style.css`, `js/app.js`, `js/bus.js`, `js/robot-ble.js`, `docs/*` |

## Contracts

### A. `js/localize.js`
```js
// Correlative scan matching on the occupancy grid. Beams: [{ angle, cm }] relative to pose heading.
export function scoreScan(map, pose, points, opts) -> number           // likelihood that the beam endpoints hit occupied cells
export function matchScan(map, guess, points, { xyWindowCm = 40, xyStepCm = 5, angWindowDeg = 20, angStepDeg = 2 }) -> { pose, score, confidence }
export function relocalize(map, points, { angStepDeg = 5, xyStepCm = 10 }) -> { pose, score, confidence, runnerUp }  // global search over free cells, for loaded maps
export function fusePose(odomPose, matchResult, { yawDeg, odomWeight, minConfidence }) -> { pose, source }           // blend; reject low-confidence matches
```
Confidence must reflect ambiguity (best vs. runner-up score). Must run in well under 500 ms for an 800 x 800 cm map with 36 beams.

Also in A: investigate the "home ignores obstacles" report with a simulator reproduction (scan, drive somewhere, goHome along a route with a thin obstacle such as the table leg). Likely causes to check: wide free cones erasing previously seen obstacles, unknown cells being cheap, inflation too small for the real robot, start-cell exception. Fix it in gridmap/planner.

### B. `js/motion.js` and Navigator legs
```js
// Drives a leg with streamed drive_speed while polling sensors, instead of blocking straight().
export async function driveLeg(bus, { cm, speed, makeCommand, signal, sensors, onSample, stopAtCm = 20 }) -> { ok, droveCm, reason: 'done'|'obstacle'|'crash'|'stall'|'aborted', samples }
export function detectCrash(samples, expected) -> { crash: boolean, reason }
```
- `sensors` is a descriptor of read expressions so the lead can plug in the names confirmed on hardware (for example encoder angle, accelerometer, yaw). Until confirmed, defaults must be configurable and missing sensors must degrade gracefully (fall back to distance-only plus time-based progress).
- Poll at 5 to 8 Hz (one combined list expression under 200 bytes per poll), stop on: distance below `stopAtCm`, stall (encoder progress far below commanded), jolt (acceleration spike), heading divergence (yaw change while driving straight). On crash: stop, back off about 5 cm, report.
- Navigator: use `driveLeg` for straight legs (keep blocking `turn` for rotations, then verify with yaw if available); on crash or stall mark the obstacle in the map at the robot front, set `pose uncertain`, scan and relocalize via `matchScan` (import from `./localize.js`; agent A writes it concurrently with the contract above), then replan.
- SimRobot: emulate encoder angles, acceleration spikes on collision and yaw so tests can exercise crash detection.

### C. UI, storage, app
- `js/mapcontrols.js`: `attachMapControls(canvas, { getView, setView, onTap, onChange })` with wheel zoom, pinch zoom, drag pan, tap detection (movement < 8 px), plus `fit` and `follow` helpers. `mapview.js` view model gains zoom/pan without breaking `fitView`.
- `js/mapstore.js`: `saveMap(name, map, meta)`, `listMaps()`, `loadMap(name)`, `deleteMap(name)` in localStorage (compact encoding, e.g. quantised log-odds as base64), `exportMap(map, meta) -> Blob`, `importMap(file) -> { map, meta }`.
- PWA: `manifest.webmanifest` (name "mBot2 Control", short_name "mBot2", display standalone, theme/background colours, icons 192 and 512 PNG plus maskable), `sw.js` cache-first for app files with a versioned cache name, `icons/` generated (simple robot glyph). Must work under the GitHub Pages path `/mbot2-control/` (relative URLs, scope).
- README: link to https://d57udy.github.io/mbot2-control/ in the first lines, plus "install as app" instructions.

## Tests
Unit and simulator integration tests per stream; `npm test` must stay under about 15 s. Lead adds browser checks and `docs/UAT-v0.5.md`.
