# v0.4 plan: mapping and navigation

Date: 2026-10-03. Goal: the robot scans, remembers what it saw in a map, knows where it is, and drives to a goal around obstacles; it can explore unknown space and return to the start. Hardware facts that constrain the design are in `research/07-hardware-session.md` (200-byte script limit, imports reboot the robot, motors keep running after a disconnect, ~90 ms per query).

## Frames and units

- World frame = the robot's pose when the map was reset (start). Origin (0, 0) is the start position.
- **x** = cm to the right of the start, **y** = cm forward of the start.
- **heading** in degrees, 0 = start forward direction, **clockwise positive** (same sign as `turn`), normalised to -180..180.
- Moving `d` cm at heading `h`: `x += d * sin(h)`, `y += d * cos(h)`.
- Scan points from `js/scan.js` are `{ angle, cm }` relative to the heading at scan time; `cm` is null on a failed read, `>= 300` (`NO_ECHO_CM`) means nothing in range. The ultrasonic sensor sits about 6 cm ahead of the robot centre; the beam is roughly 15 to 20 degrees wide.

## Available building blocks (lead, done)

- Bus commands: `turn { deg, wait, speed }`, `straight { cm, wait, speed }` (gyro based on the robot), `read { sensor: 'distance' | 'yaw' }`, `stop`. `bus.stamped()` returns a `makeCommand` bound to the current stop generation so STOPP cancels multi-step tasks; use it for every navigation task.
- `bus.onCommand((cmd, result) => ...)` fires after every submitted command.
- `scan(bus, { steps, signal, makeCommand, onPoint })`, `findOpenings`, `describeScan`, `normAngle`, `NO_ECHO_CM` in `js/scan.js`.
- SimRobot (`js/robot-sim.js`) with obstacles, `timeScale`, `yaw()`; ground truth in `sim.state` (room coordinates, heading -90 = up). Use it for integration tests.

## Module contracts

### `js/pose.js` (agent "mapping")
```js
export class PoseTracker {
  constructor({ x = 0, y = 0, heading = 0 } = {})
  get pose()                       // { x, y, heading }
  reset(pose = { x: 0, y: 0, heading: 0 })
  applyTurn(deg)                   // heading += deg
  applyStraight(cm)                // move along heading
  applyDrive(leftRpm, rightRpm, dtSec)  // differential drive integration (wheel 6.5 cm, track 12 cm)
  correctHeading(yawDeg)           // optional gyro correction relative to the yaw at reset
  attach(bus)                      // subscribes to bus.onCommand: successful turn/straight update the pose; returns unsubscribe
  trail                            // array of past {x, y} (capped)
}
```

### `js/gridmap.js` (agent "mapping")
```js
export class GridMap {
  constructor({ cellCm = 5, sizeCm = 800 })     // square, centred on the origin
  clear()
  integrateScan(pose, points, { sensorOffsetCm = 6, beamDeg = 16, maxRangeCm = 250 })  // log-odds update: cells along each beam become free, the hit cell occupied; no-echo beams clear up to maxRange
  cell(x, y) -> 'unknown' | 'free' | 'occupied'
  isTraversable(x, y, { inflateCm = 14, allowUnknown = false })
  frontiers({ minCells = 3 }) -> [{ x, y, size }]   // clusters of free cells next to unknown
  forEachCell(fn(x, y, state, p))                  // for drawing
  get bounds()                                     // { minX, maxX, minY, maxY } of known cells
  describe(pose) -> string                         // compact text for the LLM (< 400 chars)
  toJSON() / static fromJSON()                     // optional persistence
}
```

### `js/planner.js` (agent "mapping")
```js
export function planPath(map, from, to, { inflateCm = 14, allowUnknown = true }) -> [{ x, y }] | null  // A* on the grid, 8-connected
export function simplifyPath(path, map, opts) -> [{ x, y }]                                              // line-of-sight pruning
export function pathToMoves(pose, path, { maxSegCm = 40 }) -> [{ turnDeg, cm, to: { x, y } }]
```

### `js/navigate.js` (agent "mapping")
```js
export class Navigator {
  constructor({ bus, map, pose, scan, onEvent, steps = 12, safetyCm = 20 })
  async scanHere({ signal })                 // scan, integrate into map, return { points }
  async goTo({ x, y }, { signal, tolCm = 10, maxLegs = 12 }) -> { ok, reached, pose, legs, note }
      // plan -> for each leg: turn (wait), read distance, shorten if needed, straight (wait), update pose;
      // rescan + replan when blocked or after unknown space; never drive closer than safetyCm to a reading
  async explore({ signal, maxMoves = 6 }) -> { ok, moves, frontiersLeft }   // frontier exploration
  async goHome({ signal })                   // goTo(0, 0) then turn to heading 0
  describe() -> string                       // pose + map summary for the LLM
}
```
Every bus command in a navigation task uses one `bus.stamped()` makeCommand created at the start of that task. Abort (signal) and stop-generation cancellation must leave the robot stopped.

### `js/mapview.js` (agent "mapview")
```js
export function drawMap(canvas, map, pose, { path, goal, frontiers, trail, lastScan, view } = {})  // top-down, start at centre or auto-fit to bounds, forward = up
export function screenToWorld(canvas, view, clientX, clientY) -> { x, y }                         // for tap-to-goal
export function fitView(canvas, map, pose) -> view
```
Colours from CSS custom properties (`--surface`, `--border`, `--text`, `--muted`, `--accent`, `--stop`, `--ok`) so light and dark mode work.

### AI tools (agent "mapview", `js/tools.js`)
New tools: `navigate_to { x_cm, y_cm }` (map frame), `explore_room { max_moves }`, `go_home {}`, `describe_map {}`. `scan_surroundings` also integrates into the map when a navigator is injected. `createToolExecutor` gains an optional `navigator` dependency; existing tools keep working without it. System prompt explains the frame (x right, y forward from the start).

## Integration (lead)

`index.html` / `app.js`: map canvas in the scan panel (tap = drive there), buttons Scannen / Erkunden / Nach Hause / Karte löschen, pose tracker attached to the bus, sim ground-truth overlay, AI tools wired.

## Tests

- Unit: pose math, grid updates (free/occupied/unknown), inflation, frontiers, A* (around a wall, no path, unknown handling), path simplification and moves.
- Integration (sim, `timeScale`): goTo around the sofa reaches within 15 cm without collision; explore increases known area and never collides; goHome returns within 15 cm; abort mid-leg stops the robot; tools navigate_to/describe_map with a mocked LLM.
- UAT additions in `docs/UAT-v0.4.md` (lead).
