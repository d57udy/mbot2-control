# Motion sensors for crash detection and odometry

Research date: 2026-10-03. Question: which CyberPi / mBot2 Live Mode calls give wheel encoder angle and speed, acceleration, gyro rate and yaw, with which units, ports and signs, so that `js/motion.js` can detect crashes and measure driven distance from the wheels instead of counting commanded moves.

No hardware was available for this note. Everything marked **UNVERIFIED** must be checked with the probe list in section 4 before the expressions go into `BLE_SENSORS` in `js/motion.js`.

## Sources

| Tag | Source |
|---|---|
| **[FW]** | CyberPiOS 44.01.011 image `repos/perfecxx/firmware/CyberPi_firmware_44_01_011-ht2.bin`, `strings` dump (`fw44.txt` in the session scratchpad). The robot runs 44.01.013 (research/07), same line |
| **[MB]** | PyPI `makeblock` 0.1.8, `makeblock/modules/cyberpi/api_cyberpi_api.py` (Makeblock's own host library; mirrors the on-robot API names and argument lists) |
| **[API]** | Makeblock "Python API Documentation for CyberPi" (yuque, PDF text in local copy `api.txt`) |
| **[PX]** | PerfecXX/mBot2 MicroPython examples (`example/micropython/cyberpi/05-Motion Sensing/*`, `mBot2/01-mBot2 Chassis/*`) |
| **[DS]** | DrorSh/mbot_python `docs/commands.md` |
| **[BB]** | Barry Butler, "mBot2 / CyberPi Python Code" v2.2 (local copy `manual.txt`) |

## 1. Names that exist in the firmware

From the [FW] qstr table (a name in the table proves it exists in the image, not its behaviour):

| Area | Qstrs (line in `fw44.txt`) |
|---|---|
| `mbot2.py` encoder motor API | `EM_set_power`, `EM_turn`, `EM_get_angle`, `EM_get_speed` (printed as `OEM_get_speed`, the `O` is the qstr length byte), `EM_get_power`, `EM_reset_angle`, `EM_lock` (19889-19897), plus `port_table`, `_get_port` (19874) and `__DISTANCE_TO_ANGLE_FOCTOR`, `__SPEED_RPM_TO_DPS_FACTOR` (19886-19888) |
| Shield driver below it | `encoder_motor_get_positon` (sic), `encoder_motor_get_speed`, `encoder_motor_get_power`, `encoder_motor_reset_position` (19940-19947) |
| CyberPi motion sensor | `get_acc`, `get_gyro`, `get_rotation`, `reset_rotation` (sorted ROM table, 13054, 13153, 13222, 14050), `driver.motion_sensor` with `get_shakeval`, `get_shake_strength`, `get_acceleration`, `get_roll`, `get_pitch`, `get_yaw`, `reset_yaw`, `get_gyroscope`, `reset_rotation`, `is_shake` (18086-18115), `__motion_is_shake`, `get_accel` (18278) |
| Robot-side clock | `timer`, `get_time` (could timestamp samples on the robot; not needed so far) |

## 2. Signatures, units and conventions

| Call | Signature [MB] | Unit / range | Notes |
|---|---|---|---|
| `mbot2.EM_get_angle(port)` | `EM_get_angle(self, port)` (api_cyberpi_api.py:1182) | wheel degrees, cumulative since boot or `EM_reset_angle` (**UNVERIFIED**, inferred from `encoder_motor_get_positon`) | 6.5 cm wheel: **0.0567 cm per wheel degree** (pi x 6.5 / 360) |
| `mbot2.EM_get_speed(port)` | `EM_get_speed(self, port)` (:1185) | RPM, signed (**UNVERIFIED**) | Same units as `drive_speed` |
| `mbot2.EM_get_power(port)` | (:1188) | -100..100 (**UNVERIFIED**) | A stalled wheel under closed-loop speed control drives power toward the limit; a possible extra stall signal |
| `mbot2.EM_reset_angle(port)` | (:1191) | | Not needed: `driveLeg` works with differences |
| `cyberpi.get_acc(axis)` | `get_acc(axis)` (:145), axis `'x'`,`'y'`,`'z'` [PX 06-get_accelerometer.py] | m/s², "measures Earth's gravity; on a desk z is -9.8" [API] | Board axes; which one points forward on the mBot2 is **UNVERIFIED**. `driveLeg` only uses the change of the vector, so the mapping does not matter |
| `cyberpi.get_gyro(axis)` | (:148) | deg/s [API] | Rate, not angle |
| `cyberpi.get_rotation(axis)` | (:151) | degrees, counterclockwise positive [API], accumulated since `reset_rotation` | Alternative heading source with a documented sign |
| `cyberpi.get_yaw()` | (:139) | degrees, about the z axis; "no compass is configured" [API], so it is integrated gyro, not a compass, and it drifts | Already used by `robot-ble.js`; sign and range **UNVERIFIED** |
| `cyberpi.get_shakeval()` | (:124) | 0..100 [BB manual.txt:784, API] | Computed on the robot at the IMU rate, so it may catch a short impact that a 6 Hz poll of `get_acc` misses (**UNVERIFIED**) |

Ports: the factory script in the image calls `mbot2.EM_stop("ALL")` ([FW] line 61946) and the module has `port_table` / `_get_port`, which suggests ports are names (`"EM1"`, `"EM2"`, `"ALL"`) translated by a table. mBlock-generated code is believed to use `"EM1"`; integer ports `1`/`2` may also be accepted. Ranked candidates below.

Signs: the motors are mirrored (research/05 section 2b): forward is EM1 positive, EM2 negative. Expect `EM_get_angle("EM1")` to grow and `EM_get_angle("EM2")` to shrink while driving forward. `makeBleSampler` negates EM2 when `sensors.mirrored` is true (default) and swaps when `sensors.swap` is true, matching `robot.wheels`.

The owner's "compass" is the IMU yaw. There is no magnetometer on the CyberPi [API], so yaw is drift-prone over minutes but fine over one leg (seconds).

## 3. Why a low-rate poll still catches crashes

Each Live Mode query costs about 90 ms (research/07), so one combined list per poll gives 5 to 8 samples per second. A wall impact lasts tens of milliseconds, so an acceleration sample will often miss it. `driveLeg` therefore combines several detectors, each of which works at low rate:

| Detector | Signal | Works without |
|---|---|---|
| Stall | encoder progress below 30 % of the commanded progress over 400 ms (wheels stopped against an obstacle) | accelerometer |
| Slip (off by default, `opts.slip`) | encoders progress more than 8 cm in the window but the ultrasonic range ahead shrinks by less than 15 % of that. False positives in the simulator when driving past a side object (the nearest echo in the cone stays at a constant range) | accelerometer |
| Heading | yaw change minus encoder-implied heading change above 12 degrees (glancing hit turns the robot; this is the owner's "compass vs counted wheel rotations") | accelerometer |
| Jolt | change of the acceleration vector between two samples above 6 m/s², or `get_shakeval()` above 40 | encoders |
| Obstacle | ultrasonic below `stopAtCm` | everything else |

With no encoders the leg falls back to time x commanded speed for distance and only the ultrasonic, yaw and jolt detectors run.

## 4. Probe list for the hardware session

Send each line as a single query (all are under 200 bytes, none imports anything). Stop at the first form that answers with a number.

| # | Expression | Expect |
|---|---|---|
| 1 | `[hasattr(mbot2,n) for n in ('EM_get_angle','EM_get_speed','EM_get_power','EM_reset_angle')]` | `[True, True, True, True]` |
| 2 | `[hasattr(cyberpi,n) for n in ('get_acc','get_gyro','get_rotation','get_shakeval','get_yaw')]` | all `True` |
| 3a | `mbot2.EM_get_angle("EM1")` | number (degrees) |
| 3b | `mbot2.EM_get_angle(1)` | number, if 3a errors |
| 3c | `mbot2.EM_get_angle("em1")` | number, if 3a and 3b error |
| 4 | `[mbot2.EM_get_speed("EM1"),mbot2.EM_get_speed("EM2")]` (port form from 3) | about `[0, 0]` at rest |
| 5 | `[cyberpi.get_acc(a) for a in 'xyz']` | magnitude about 9.8 at rest; note which axis carries gravity |
| 6 | `[cyberpi.get_gyro(a) for a in 'xyz']` | about `[0, 0, 0]` at rest |
| 7 | `[cyberpi.get_yaw(),cyberpi.get_rotation('z'),cyberpi.get_shakeval()]` | yaw, rotation, shake 0 |
| 8 | Lift the robot, send `mbot2.drive_speed(30,-30)`, then 4 a few times, then `mbot2.EM_stop()` | EM1 speed about +30, EM2 about -30; angles change with the same signs |
| 9 | On the floor: `mbot2.straight(20)`, then 3 for both ports | EM1 about +353 degrees, EM2 about -353 (20 cm / 0.0567) |
| 10 | Turn right with `mbot2.turn(90)`, then 7 | sign of yaw and of `get_rotation('z')` for a clockwise turn |
| 11 | Push the robot by hand into a wall while 7 and 5 are polled | size of the jolt and shake values |
| 12 | The full poll expression `makeBleSampler` builds (printed by `buildSampleExpr(SENSOR_EXPR)` in `js/motion.js`) | an 8 element list, about 90 ms round trip |

The combined poll in its full form (port form `"EM1"`) is 168 bytes:

```
(lambda c,m:[c.ultrasonic2.get(1),m.EM_get_angle("EM1"),m.EM_get_angle("EM2"),c.get_acc('x'),c.get_acc('y'),c.get_acc('z'),c.get_yaw(),c.get_shakeval()])(cyberpi,mbot2)
```

(the exact string comes from `buildSampleExpr`; the test suite checks that it stays under 200 bytes).

After the session, update `SENSOR_EXPR` (port form), `BLE_SENSORS` (which entries are confirmed) and `yawSign` in `js/motion.js`, and record the measured jolt and shake values to tune `MOTION.joltMs2` and `MOTION.shakeLimit`.

## 5. Field test v0.5.4: oscillating turns (2026-10-04)

Log: turns -72, -2, -44, -12, -149, then -69, -125, +104, -122, +104 with 20 to 30 cm legs, and a door frame hit that was not detected.

Reproduced in the simulator with `turnError: -2` (the blocking `mbot2.turn(deg)` rotating by -deg). Every other imperfection was on as well: integer, unbounded yaw, ±1° encoder noise and 200 ms sensor latency. The gyro keeps the estimate right, so each next plan asks to undo the last turn, and that undo runs backwards too. The simulated legs gave -149, +99, -167, +124, -145, +100, the same pattern as the log. With a correct `mbot2.turn` but up to ±10 % turn error, there is no oscillation. A second defect made it worse: the gyro reference (`yawRef`) was set at the first correction, after the first turn, so that turn's error stayed in every later heading.

Hardware check still open: send `mbot2.turn(90)` and read `cyberpi.get_yaw()` before and after. Clockwise positive means yaw rises by about 90.

Fixes in `js/navigate.js` and `js/motion.js`:
- **Turns:** they are closed-loop on the gyro by default (`turnInPlace`). The robot spins with `drive` frames and stops on yaw (±1.5°, correction passes at creep speed). If the robot turns the wrong way, it learns `spinSign`. Without a sampler, the robot falls back to `mbot2.turn`. With the gyro on, it learns the sign of that turn as well (`turnSign`).
- **Gyro reference:** `yawRef` is taken before the first motion of every task.
- **Scan matching:** the match moves only x/y while the gyro is on. Routine corrections are capped at 30 cm and 30°.
- **Glancing hits on straight legs:**

| Check | Trigger |
|---|---|
| Per-wheel stall | One wheel makes less than 30 % of its commanded progress over 400 ms, seen in two consecutive windows |
| Twist | Yaw changes more than 10° |
| Wheel | The heading implied by the encoder difference changes more than 8° |
