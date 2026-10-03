# Continuous sweep scan

Date: 2026-10-04. Owner request: 16 distances are not enough and step-by-step rotation is slow. Can the robot rotate continuously and still get usable angular resolution?

Code: `sweepScan`, `resampleSweep`, `spinRpmForRate`, `encoderRotationDeg` in `js/scan.js`; tests in `test/scan.test.mjs`. The step scan (`scan()`) is unchanged.

## How it works

1. One `sample()` at rest gives the zero heading (yaw, encoders) and a first distance.
2. A streamed in-place spin starts: `drive { left: +rpm, right: -rpm }` (clockwise, same sign as `turn`; a negative `speedDegS` spins counterclockwise), re-sent every 300 ms. The first 300 ms and the last 15 degrees run at half speed (short ramp). Stamped commands (`bus.stamped()`) let a bus stop cut the stream; a cancelled drive ends the sweep with an AbortError.
3. Samples are taken back to back (one combined query per sample, about 90 ms each). Each sample adds `{ t, rotation }` to a track; each distance is placed at the rotation interpolated at `t - latencyMs`.
4. The sweep ends at 380 degrees of rotation (20 degrees overlap), on abort, or after `maxDurationMs` (20 s). A stop is always sent (`try`/`finally`), after the drive stream has settled so no late drive frame can restart the wheels. One more sample at rest measures the coast; `turnedDeg` is the net heading change normalised to -180..180 (about +20 to +30 after a clockwise sweep) and `totalTurnDeg` the unwrapped rotation (about 380 to 390). The robot is not turned back; the caller updates the pose heading by `turnedDeg`. Drive commands carry `args.leg = true` so the app skips its own drive integration for them.
5. Post-processing: angles normalised to -180..180, sorted, readings within 1 degree merged (lower median: of two readings the nearer one), readings of 2 cm or less dropped, 300 cm kept as "no echo".

Result: `{ points, durationMs, method: 'sweep', rotationSource: 'yaw'|'encoder'|'time', yawSign, turnedDeg, totalTurnDeg, samples, coverageDeg }`.

### Rotation sources

| Source | When | Notes |
|---|---|---|
| Gyro yaw | first sample has `yaw` | Unwrapped with the shortest delta, so the ±180 wrap does not matter. The firmware sign is unverified: the sign is detected once the raw yaw has moved 20 degrees, against the commanded direction (or against the encoders if present, which tells a reversed gyro apart from a robot that really turns the other way). Points are buffered until then. |
| Wheel encoders | no yaw, `encL`/`encR` present | rotation = (encL - encR) x D / (2 x track) degrees, wheel angles in degrees and forward-positive (the sampler undoes the mirrored motor). Derivation: (encL - encR) / 360 x pi D is the wheel path difference in cm; divided by the track it is the rotation in radians. |
| Commanded rate x time | neither | Integrates the drive commands actually sent. Ignores motor lag and slip; coarse (a few degrees, growing over the turn). |

Wheel speed for a rate: each wheel moves at v = rpm / 60 x pi D and the robot turns at 2v / track, so rpm = degS x track x 60 / (360 x D). With D = 6.5 cm and track = 12 cm, 45 deg/s needs 13.8 RPM (clamped to 5..40 RPM). The SimRobot uses the same model. The real track width is an estimate; with the gyro as source it only changes the actual spin rate, not the angles.

### Latency compensation

Distance and yaw are read in the same script, but the ultrasonic value is the result of the module's last ping and is older than the gyro value. `latencyMs` (default 45 ms, a guess to be calibrated) shifts each distance back in time; the angle is interpolated between the neighbouring rotation samples. At 45 deg/s an uncorrected 45 ms is a 2 degree bias in the spin direction; the bias is systematic, so it shows up as walls rotated clockwise when maps from clockwise sweeps are compared with step scans.

## Expected resolution and duration on hardware

About 90 ms per sample plus a drive frame (no reply, a write only) every 300 ms. Estimates, not measured yet:

| Speed | Degrees per sample | Readings per turn | Duration (380 degrees + ramp + stop) |
|---|---|---|---|
| 30 deg/s | 2.7 to 3 | about 120 | about 13.5 s |
| 45 deg/s (default) | 4 to 4.5 | about 85 | about 9 s |
| 60 deg/s | 5.4 to 6 | about 62 | about 7 s |

For comparison, the 12-step scan gives 12 readings with a blocking turn, a read and a 120 ms settle per step.

### Why 3 to 6 degrees spacing is enough

The ultrasonic beam is roughly 15 to 20 degrees wide (`docs/PLAN-v0.4.md`) and reports the nearest echo anywhere in that cone. Every reading is already a blur over 15 to 20 degrees, so a thin table leg shows up across a 15 to 20 degree arc regardless of how finely we sample. Sampling at a quarter to a third of the beam width (4 to 6 degrees) captures every feature the sensor can resolve and locates edges to within a sample step; finer sampling mainly adds redundancy. The rotation during one ping (sound flight under 20 ms for 3 m) is under 1 degree at 45 deg/s, so motion blur is negligible.

`resampleSweep(points, binDeg = 3)` gives evenly spaced bins (nearest echo per bin, `cm: null` for empty bins) for consumers that want uniform angles. `findOpenings` now weights each point by half the gap to its neighbours (capped), so it works on uneven dense points; `describeScan` summarises dense scans as 12 directions (nearest echo per 30 degree sector) so the LLM text stays short.

## Risks

- **Bluetooth drop mid-sweep**: there is no robot-side watchdog on firmware 44.01.013 (`research/07-hardware-session.md`), so the robot keeps spinning in place until power off. A spin in place is the least harmful motion (no travel), but the robot will not stop by itself. Mitigation options: a time-limited spin script if one fits in 200 bytes without imports, or the timed-burst safe mode.
- **Yaw drift**: the gyro drifts slowly (degrees per minute); over a 9 s sweep this is well under 1 degree. Larger issue: the sign and range of `cyberpi.get_yaw()` are unverified; the auto-detection covers the sign, the unwrap covers any ±180 range.
- **Wheel slip**: irrelevant with the gyro; with encoders, slip on smooth floors makes the robot turn less than the wheels say (angles stretched). The time fallback is worse.
- **Low RPM**: 13.8 RPM is slow for the encoder motors; if the spin is jerky, raise `speedDegS` to 60 (20.8 RPM).
- **Bus guard**: `drive` with left + right = 0 is never blocked by the obstacle guard, so close obstacles do not stop the spin.
- **Sensor dropouts**: missing yaw samples are interpolated across; missing distances leave gaps that `resampleSweep` reports as `null`.

## Hardware verification steps

1. Place the robot 50 to 80 cm from a wall with a clear surround and one narrow object (for example a bottle) at a known direction, such as 90 degrees right.
2. Run the step scan (12 steps) and note readings at 0, 90, 180 and -90.
3. Run `sweepScan(bus, { sample: makeBleSampler(robot), makeCommand: bus.stamped() })` with defaults. Check in the log: `rotationSource` is `yaw`, `yawSign` (record it, then fix the default in the sampler if it is -1), `samples` about 85, `durationMs` about 9 s, `coverageDeg` 360, `totalTurnDeg` about 380 to 390 and `turnedDeg` matching the physical end heading (mark the floor).
4. Compare: the sweep's nearest reading per 30 degree sector should match the step scan within a few cm; the bottle should appear centred at 90 degrees within ±5.
5. Latency calibration: run one sweep clockwise and one with a negative `speedDegS` (counterclockwise) at 60 deg/s. The bottle centre differs by 2 x rate x (true latency - latencyMs); set `latencyMs` so both agree.
6. Repeat at 30 and 60 deg/s; check the bottle's angular position is stable.
7. Abort test: press Stop mid-sweep; the robot must stop within one frame and the scan must report an abort, not partial results.
8. Encoder fallback: with a sampler that reads encoders but not yaw, repeat step 3 and compare `turnedDeg` with the floor mark (shows slip).
9. Disconnect test only with a hand ready on the power switch: confirm the spin continues after a link loss (expected) to document the risk.
