# v0.9 plan (proposed): floor colour layer and colour landmarks

Date: 2026-10-11. Status: idea agreed in discussion, nothing built yet. State of the app: v0.8.3, field-tested (see `research/07-hardware-session.md`).

## Idea

The Quad RGB sensor under the front of the robot sees a spot of about 1 cm per probe (four probes side by side: L2, L1, R1, R2). It cannot replace the ultrasonic sensor, but it is precise and reliable where the robot drives, so it can add a floor layer to the map and act as a position check.

## 1. Floor colour layer

- While driving, read the floor colour of the four probes together with the other sensors and paint the map cells under each probe (pose plus probe offset) in a separate layer that can be switched on and off.
- Group similar colours into floor types automatically (for example parquet, carpet); the owner can name them.
- Uses: wheel calibration per floor type (carpet slips differently), and knowing in advance where a route crosses a floor transition, which is where heading drift came from in the field.

## 2. Colour patterns as landmarks

| Type | Effect | Expected reliability |
|---|---|---|
| Coloured tape markers (for example red at the start, blue at doorways) | A unique colour gives an exact position check (1 to 2 cm) when driven over; the times at which the four probes reach the tape edge also give the crossing angle, so heading too | Very high |
| Floor-type boundaries (parquet to carpet) | Crossing a known, straight boundary fixes the position across it, like a wall | Good if straight and clearly different |
| Natural patterns (rug patterns, tiles) | Match the colour sequence along the path against the stored floor layer | Weaker: repeats, small spot |

## Limits

- Only where the robot drives: landmarks help only on routes that cross them.
- A colour read is another Bluetooth round trip (about 120 to 250 ms; a script may be at most 200 bytes), so about 3 to 4 readings per second next to the other sensors: one reading every 3 to 5 cm at normal speed, enough for markers of about 5 cm.
- Colours depend on light; use the sensor's own fill light for steady readings.

## Proposed order

1. Floor test on the robot: drive slowly across a parquet/carpet boundary and over coloured tape; measure how stable the readings are and how long a read takes.
2. Floor colour layer on the map with automatic floor types.
3. Tape markers as landmarks: position and heading correction when driven over.
4. Floor-type boundaries as additional position checks.

## Open items carried over

- Git history still contains the old commit email and one device address in an earlier version of a research note; removing them needs a history rewrite and force push (not done, waiting for the owner's decision).
- Remaining UAT on the robot: AI conversation with a real API key, crash contacts with a low box, saved maps with relocalization.
