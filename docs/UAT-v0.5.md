# User acceptance test, v0.5

Real mBot2 on the floor, 2 to 3 obstacles. Phone (Chrome) or desktop Chrome. First run **Sensor-Test** (Befehl senden) and paste the log, so the sensor names can be confirmed.

| # | Test | Pass criterion | Result |
|---|---|---|---|
| S1 | Sensor-Test, then turn the robot 90° clockwise by hand, Sensor-Test again | yaw changes by about ±90; encoders/acceleration answer (or show "--") | |
| S2 | Umgebung scannen → Kontinuierlich → Scannen | Robot spins once smoothly (about 10 s), map shows walls and objects densely | |
| S3 | Schrittweise 12 → Scannen | Still works as before | |
| Z1 | Map: pinch / mouse wheel | Zooms around the fingers / cursor | |
| Z2 | Map: drag | Pans; a short tap still sets a goal | |
| Z3 | Ganze Karte / Roboter folgen | Fits everything / keeps the robot centred | |
| N1 | Tap a goal behind an obstacle, then Nach Hause | Both routes avoid the obstacle (the old "home ignores obstacles" bug) | |
| C1 | Put a box in the path that the scan did not see (low or behind), let it drive into it | Crash/stall detected, robot stops, backs off, map shows the contact, it rescans and replans | |
| C2 | After C1 | Robot arrow on the map still matches reality within about 15 cm | |
| L1 | Drive around with the joystick for 1 minute, then Scannen | Map does not smear; position snaps back to the right place | |
| M1 | Kartenname "Wohnzimmer" → Speichern; Karte löschen; Laden | Map comes back | |
| M2 | After loading: move the robot somewhere else in the room, Scannen | Log says "Auf der Karte gefunden" with a sensible position; arrow is right | |
| M3 | Export → file; Import it | Same map | |
| A1 | Chrome menu → App installieren / Zum Startbildschirm | Icon on the home screen; opens full screen; Verbinden works | |
| R1 | GitHub README | Link to the app near the top | |
