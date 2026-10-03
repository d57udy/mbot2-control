# User acceptance test, v0.4: mapping and navigation

Real mBot2 on the floor with 2 to 3 obstacles (box, chair legs, wall). Speed **Normal** unless noted. Record PASS/FAIL and notes.

| # | Test | Pass criterion | Result |
|---|---|---|---|
| N1 | Gyro: send `py:cyberpi.get_yaw()`, turn the robot by hand 90° clockwise, read again | Value changes by about +90 (note sign and range) | |
| N2 | `{"cmd":"turn","args":{"deg":90}}` at Langsam and Schnell | Turns about 90° both times, visibly different speed | |
| N3 | Umgebung scannen → Scannen | Map shows walls/objects roughly where they are; robot arrow faces the right way | |
| N4 | Tap a free spot about 1 m ahead on the map | Robot drives there in short legs, ends within about 15 cm | |
| N5 | Tap a spot behind an obstacle | Robot drives around it, never touches it | |
| N6 | Put a box in the path while it drives | It stops before the box, rescans, finds another way or reports blocked | |
| N7 | Erkunden | Map grows into unexplored areas; no collisions; stops when nothing new is reachable | |
| N8 | Nach Hause | Returns to the start within about 20 cm and faces the start direction | |
| N9 | STOPP during any of the above | Stops at once; no further movement afterwards | |
| N10 | Kompass-Korrektur on, repeat N4 and N8 | Same or better end position | |
| N11 | AI: "Erkunde das Zimmer" / "Fahr zurück zum Start" / "Was siehst du auf deiner Karte?" | Uses explore_room / go_home / describe_map and acts accordingly | |
| N12 | Karte löschen | Map empties, current position becomes the new start | |
