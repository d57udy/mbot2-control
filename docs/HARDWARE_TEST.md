# First hardware session checklist

Goal: settle the open questions from `research/01-ble-protocol.md` in about 30 minutes. Do it on a floor with space, robot away from stairs and table edges.

## Before connecting

- [ ] mBot2 charged, switched on, CyberPi on the **home screen**, no program running.
- [ ] mBlock app/PC and the Makeblock Bluetooth dongle **not** connected to the robot.
- [ ] Robot **not** paired in Android Bluetooth settings (if it is, "Forget" it).
- [ ] Phone: Bluetooth on, Chrome up to date, "Nearby devices" permission allowed for Chrome. On Android 11 or older, location services on.
- [ ] Note firmware: send `py:cyberpi.get_firmware_version()` from the test box after connecting.

## Connection

1. [ ] Tap **Verbinden**. Does the chooser list a `Makeblock_LE...` device? Record the exact name.
   - If nothing appears: Settings, enable **Alle Bluetooth-Geräte zeigen**, try again, and note the name the robot shows up as.
2. [ ] Log line `Visible services: ...` and `Using service ...`. Record which UUID worked.
3. [ ] Log shows `Live mode ready`. If it says "No handshake reply", note it and continue.
4. [ ] Battery readout shows a value.

## Motion and safety (lift the robot or give it space)

5. [ ] Hold ▲ for 2 s, release. Smooth motion or stutter? Stops on release within how long?
6. [ ] Repeat with burst 0.3 and 0.6 in settings. Which feels best?
7. [ ] ◀ and ▶ spin the correct way.
8. [ ] Test box: `{"cmd":"turn","args":{"deg":90}}`. Turns right about 90°?
9. [ ] **Disconnect test**: hold ▲, then switch Bluetooth off on the phone. Does the robot stop within the burst time? Then try `py:mbot2.forward(30)` (no duration), and while it drives, switch Bluetooth off. Does the robot stop by itself? (Expected: no. Catch it.)
10. [ ] Press STOPP during a `turn` of 360°. Does it stop immediately or only after the turn finishes?
11. [ ] Drive toward a wall. Does the obstacle guard refuse forward moves under 15 cm?

## Speed

12. [ ] Settings: chunk size 100, then 180. Do commands still work? (If yes, larger chunks lower latency.)
13. [ ] Test box `py:cyberpi.get_battery()` ten times; note the rough reply time from the log timestamps.

## Voice

14. [ ] Tap **Sprache an**, allow the microphone. Say "vorwärts", "links", "rechts 45 Grad", "zurück zwei Sekunden", "Licht blau", "stopp".
15. [ ] Does "stopp" interrupt a running "vorwärts drei Sekunden"?
16. [ ] Do you hear a beep on every restart? How often does it restart while silent?
17. [ ] Lock the screen while listening and driving: robot stops, microphone turns off.

Record results in this file (or an issue) and adjust `js/protocol.js` / defaults accordingly.

## v0.2: joystick, watchdog, lights, floor sensor

Run **Diagnose** first (Befehl senden) and paste the log output into `research/` so the API lists are on record.

18. [ ] Header shows **Watchdog an** after connecting. If it shows "kein Watchdog", copy the log lines.
19. [ ] Wheels off the ground, Joystick mode: push up. Both wheels forward? If the robot spins, untick **Motoren gespiegelt**. If left/right are swapped, tick **Räder tauschen**.
20. [ ] Watchdog test: start driving with the joystick, then switch phone Bluetooth off. Wheels stop within about half a second?
21. [ ] Release the joystick: smooth slow-down, no run-on. STOPP: immediate?
22. [ ] Drive toward a wall: slows from about 40 cm, stops at 15 cm. Reversing still works.
23. [ ] Back LEDs: each colour picker changes the matching LED. Which physical LED is 1?
24. [ ] Eyes: the two sliders switch the left and right eye. Try the emotion chips. Try the experimental per-LED sliders and note whether anything lights.
25. [ ] Floor sensor: robot on white paper with a black line. Which probes light up as "on"? Does 1 mean line or background? Cover only the rightmost probe: does R2 change?
26. [ ] Tick **Farben erkennen** and hold coloured paper under the sensor. Are the names right?
