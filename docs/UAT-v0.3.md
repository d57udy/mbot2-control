# User acceptance test, v0.3

Run with the real mBot2 and the Android phone on https://d57udy.github.io/mbot2-control/. Each test has a pass criterion. Record PASS/FAIL and notes in the last column (or open an issue). Earlier checklists: `docs/HARDWARE_TEST.md`.

Setup: robot charged, home screen, no program running; floor space of about 2 x 2 m with a few objects (box, chair leg, wall); phone volume up; an Anthropic API key with a spending limit for the conversation tests.

## A. Connection and regression

| # | Test | Pass criterion | Result |
|---|---|---|---|
| A1 | Connect | Status "verbunden", header shows "Watchdog an" | |
| A2 | Diagnose (Befehl senden) | Log shows firmware version and `dir()` lists; copy the output into an issue | |
| A3 | Joystick drive 10 s, release | Smooth motion, stops within 0.5 s of release | |
| A4 | Bluetooth off while driving | Robot stops within about 0.5 s | |

## B. Eye LEDs (ultrasonic sensor)

| # | Test | Pass criterion | Result |
|---|---|---|---|
| B1 | Open "Lichter". Only supported emotion buttons are shown | Every visible button plays an animation on the eyes | |
| B2 | Press an emotion while the joystick tab is open (distance polling running) | Animation plays fully, not cut off | |
| B3 | Press two emotions quickly in a row | Second one plays after (or replaces) the first; no error in the log other than a clear "busy" note | |
| B4 | Eye sliders left/right | Each eye follows its slider | |
| B5 | Any failing button | Log shows a readable error naming the effect (not silent) | |

## C. Environment scan

| # | Test | Pass criterion | Result |
|---|---|---|---|
| C1 | "Umgebung scannen" → Scannen (12) | Robot turns 12 x 30° and ends facing the start direction (± 15°) | |
| C2 | Radar plot | Near objects appear in the right direction (object on the robot's right shows on the right) | |
| C3 | Open directions | Chips list directions that are actually free | |
| C4 | Tap an opening chip | Robot turns toward it and drives forward, stopping at least 15 cm before anything | |
| C5 | Erkunden | Robot scans, moves, scans again (up to 3 moves) without touching anything | |
| C6 | Press STOPP during a scan | Robot stops within 1 s; scan reports "abgebrochen" | |

## D. Conversation (KI)

| # | Test | Pass criterion | Result |
|---|---|---|---|
| D1 | Settings → KI: paste key, choose model | No error; key field masked | |
| D2 | Voice mode "Gespräch (KI)", mic on, say "Hallo, wie heißt du?" | Spoken German answer within about 3 s, eyes/LEDs react | |
| D3 | "Wie fühlst du dich?" | Emotion shown (eyes effect + LED colour) matching the answer | |
| D4 | "Fahr ein Stück nach vorne" | Robot drives forward a short distance and says what it did | |
| D5 | "Dreh dich nach links" | Turns left about 90° | |
| D6 | "Schau dich mal um" | Robot runs a scan, then describes what it found (e.g. "Rechts ist frei") | |
| D7 | "Fahr dahin, wo am meisten Platz ist" | Scans if needed, turns toward the best opening, drives, stops before obstacles | |
| D8 | Say "stopp" while it drives or talks | Robot stops immediately, speech stops, agent loop ends | |
| D9 | Robot speech is not picked up as a new command | No self-triggered replies | |
| D10 | Wrong API key | Clear German error message in the chat, robot stays still | |
| D11 | English: switch language to English and talk | Answers in English | |
| D12 | Ask a second question while the robot is still thinking | First request is dropped, second is answered | |
| D13 | Ask "Fahr dahin, wo Platz ist" and press STOPP while it turns | It does not drive off after the turn finishes | |
| D14 | Grab the joystick while the AI or a scan is moving the robot | Manual control takes over immediately | |
| D15 | Say "Stopp" twice in one session (with something in between) | Both stop the robot | |

## E. Safety

| # | Test | Pass criterion | Result |
|---|---|---|---|
| E1 | Ask the AI to "drive 3 meters forward" toward a wall 50 cm away | It refuses or stops at least 15 cm before the wall | |
| E2 | Lock the screen during a conversation | Robot stops, mic off | |
| E3 | Switch to another app during a scan | Robot stops | |
