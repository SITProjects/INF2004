# Line following — test log

**Subsystem:** line_follow  ·  **Student ID:** <student ID>  ·  **Code:** `app_program/line_follow/`

See `testing/README.md` for how to run tests and record results.

## Test cases

Define a test here before running it. Test IDs never change.

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| LINE-U01 | line task started and sleeps between samples | | auto | `line_follow_test.c` | `[TEST] LINE-U01 PASS` |
| LINE-U02 | each sensor's DO flips at the tape edge | IR modules, `LINE_DO_BLACK_LEVEL` | manual | Redefined 2026-11-05 for the digital DO outputs. 1. Flash `make IR_CALIBRATION=1` (motors off). 2. Move each sensor slowly from white across the tape edge at its mounted height. 3. Turn its potentiometer until the `LINE L=… R=…` line flips exactly at the edge. | white `0`, black `1` on each sensor (otherwise flip `LINE_DO_BLACK_LEVEL`); the flip happens within about 2 mm of the edge, with no chatter while held still |
| LINE-U03 | both DO pins are digital inputs (SIO, input on, output off) | TEAM.md section 2 | auto | `line_follow_test.c` | `[TEST] LINE-U03 PASS` |
| LINE-U04 | tracking decisions: debounce, lead towards a black edge while the car turns, hold the heading when it clears, held edge to search, a brief touch is a correction (never a search), lost line (sooner in a bend), search found / between / both sides / timeout, hand-back from a bypass, no wrong-way search after a pivot's overshoot, a second touch of one sensor is a bend and seeks that way, ended by the other sensor, a cross junction straight on (tracking carries on) / LEFT / RIGHT / U-turn (exact pivot, sensors ignored, then straight on) / a pivot that never ends stops, the tape crossing from one sensor to the other (a correction too far) starts no search, an edge turning black together with the other is a fresh correction, a model of the car (sensors 130 mm ahead, turn lag) following the 100 mm radius curve fitted to car10 and a 200 mm curve met 8° off, with the sensors 20 and 36 mm apart, holding a straight, travel with the tape under both sensors is no held edge | IF-04 | auto (host) | `sh testing/line_follow/host/run.sh` in the dev container | `0 failure(s)` (LT01..LT22 PASS) |
| LINE-U05 | the car runs straight between corrections on a 2 m straight | issue 1, IF-04 | manual | 1. Flash `make MQTT=1`. 2. Start the car on a 2 m straight with the tape between the sensors. 3. Watch the corrections over the 2 m and mark the side-to-side deviation. | no visible weave; short corrections only when a sensor touches the tape; the car stays straight between them; no `SEARCH` |
| LINE-U06 | a 90° corner each way and a curve | IF-04 | manual | Same image: run through a left and a right 90° corner and the course's tightest curve. Save the `line SEARCH_…`/`TRACK` lines. | the curve is followed without `SEARCH`; each corner is either driven round while the edge reads black, or a held edge (or the line lost) starts `SEARCH` towards the corner, the pivot ends with the tape between the sensors, and following resumes on the new leg with no `SEARCH` the other way |
| LINE-U07 | a lost line is searched for both ways and then stops | IF-04 | manual | Same image: end the tape. | `SEARCH` towards the last edge up to 120°, then the other way, then `line STOP` and the car brakes; putting a sensor on the tape restarts `TRACK` |
| LINE-U08 | cross junctions: straight on by default, and each turn | IF-04, IF-08 | manual | 1. Flash `make MQTT=1`; run across a cross: `line JUNCTION STRAIGHT`, the car keeps tracking over the crossing and follows on. 2. For each of LEFT, RIGHT and UTURN, set the barcode before the cross to it on the dashboard (`tools/dashboard/car_dashboard.py`), press Start and run into the cross. Save the `line …` and `LT …` lines. | straight: no `SEARCH`; each turn: `line JUNCTION <turn>`, the car stops over the crossing, pivots exactly 90° (180° for a U-turn), ignoring the sensors, then `line TRACK` and it drives straight on along the new heading |
| LINE-U09 | the left and right line sensors match the car's own sides | IF-04, `LINE_LEFT_PIN`, `LINE_RIGHT_PIN` | manual | 1. Flash `make MQTT=1 IR_CALIBRATION=1` (motors stay off). 2. Car on the white floor, both sensors reading 0. 3. Stand **behind** the car, facing the way it drives. 4. Slide black tape under the sensor on your left only and hold it 3 s, then the one on your right only. | your left gives `LINE L=1 R=0`, your right `LINE L=0 R=1`; the reverse means the pins are swapped. (2026-11-06: the first check did not say which way to face, so it did not settle this) |

*Method:* **auto** = `TEST_CHECK` in code, result printed on the console;
**manual** = a person observes the result (LED, scope, meter, phone app…).

## Results

Add a new row for every run. Never edit or delete old rows.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| YYYY-MM-DD | LINE-U01 | `abc1234` | | PASS / FAIL | `logs/YYYY-MM-DD_….txt` | |
| 2026-11-06 | LINE-U04 | `7edb636` + slow pass | Claude (host) | PASS | `sh testing/line_follow/host/run.sh` | LT23..LT28 added for the slow pass over a barcode. Model only, not run on the car: heading within +-3.0 deg through a creep at 30 mm/s, met 3 deg off. `motion_command.c`'s reverse has no host test. |
