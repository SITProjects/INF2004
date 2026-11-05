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
| LINE-U04 | tracking decisions: debounce, one step per edge, cool-down, step limit to search, quick crossing to search, junction, lost line, search found / between / both sides / timeout, hand-back from a bypass | IF-04 | auto (host) | `sh testing/line_follow/host/run.sh` in the dev container | `0 failure(s)` (LT01..LT12 PASS) |
| LINE-U05 | the car runs straight between corrections on a 2 m straight | issue 1, IF-04 | manual | 1. Flash `make MQTT=1`. 2. Start the car on a 2 m straight with the tape between the sensors. 3. Count `line TRACK` steps (heading changes) over the 2 m and mark the side-to-side deviation. | no visible weave; a few 4° steps at most per metre; the car stays straight between steps |
| LINE-U06 | a 90° corner each way and a curve | IF-04 | manual | Same image: run through a left and a right 90° corner and the course's tightest curve. Save the `line SEARCH_…`/`TRACK` lines. | each corner: a quick crossing or held edge starts `SEARCH` towards the corner, the pivot ends with the tape between the sensors, and following resumes on the new leg |
| LINE-U07 | a lost line is searched for both ways and then stops | IF-04 | manual | Same image: end the tape. | `SEARCH` towards the last edge up to 120°, then the other way, then `line STOP` and the car brakes; putting a sensor on the tape restarts `TRACK` |

*Method:* **auto** = `TEST_CHECK` in code, result printed on the console;
**manual** = a person observes the result (LED, scope, meter, phone app…).

## Results

Add a new row for every run. Never edit or delete old rows.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| YYYY-MM-DD | LINE-U01 | `abc1234` | | PASS / FAIL | `logs/YYYY-MM-DD_….txt` | |
