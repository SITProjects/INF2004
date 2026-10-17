# Obstacle — test log

**Subsystem:** obstacle  ·  **Student ID:** <student ID>  ·  **Code:** `app_program/obstacle/`

See `testing/README.md` for how to run tests and record results.

## Test cases

Define a test here before running it. Test IDs never change.

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| OBS-U01 | scan task started (obstacle module initialised) and sleeps between scans | | auto | `obstacle_test.c` | `[TEST] OBS-U01 PASS` |
| OBS-U02 | servo PWM slice runs at 50 Hz (1 MHz counter, wrap 19999) | | auto | `obstacle_test.c` | `[TEST] OBS-U02 PASS` |
| OBS-U03 | clear path ahead reports CLEAR | | manual | 1. Flash normal image. 2. Keep 20 cm in front of the sensor clear. 3. Watch the console. | `obstacle: CLEAR, angle 90` |
| OBS-U04 | obstacle closer than 5 cm starts a side scan | | manual | 1. Hold a hand 3 cm in front of the sensor. 2. Watch the servo and console. | servo turns to 60° then 120°; console shows `SCANNING`, then `PATH_FOUND` or `BLOCKED` |

*Method:* **auto** = `TEST_CHECK` in code, result printed on the console;
**manual** = a person observes the result (LED, scope, meter, phone app…).

## Results

Add a new row for every run. Never edit or delete old rows.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| YYYY-MM-DD | OBS-U01 | `abc1234` | | PASS / FAIL | `logs/YYYY-MM-DD_….txt` | |
