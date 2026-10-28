# Obstacle — test log

**Subsystem:** obstacle  ·  **Student ID:** <student ID>  ·  **Code:** `app_program/obstacle/`

See `testing/README.md` for how to run tests and record results.

## Test cases

Define a test here before running it. Test IDs never change.

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| OBS-U01 | scan task started (obstacle module initialised) and sleeps between scans | | auto | `obstacle_test.c` | `[TEST] OBS-U01 PASS` |
| OBS-U02 | servo PWM slice runs at 50 Hz (1 MHz counter, wrap 19999) | | auto | `obstacle_test.c` | `[TEST] OBS-U02 PASS` |
| OBS-U03 | clear path ahead reports CLEAR | | manual | 1. Flash normal image. 2. Keep at least 40 cm in front of the sensor clear (30 cm candidate threshold). 3. Watch the console. | `obstacle: [0x11] CLEAR dir 90` |
| OBS-U04 | obstacle inside the configured detection threshold starts a side scan | | manual | 1. Hold a hand 3 cm in front of the sensor. 2. Watch the servo and console. | servo turns to 60° then 120°; console reports `PATH_FOUND` or `BLOCKED` after both side positions finish |
| OBS-U10 | packed obstacle fields at boundaries and invalid values | robot_types.h | auto | `obstacle_test.c`: valid flags, min/max/one-beyond/invalid fields, edge sentinels, width saturation and sequence wrap | `[TEST] OBS-U10 PASS`; neighbouring fields unchanged |

Additional regression cases (U05–U09 and U11 remain reserved for the sweep plan):

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| OBS-U12 | no-return and beyond-range policy | driver → scanner | manual | Scope TRIG/ECHO in open space; include a pulse lasting about 38 ms, if produced by this module. Record trigger-to-rise, pulse duration, driver outcome and scanner status. | NO_ECHO then CLEAR; late falling edge does not overwrite a result; next trigger waits for configured interval |
| OBS-U13 | malformed or stuck-high echo | driver → scanner | manual | Check ECHO high before trigger, duplicate/ambiguous edges and a zero pulse using controlled input. | FAULT; never classed as a no-return clear reading |
| OBS-U14 | directional no-return ranking | scanner | manual | Compare a no-return side against a finite clear side, swap sides, then test two no-return sides. | No-return side wins; two no-return sides select left; width remains unavailable and edge flags remain clear |
| OBS-U15 | active pin allocation | config → hardware | manual | Scope trigger output and inject ECHO on GP17; verify GP2/GP3 are not configured by obstacle. | TRIG GP16, ECHO GP17, servo GP12; one configured scan priority |

Compile and simulated regression results are software evidence only. Physical
scope observations, voltage checks and actual sensor behavior remain pending.

*Method:* **auto** = `TEST_CHECK` in code, result printed on the console;
**manual** = a person observes the result (LED, scope, meter, phone app…).

## Results

Add a new row for every run. Never edit or delete old rows.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| YYYY-MM-DD | OBS-U01 | `abc1234` | | PASS / FAIL | `logs/YYYY-MM-DD_….txt` | |
