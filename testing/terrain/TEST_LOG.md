# Terrain — test log

**Subsystem:** terrain  ·  **Student ID:** <student ID>  ·  **Code:** `app_program/terrain/`

See `testing/README.md` for how to run tests and record results.

## Test cases

Define a test here before running it. Test IDs never change.

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| TER-U01 | IMU set up over I2C0 and delivering samples at about 100 Hz | | auto | `terrain_test.c`, GY-511 on Grove 3 | `[TEST] TER-U01 PASS` |
| TER-U02 | calibration becomes VALID within 5 s of power-up | | auto | `terrain_test.c`; car still on level ground during power-up | `[TEST] TER-U02 PASS` |
| TER-U03 | tilted car is refused | | manual | 1. Flash normal image with the car on a slope. 2. Watch the console. | `terrain: calibration NOT_LEVEL` |

*Method:* **auto** = `TEST_CHECK` in code, result printed on the console;
**manual** = a person observes the result (LED, scope, meter, phone app…).

## Results

Add a new row for every run. Never edit or delete old rows.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| YYYY-MM-DD | TER-U01 | `abc1234` | | PASS / FAIL | `logs/YYYY-MM-DD_….txt` | |
