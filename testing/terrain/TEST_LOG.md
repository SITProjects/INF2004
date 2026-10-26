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
| TER-U04 | pitch and roll read about 0 at rest after calibration | | auto | `terrain_test.c`; car still on level ground during power-up | `[TEST] TER-U04 PASS` |
| TER-U05 | pitch and roll match known angles | | manual | 1. Set `TERRAIN_DEBUG_PRINT_MS` to 500 locally (never commit it), build and flash. 2. Let it calibrate on level ground. 3. Tilt nose up to 10°, 20° and 30°, then nose down, checking each angle with a phone level app. 4. Repeat with the right side up, then the left side up. 5. Note the printed `pitch` and `roll` at each angle. | Within ±2° of the phone at each angle; nose up and right side up are positive |
| TER-U06 | record the level reference (after every remount) | | auto (bench) | 1. Set `TERRAIN_TEST_BENCH` to 1 locally (never commit it), build `make TEST=1` and flash. 2. Put the car on a floor checked to be flat. 3. Keep it still when the countdown ends. 4. Paste the printed `#define TERRAIN_LEVEL_REF_*` lines into `terrain_config.h` and rebuild. | `[TEST] TER-U06 PASS` and three `#define` lines |
| TER-U07 | each car axis points the right way after mounting | | auto (bench) | Same build as TER-U06; runs straight after it. Follow the console: 1. car flat; 2. nose lifted about 30°; 3. right side lifted about 30°. Hold still after each countdown until the reading prints. | Three `[TEST] TER-U07 PASS` lines; a FAIL names the `TERRAIN_CAR_*_FROM/SIGN` setting to check |
| TER-U08 | movement during calibration is retried | | manual | 1. Flash the normal image. 2. Shake the car gently for the first 3 s after power-up, then put it down flat and still. | `terrain: calibration VALID` about 2.5 s after the car is still |

*Method:* **auto** = `TEST_CHECK` in code, result printed on the console;
**auto (bench)** = `TEST_CHECK` in code that needs a person to move the car on
cue, built only with `TERRAIN_TEST_BENCH` set to 1;
**manual** = a person observes the result (LED, scope, meter, phone app…).

## Results

Add a new row for every run. Never edit or delete old rows.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| YYYY-MM-DD | TER-U01 | `abc1234` | | PASS / FAIL | `logs/YYYY-MM-DD_….txt` | |
