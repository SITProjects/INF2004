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
| TER-U11 | IMU fault raised when the sensor stops and cleared when it is back | | manual | 1. Flash the normal image with the IMU connected; wait for `terrain: calibration VALID`. 2. Unplug the IMU cable from Grove 3. 3. Plug it back in. | After step 2: `terrain: IMU fault raised` within about 0.1 s (10 failed reads); after step 3: `terrain: IMU fault cleared`. Calibration stays VALID |

*Method:* **auto** = `TEST_CHECK` in code, result printed on the console;
**auto (bench)** = `TEST_CHECK` in code that needs a person to move the car on
cue, built only with `TERRAIN_TEST_BENCH` set to 1;
**manual** = a person observes the result (LED, scope, meter, phone app…).

## Results

Add a new row for every run. Never edit or delete old rows.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| YYYY-MM-DD | TER-U01 | `abc1234` | | PASS / FAIL | `logs/YYYY-MM-DD_….txt` | |
| 2026-11-02 | TER-U06 | `4a4be96` | ann | PASS | `logs/2026-11-02_ter_u06_level_ref.txt` | Loose IMU taped flat on a levelled desk; new reference X -0.470, Y 0.743, Z 8.491 m/s^2. TER-U07 not done (IMU stayed taped) |
| 2026-11-02 | TER-U01 | `4a4be96` + new reference | ann | PASS | `logs/2026-11-02_ter_u01_u02_u04.txt` | Packed code, loose IMU |
| 2026-11-02 | TER-U02 | `4a4be96` + new reference | ann | PASS | `logs/2026-11-02_ter_u01_u02_u04.txt` | VALID about 2.6 s after boot |
| 2026-11-02 | TER-U04 | `4a4be96` + new reference | ann | PASS | `logs/2026-11-02_ter_u01_u02_u04.txt` | Pitch and roll within 1 degree |
| 2026-11-02 | TER-U11 | `4a4be96` + new reference | ann | PASS | `logs/2026-11-02_ter_u11_imu_fault.txt` | Raised on unplug, cleared on replug, calibration stayed VALID. Sampling after the reconnect not checked |
