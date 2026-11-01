# Motion — test log

**Subsystem:** motion  ·  **Student ID:** <student ID>  ·  **Code:** `app_program/motion/`

See `testing/README.md` for how to run tests and record results.

## Test cases

Define a test here before running it. Test IDs never change.

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| MOT-U01 | both motors run 10 kHz PWM and are braked after start-up | | auto | `motion_test.c` | `[TEST] MOT-U01 PASS` |
| MOT-U02 | wheels stay still at power-up | | manual | 1. Flash normal image with the car lifted. 2. Watch both wheels for 10 s. | neither wheel turns; console shows `motion: motors initialized and braked` |
| MOT-U03 | both encoders count on PIO0 and hold still at rest | step 2 | auto | `motion_test.c`; car lifted, wheels untouched | `[TEST] MOT-U03 PASS` |
| MOT-U04 | one forward wheel revolution gives `MOTION_ENCODER_TICKS_PER_REV` counts, about 2 mm per count or better | step 2, A3, NFR8 | manual | 1. Set `MOTION_MANUAL_TESTS` to 1 in `motion_config.h` and flash `make TEST=1` with the car lifted. 2. Mark each tyre. 3. When `MOT-U04/U05 turn each wheel` appears, turn the left wheel exactly one revolution forward, then the right. 4. Read the `left`/`right` lines. | each count rises by the same amount each time (record it in `motion_config.h`); wheel circumference ÷ count ≤ 2 mm |
| MOT-U05 | counts go negative when a wheel turns backward | step 2, A3 | manual | Same run as MOT-U04: turn each wheel one revolution backward. | each count falls by the MOT-U04 amount; set `MOTION_*_ENCODER_INVERTED` for a wheel that counts the wrong way |
| MOT-U06 | each motor terminal drives the right wheel in the right direction | step 1 | manual | 1. Set `MOTION_MANUAL_TESTS` to 1 in `motion_config.h` and flash `make TEST=1` with the car lifted. 2. Watch each wheel as the console names it: `LEFT wheel FORWARD`, `LEFT wheel BACKWARD`, `RIGHT wheel FORWARD`, `RIGHT wheel BACKWARD` (1.5 s each at 40 % duty). | only the named wheel turns, in the named direction (forward = the car would drive forward). Wrong wheel: swap the M1/M2 motor cables. Wrong direction: set `MOTION_*_MOTOR_INVERTED`. Once the encoders are wired, the named wheel's `encoder change` is positive forward and negative backward |

*Method:* **auto** = `TEST_CHECK` in code, result printed on the console;
**manual** = a person observes the result (LED, scope, meter, phone app…).
MOT-U04..U06 run only with `MOTION_MANUAL_TESTS` set to 1; set it back to 0
before committing, so a normal `make TEST=1` never drives the motors.

## Results

Add a new row for every run. Never edit or delete old rows.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| YYYY-MM-DD | MOT-U01 | `abc1234` | | PASS / FAIL | `logs/YYYY-MM-DD_….txt` | |
