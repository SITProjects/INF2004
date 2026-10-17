# Motion — test log

**Subsystem:** motion  ·  **Student ID:** <student ID>  ·  **Code:** `app_program/motion/`

See `testing/README.md` for how to run tests and record results.

## Test cases

Define a test here before running it. Test IDs never change.

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| MOT-U01 | both motors run 10 kHz PWM and are braked after start-up | | auto | `motion_test.c` | `[TEST] MOT-U01 PASS` |
| MOT-U02 | wheels stay still at power-up | | manual | 1. Flash normal image with the car lifted. 2. Watch both wheels for 10 s. | neither wheel turns; console shows `motion: motors initialized and braked` |

*Method:* **auto** = `TEST_CHECK` in code, result printed on the console;
**manual** = a person observes the result (LED, scope, meter, phone app…).

## Results

Add a new row for every run. Never edit or delete old rows.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| YYYY-MM-DD | MOT-U01 | `abc1234` | | PASS / FAIL | `logs/YYYY-MM-DD_….txt` | |
