# Line following — test log

**Subsystem:** line_follow  ·  **Student ID:** <student ID>  ·  **Code:** `app_program/line_follow/`

See `testing/README.md` for how to run tests and record results.

## Test cases

Define a test here before running it. Test IDs never change.

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| LINE-U01 | line task started (ADC device opened) and sleeps between samples | | auto | `line_follow_test.c` | `[TEST] LINE-U01 PASS` |
| LINE-U02 | sensor readings and commands follow the surface | | manual | 1. Flash normal image. 2. Move the sensors over white, then a black line under the left, the right, and both sensors. | console every 200 ms; left black → `STEER_LEFT`, right black → `STEER_RIGHT`, both black → `STOP` then `JUNCTION`, both white → `SEARCH` |

*Method:* **auto** = `TEST_CHECK` in code, result printed on the console;
**manual** = a person observes the result (LED, scope, meter, phone app…).

## Results

Add a new row for every run. Never edit or delete old rows.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| YYYY-MM-DD | LINE-U01 | `abc1234` | | PASS / FAIL | `logs/YYYY-MM-DD_….txt` | |
