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
| MOT-U07 | after start-up the move state is STOPPED, odometry (distance and heading) holds at rest, and a 0 mm move, 0° turns and a turn above `MOTION_MAX_TURN_DEG` are refused | step 3 | auto | `motion_test.c`; car lifted, wheels untouched | `[TEST] MOT-U07 PASS` |
| MOT-U08 | `motion_move_forward(1000)` drives 1 m | step 3, NFR8, A3 | manual | 1. Set `MOTION_MOVE_TESTS` to 1 in `motion_config.h` and flash `make TEST=1`. 2. Put the car on the floor beside a ruler or tape. 3. When `MOT-U08 car on the floor` appears, mark the rear axle. 4. After the move, measure the distance and read the `odometry` line. | car moves 970–1030 mm; `state 0` (finished, not stalled). If the ruler and odometry differ, scale `MOTION_WHEEL_DIAMETER_MM` by ruler ÷ odometry and repeat until they agree within about one count |
| MOT-U09 | `motion_move_backward(100)` reverses 10 cm | step 3, NFR8, A3 | manual | Same run as MOT-U08: mark the car again when `MOT-U09` appears, then measure. | car reverses 97–103 mm; odometry about −100 |
| MOT-U10 | a held wheel stalls the move | step 3, FR12 | manual + auto | Same run: when `MOT-U10` appears, lift the car and hold the left wheel firmly still before the wheels start. | right wheel spins for about 300 ms, then both brake; `[TEST] MOT-U10 PASS` (stalled after about 300 ms, braked, a new move refused, cleared by stop) |
| MOT-U11 | `motion_move_turn_left(360)` ×3 returns to the start heading; calibrates `MOTION_TRACK_WIDTH_MM` | step 4, NFR8 | manual | 1. Set `MOTION_TURN_TESTS` to 1 in `motion_config.h` and flash `make TEST=1`. 2. Put the car on the floor and lay tape along its side. 3. When `MOT-U11 car on the floor` appears, mark the heading. 4. After three turns, measure the total angle A turned (1080° + the error). Calibrate on the floor the demo runs on: slip differs between surfaces. | final heading within a few degrees of the start; each run `state 0`. Otherwise set `MOTION_TRACK_WIDTH_MM` to old × 1080 ÷ the mean of A from MOT-U11 and MOT-U12, and repeat. If the turn judders or stalls, raise `MOTION_TURN_SPEED_DEG_S` |
| MOT-U12 | `motion_move_turn_right(360)` ×3 returns to the start heading | step 4, NFR8 | manual | Same run: re-mark the heading when `MOT-U12` appears, then measure. | as MOT-U11. If left is right and right is off, the track width is not the cause (it is the same both ways): check the weaker motor and encoder |
| MOT-U13 | 90° turns left and right are accurate | step 4, NFR8 | manual | Same run: when `MOT-U13` appears, mark the heading; after each turn measure it, then re-mark before the next (8 s). Five left, then five right. | each turn 85–95°. If MOT-U11 is right but every 90° turn is off by about the same amount, set `MOTION_TURN_STOP_EARLY_DEG` to that overshoot (compare the `coast` value) |
| MOT-U14 | `motion_move_u_turn()` turns 180° | step 4, NFR8 | manual | Same run: measure five U-turns as in MOT-U13. | each turn 175–185° |
| MOT-U15 | a turn replaces a move in progress, a move replaces a turn, and a stop ends a turn at once | step 4, statechart 6.4 | manual + auto | Same run: when `MOT-U15` appears, give the car 1 m clear ahead and mark its heading. It drives forward, turns left 90° about 1 s in, starts another turn that a 100 mm reverse replaces after 0.5 s, then starts a turn that is stopped after 0.5 s. | forward motion ends when the turn starts; first turn 85–95° from the start heading (it pivots slightly ahead of the start point while the forward momentum dies; that is expected); the second turn ends in a reverse; car brakes at once on the stop; `[TEST] MOT-U15 PASS` |
| MOT-U16 | a held wheel stalls a turn | step 4, FR12 | manual + auto | Same run: when `MOT-U16` appears, lift the car and hold the left wheel firmly still before the wheels start. | right wheel spins for about 300 ms, then both brake; `[TEST] MOT-U16 PASS` |
| MOT-U17 | a turn that makes no progress while the wheels turn times out | step 4 | manual | 1. Flip one `MOTION_*_ENCODER_INVERTED` flag in `motion_config.h` (keep `MOTION_TURN_TESTS` at 1) and flash `make TEST=1`; the earlier turn tests will misbehave, so keep the car lifted throughout. 2. Read the `MOT-U17` line. 3. Flip the flag back. | with the flag flipped: both wheels spin, then brake after about 3.5 s; `state 4` (MOTION_STATE_TIMED_OUT) and `retry 4` (MOTION_ERR_TIMED_OUT). With correct flags the same run shows `state 0` |
| MOT-U18 | the PID controller's P, I and D terms, output limits, anti-windup and reset give hand-worked values | step 5, report 8.1 | auto | `motion_test.c`; no motors or encoders needed | `[TEST] MOT-U18 PASS` |
| MOT-U19 | feed-forward duty against wheel speed, open loop; top speed | step 5 | manual | 1. Set `MOTION_SPEED_TESTS` to 1 in `motion_config.h` and flash `make TEST=1` with the car lifted. 2. When `MOT-U19 keep the car lifted` appears, keep clear: both wheels step from duty 100 to 1000 permille, 1.5 s each. 3. Save the `duty … left … right` lines. | speed rises steadily with duty on both wheels. Calibrate `MOTION_FF_OFFSET` and `MOTION_FF_GAIN` from a straight line through the lines above the start-up duty, and set `MOTION_MAX_SPEED_MM_S` 10–15 % below the slower wheel's speed at 1000 |
| MOT-U20 | the speed loop follows 200 → 100 → 200 mm/s steps | step 5, report 8.1 | manual | Same run, still lifted: when `MOT-U20` appears the wheels step speed for 4.5 s; the per-update lines (`ms setL setR speedL speedR dutyL dutyR`) print afterwards. Plot them for the PID tuning report. | each wheel settles within ±5 % of each setpoint within about 0.5 s, without sustained oscillation. Tune `MOTION_SPEED_KP` (KI 0 first: raise until it overshoots, then halve), then `MOTION_SPEED_KI`, then `MOTION_SPEED_WINDOW_UPDATES` |
| MOT-U21 | 200 mm/s holds on the floor on battery alone, open loop against speed loop | step 5, A4 | manual | Same run: when `MOT-U21` appears, unplug USB and put the car on the floor, 2 m clear ahead (15 s). It drives 800 mm open loop; reposition it in the 10 s pause; it drives 800 mm with the speed loop. Plug USB back in and open the terminal within 30 s to get both runs' lines. Run once on a full battery and once on a low one, and note the battery voltage of each. | speed-loop `settled` speeds within ±5 % of 200 on both batteries; the open-loop run shows the difference the loop makes (report graph) |
| MOT-U22 | the test loops' updates come every `MOTION_UPDATE_PERIOD_MS`, which the speed and PI calculations assume | step 5 | auto | `motion_test.c`; times 50 updates (delay, then `motion_move_update()`) with the system timer; no motors | `[TEST] MOT-U22 PASS` and the `took … ms` line within 5 % of 1000 ms. If it fails, measured speeds are off by the same ratio; fix the loop timing before tuning with MOT-U19..U21 |

*Method:* **auto** = `TEST_CHECK` in code, result printed on the console;
**manual** = a person observes the result (LED, scope, meter, phone app…).
MOT-U04..U06 run only with `MOTION_MANUAL_TESTS` set to 1, MOT-U08..U10
only with `MOTION_MOVE_TESTS` set to 1, and MOT-U11..U16 only with
`MOTION_TURN_TESTS` set to 1 (MOT-U17 is the last of these), and
MOT-U19..U21 only with `MOTION_SPEED_TESTS` set to 1; set all four back to 0
before committing, so a normal `make TEST=1` never drives the motors.
MOT-U19..U21 also need MOT-U04 and MOT-U08 first. MOT-U08..U16 need
working encoders and a measured `MOTION_ENCODER_TICKS_PER_REV` (MOT-U04)
first, and MOT-U11..U16 also a calibrated `MOTION_WHEEL_DIAMETER_MM`
(MOT-U08). To measure a turn, see how far the tape line has moved sideways
500 mm from the pivot: 5° is about 44 mm, 3° about 26 mm.

## Results

Add a new row for every run. Never edit or delete old rows.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| YYYY-MM-DD | MOT-U01 | `abc1234` | | PASS / FAIL | `logs/YYYY-MM-DD_….txt` | |
