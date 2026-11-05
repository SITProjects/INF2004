# Motion — test log

**Subsystem:** motion  ·  **Student ID:** <student ID>  ·  **Code:** `app_program/motion/`

See `testing/README.md` for how to run tests and record results.

## Test cases

Define a test here before running it. Test IDs never change.

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| MOT-U01 | both motors run 20 kHz PWM and are braked after start-up | | auto | `motion_test.c` | `[TEST] MOT-U01 PASS` |
| MOT-U02 | wheels stay still at power-up | | manual | 1. Flash normal image with the car lifted. 2. Power up and watch both wheels until the first commanded movement: `search reacquisition` after the 5 s line-adapter delay, or an obstacle bypass leg (`avoid … SEND`), which is not held back by that delay and can come sooner if something is in front of the sensor. Keep 40 cm clear ahead to test the full 5 s. 3. Repeat for 3 power-ups. | neither wheel turns or twitches before that command; console shows `motion: motors initialized and braked`. Movement after the command is the normal start, not a failure |
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
| MOT-U21 | 200 mm/s holds on the floor on battery alone, open loop against speed loop | step 5, A4 | manual | Same run: when `MOT-U21` appears, unplug USB and put the car on the floor, 2 m clear ahead (15 s). It drives 800 mm open loop; reposition it in the 10 s pause; it drives 800 mm with the speed loop. Plug USB back in and reopen the terminal (or restart it if it lost the port): both runs' lines print once it has been open for 2 s, within 5 min. If the boot banner appears instead, the Pico reset and the run is lost. Run once on a full battery and once on a low one, and note the battery voltage of each. | speed-loop `settled` speeds within ±5 % of 200 on both batteries; the open-loop run shows the difference the loop makes (report graph) |
| MOT-U22 | the test loops' updates come every `MOTION_UPDATE_PERIOD_MS`, which the speed and PI calculations assume | step 5 | auto | `motion_test.c`; times 50 updates (delay, then `motion_move_update()`) with the system timer; no motors | `[TEST] MOT-U22 PASS` and the `took … ms` line within 5 % of 1000 ms. If it fails, measured speeds are off by the same ratio; fix the loop timing before tuning with MOT-U19..U21 |
| MOT-U23 | when stopped, the straight-line loop has no error or correction, and it switches off and on without driving the motors | step 6 | auto | `motion_test.c`; car lifted, wheels untouched | `[TEST] MOT-U23 PASS` |
| MOT-U24 | the straight-line loop steers against the error, forward and backward | step 6 | manual | 1. Set `MOTION_SYNC_TESTS` to 1 in `motion_config.h` and flash `make TEST=1` with the car lifted. 2. When `MOT-U24 keep the car lifted` appears, wait for the wheels to turn, then lightly slow the LEFT wheel with a finger for about a second; do not stop it, or the move stalls. The car drives forward 1 m, then backward 1 m. 3. Save the `ms errTicks corrDegS setL setR` lines. | while the left wheel is slowed: forward, `errTicks` grows positive, `corrDegS` goes negative and `setL` rises above `setR`; backward, `errTicks` grows negative, `corrDegS` goes positive and `setL` becomes more negative than `setR`. In both, the slowed wheel is driven harder and `errTicks` returns towards 0 after you let go. If `corrDegS` has the same sign as `errTicks`, the loop is reversed: fix this before the floor tests |
| MOT-U25 | sideways drift over 1 m, straight-line loop off against on | step 6, NFR8, A3 | manual | Same run: when each `MOT-U25 … run` line appears (20 s apart), put the car on the start line with tape along its side. After each run measure the sideways offset of the rear axle from the tape line and the distance, and note the `odometry … heading … max error` line. Three runs with sync OFF, then three with sync ON. Calibrate on the demo floor. | sync ON: drift < 20 mm on every run and distance 970–1030 mm; record both sets for the motion accuracy evaluation. Tune `MOTION_SYNC_KP` (KI 0 first: raise until the car weaves, then halve), then `MOTION_SYNC_KI` if the heading ends off the same way every run. If sync ON still drifts one way while `heading` reads about 0, the wheels differ in diameter (not something the encoders can see) |
| MOT-U26 | the straight-line loop recovers from a nudge and holds a reverse straight | step 6 | manual | Same run: when `MOT-U26` appears, give the car 1 m clear ahead; about halfway push its front about 1 cm sideways. Save the trace lines. Then re-mark it for the 500 mm reverse and measure its drift. | after the push `errTicks` returns to within ±1 within about 0.5 s without swinging back and forth; the car carries on parallel to its old path (the loop holds heading, not position). Reverse: drift < 10 mm, `state 0` |
| MOT-U27 | follow and the line input refuse out-of-range values, stop-at-line needs a move in progress, and the line counts as seen or gone only after `MOTION_LINE_SEEN_SAMPLES` readings agree, and follow refuses a line reading older than `MOTION_LINE_TIMEOUT_MS`; no motors | step 6b | auto | `motion_test.c`; car lifted, wheels untouched | `[TEST] MOT-U27 PASS` |
| MOT-U28 | following steers towards a hand-fed line error, and stops when the errors stop | step 6b | manual + auto | 1. Set `MOTION_FOLLOW_TESTS` to 1 in `motion_config.h` and flash `make TEST=1` with the car lifted. 2. When `MOT-U28 keep the car lifted` appears, keep clear: the wheels turn for four 1.5 s steps with line errors 0, +300, 0, −300. 3. Save the `error … turn … set … speed` lines. | `[TEST] MOT-U28 PASS`: error +300 gives a + turn and the right wheel's setpoint and speed above the left's; −300 the reverse; 0 equal. Both wheels brake within about 100 ms of the last error (`stopped … ms` ≤ 140). If the speeds do not follow the setpoints, tune the speed loop (MOT-U20) first |
| MOT-U29 | a move or turn asked to stop at the line ends when the line comes into view, not when it starts on it; otherwise it ends at its distance | step 6b, design review 6.4 | manual + auto | Same run, still lifted: when `MOT-U29` appears, keep clear for four short runs with a pretend line: (a) forward, line appears; (b) 360° turn that starts on the line, line goes, then returns; (c) 200 mm forward not asked to stop, crosses the line; (d) 200 mm asked to stop, never sees it. | `[TEST] MOT-U29 PASS`; (a) and (b) end within `MOTION_LINE_SEEN_SAMPLES` updates of the line appearing, (c) and (d) end at their distance |
| MOT-U30 | following arcs the right way on the floor at the turn rate asked for | step 6b | manual | Same run: when `MOT-U30` appears, put the car on the floor with 1 m clear all round. It follows with error +300 (arc left), 0 (straight), −300 (arc right), 1.5 s each. Save the `heading … (asked …)` lines. | arcs left, then straight, then right; each phase's `heading` within about 20 % of `asked` (135° with the default `MOTION_FOLLOW_KP`: 90 °/s for 1.5 s); the straight phase within a few degrees of 0. A large shortfall means the inner wheel cannot slow enough: check `MOTION_FOLLOW_MAX_DEG_S` and the speed loop |
| MOT-U33 | motion command words pack into the 22-bit layout, saturate at 1023 mm and 180° without touching neighbouring fields, decode back, and leave bits 22..31 clear | step 9, Tribunal 1 | auto | `motion_test.c`; no motors | `[TEST] MOT-U33 PASS` |
| MOT-U34 | tMotion wakes every `MOTION_UPDATE_PERIOD_MS` within 1 ms while every other task runs | step 9, NFR1, A1 | auto | `motion_test.c`; tMotion runs 1 s with the TIMER stamped at each wake-up; no motors. Run on the full image (all subsystems started, `make TEST=1`, Wi-Fi on), not the motion-only one | `[TEST] MOT-U34 PASS`; the `period … us` line within 19000..21000; record the `longest update` (loop execution time) for the Tri 1 before/after comparison |
| MOT-U35 | through `MBF_MOTION_CMD`, tMotion refuses a word with reserved bits, a 0 mm move and a turn above 180°, but not a stop; INHIBIT brakes within 20 ms, reports INHIBITED and discards a command sent meanwhile; clearing it returns to STOPPED | step 9, NFR6 | auto | `motion_test.c`; no motors (the discarded command is an invalid one) | `[TEST] MOT-U35 PASS`; `INHIBIT to brake … us` ≤ 20000 |
| MOT-U36 | through tMotion on the lifted car: (a) a 300 mm move ends with MOVE_DONE, reason DIST; (b) a move replaced by another gives MOVE_DONE only for the second; (c) STOP ends a move without MOVE_DONE; (d) INHIBIT during a move brakes within 20 ms, discards a move sent meanwhile, and clearing it gives no MOVE_DONE; (e) FOLLOW runs on the interim line input and ends with FOLLOW_ENDED once readings stop | step 9, design review 6.4, NFR6 | manual + auto | 1. Set `MOTION_TASK_TESTS` to 1 in `motion_config.h` and flash `make TEST=1` with the car lifted. 2. When `MOT-U36 keep the car lifted` appears, keep clear for about 20 s. | `[TEST] MOT-U36 PASS`; every part `1`; `INHIBIT to brake … us` ≤ 20000. Watch the wheels: each part stops when it should |
| MOT-U37 | through tMotion, a held wheel raises STALL and mode STALLED; a new move is refused until STOP | step 9, FR12 | manual + auto | Same run: when `MOT-U37` appears, hold the LEFT wheel firmly still before the right one starts (10 s). | `[TEST] MOT-U37 PASS`; the right wheel brakes within about 0.3–0.8 s |
| MOT-U38 | `motion_get_odometry()` (IF-05, for terrain) refuses NULL, its update count rises by one per period while tMotion runs, and at rest distance and heading hold and every speed is 0 | step 9, IF-05 | auto | `motion_test.c`; reads the snapshot from the test task for 200 ms while tMotion runs; no motors | `[TEST] MOT-U38 PASS`; `updates` within 2 of 10 |
| MOT-U39 | line following restarts after a stall or timed-out recovery turn, at most 3 times in a row | line adapter (`motion_command.c`) | manual | Normal firmware, following a line: hold one wheel for about 1 s, release; repeat; then hold it until the car gives up; after that, STOP then FOLLOW, run 3 s, and hold again | Each hold: `fault 3, retry n of 3 in 500 ms` (3 = STALLED), then `fault cleared, searching again` and following resumes. The fourth fault in a row latches with `STOP then FOLLOW or restart required` and the car stays stopped. After 3 s of running without a fault the count starts again at 1 |

*Method:* **auto** = `TEST_CHECK` in code, result printed on the console;
**manual** = a person observes the result (LED, scope, meter, phone app…).
MOT-U04..U06 run only with `MOTION_MANUAL_TESTS` set to 1, MOT-U08..U10
only with `MOTION_MOVE_TESTS` set to 1, and MOT-U11..U16 only with
`MOTION_TURN_TESTS` set to 1 (MOT-U17 is the last of these), and
MOT-U19..U21 only with `MOTION_SPEED_TESTS` set to 1, MOT-U24..U26
only with `MOTION_SYNC_TESTS` set to 1, and MOT-U28..U30 only with
`MOTION_FOLLOW_TESTS` set to 1, and MOT-U36..U37 only with
`MOTION_TASK_TESTS` set to 1; set all seven back to 0 before
committing, so a normal `make TEST=1` never drives the motors.
MOT-U01..U32 call the motion module directly, so `motion_test()` first
holds tMotion (`motion_task_hold()`) and releases it for MOT-U34 on.
MOT-U28..U30 need a tuned speed loop (MOT-U20) and a calibrated
`MOTION_TRACK_WIDTH_MM` (MOT-U11). Following a real line (MOT-U31) and
stopping at real tape (MOT-U32) are not defined yet: they need a signed
line error and a line-seen flag from line_follow.
MOT-U19..U21 also need MOT-U04 and MOT-U08 first, and MOT-U24..U26 a
calibrated `MOTION_TRACK_WIDTH_MM` (MOT-U11) and a tuned speed loop
(MOT-U20, MOT-U21). MOT-U08..U16 need
working encoders and a measured `MOTION_ENCODER_TICKS_PER_REV` (MOT-U04)
first, and MOT-U11..U16 also a calibrated `MOTION_WHEEL_DIAMETER_MM`
(MOT-U08). To measure a turn, see how far the tape line has moved sideways
500 mm from the pivot: 5° is about 44 mm, 3° about 26 mm.

## Test sessions

The tests run in these sessions, in this order: each session may
calibrate a value the later ones need, so build each image only when its
session starts. A session's image and log share one name:
`S<nn>_MOT-<tests>.uf2` and `YYYY-MM-DD_S<nn>_MOT-<tests>.txt`; the
commit tested goes in the result row. *Motion only* means a local, uncommitted `app_main.c` that starts and
tests motion alone, built with `WIFI=none`; *full* means the committed
`app_main.c` with Wi-Fi.

| Session | Tests | Build | Image | Log | Before it |
|---|---|---|---|---|---|
| S00 | host line acquisition | `sh testing/motion/host/run.sh` | — | `S00_MOT-host.txt` | — |
| S01 | MOT-U01, U03, U07, U18, U22, U23, U27, U33, U34, U35, U38 | full, `make TEST=1` | `S01_MOT-auto.uf2` | `S01_MOT-auto.txt` | — |
| S02 | MOT-U02 | full, `make` | `S02_MOT-U02.uf2` | `S02_MOT-U02.txt` | — |
| S03 | MOT-U04..U06 | motion only, `MOTION_MANUAL_TESTS` 1; car lifted | `S03_MOT-U04-U06.uf2` | `S03_MOT-U04-U06.txt` | — |
| S04 | MOT-U19..U21 | motion only, `MOTION_SPEED_TESTS` 1 | `S04_MOT-U19-U21.uf2` | `S04_MOT-U19-U21.txt` | S01 (MOT-U22), S03 |
| S05 | MOT-U08..U10 | motion only, `MOTION_MOVE_TESTS` 1 | `S05_MOT-U08-U10.uf2` | `S05_MOT-U08-U10.txt` | S04 |
| S06 | MOT-U11..U16 | motion only, `MOTION_TURN_TESTS` 1 | `S06_MOT-U11-U16.uf2` | `S06_MOT-U11-U16.txt` | S05 |
| S07 | MOT-U17 | as S06 with `MOTION_LEFT_ENCODER_INVERTED` flipped; car lifted | `S07_MOT-U17_ENC-FLIPPED.uf2` | `S07_MOT-U17.txt` | S06 |
| S08 | MOT-U24..U26 | motion only, `MOTION_SYNC_TESTS` 1 | `S08_MOT-U24-U26.uf2` | `S08_MOT-U24-U26.txt` | S04, S06 |
| S09 | MOT-U28..U30 | motion only, `MOTION_FOLLOW_TESTS` 1 | `S09_MOT-U28-U30.uf2` | `S09_MOT-U28-U30.txt` | S04, S06 |
| S10 | MOT-U36, U37 | motion only, `MOTION_TASK_TESTS` 1 | `S10_MOT-U36-U37.uf2` | `S10_MOT-U36-U37.txt` | — |
| S11 | MOT-U39 | full, `make`, on a real line | `S11_MOT-U39.uf2` | `S11_MOT-U39.txt` | S09 |
| S12 | as S01, on the commit to be merged or demonstrated | full, `make TEST=1` | `S12_MOT-auto-final.uf2` | `S12_MOT-auto-final.txt` | all |

Every `make TEST=1` image also runs the automatic checks, so MOT-U01,
U03, U07, U18, U22, U23 and U27 appear in every test session's log; record
them from S01. If a session changes a calibration value, commit it and
rerun that session before the next one.

## Results

Add a new row for every run. Never edit or delete old rows.

The motor PWM changed from 10 kHz to 20 kHz in `b111dd4`. Earlier runs
and logs used 10 kHz, so their MOT-U01 lines say 10 kHz. Those results
do not depend on the PWM frequency: encoder counts, loop timing, wheel
diameter and stall detection. MOT-U08/U09 are rerun at 20 kHz to
confirm the stop-early distance, and MOT-U19/U20 are calibrated at
20 kHz.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| YYYY-MM-DD | MOT-U01 | `abc1234` | | PASS / FAIL | `logs/YYYY-MM-DD_….txt` | |
| 2026-11-03 | MOT-U01 | `ad5e45f` + local | Felicia Chua | PASS | `logs/2026-11-03_MOT-U04-U06.txt` | `make TEST=1 WIFI=none`, motion-only `app_main.c`, `MOTION_MANUAL_TESTS` 1 |
| 2026-11-03 | MOT-U03 | `ad5e45f` + local | Felicia Chua | PASS | `logs/2026-11-03_MOT-U04-U06.txt` | |
| 2026-11-03 | MOT-U04 | `ad5e45f` + local | Felicia Chua | PASS | `logs/2026-11-03_MOT-U04-U06.txt` | 5 turns each way: left ≈2497, right ≈2531 counts per rev (an unlogged 2026-11-02 run gave ≈2480–2500); set `MOTION_ENCODER_TICKS_PER_REV` 2490, ≈0.08 mm per count; MOT-U08 tunes the scale |
| 2026-11-03 | MOT-U05 | `ad5e45f` + local | Felicia Chua | PASS | `logs/2026-11-03_MOT-U04-U06.txt` | both counts fall by the same amount backward, with `MOTION_RIGHT_ENCODER_INVERTED` true |
| 2026-11-03 | MOT-U06 | `ad5e45f` + local | Felicia Chua | PASS | `logs/2026-11-03_MOT-U04-U06.txt` | each wheel moves only its own count, + forward. Unlogged 2026-11-02 runs showed each motor moving the other side's count: the encoders are wired left GP13/14, right GP2/3, so the pins were swapped in `motion_config.h` (TEAM.md updated) |
| 2026-11-03 | MOT-U07 | `ad5e45f` + local | Felicia Chua | PASS | `logs/2026-11-03_MOT-U04-U06.txt` | |
| 2026-11-03 | MOT-U18 | `ad5e45f` + local | Felicia Chua | PASS | `logs/2026-11-03_MOT-U04-U06.txt` | |
| 2026-11-03 | MOT-U22 | `ad5e45f` + local | Felicia Chua | PASS | `logs/2026-11-03_MOT-U04-U06.txt` | 1000 ms. Unlogged 2026-11-02 runs took 1500 ms: `tk_dly_tsk()` adds one 10 ms tick, so the test loops now ask for one tick less |
| 2026-11-03 | MOT-U23 | `ad5e45f` + local | Felicia Chua | PASS | `logs/2026-11-03_MOT-U04-U06.txt` | |
| 2026-11-03 | MOT-U27 | `ad5e45f` + local | Felicia Chua | PASS | `logs/2026-11-03_MOT-U04-U06.txt` | |
| 2026-11-03 | MOT-U02 | `4fe0403` + local | Felicia Chua | PASS | `logs/2026-11-03_MOT-U02.txt` | normal image, dual-core (`make WIFI=none`), all subsystems started, only the motors connected; 3 USB power-ups with the motor supply (power bank on VIN) on: neither wheel moved. VIN-only power-up not tested |
| 2026-11-03 | MOT-U08 | `dac27e4` + local | Felicia Chua | PASS | `logs/2026-11-03_MOT-U08-U10_calibration.txt` | calibration, before the stop-early fix; 3 runs: ruler ≈1005 mm each, odometry 1005, 1003, 1010; ruler and odometry agree, so `MOTION_WHEEL_DIAMETER_MM` 64 kept |
| 2026-11-03 | MOT-U09 | `dac27e4` + local | Felicia Chua | FAIL | `logs/2026-11-03_MOT-U08-U10_calibration.txt` | calibration; 3 runs: ruler ≈105 mm each (limit 97–103), odometry −114, −108, −111. Both moves stop about 5 mm past the target (coast after braking), so `MOTION_MOVE_STOP_EARLY_MM` set to 5 |
| 2026-11-03 | MOT-U10 | `dac27e4` + local | Felicia Chua | PASS | `logs/2026-11-03_MOT-U08-U10_calibration.txt` | calibration; 3 runs: stalled after 740, 580, 500 ms, braked, refused, cleared |
| 2026-11-03 | MOT-U10 | `dac27e4` + local | Felicia Chua | PASS | `logs/2026-11-03_MOT-U08-U10.txt` | with `MOTION_MOVE_STOP_EARLY_MM` 5; 3 runs: stalled after 460, 540, 720 ms (above the 300 ms timeout: small wheel movement in the grip restarts it), braked, refused, cleared. U08/U09 odometry 1003, 1003, 1002 / −110, −110, −106; ruler check of U08/U09 pending |
| 2026-11-05 | MOT-U01 | `0a0d6bd` + local | Felicia Chua | PASS | `logs/2026-11-05_S03_MOT-U04-U06.txt` | S03, `make TEST=1 WIFI=none`, motion-only `app_main.c`, `MOTION_MANUAL_TESTS` 1, 20 kHz |
| 2026-11-05 | MOT-U03 | `0a0d6bd` + local | Felicia Chua | PASS | `logs/2026-11-05_S03_MOT-U04-U06.txt` | S03 |
| 2026-11-05 | MOT-U07 | `0a0d6bd` + local | Felicia Chua | PASS | `logs/2026-11-05_S03_MOT-U04-U06.txt` | S03 |
| 2026-11-05 | MOT-U18 | `0a0d6bd` + local | Felicia Chua | PASS | `logs/2026-11-05_S03_MOT-U04-U06.txt` | S03 |
| 2026-11-05 | MOT-U22 | `0a0d6bd` + local | Felicia Chua | PASS | `logs/2026-11-05_S03_MOT-U04-U06.txt` | S03; 50 updates took 1000 ms |
| 2026-11-05 | MOT-U23 | `0a0d6bd` + local | Felicia Chua | PASS | `logs/2026-11-05_S03_MOT-U04-U06.txt` | S03 |
| 2026-11-05 | MOT-U27 | `0a0d6bd` + local | Felicia Chua | PASS | `logs/2026-11-05_S03_MOT-U04-U06.txt` | S03 |
| 2026-11-05 | MOT-U06 | `0a0d6bd` + local | Felicia Chua | PASS | `logs/2026-11-05_S03_MOT-U04-U06.txt` | S03; encoder change: LEFT FORWARD +5891 / right 0, LEFT BACKWARD −5853 / 0, RIGHT FORWARD 0 / +6046, RIGHT BACKWARD 0 / −5820 |
| 2026-11-05 | MOT-U04 | `0a0d6bd` + local | Felicia Chua | PASS | `logs/2026-11-05_S03_MOT-U04-U06.txt` | S03; 5 revolutions each way per wheel, counted between turning points. Left 2503–2584, mean 2539; right 2497–2555, mean 2525 (single-reading overshoots excluded: left 2621, right 2578 and 2579). `MOTION_ENCODER_TICKS_PER_REV` 2490 → 2532 (mean of both wheels); ≈0.08 mm per count. MOT-U08 must be repeated: the 64 mm wheel diameter was checked at 2490 |
| 2026-11-05 | MOT-U05 | `0a0d6bd` + local | Felicia Chua | PASS | `logs/2026-11-05_S03_MOT-U04-U06.txt` | S03; each backward revolution lowers the count by about one revolution; after 5 forward/backward pairs the left ends at 6 (start −16) and the right at 6 (start −21) |
| 2026-11-05 | MOT-U33 | `0a0d6bd` + local | Felicia Chua | PASS | `logs/2026-11-05_S03_MOT-U04-U06.txt` | S03 |
| 2026-11-05 | MOT-U34 | `0a0d6bd` + local | Felicia Chua | PASS | `logs/2026-11-05_S03_MOT-U04-U06.txt` | S03, motion only: period 19993..20009 us, longest update 440 us. Not the full-load result the test asks for; that comes from S01 |
| 2026-11-05 | MOT-U35 | `0a0d6bd` + local | Felicia Chua | PASS | `logs/2026-11-05_S03_MOT-U04-U06.txt` | S03; INHIBIT to brake 69 us |
| 2026-11-05 | MOT-U38 | `0a0d6bd` + local | Felicia Chua | PASS | `logs/2026-11-05_S03_MOT-U04-U06.txt` | S03; 11 updates in 200 ms |
| 2026-11-05 | MOT-U01 | `96a2911` + local | Felicia Chua | PASS | `logs/2026-11-05_S01_MOT-auto.txt` | S01, `make TEST=1` with Wi-Fi; local `app_main.c` starts every subsystem's task but runs only `motion_test()`; car lifted |
| 2026-11-05 | MOT-U03 | `96a2911` + local | Felicia Chua | PASS | `logs/2026-11-05_S01_MOT-auto.txt` | S01 |
| 2026-11-05 | MOT-U07 | `96a2911` + local | Felicia Chua | PASS | `logs/2026-11-05_S01_MOT-auto.txt` | S01 |
| 2026-11-05 | MOT-U18 | `96a2911` + local | Felicia Chua | PASS | `logs/2026-11-05_S01_MOT-auto.txt` | S01 |
| 2026-11-05 | MOT-U22 | `96a2911` + local | Felicia Chua | PASS | `logs/2026-11-05_S01_MOT-auto.txt` | S01; 50 updates took 1000 ms with every task running |
| 2026-11-05 | MOT-U23 | `96a2911` + local | Felicia Chua | PASS | `logs/2026-11-05_S01_MOT-auto.txt` | S01 |
| 2026-11-05 | MOT-U27 | `96a2911` + local | Felicia Chua | PASS | `logs/2026-11-05_S01_MOT-auto.txt` | S01 |
| 2026-11-05 | MOT-U33 | `96a2911` + local | Felicia Chua | PASS | `logs/2026-11-05_S01_MOT-auto.txt` | S01 |
| 2026-11-05 | MOT-U34 | `96a2911` + local | Felicia Chua | PASS | `logs/2026-11-05_S01_MOT-auto.txt` | S01, full load: obstacle, line_follow, terrain and communication tasks running. 52 updates, period 19925..20050 us (limit 19000..21000), longest update 343 us. Wi-Fi radio up but not joined (placeholder SSID, retrying); terrain IMU not connected (setup retrying) |
| 2026-11-05 | MOT-U35 | `96a2911` + local | Felicia Chua | PASS | `logs/2026-11-05_S01_MOT-auto.txt` | S01; INHIBIT to brake 200 us (limit 20000) |
| 2026-11-05 | MOT-U38 | `96a2911` + local | Felicia Chua | PASS | `logs/2026-11-05_S01_MOT-auto.txt` | S01; 11 updates in 200 ms, distance 0 mm, heading 0 deg |
| 2026-11-05 | MOT-U19 | `d5479f8` + local | Felicia Chua | PASS | `logs/2026-11-05_S04_MOT-U19-U21_calibration.txt` | S04 calibration, `make TEST=1 WIFI=none`, motion-only `app_main.c`, `MOTION_SPEED_TESTS` 1, lifted. Speed rises steadily with duty on both wheels: 250 → 194/204, 1000 → 728/749 mm/s. Fit over duty 200–600, both wheels: `MOTION_FF_OFFSET` 0.009, `MOTION_FF_GAIN` 0.00122 (worst miss 15 permille; 253 for 200 mm/s); `MOTION_MAX_SPEED_MM_S` 620 (15 % below the slower wheel's 728) |
| 2026-11-05 | MOT-U20 | `d5479f8` + local | Felicia Chua | FAIL | `logs/2026-11-05_S04_MOT-U19-U21_calibration.txt` | S04 calibration, before the U19 fit was applied (feed-forward 0.20 + 0.0012 × speed). First 200 mm/s step starts at duty 440 (253 needed): overshoot to 301/309 mm/s (+53 %), still 212/212 at 1500 ms. The 100 mm/s step and the second 200 mm/s step are within ±5 % after about 0.25 s. The `settled` line averages both setpoints and is not a pass measure |
| 2026-11-05 | MOT-U21 | `d5479f8` + local | Felicia Chua | PASS | `logs/2026-11-05_S04_MOT-U19-U21_calibration.txt` | S04 calibration, old feed-forward as for U20; floor, battery voltage not recorded. Open loop settled 304/308 mm/s (52 % fast); speed loop overshoot to 272 mm/s for about 0.3 s, then settled 202/201 mm/s (within ±5 %), at duty about 300–310 on the floor. Repeat after the U19 fit is applied; low-battery case not run |
| 2026-11-05 | MOT-U01 | `cd0d88b` + local | Felicia Chua | PASS | `logs/2026-11-05_S04_MOT-U19-U21.txt` | S04, `make TEST=1 WIFI=none`, motion-only `app_main.c`, `MOTION_SPEED_TESTS` 1 |
| 2026-11-05 | MOT-U03 | `cd0d88b` + local | Felicia Chua | PASS | `logs/2026-11-05_S04_MOT-U19-U21.txt` | S04 |
| 2026-11-05 | MOT-U07 | `cd0d88b` + local | Felicia Chua | PASS | `logs/2026-11-05_S04_MOT-U19-U21.txt` | S04 |
| 2026-11-05 | MOT-U18 | `cd0d88b` + local | Felicia Chua | PASS | `logs/2026-11-05_S04_MOT-U19-U21.txt` | S04 |
| 2026-11-05 | MOT-U22 | `cd0d88b` + local | Felicia Chua | PASS | `logs/2026-11-05_S04_MOT-U19-U21.txt` | S04; 50 updates took 1000 ms |
| 2026-11-05 | MOT-U23 | `cd0d88b` + local | Felicia Chua | PASS | `logs/2026-11-05_S04_MOT-U19-U21.txt` | S04 |
| 2026-11-05 | MOT-U27 | `cd0d88b` + local | Felicia Chua | PASS | `logs/2026-11-05_S04_MOT-U19-U21.txt` | S04 |
| 2026-11-05 | MOT-U19 | `cd0d88b` + local | Felicia Chua | PASS | `logs/2026-11-05_S04_MOT-U19-U21.txt` | S04, lifted; repeat of the calibration sweep: duty 250 → 197/205, 1000 → 723/750 mm/s |
| 2026-11-05 | MOT-U20 | `cd0d88b` + local | Felicia Chua | PASS | `logs/2026-11-05_S04_MOT-U19-U21.txt` | S04, lifted, feed-forward 0.009 + 0.00122 × speed (starts at duty 252). 0 → 200 mm/s: peak 215/223 (+8/+12 %), within ±5 % from 300 ms; 100 → 200: peak 227/230, within ±5 % from 240 ms; 200 → 100: undershoot to 72/69, right wheel within ±5 % only from about 620 ms (dips to 92–94). No sustained oscillation. The `settled` line averages both setpoints |
| 2026-11-05 | MOT-U21 | `cd0d88b` + local | Felicia Chua | PASS | `logs/2026-11-05_S04_MOT-U19-U21.txt` | S04, floor, USB left connected, battery voltage not recorded. Open loop settled 163/162 mm/s (19 % slow on the floor); speed loop settled 199/199 mm/s, within ±5 % from 240 ms and 189–205 after, at duty about 295–310 |
| 2026-11-05 | MOT-U21 | `cd0d88b` + local | Felicia Chua | NOT RUN | — | Low-battery case |
| 2026-11-05 | MOT-U33 | `cd0d88b` + local | Felicia Chua | PASS | `logs/2026-11-05_S04_MOT-U19-U21.txt` | S04 |
| 2026-11-05 | MOT-U34 | `cd0d88b` + local | Felicia Chua | PASS | `logs/2026-11-05_S04_MOT-U19-U21.txt` | S04, motion only: period 19991..20011 us, longest update 417 us |
| 2026-11-05 | MOT-U35 | `cd0d88b` + local | Felicia Chua | PASS | `logs/2026-11-05_S04_MOT-U19-U21.txt` | S04; INHIBIT to brake 62 us |
| 2026-11-05 | MOT-U38 | `cd0d88b` + local | Felicia Chua | PASS | `logs/2026-11-05_S04_MOT-U19-U21.txt` | S04; 11 updates in 200 ms |
| 2026-11-05 | MOT-U08 | `5336d44` + local | Felicia Chua | FAIL | `logs/2026-11-05_S05_MOT-U08-U10.txt` | S05 session 18:13, `make TEST=1 WIFI=none`, motion-only `app_main.c`, `MOTION_MOVE_TESTS` 1: `state 2 after 300 ms`, odometry 21 mm, car turned about 23° right. The right wheel did not turn on the floor (it did lifted, U06 check): a loose right motor pin; reseated |
| 2026-11-05 | MOT-U09 | `5336d44` + local | Felicia Chua | FAIL | `logs/2026-11-05_S05_MOT-U08-U10.txt` | S05 session 18:13: `state 2 after 300 ms`, odometry −21 mm; same loose pin as U08 |
| 2026-11-05 | MOT-U08 | `5336d44` + local | Felicia Chua | PASS | `logs/2026-11-05_S05_MOT-U08-U10.txt` | S05, pin reseated; 3 runs: ruler about 1000 mm each, odometry 1003, 1004, 1004; `state 0`. Ruler and odometry agree, so `MOTION_WHEEL_DIAMETER_MM` 64 kept at 2532 counts per rev |
| 2026-11-05 | MOT-U09 | `5336d44` + local | Felicia Chua | FAIL | `logs/2026-11-05_S05_MOT-U08-U10.txt` | S05; 3 runs: ruler 75, 80, 85 mm (limit 97–103), odometry −104 each, `state 0`. Castor flip, confirmed by watching both wheels: after the forward move the castor must swing 180°, and the car twists instead of moving straight back. The gentler start (duty 252 instead of 440) flips it more slowly; on 2026-11-03 at 440 the ruler read about 105 mm. Short reverses in the obstacle bypass are affected the same way |
| 2026-11-05 | MOT-U10 | `5336d44` + local | Felicia Chua | PASS | `logs/2026-11-05_S05_MOT-U08-U10.txt` | S05; 3 runs: stalled after 500, 720, 940 ms (above 300 ms: small wheel movement in the grip restarts the timer, as on 2026-11-03), braked, refused, cleared |
| 2026-11-05 | MOT-U08 | `32dd01f` + local | Felicia Chua | PASS | `logs/2026-11-05_S05_MOT-U08-U10_run2.txt` | S05 after `MOTION_FF_OFFSET` 0.05 (castor flip) and `MOTION_MOVE_STOP_EARLY_MM` 9 (coast with the stronger start); 3 runs: odometry 998, 996, 997 mm, `state 0`. Ruler agreed with odometry within a few mm on each run (tester's reading; individual ruler values not recorded) |
| 2026-11-05 | MOT-U09 | `32dd01f` + local | Felicia Chua | PASS | `logs/2026-11-05_S05_MOT-U08-U10_run2.txt` | S05, same settings; 3 runs: odometry −99, −102, −101 mm, `state 0`; ruler agreed with odometry within a few mm (tester's reading; individual ruler values not recorded). Castor flip no longer shortens the reverse: an earlier single run at offset 0.05 and stop-early 5 measured 105 mm by ruler against −104 by odometry |
| 2026-11-05 | MOT-U10 | `32dd01f` + local | Felicia Chua | PASS | `logs/2026-11-05_S05_MOT-U08-U10_run2.txt` | S05; 3 runs: stalled after 1440, 960, 720 ms (grip movement restarts the 300 ms timer), braked, refused, cleared |
| 2026-11-05 | MOT-U19 | `897215d` + local | Felicia Chua | PASS | `logs/2026-11-05_S04_MOT-U19-U21_rerun.txt` | S04 rerun with the final settings (`MOTION_FF_OFFSET` 0.05, `MOTION_SPEED_KI` 0.006, stop-early 9), lifted: duty 250 → 190/201, 1000 → 735/749 mm/s |
| 2026-11-05 | MOT-U20 | `897215d` + local | Felicia Chua | PASS | `logs/2026-11-05_S04_MOT-U19-U21_rerun.txt` | S04 rerun, lifted, starts at duty 294. 0 → 200 mm/s: peak 235/242 (+17/+21 %, the floor-tuned offset overshoots lifted), within ±5 % from 340/360 ms (right briefly 211–212 until 520 ms); 200 → 100: undershoot 65/61, within ±5 % by about 500 ms; 100 → 200: peak 235/238, within ±5 % from 260 ms. No sustained oscillation. Two earlier reruns that day, logs not kept: at KI 0.003 the 0 → 200 step settled in about 0.8 s (FAIL), and at KI 0.006 the right wheel slowed once per revolution (204 → 172 mm/s with duty rising) until the tight spot was freed |
| 2026-11-05 | MOT-U21 | `897215d` + local | Felicia Chua | PASS | `logs/2026-11-05_S04_MOT-U19-U21_rerun.txt` | S04 rerun, floor, battery voltage not recorded. Open loop settled 190/189 mm/s; speed loop within ±5 % from 220 ms, settled 198/196. In the last 0.5 s of the speed-loop run the right wheel's duty climbed to 409 while it slowed to 182–186 (left about 285): watch for floor or wheel friction |
| 2026-11-05 | MOT-U21 | `897215d` + local | Felicia Chua | NOT RUN | — | Low-battery case, final settings |
| 2026-11-05 | MOT-U08 | `897215d` + local | Felicia Chua | PASS | `logs/2026-11-05_S05_MOT-U08-U10_rerun.txt` | S05 confirmation run for `MOTION_SPEED_KI` 0.006 (single run): odometry 998 mm, `state 0`; ruler agreed with odometry within a few mm (tester's reading; ruler value not recorded) |
| 2026-11-05 | MOT-U09 | `897215d` + local | Felicia Chua | PASS | `logs/2026-11-05_S05_MOT-U08-U10_rerun.txt` | S05 confirmation run (single run): odometry −101 mm, `state 0`; ruler agreed with odometry within a few mm (tester's reading; ruler value not recorded) |
| 2026-11-05 | MOT-U10 | `897215d` + local | Felicia Chua | PASS | `logs/2026-11-05_S05_MOT-U08-U10_rerun.txt` | S05 confirmation run: stalled after 500 ms (grip movement restarts the 300 ms timer), braked, refused, cleared |
| 2026-11-05 | Host line acquisition | `0a0d6bd` | Felicia Chua | PASS | `logs/2026-11-05_S00_MOT-host.txt` | S00, `sh testing/motion/host/run.sh`: 12 checks, 0 failures. Software check only; no motor or sensor hardware |
| 2026-11-05 | MOT-U02 | `4707ca2` + local | Felicia Chua | PASS | `logs/2026-11-05_S02_MOT-U02.txt` (filtered) | S02, normal image with Wi-Fi, `MOTION_ONLY` 0, car lifted; 3 boots, `motors initialized and braked` each time; tester saw no wheel movement or twitch before the first commanded movement. First command: boot 1 line-adapter turn at about 5.4 s (path clear); boots 2 and 3 obstacle `reverse for room` at 5.81 s and 3.91 s (something in front of the sensor), so boot 3 is not a full 5 s of stillness. Wi-Fi not joined (placeholder SSID); terrain `NOT_LEVEL` because lifted |
| 2026-11-05 | MOT-U11 | `9fab33c` + local | Felicia Chua | FAIL | `logs/2026-11-05_S06_MOT-U11-U16_calibration.txt` | S06 calibration, motion-only `app_main.c`, `MOTION_TURN_TESTS` 1, track width 107.5 mm. By eye about 10° short per 360° (A about 1050; not taped); encoder total 1087°, `state 0` each run, so the encoders read more than the car turned. `MOTION_TRACK_WIDTH_MM` 107.5 → 110.6 (107.5 × 1080 ÷ 1050). Repeat by tape |
| 2026-11-05 | MOT-U12 | `9fab33c` + local | Felicia Chua | FAIL | `logs/2026-11-05_S06_MOT-U11-U16_calibration.txt` | S06 calibration; the same eye estimate as MOT-U11, not recorded per direction; encoder total −1089°, `state 0` each run |
| 2026-11-05 | MOT-U13 | `9fab33c` + local | Felicia Chua | PASS | `logs/2026-11-05_S06_MOT-U11-U16_calibration.txt` | S06 calibration; 5 left and 5 right looked good by eye, each turn not measured; encoder 91–93° each way, coast 1–2°. Repeat after the track width change |
| 2026-11-05 | MOT-U14 | `9fab33c` + local | Felicia Chua | PASS | `logs/2026-11-05_S06_MOT-U11-U16_calibration.txt` | S06 calibration; about 5° off by eye, at the ±5° limit, each turn not measured; encoder 181–183°, coast 1–2°. Repeat after the track width change |
| 2026-11-05 | MOT-U15 | `9fab33c` + local | Felicia Chua | PASS | `logs/2026-11-05_S06_MOT-U11-U16_calibration.txt` | S06 calibration; moving 1, replaced 1, turn done 1, move replaced turn 1, stopped 1; encoder turn heading 92° (target 90°) |
| 2026-11-05 | MOT-U16 | `9fab33c` + local | Felicia Chua | FAIL | `logs/2026-11-05_S06_MOT-U11-U16_calibration.txt` | S06 calibration; test not performed as written: the tester forgot to hold the left wheel, so the turn finished normally (`stalled 0 after 1080 ms`). Says nothing about stall detection. Repeat with the wheel held |
| 2026-11-05 | MOT-U11 | `3abd3cc` + local | Felicia Chua | PASS | `logs/2026-11-05_S06_MOT-U11-U16.txt` | S06 rerun, motion-only `app_main.c`, `MOTION_TURN_TESTS` 1, track width 110.6 mm. Tester reports the 360° turns are no longer off; no angle recorded. Encoder total 1087°, `state 0` each run |
| 2026-11-05 | MOT-U12 | `3abd3cc` + local | Felicia Chua | PASS | `logs/2026-11-05_S06_MOT-U11-U16.txt` | S06 rerun; tester reports the 360° turns are no longer off; no angle recorded. Encoder total −1088°, `state 0` each run |
| 2026-11-05 | MOT-U13 | `3abd3cc` + local | Felicia Chua | PASS | `logs/2026-11-05_S06_MOT-U11-U16.txt` | S06 rerun; 5 left and 5 right; tester reports the turns correct; no angle recorded. Encoder left 92–93°, right 92–94°, coast 1–2° |
| 2026-11-05 | MOT-U14 | `3abd3cc` + local | Felicia Chua | PASS | `logs/2026-11-05_S06_MOT-U11-U16.txt` | S06 rerun; 5 U-turns; tester reports the turns correct; no angle recorded. Encoder 182–183°, coast 1–2° |
| 2026-11-05 | MOT-U15 | `3abd3cc` + local | Felicia Chua | PASS | `logs/2026-11-05_S06_MOT-U11-U16.txt` | S06 rerun; moving 1, replaced 1, turn done 1, move replaced turn 1, stopped 1; encoder turn heading 92° (target 90°) |
| 2026-11-05 | MOT-U16 | `3abd3cc` + local | Felicia Chua | PASS | `logs/2026-11-05_S06_MOT-U11-U16.txt` | S06 rerun, left wheel held; stalled after 420 ms (expect about 300), braked, refused, cleared |
| 2026-11-05 | MOT-U17 | `b2a4b6d` + local | Felicia Chua | NOT RUN | `logs/2026-11-05_S07_MOT-U17_run1.txt` | S07 first run, `MOTION_LEFT_ENCODER_INVERTED` flipped, `MOTION_TURN_TESTS` 1, car lifted. `MOT-U17 turn refused`: the timed-out turn in MOT-U15 was never stopped, so MOT-U16 and MOT-U17 were refused. A test sequencing fault, not a motion fault; fixed by a `motion_move_stop()` after MOT-U15 |
| 2026-11-05 | MOT-U17 | `b2a4b6d` + local | Felicia Chua | PASS | `logs/2026-11-05_S07_MOT-U17.txt` | S07 rerun, `make TEST=1 WIFI=none`, car lifted. Local changes, none of them in the commit: `motion_config.h` `MOTION_LEFT_ENCODER_INVERTED` false → true and `MOTION_TURN_TESTS` 0 → 1, `app_main.c` motion-only (`MOTION_ONLY` 1), and the `motion_move_stop()` after MOT-U15 in `motion_test.c`. Tester's verdict read from the informational line `MOT-U17 state 4 after 3500 ms, retry 4`, which matches the expected result; the firmware prints no MOT-U17 PASS line. The log's `motion SUMMARY 11 passed, 2 failed` counts MOT-U15 and MOT-U16, which FAIL here because of the flipped flag; their results come from S06 |
| 2026-11-05 | MOT-U28 | `d6f1522` + local | Felicia Chua | PASS | `logs/2026-11-05_S09_MOT-U28-U30.txt` | S09, car lifted. Local changes, none in the commit: `motion_config.h` `MOTION_FOLLOW_TESTS` 0 → 1, `app_main.c` motion-only (`MOTION_ONLY` 1). Error +300: set L 33 R 206, speed L 33 R 207 mm/s; −300: set L 206 R 33, speed L 207 R 33; 0: equal. Stopped 100 ms after the last error (the check allows 140; the printed `limit 100` is the timeout alone) |
| 2026-11-05 | MOT-U29 | `d6f1522` + local | Felicia Chua | PASS | `logs/2026-11-05_S09_MOT-U28-U30.txt` | S09, car lifted; (a) and (b) ended 2 updates after the line appeared, (c) and (d) ended at their distance |
| 2026-11-05 | MOT-U30 | `d6f1522` + local | Felicia Chua | PASS | `logs/2026-11-05_S09_MOT-U28-U30.txt` | S09, floor; same local changes as MOT-U28. Tester's verdict: the firmware prints no MOT-U30 PASS line. Tester saw the car arc left, run straight, then arc right. `heading` lines: 127° (asked 135°, about 6 % off), 2° (asked 0°), −131° (asked −135°, about 3 % off), all within the limits. Each phase ran 1.5 s; the prompt's `1 s each` is rounded down by whole-second printing |

## Host regression: post-obstacle line acquisition

Run `sh testing/motion/host/run.sh` (host C compiler and sanitizer runtime).
This links the production `motion_move.c` and PID module to a simulated
stationary drive; no motor or sensor hardware is accessed.

Checks include a confirmed line already visible before the return/inward
turn is armed, fresh detection debounce, stale visibility, resetting the
mode on a new command, invalid state and retaining the edge-only API's
leave-and-reenter behaviour. Queued stop-at-line uses acquisition semantics;
`motion_move_stop_at_line()` remains edge-only for callers that need it.

Physical validation: with the car beyond the obstacle, start the inward turn
and return while the sensors already cover the line, then repeat with an
initially white surface and an approaching line. Both must brake on confirmed
line evidence and then align/follow. Record raw IR values and `line found,
aligning`; stable tracking and physical braking are NOT RUN until observed.

Host result (2026-11-04): 12 checks PASS with ASan/UBSan. The same regression
against the previous edge-only queued behaviour fails the already-visible
return, already-visible inward turn, and fresh-refresh cases (3 failures).
This is a software reproduction; it does not establish which failure the
reported physical run encountered without its current console capture.
