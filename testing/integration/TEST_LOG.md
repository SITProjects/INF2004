# Integration — test log

Tests that two or more subsystems work together. There is one test (or more) for every interface listed in TEAM.md section 5. Both owners agree on the test and sign off the result.

No interfaces exist yet, so there are no integration tests yet.

See `testing/README.md` for how to run tests and record results.

## Test cases

Define a test here before running it. Test IDs never change.

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| INT-01 | | IF-01 | | | |
| INT-02 | start-up hold | IF-06 | manual | Bypass enabled; power on with a board 150 mm ahead, then remove it | Motors stay held (`motors held until the path is confirmed clear`); FOLLOW only after removal |
| INT-03 | scanner failure | IF-06 | manual | Unplug the ultrasonic, power on | `scanner failed, motors stay held`; the car never moves |
| INT-04 | stop | IF-06, IF-02 | manual | Following at 120 mm/s; place a board in the path | `avoid IDLE -> STOPPING` and mode INHIBITED in the same loop as `stop request ACTIVE`; the car stops before the critical distance |
| INT-05 | false alarm | IF-06 | manual | Remove the board before confirmation | Releases, FOLLOW, keeps following |
| INT-06 | box left | IF-06, IF-01, IF-03 | manual | 160 mm box centred on a straight; room on the left | PREPARE before the sidestep, pass and return; VERIFY CLEAR; `line found`; FOLLOW; total time recorded |
| INT-07 | box right | IF-06 | manual | Box shifted left of the line | Mirror of INT-06 |
| INT-08 | edge outside the sweep | IF-06 | manual | Box wider than the sweep sees on the gap side | Maximum sidestep; corridor SHIFT or CLEAR; passes without contact, or BLOCKED_WAIT then FAULT |
| INT-09 | repeated sidestep | IF-06 | manual | Box wider than its estimate | Extra sidestep(s); the return leg is longer by the same total; `line found` |
| INT-10 | blocked | IF-06 | manual | Boards on both sides | WAIT_CLEAR on the line; removing the front board gives FOLLOW |
| INT-11 | stall retry | IF-06, IF-03 | manual | Hold a wheel briefly during the pass | RETRY_WAIT, then the remainder of the pass and the rest of the box |
| INT-12 | stall fault | IF-06 | manual | Hold a wheel through the retry | FAULT; clearing the path does not restart the car |
| INT-13 | critical during a move | IF-06 | manual | Hand within 80 mm during the pass | INHIBIT, BLOCKED_WAIT; hand away resumes the pass |
| INT-14 | line missed | IF-06 | manual | `OBSTACLE_AVOID_RETURN_EXTRA_MM` set too short (temporary) | One search move; if still missed, FAULT |
| INT-15 | obstacle state for other tasks | IF-07 | manual | A task polls `obstacle_get_state()` and waits on `obstacle_flg_event` while a board comes and goes | STOP and CRITICAL follow the scanner; RESULT on each new sequence; the copy is never torn |
| INT-17 | clearest side selection | obstacle scanner → planner | manual | Arrange two clear side paths with unequal measured ranges but a shorter sidestep on the nearer side; repeat mirrored | Actual turn follows the farther clear side, independent of sidestep size. Capture raw ends data and chosen turn; physical result NOT RUN |

*Method:* **auto** = `TEST_CHECK` in code, result printed on the console;
**manual** = a person observes the result (LED, scope, meter, phone app…).

## Results

Add a new row for every run. Never edit or delete old rows.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| YYYY-MM-DD | INT-01 | `abc1234` | | PASS / FAIL | `logs/YYYY-MM-DD_….txt` | |
