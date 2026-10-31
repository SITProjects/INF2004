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
| OBS-U04 | obstacle inside the configured detection threshold starts a side scan | | manual | 1. Place a flat board at 150 mm (inside the configured detection distance and above critical release). 2. Watch the servo and console through both side positions and return to centre. | servo turns to 60° then 120°; console reports `PATH_FOUND` or `BLOCKED` after both side positions finish |
| OBS-U10 | packed obstacle fields at boundaries and invalid values | robot_types.h | auto | `obstacle_test.c`: valid flags, min/max/one-beyond/invalid fields, edge sentinels, width saturation and sequence wrap | `[TEST] OBS-U10 PASS`; neighbouring fields unchanged |

Additional regression cases (U05–U09 and U11 remain reserved for the sweep plan):

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| OBS-U12 | no-return and beyond-range policy | driver → scanner | manual | Scope TRIG/ECHO in open space; include a pulse lasting about 38 ms, if produced by this module. Record trigger-to-rise, pulse duration, driver outcome and scanner status. | NO_ECHO then CLEAR; late falling edge does not overwrite a result; next trigger waits for configured interval |
| OBS-U13 | malformed or stuck-high echo | driver → scanner | manual | Check ECHO high before trigger, duplicate/ambiguous edges and a zero pulse using controlled input. | FAULT; never classed as a no-return clear reading |
| OBS-U14 | directional no-return ranking | scanner | manual | Compare a no-return side against a finite clear side, swap sides, then test two no-return sides. | No-return side wins; two no-return sides select left; width remains unavailable and edge flags remain clear |
| OBS-U15 | active pin allocation | config → hardware | manual | Scope GP16 TRIG width and nominal forward trigger cadence in a stable clear-path centre scan (allow polling jitter; side scans have different cadence); scope GP12 servo period/centre width. Confirm GP17 input changes with sensor distance. Inspect GPIO function/output-enable registers or debugger pinmux state and obstacle source to establish ownership of GP2/GP3; quiet pins alone do not establish pin allocation. | TRIG GP16, ECHO GP17, servo GP12; one configured scan priority |
| OBS-U16 | ping median | `obstacle_decision.c` | auto | `obstacle_test.c`: zeros ignored (not majority voting: `{0, 0, 300}` gives 300 us), two nonzero samples averaged without overflow, all-zero gives 0, NULL/zero count gives 0, only the first 3 samples used | `[TEST] OBS-U16 PASS` |
| OBS-U17 | position result after its pings | `obstacle_decision.c` | auto | `obstacle_test.c`: exhausted retries give FAULT regardless of median; zero median gives NO_ECHO; otherwise VALID | `[TEST] OBS-U17 PASS` |
| OBS-U18 | centre decision at the detection threshold | `obstacle_decision.c` | auto | `obstacle_test.c`: NO_ECHO, echo at and above `OBSTACLE_DETECT_ECHO_US` → CLEAR; one below and 1 µs → scan sides; FAULT/PENDING → UNKNOWN | `[TEST] OBS-U18 PASS` |
| OBS-U19 | side selection | `obstacle_decision.c` | auto | `obstacle_test.c`: 14 cases covering both near, threshold boundary, ties, farther side, no-echo ranking, faults with and without a clear side | `[TEST] OBS-U19 PASS`; on failure the console names the failing case number |

Critical/freshness cases (all on-target and physical results remain pending):

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| OBS-U20 | critical predicate boundary/status handling | `obstacle_decision.c` | auto | VALID at `OBSTACLE_CRITICAL_ECHO_US - 1`, at threshold and UINT16_MAX; NO_ECHO/FAULT/PENDING | Only VALID below threshold is critical |
| OBS-U21 | critical release predicate | `obstacle_decision.c` | auto | VALID at `OBSTACLE_CRITICAL_RELEASE_ECHO_US - 1` and at threshold; NO_ECHO/FAULT/PENDING | Only VALID below release threshold blocks release |
| OBS-U22 | strict wrap-safe elapsed limit | `obstacle_decision.c` | auto | Equal, one above, and uint32 wrap: now 5, since 0xFFFFFFF0, limits 20/21 | Equality false; 21 elapsed exceeds 20 but not 21 |
| OBS-U23 | forward freshness watchdog and recovery | scanner | auto (bench: clear path) | Await clear/released snapshot; request trigger hold and await scanner acknowledgement. Save stale-event counter/time baseline. Wait from configured `OBSTACLE_FORWARD_MAX_AGE_MS` plus scheduling/transaction margin, then release hold on every exit and await fresh clear recovery. | Specific stale watchdog event increments with timestamp; UNKNOWN and stop asserted; subsequently fresh CLEAR/released. Stop alone is insufficient evidence |
| OBS-U24 | side-scan watchdog and recovery | scanner | auto (bench: board) | Place board at 150 mm when prompted. Await side scan; request trigger hold and await acknowledgement; await trigger-hold/drain acknowledgement. Save scan-timeout event baseline. Wait `OBSTACLE_SCAN_TIMEOUT_MS` plus polling margin; release hold on every exit, remove board, await centre recovery. | Specific side-timeout event increments with timestamp; sides cancelled, UNKNOWN and stop asserted; fresh CLEAR/released after removal |
| OBS-U25 | critical hysteresis through distinct centre groups | scanner → obstacle word | manual | Hold a flat board below `OBSTACLE_CRITICAL_DISTANCE_MM` until observed centre critical set. Hold separately within critical/release band until a completed centre group is observed; then hold above `OBSTACLE_CRITICAL_RELEASE_DISTANCE_MM` through another completed centre group. Record distance, centre-group observations and flags at each stage; allow complete side scan and return before each centre group. | Critical sets below threshold, remains set during band centre group, and clears only after completed above-release centre group with no blocking VALID sample. Two-second holds alone are inadequate during sides |
| OBS-U26 | confirmed ECHO disconnection fault progression | driver → scanner | manual | From fresh CLEAR/released, document electrical ECHO input level/bias, unplug ECHO and confirm input state (floating is inconclusive). Save individual fault outcomes, first stop assertion, group exhaustion and UNKNOWN timestamps. Reconnect and verify recovery. | First consumed FAULT requests stop; exhausted configured group attempts publish UNKNOWN (not necessarily the first fault or exactly three attempts). No unplug-to-event bound without instrumented disconnection origin; elapsed from confirmed first fault is separately reported |
| OBS-U27 | critical timing measurement | scanner | manual / APP_TEST instrumentation | Capture matched consumed-centre-reading and critical-set microsecond timestamps/sequence using APP_TEST instrumentation; save actual delta and timer resolution for five set transitions. Never subtract adjacent console lines. Physical arrival timing needs separate future instrumentation. | Flag sets within the same scanner update; report measured consumed-reading→critical-set interval separately from trigger→event and physical-arrival→event. No 95 ms critical deadline: forward period + transaction + polling tick is a healthy forward freshness budget only |

Calibration procedures are in [CALIBRATION.md](CALIBRATION.md). These hardware
tests are pending; firmware builds do not constitute a measurement or PASS.

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| OBS-U28 | Distance scale and offset | raw echo → distance | manual, mode 1 | Nine distances 50–1000 mm; record outcome counts, valid medians and fit residuals; see calibration procedure | Residuals within ±max(5 mm, 2%); VALID spread ≤35 us through 500 mm; invalid samples investigated |
| OBS-U29 | Centre trim and side angles | logical → servo command | manual, modes 2/4 | Mechanically measure centre and 60/90/120; wall sweep is coarse only; trim = raw centre command −90 | Actual angles within ±3 degrees; record wall asymmetry and mounting effects |
| OBS-U30 | Servo travel and settle margin | move → usable ping | manual, video/modes 3/4 | Measure command-to-settled in both directions; resting reference and ten trials per margin; record actual trigger delays | All ten VALID echoes within ±30 us at selected and larger margins; uncertainty and reverse-direction validation recorded |
| OBS-U31 | Critical distance geometry | vehicle envelope | manual, ruler | Measure a, s, W; confirm pivot/envelope assumptions and calculate proposed critical/release thresholds | Measurements and calculation recorded; thresholds ordered; no claim of braking validation |
| OBS-U32 | Sensor limits | echo result semantics | manual, mode 1 | Minimum range, angled board, narrow and soft targets; record individual results and VALID percentages | Blind spots and demo-course risks documented; no automatic claim of reliable detection |

For OBS-U12 additionally capture 20 open-space and 20 300 mm scope samples,
record missing rises and the driver's result, and reconcile the driver's
pre-pulse timeout origin with TRIG-falling measurements. Do not require an
unobserved all-rise response or presume every pulse below 23000 us is clear.
The current no-rise FAULT policy remains until hardware evidence supports a
separate decision. All calibration observations and normal-mode follow-up
verification are pending.

Compile and simulated regression results are software evidence only. Physical
scope observations, voltage checks and actual sensor behavior remain pending.

*Method:* **auto** = `TEST_CHECK` in code, result printed on the console;
**manual** = a person observes the result (LED, scope, meter, phone app…).

## Results

Add a new row for every run. Never edit or delete old rows. Hardware has not
been run for this implementation. Unavailable subcases are NOT RUN with a reason;
never use PASS for a build, host simulation, lost log, or unperformed procedure.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| YYYY-MM-DD | OBS-U01 | `abc1234` | | PASS / FAIL | `logs/YYYY-MM-DD_….txt` | |

U26 and U27 use TEST=1 after the entire automatic run ends and all trigger
holds have been released. U26 records the APP_TEST centre FAULT timestamp,
retry-before-increment and attempt diagnostic with the UNKNOWN publication;
it is a software-consumption timestamp. U25 observes completed centre decisions
through the subsequent `side scan started` event while below detection, or
CLEAR publication above detection, alongside critical set/clear events.
