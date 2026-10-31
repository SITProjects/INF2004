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
| OBS-U04 | obstacle inside the configured detection threshold starts a side scan | | manual | 1. Hold a hand 3 cm in front of the sensor. 2. Watch the servo and console. | servo turns to 60° then 120°; console reports `PATH_FOUND` or `BLOCKED` after both side positions finish |
| OBS-U10 | packed obstacle fields at boundaries and invalid values | robot_types.h | auto | `obstacle_test.c`: valid flags, min/max/one-beyond/invalid fields, edge sentinels, width saturation and sequence wrap | `[TEST] OBS-U10 PASS`; neighbouring fields unchanged |

Additional regression cases (U05–U09 and U11 remain reserved for the sweep plan):

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| OBS-U12 | no-return and beyond-range policy | driver → scanner | manual | Scope TRIG/ECHO in open space; include a pulse lasting about 38 ms, if produced by this module. Record trigger-to-rise, pulse duration, driver outcome and scanner status. | NO_ECHO then CLEAR; late falling edge does not overwrite a result; next trigger waits for configured interval |
| OBS-U13 | malformed or stuck-high echo | driver → scanner | manual | Check ECHO high before trigger, duplicate/ambiguous edges and a zero pulse using controlled input. | FAULT; never classed as a no-return clear reading |
| OBS-U14 | directional no-return ranking | scanner | manual | Compare a no-return side against a finite clear side, swap sides, then test two no-return sides. | No-return side wins; two no-return sides select left; width remains unavailable and edge flags remain clear |
| OBS-U15 | active pin allocation | config → hardware | manual | Scope trigger output and inject ECHO on GP17; verify GP2/GP3 are not configured by obstacle. | TRIG GP16, ECHO GP17, servo GP12; one configured scan priority |
| OBS-U16 | ping median | `obstacle_decision.c` | auto | `obstacle_test.c`: zeros ignored (not majority voting: `{0, 0, 300}` gives 300 us), two nonzero samples averaged without overflow, all-zero gives 0, NULL/zero count gives 0, only the first 3 samples used | `[TEST] OBS-U16 PASS` |
| OBS-U17 | position result after its pings | `obstacle_decision.c` | auto | `obstacle_test.c`: exhausted retries give FAULT regardless of median; zero median gives NO_ECHO; otherwise VALID | `[TEST] OBS-U17 PASS` |
| OBS-U18 | centre decision at the detection threshold | `obstacle_decision.c` | auto | `obstacle_test.c`: NO_ECHO, echo at and above `OBSTACLE_DETECT_ECHO_US` → CLEAR; one below and 1 µs → scan sides; FAULT/PENDING → UNKNOWN | `[TEST] OBS-U18 PASS` |
| OBS-U19 | side selection | `obstacle_decision.c` | auto | `obstacle_test.c`: 14 cases covering both near, threshold boundary, ties, farther side, no-echo ranking, faults with and without a clear side | `[TEST] OBS-U19 PASS`; on failure the console names the failing case number |

Calibration procedures are in [CALIBRATION.md](CALIBRATION.md). OBS-U20–U27
remain reserved for the critical/freshness plan. These hardware tests are
pending; firmware builds do not constitute a measurement or PASS.

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

Add a new row for every run. Never edit or delete old rows.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| YYYY-MM-DD | OBS-U01 | `abc1234` | | PASS / FAIL | `logs/YYYY-MM-DD_….txt` | |
