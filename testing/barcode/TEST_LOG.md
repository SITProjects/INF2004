# Barcode — test log

**Subsystem:** barcode  ·  **Student ID:** <student ID>  ·  **Code:** `app_program/barcode/`

See `testing/README.md` for how to run tests and record results.

## Test cases

Define a test here before running it. Test IDs never change.

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| BAR-U01 | the decoder reads a built-in `*INF2004*` forwards and backwards | Code 39 | auto | `barcode_test.c` | `[TEST] BAR-U01 PASS` |
| BAR-U02 | tBarcode attached its interrupt and sleeps between polls | TEAM.md section 4 | auto | `barcode_test.c` | `[TEST] BAR-U02 PASS` |
| BAR-U03 | GP28 is a digital input (SIO, input on, output off) | TEAM.md section 2 | auto | `barcode_test.c` | `[TEST] BAR-U03 PASS` |
| BAR-U04 | every symbol both ways, ±20 % width jitter, rejection of a weak ratio / no frame / inner '*' / truncation / small buffer / empty symbol, scan assembly across the timer wrap, back-to-back symbols, glitches, over-long bars ending a scan at the last bar, chatter on every edge, thin bars with a stretched white gap, a stale level between scans, a real scan from the car, a flicker at the first bar, partial reads of logged scans | Code 39 | auto (host) | `sh testing/barcode/host/run.sh` in the dev container | `0 failure(s)` (BT01..BT14 PASS) |
| BAR-U05 | the barcode sensor's DO flips at bar edges | IR module | manual | 1. Flash `make MQTT=1 IR_CALIBRATION=1` (motors off) and log the MQTT console. 2. Push the car by hand along the line past a barcode, steadily, so the sensor passes over it at its mounted height. 3. The image prints `barcode: B=<level> raw=<edges> kept=<edges>` twice a second: turn the potentiometer until `raw` stays near 0 on plain white and on a black bar, with `B` following. 4. Push again until scans print. | `barcode: <text> dir=fwd` or `rev`; a `FAIL ratio` or `FAIL length` means the threshold is off or a bar was missed, and the `barcode: widths` line after it shows which |
| BAR-U06 | the car reads a barcode beside the line at follow speed, both directions | Code 39, IF-04 | manual | 1. Flash `make MQTT=1`. 2. Follow a line past a barcode placed 2 cm beside it, then the other way round. 3. Save the console and the MQTT records. | `barcode: <text> dir=fwd` one way and `dir=rev` the other, and a `barcode <text> fwd/rev` MQTT record each time; no `FAIL` lines |
| BAR-U07 | the car stops for a barcode, backs up, creeps over it at 30 mm/s and speeds up again | Code 39, IF-08 | manual | 1. Flash `make MQTT=1` with `BARCODE_EDGE_REPORT` 1. 2. Start the dashboard, press Start and drive at a barcode on a straight. 3. Expect the events `line BARCODE BRAKE`, `BACK`, `CREEP`, then a read or FAIL, then `DONE`. 4. Check the sensor was before the first bar when the creep began, and that the car rejoins the line at 80 mm/s. |

*Method:* **auto** = `TEST_CHECK` in code, result printed on the console;
**manual** = a person observes the result (LED, scope, meter, phone app…).

## Results

Add a new row for every run. Never edit or delete old rows.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| YYYY-MM-DD | BAR-U01 | `abc1234` | | PASS / FAIL | `logs/YYYY-MM-DD_….txt` | |
| 2026-11-06 | BAR-U04 | `419d3f0` | Claude (host) | PASS | `sh testing/barcode/host/run.sh` | BT01..BT11, 0 failures |
| 2026-11-06 | BAR-U06 | `419d3f0` + `BARCODE_EDGE_REPORT` 1 | Kitsunez | PARTIAL | `logs/2026-11-06_BAR-U06_run3_mqtt.txt` (not committed) | Two runs round the course at 80 mm/s, 3 full passes (about 30 edges each): 2 read `F dir=fwd`, both with the heading steady; the third failed (`FAIL length`, 17 elements) with the heading swinging. No wrong text. `dir=rev` not tested: the car only went one way. Small sample |
| 2026-11-06 | BAR-U04 | `72180b5` + long black fix | Claude (host) | PASS | `sh testing/barcode/host/run.sh` | BT01..BT17, 0 failures |
| 2026-11-06 | BAR-U06 | `72180b5` + `BARCODE_EDGE_REPORT` 1 | Kitsunez | FAIL | `../system/logs/2026-11-06_dash2_mqtt.txt` (not committed) | 7 passes, 0 clean reads: 4 dropped with no result by a black over 400 ms, 2 short scans, 1 false partial `.`. 22 to 25 edges a pass, against about 31 before. |
| 2026-11-06 | BAR-U06 | `e6be5e4` + `BARCODE_EDGE_REPORT` 1 | Kitsunez | FAIL | `../system/logs/2026-11-06_dash3_mqtt.txt` (not committed) | 5 passes of the smaller barcode (about 190 mm, `*F*` backwards), no read. The gap between the two wide bars of `*` read 64 ms at 18:25 and 12 ms here, and in 3 passes DO did not flip for it at all: the sensor has read far blacker since some time between 18:40 and 18:55. Re-run BAR-U05. |
| 2026-11-06 | BAR-U04 | `7edb636` + slow pass | Claude (host) | PASS | `sh testing/barcode/host/run.sh` | BT01..BT20, 0 failures. `barcode.c`'s phase handling has no host test. |
| 2026-11-07 | BAR-U04 | hump holds the trigger | Claude (host) | PASS | `sh testing/barcode/host/run.sh` | BT21 to BT26 added. Not run on the car or on a real hump. On a hump the sensor sits too close to the floor to trust, so edges cannot start a slow pass until 0.5 s after it, at most 4 s per hump: a car calibrated on a slope would otherwise read as on a hump for the whole run. Needs calibration VALID. In the model the hump state starts 40 to 61 mm in at 120 mm/s, at 6 to 8 mm of lift, so the start of the ramp is not covered. |
| 2026-11-07 | BAR-U01 | longer back-up | Claude (host) | NOT RUN | `testing/system/logs/2026-11-06_dash5_mqtt.txt` boots 35 to 48 | The user reported the car missing barcodes. 32 passes at 120 mm/s, 22 read: the car ended a median 74 mm back from braking, and late triggers left it inside the barcode (23 of 29 elements, boot 48). LINE_BARCODE_REVERSE_MM 100 to 170, LINE_BARCODE_CREEP_MM 450 to 520, BARCODE_CREEP_QUIET_US 5 to 7 s so the longer run-up does not end the pass. Costs about 2.3 s more per pass. Whether the hump hold makes triggers late is not established. |
