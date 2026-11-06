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
| BAR-U04 | every symbol both ways, ±20 % width jitter, rejection of a weak ratio / no frame / inner '*' / truncation / small buffer / empty symbol, scan assembly across the timer wrap, back-to-back symbols, glitches and over-long bars, chatter on every edge | Code 39 | auto (host) | `sh testing/barcode/host/run.sh` in the dev container | `0 failure(s)` (BT01..BT08 PASS) |
| BAR-U05 | the barcode sensor's DO flips at bar edges | IR module | manual | 1. Flash `make MQTT=1 IR_CALIBRATION=1` (motors off) and log the MQTT console. 2. Push the car by hand along the line past a barcode, steadily, so the sensor passes over it at its mounted height. 3. Turn the potentiometer until scans print. | `barcode: <text> dir=fwd` or `rev`; a `FAIL ratio` or `FAIL length` means the threshold is off or a bar was missed, and the `barcode: widths` line after it shows which |
| BAR-U06 | the car reads a barcode beside the line at follow speed, both directions | Code 39, IF-04 | manual | 1. Flash `make MQTT=1`. 2. Follow a line past a barcode placed 2 cm beside it, then the other way round. 3. Save the console and the MQTT records. | `barcode: <text> dir=fwd` one way and `dir=rev` the other, and a `barcode <text> fwd/rev` MQTT record each time; no `FAIL` lines |

*Method:* **auto** = `TEST_CHECK` in code, result printed on the console;
**manual** = a person observes the result (LED, scope, meter, phone app…).

## Results

Add a new row for every run. Never edit or delete old rows.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| YYYY-MM-DD | BAR-U01 | `abc1234` | | PASS / FAIL | `logs/YYYY-MM-DD_….txt` | |
