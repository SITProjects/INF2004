# Communication (Wi-Fi) — test log

**Subsystem:** communication  ·  **Student ID:** <student ID>  ·  **Code:** `app_program/communication/`

See `testing/README.md` for how to run tests and record results.

## Test cases

Define a test here before running it. Test IDs never change.

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| COM-U01 | kernel radio service has the CYW43 radio ready within 10 s | | auto | `communication_test.c` | `[TEST] COM-U01 PASS` |
| COM-U02 | station joins the network and gets an IP within 30 s | | auto | `communication_test.c`; network from `config/wifi_secrets.h` in range | `[TEST] COM-U02 PASS` |
| COM-U03 | link loss is reported | | manual | 1. Flash normal image and wait for `Wi-Fi connected`. 2. Switch the access point off. | `Wi-Fi unavailable; the radio service retries automatically`, then `Wi-Fi connected` again once the AP is back |

*Method:* **auto** = `TEST_CHECK` in code, result printed on the console;
**manual** = a person observes the result (LED, scope, meter, phone app…).

## Results

Add a new row for every run. Never edit or delete old rows.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| YYYY-MM-DD | COM-U01 | `abc1234` | | PASS / FAIL | `logs/YYYY-MM-DD_….txt` | |
