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
| COM-U04 | MQTT heartbeat round trip | MQTT=1, configured broker | manual | Follow MQTT_HEARTBEAT.md; observe at least three TX/RX pairs and subscriber messages | `hb 0`, `hb 1`, `hb 2` on MQTT_TOPIC, approximately 5 s apart; first heartbeat after connection + 5 s |
| COM-U05 | broker unavailable and restart | MQTT reconnect | manual | Boot with broker stopped; start it, then stop and restart it twice | Other tasks continue; connection eventually recovers after timeout/retry; heartbeat cadence stays about 5 s without duplicates |
| COM-U06 | Wi-Fi loss and recovery with MQTT | radio ownership | manual | After heartbeats arrive, disable AP, then restore it | Wi-Fi and MQTT recover without reset; heartbeats pause while disconnected and resume with continuing counter |
| COM-U07 | bounded MQTT receive logging | fragment handling | manual | Publish a payload longer than 127 bytes to MQTT_TOPIC; then publish a normal message | Long message is truncated and marked; subsequent normal message prints correctly; no reset |

*Method:* **auto** = `TEST_CHECK` in code, result printed on the console;
**manual** = a person observes the result (LED, scope, meter, phone app…).

## Results

Add a new row for every run. Never edit or delete old rows.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| YYYY-MM-DD | COM-U01 | `abc1234` | | PASS / FAIL | `logs/YYYY-MM-DD_….txt` | |
