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
| COM-U04 | telemetry only, no heartbeat | MQTT=1, configured broker | manual | Follow MQTT_HEARTBEAT.md; observe telemetry and monitor base MQTT_TOPIC for 15 s | Obstacle records arrive on MQTT_TOPIC/telemetry; no hb messages on base topic |
| COM-U05 | broker unavailable and restart | MQTT reconnect | manual | Boot with broker stopped; start it, then stop and restart it twice | Other tasks continue; connection recovers and queued telemetry resumes |
| COM-U06 | Wi-Fi loss and recovery with MQTT | radio ownership | manual | After telemetry arrives, disable AP, then restore it | Wi-Fi and MQTT recover without reset; queued samples replay with original capture times |
| COM-U07 | bounded MQTT receive logging (retired) | no subscriptions | manual | Not applicable after heartbeat echo removal | Client no longer subscribes to or logs incoming topic messages |
| COM-U08 | Wi-Fi outage telemetry replay | IF-01, MQTT=1 | manual | Follow TELEMETRY_REPLAY.md; interrupt AP for 20-30 s while keeping power and local subscriber | Accepted samples replay with original sequence/capture time; PUBACK removes them; duplicates allowed |
| COM-U09 | Broker outage telemetry replay | IF-01, QoS 1 | manual | Stop broker 20 s, restart; inspect serial/broker ACK logs | Pending records survive reconnect and drain; other tasks continue |
| COM-U10 | Outbox capacity and overflow | IF-01, 128 records | manual | Interrupt Wi-Fi over 140 s then restore | Full rejects newest visibly; oldest accepted data replays unchanged; new samples resume |


*Method:* **auto** = `TEST_CHECK` in code, result printed on the console;
**manual** = a person observes the result (LED, scope, meter, phone app…).

## Results

Add a new row for every run. Never edit or delete old rows.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| YYYY-MM-DD | COM-U01 | `abc1234` | | PASS / FAIL | `logs/YYYY-MM-DD_….txt` | |
