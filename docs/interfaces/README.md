# Interfaces

Interfaces between subsystems are kernel objects (mailboxes, message
buffers, event flags, shared data under a mutex) created by the subsystem
that owns the data and published in its `<name>.h`. Each one gets an ID in
`TEAM.md` section 5 and an integration test in
`testing/integration/TEST_LOG.md`.

The obstacle bypass (IF-06, off until the car is measured) is in
`app_program/obstacle/obstacle_avoid.c`, run by the scan task in
`app_program/obstacle/obstacle.c`. The line-following commands are still
open.

Functions that are already safe to call from any task:

| Function | Header | Notes |
|---|---|---|
| `terrain_monitor_get_latest()` | `app_program/terrain/terrain_monitor.h` | latest acceleration snapshot |
| `terrain_monitor_get_calibration()` | `app_program/terrain/terrain_monitor.h` | calibration details |
| `terrain_monitor_recalibrate()` | `app_program/terrain/terrain_monitor.h` | restart calibration |
| `connectivity_is_connected()` | `app_program/communication/connectivity.h` | Wi-Fi joined with an IP address |
| `obstacle_get_state()` | `app_program/obstacle/obstacle.h` | latest obstacle result, geometry, stop request and bypass state (IF-07) |
