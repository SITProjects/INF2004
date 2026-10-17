# Interfaces

Interfaces between subsystems are kernel objects (mailboxes, message
buffers, event flags, shared data under a mutex) created by the subsystem
that owns the data and published in its `<name>.h`. Each one gets an ID in
`TEAM.md` section 5 and an integration test in
`testing/integration/TEST_LOG.md`.

None exist yet: every subsystem runs on its own. The open TODOs that need
interfaces are in `app_program/obstacle/obstacle.c` (act on scan results)
and the line-following commands, which nothing drives the motors with yet.

Functions that are already safe to call from any task:

| Function | Header | Notes |
|---|---|---|
| `terrain_monitor_get_latest()` | `app_program/terrain/terrain_monitor.h` | latest acceleration snapshot |
| `terrain_monitor_get_calibration()` | `app_program/terrain/terrain_monitor.h` | calibration details |
| `terrain_monitor_recalibrate()` | `app_program/terrain/terrain_monitor.h` | restart calibration |
| `connectivity_is_connected()` | `app_program/communication/connectivity.h` | Wi-Fi joined with an IP address |
