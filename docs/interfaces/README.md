# Interfaces

Interfaces between subsystems are kernel objects (mailboxes, message
buffers, event flags, shared data under a mutex) created by the subsystem
that owns the data and published in its `<name>.h`. Each one gets an ID in
`TEAM.md` section 5 and an integration test in
`testing/integration/TEST_LOG.md`.

The obstacle bypass (IF-06, off until the car is measured) is in
`app_program/obstacle/obstacle_avoid.c`, run by the scan task in
`app_program/obstacle/obstacle.c`. Line following (IF-04) is a
latest-value request from the line task to `tMotion`
(`app_program/motion/motion_command.h`, `line_request_t` in
`app_program/line_follow/line_track.h`): a mode and an absolute target
heading. The line task restarts tracking on the current heading whenever
`motion_line_control_active()` is false, so a bypass hands back with a
FOLLOW command and nothing else.

Functions that are already safe to call from any task:

| Function | Header | Notes |
|---|---|---|
| `terrain_monitor_get_latest()` | `app_program/terrain/terrain_monitor.h` | latest acceleration snapshot |
| `terrain_monitor_get_calibration()` | `app_program/terrain/terrain_monitor.h` | calibration details |
| `terrain_monitor_recalibrate()` | `app_program/terrain/terrain_monitor.h` | restart calibration |
| `remote_command_stopped()`, `remote_command_turn()` | `app_program/communication/remote_command.h` | dashboard run state and barcode actions (IF-10) |
| `terrain_compass_get()` | `app_program/terrain/terrain_compass.h` | latest compass heading and raw field (IF-09) |
| `connectivity_is_connected()` | `app_program/communication/connectivity.h` | Wi-Fi joined with an IP address |
| `obstacle_get_state()` | `app_program/obstacle/obstacle.h` | latest obstacle result, geometry, stop request and bypass state (IF-07) |
| `motion_line_control_active()` | `app_program/motion/motion.h` | whether `tMotion` follows line requests (IF-04) |
| `line_follow_set_turn()` | `app_program/line_follow/line_follow.h` | the turn for the next cross junction, set by the barcode reader (IF-08) |
