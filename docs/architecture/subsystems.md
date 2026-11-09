# Subsystems: who covers what and how it works

What each buddy owns, the logic behind it, how the code is laid out, and how
data is packed. Everything here is taken from the code in `app_program/`; the
owner of record for each row is `TEAM.md`, which wins if the two ever differ.

## 1. Who covers what

| Buddy | Subsystem | GitHub | Code | Tasks (priority, core) | Pins |
|---|---|---|---|---|---|
| 1 | Communication | Jerome-4469 | `communication/`, `console_mirror/` | `communication_task` (13, any); the MQTT client runs in the kernel's radio service task | Radio only |
| 2 | Motion | felicia-chuaxy | `motion/` | `tMotion` (6, core 1) | GP8–11 motors, GP2–3 and GP13–14 encoders |
| 3 | Line following | kaiwenteoo | `line_follow/` | `line_follow_task` (7, any) | GP26, GP27 |
| 3 | Barcode (owner to confirm) | kaiwenteoo | `barcode/` | `tBarcode` (8, core 0) | GP28 |
| 4 | Terrain | furiiouxx (`ann` in `TEAM.md`) | `terrain/` | `terrain_task` (9, any) | GP4–5, I2C0 |
| 5 | Obstacle | Kitsuneez | `obstacle/` | `obstacle_scan_task` (11, core 0) | GP12 servo, GP16 trigger, GP17 echo |

Shared by everyone: `app_main.c` (starts each subsystem), `app.h`,
`robot_types.h` (every packed layout), `test_report.h`.

### How the pieces talk

```text
             dashboard (tools/dashboard/car_dashboard.py)
                   ^  /status, /telemetry, /console   |  /cmd
                   |                                  v
 line sensors -> line_follow --line request--> motion <-- obstacle
 barcode sensor -> barcode --turn, slow pass-->   ^         | stop, bypass moves
 GY-511 -> terrain --hump, compass-->  line_follow|         | ultrasonic + servo
                 ^-- wheel speeds, distance ------'
```

Each peripheral has one owning task, and tasks reach each other only through
the interfaces in `TEAM.md` section 5. None of the shared values is guarded by
a lock that can wait: each is one atomic word, double-buffered, or guarded by a
sequence counter, except the compass, which uses a short spin lock.

## 2. Buddy 5: Obstacle

`obstacle_scan_task` runs every 10 ms. It owns the servo and the ultrasonic
sensor, and while the bypass is enabled it is also the only task that holds or
releases the motors.

### 2.1 How an obstacle is detected

1. The servo faces ahead (90 degrees) and the sensor pings every 60 ms. The
   echo pulse is timed by the GP17 edge interrupt. No rising edge within 2 ms,
   or a pulse longer than 23 ms, counts as no echo.
2. Echo time becomes distance with the calibration
   `D = 0.1700 * t - 18` mm (t in microseconds). Ranges used for geometry add
   14 mm, measuring from the servo pivot.
3. The stop distance is 150 mm, an echo of 989 us. The first reading nearer
   than that **requests a stop at once**. The request is latched, and the
   bypass state machine sets `MOTION_FLG_INHIBIT` so `tMotion` brakes.
4. The reading is then confirmed. A group of three usable pings is taken,
   counting the first, and at least two must be near. If not, it was a false
   alarm and the stop request is released.
5. A sensor fault also requests a stop, and uses up one of two extra attempts
   rather than a vote. If those run out, the group is thrown away and three new
   votes are needed. Going 200 ms without any usable centre reading requests a
   stop too.
6. If confirmed, the median of the near echoes becomes the **centre echo**, the
   reference for the width sweep.

**Critical distance** is separate and faster. One reading nearer than 80 mm
(577 us) sets the critical flag. It clears only after a complete group with no
valid reading nearer than 100 mm (695 us), so readings between 80 and 100 mm
cannot flip it back and forth.

### 2.2 How the width is calculated

The sweep uses a grid from 50 to 130 degrees in 10 degree steps, nine points,
index 0 being 50 degrees.

1. **Both ends first.** The servo goes to 50 and 130 degrees and keeps the
   median of three pings at each. These decide which side is clearer.
2. **Walk inward.** An end that missed the obstacle is walked inward one grid
   step at a time, one ping each, until a position hits it. An end that hit is
   already the outermost hit on that side.
3. **What counts as a hit.** A flat face square to the car, seen at an angle
   `a` off centre, would read `centre / cos(a)`. A reading is a hit if it is
   valid and nearer than that plus 466 us (about 80 mm). Without the cosine, a
   wide wall would look clear at the ends. An end reading more than 100 us
   behind the flat face, while the other end is clear, is taken for something
   beside the obstacle, and the walk carries on inward.
4. **The formula** (`obstacle_width_estimate()`). Each side gives a range `r`
   and an angle `t`: the outermost hit's angle, moved out by half a grid step
   (5 degrees) when an edge was found inside the grid. Then:

   ```text
   x = r * cos(t)      y = r * sin(t)
   width = sqrt((xL - xR)^2 + (yL - yR)^2)
   ```

   The result is rounded and held between 1 and 511 mm. The two `x` values are
   also published as the left and right boundaries the bypass plans from.
5. **Close-up correction** (`obstacle_width_close_up()`). Within 200 mm, with
   both edges at least three grid steps off centre, the echoes come from the
   corners. Each side then gives `sqrt(r^2 - centre^2)`, and the sum is used if
   it is smaller than the estimate above.

The result carries a status (`PATH_FOUND`, `BLOCKED` or `UNKNOWN`), the clearer
direction, the edge indices and the width. See section 8 for how accurate it is.

### 2.3 How the bypass works

`obstacle_avoid.c` has no hardware access. The scan task calls
`obstacle_avoid_step()` once per loop with everything it read, and carries out
the actions it returns. It never blocks.

**Choosing a side.** A side's sideways need is its edge offset plus half the
car's width (83 mm) plus a 60 mm margin. A side counts as clear only if the car,
shifted aside, would pass what that end measured. The scanner's clearer side is
preferred. Otherwise the clear side needing the shorter sidestep is taken.

**Sizing the moves** (`obstacle_avoid_plan()`).

| Value | Rule |
|---|---|
| Sidestep | The side's need, at least 258 mm and at most 400 mm. The 258 mm floor is a 115 mm edge plus half the car plus the margin: on 2026-11-08 sidesteps planned from an edge read near the line hit a centred obstacle |
| Reverse first | If the obstacle is nearer than 200 mm, reverse to make 200 mm of room |
| Turn angle | `atan(sidestep / (room - margin))`, rounded up to 5 degrees, between 30 and 90 |
| Pass distance | Room, plus axle-to-pivot 60 mm, depth 200 mm, tail 162 mm and the margin, at most 1500 mm. With a known edge a shorter tail clearance is used |

**The legs, in order.**

| Leg | What the car does |
|---|---|
| `REVERSE` | Backs up, only if the plan needs room |
| `TURN_OUT` | Turns by the plan's angle toward the gap |
| `SIDESTEP` | Drives diagonally `sidestep / sin(angle)`. The sensor is held toward the obstacle and can widen the path by up to 80 mm |
| `TURN_BACK` | Turns back parallel to the line |
| `VERIFY` | Sweeps the corridor ahead: clear goes on, blocked waits, too close adds another sidestep (up to three in all, 500 mm sideways at most) |
| `PASS` | Drives past the obstacle |
| `TURN_IN` | Turns toward the line; only its second half may pick the line up |
| `RETURN` | Drives back `(lateral + 150) / sin(angle)`, stopping at any confirmed line |
| `OVERSHOOT` | Drives 130 mm on, putting the axle (not the sensors) on the line |
| `TURN_HOME` | Turns to the heading the car stopped on |
| `FOLLOW` | Hands back to line following |

**The states** (`obstacle_avoid_state_t`).

| State | Meaning |
|---|---|
| `IDLE` | Following, watching for a stop |
| `STOPPING` | INHIBIT set, waiting for `tMotion` to confirm |
| `WAIT_RESULT` | On the line, held, the sweep is running |
| `WAIT_CLEAR` | On the line, held, nothing to bypass with yet; no time limit |
| `RELEASING` | INHIBIT cleared, waiting for motion to leave it |
| `PREPARE` | Scanner rearmed, waiting for a fresh clear reading |
| `SEND` | A command waiting to be queued |
| `RUN_LEG` | Command queued, waiting for its finish event |
| `VERIFY` | Corridor sweep after a sidestep |
| `BLOCKED_WAIT` | Off the line, held, path blocked |
| `RETRY_WAIT` | Off the line, held, before one retry |
| `REPLAN` | Re-sweeping after back-offs failed |
| `FAULT` | Latched until reboot, motors held |

A critical reading during a move backs the car off 50 mm, up to twice, and then
re-sweeps. A leg that overruns twice its expected time plus one second is retried
once, and a second problem latches `FAULT` with the motors held. A line missed
twice is different: the obstacle is already passed, so the car hands over to line
following to search for it.

### 2.4 Where the code is

| File | Purpose |
|---|---|
| `obstacle.c/.h` | The task; publishes `obstacle_shared_state_t`; carries out bypass actions |
| `obstacle_scan.c/.h` | Servo timing, ping groups, detection, sweep, corridor, watch |
| `obstacle_decision.c/.h` | Pure decisions: median, centre action, clearer side |
| `obstacle_width.c/.h` | Pure geometry: hit test, width formula, range conversion |
| `obstacle_avoid.c/.h` | Pure bypass planning and state machine |
| `obstacle_calibration.c/.h` | Stationary raw measurements for calibration builds |
| `obstacle_config.h` | Every distance, time and threshold above |
| `drv_ultrasonic.c/.h` | Trigger and interrupt-timed echo |
| `drv_servo.c/.h` | 50 Hz PWM servo |
| `drv_gpio_irq.c/.h` | Shares the one GPIO interrupt between pins |
| `obstacle_test.c` | On-target tests |

## 3. Buddy 2: Motion

`tMotion` wakes every 20 ms on core 1, or at once when `MOTION_FLG_INHIBIT` is
set, and is the only task that drives the motors or reads the encoders. It
discards commands while inhibited, otherwise runs queued commands in order,
services the line adapter, then updates control.

- **Speed loop.** Feed-forward duty plus a PI loop on each wheel, from encoder
  speeds, so speed holds under load.
- **Moves and turns.** Moves run at 300 mm/s and turns at 90 degrees/s, each
  ending when the encoders reach the target, or at a confirmed line if asked. A
  sync loop holds the heading during a move.
- **Following.** 120 mm/s (30 mm/s when slowed). A heading PI loop holds the
  target heading from the line task, and this module never reads line sensors.
- **Safety.** A request older than 100 ms stops following, and a stall or
  timeout latches until cleared.

| File | Purpose |
|---|---|
| `motion.c/.h` | The task, kernel objects, status and odometry publishing |
| `motion_command.c/.h` | Takes line requests from two lock-free slots and applies them |
| `motion_move.c/.h` | Odometry, moves, turns, following, stall detection |
| `motion_drive.c/.h` | Feed-forward and speed loop per wheel |
| `motion_pid.c/.h` | PID with limits and anti-windup |
| `drv_motor.c/.h`, `drv_encoder.c/.h` | PWM H-bridge; PIO quadrature counters |
| `motion_config.h`, `motion_command_config.h` | Speeds, gains, timeouts |

## 4. Buddy 3: Line following and barcode

### Line following

Two digital sensors straddle the tape, so both read white while centred. Every
10 ms `line_track_update()` turns the two levels and the odometry into a
request.

- **Turn while black.** While one sensor is black, the target heading leads the
  car toward it. Once clear, the car holds the heading it reached.
- **Bends.** The same side touching again soon means a bend, and the car keeps
  turning toward the last edge while both read white.
- **Search.** An edge black for 200 mm, or a long white run (250 mm in a bend),
  starts a pivot search toward the last edge.
- **Junctions.** Both sensors going black together is a cross. Straight on, the
  heading is held over it. For a turn, the car drives 130 mm on and pivots
  exactly 90 or 180 degrees.
- **Barcodes.** When the reader sees one coming, the car brakes, backs up
  170 mm and tracks over it at 30 mm/s, then holds its heading for 200 mm
  (settle) so the next crossing is met square.
- **Humps.** Terrain's report slows following to 30 mm/s.

### Barcode

The GP28 interrupt timestamps every edge into a queue, after a chatter filter.
`tBarcode` turns the edges into element widths and decodes Code 39 in either
direction, comparing widths with each other, so speed does not matter. A white
gap or black over 1.07 s ends a scan. Four edges close together mean a barcode
is under the sensor at follow speed. The decoded text is looked up in the map
from the dashboard to choose the next junction's turn.

| File | Purpose |
|---|---|
| `line_follow/line_follow.c/.h` | The task: sampling, requests, phase handling |
| `line_follow/line_track.c/.h` | All tracking decisions, hardware-free |
| `line_follow/drv_line_sensor.c/.h` | The two digital inputs |
| `line_follow/line_follow_config.h` | Pins and tuning |
| `barcode/barcode.c/.h` | The task, decode, turn lookup |
| `barcode/barcode_scan.c/.h` | Edges to widths, trigger, chatter filter |
| `barcode/barcode_code39.c/.h` | The decoder |
| `barcode/drv_barcode.c/.h` | The edge interrupt and queue |
| `barcode/barcode_config.h` | Timing |

## 5. Buddy 4: Terrain

`terrain_task` reads the GY-511 about every 10 ms. The car stands still for
calibration at each start. Readings are given out only once calibration is
valid.

- **Tilt.** Pitch and roll from the corrected accelerometer.
- **Humps.** The course's humps are ramps under one wheel, so the detector
  follows the size of the roll: climbing, top, then a drop from the peak. A
  slowed pass needs 60 mm of travel to count. Height comes from the peak lean
  and the track width.
- **Collisions.** A hard push in level running, or, at line speed, the wheels
  falling from at least 90 mm/s to half within 30 samples while still following.
  A lean in the last 5 samples is taken for the hit itself; anything older is a
  hump slowing the car.
- **Other.** Turn rate from motion's wheel speeds, a compass from the
  magnetometer, and a run summary of humps, heights, collisions and turns.

| File | Purpose |
|---|---|
| `terrain.c/.h` | The task |
| `terrain_monitor.c/.h` | Coordinator, calibration, published readings |
| `terrain_process.c/.h` | Axis mapping, correction, tilt, field helpers |
| `terrain_detect.c/.h` | Hump, collision, turning and speed state machines |
| `terrain_height.c/.h` | Hump height |
| `terrain_event.c/.h`, `terrain_summary.c/.h` | Event queue; run summary |
| `terrain_hint.c/.h` | Movement hints taken from motion |
| `terrain_compass.c/.h` | Magnetometer as a compass |
| `drv_imu.c/.h`, `drv_imu_bus.c/.h` | GY-511 driver over the kernel I2C driver |
| `terrain_config.h` | Thresholds |

## 6. Buddy 1: Communication

The radio belongs to the kernel's service task, so application tasks never call
it. `communication_task` only watches the connection. The MQTT client runs
inside the radio task and carries:

| Topic | Direction | Content |
|---|---|---|
| `<topic>/cmd` | dashboard to car | `start`, `stop`, `clear`, `map <barcode> <turn>` |
| `<topic>/status` | car to dashboard | 24-byte packed status, twice a second, best effort |
| `<topic>/telemetry` | car to dashboard | 12-byte packed records, acknowledged, replayed after an outage |
| `<topic>/console` | car to dashboard | Every console line, as text. This is the only debug view, so it stays text |

`tools/dashboard/car_dashboard.py` is the broker, the log and the web page. It
decodes both binary topics and writes the same readable lines to the log.

| File | Purpose |
|---|---|
| `communication.c/.h` | The Wi-Fi status task |
| `connectivity.c/.h` | Connection state from the radio service |
| `mqtt_client.c/.h` | Connection, telemetry outbox, status, command topic |
| `telemetry_queue.c/.h`, `telemetry_queue_port.c/.h` | Fixed queue and its lock |
| `car_status.c/.h` | Reads every subsystem and packs the status |
| `remote_command.c/.h` | Parses commands, holds the barcode map |
| `console_mirror/` | Copies console lines to MQTT |

## 7. Minimum-bit data

**The rule.** Data that is stored, shared between tasks or sent uses only the
bits its range needs, packed with masks and shifts, saturating at the field's
limit. Layouts live in `robot_types.h`, or beside the module for state private
to it. Calculations unpack first and stay full size, so arithmetic does not
have to think about field widths.

**Helpers.** `robot_field_put()` and `robot_field_get()`, with
`robot_field_from_signed()` and `robot_field_to_signed()` for two's complement.
`robot_mark16()` and `robot_mark16_near()` keep the low 16 bits of a running
distance and read it back wrap-safely.

### Shared and sent

| Data | Bits | Field ranges and why |
|---|---|---|
| Motion command (IF-01) | 22 | Type 2, stop at line 1, reverse 1, distance 10 (1023 mm), angle 8 (180 degrees) |
| Motion status | 5 | Mode 3 (seven modes), done reason 2 |
| Line request (IF-04) | 30 | Mode 2, four flags, reverse 8 (the 170 mm reverse), heading 16 (about 91 turns each way) |
| Odometry (IF-05) | 81 in 3 words | Distance 24 (8.3 km each way), count 8, heading 16, three speeds 11 (the car tops out at 620 mm/s) |
| Obstacle result | 33 | Status 3, direction 2, edge flags 2, critical 1, edges 4 and 4, width 9 (511 mm), sequence 8 |
| Obstacle geometry (IF-07) | 64 | Ranges 12 (4 m, the sensor's limit), offsets 13 signed, two flags |
| Obstacle control | 9 | Stop 1, bypass state 4 (13 states), fault 4 (9 reasons) |
| Watch reading | 24 | Echo 15, sequence 8, active 1 |
| Turn and barcode phase (IF-08) | 8 each | One byte each, written by one task and read by another |
| Compass (IF-09) | 57 in 2 words | Axes 12 (the sensor's own range), heading 12 (0 to 3599), count 8, valid 1 |
| Barcode map entry (IF-10) | 20 | Three 6-bit Code 39 characters, turn 2. One word, so one write replaces an entry with no lock |
| Barcode edge | 32 | Level 1, time 31 in 2 us steps, keeping the clock's top bit so differences still work across its wrap |
| Telemetry record | 96 | Type 4, sequence 8, time 20 (10 ms ticks, 2.9 h), payload 64 |
| Car status | 174 in 6 words | Field list in `robot_types.h` |

### Stored inside a subsystem

| Data | Change |
|---|---|
| `line_track_t` | 14 facts in a flags word; 16 small values in 30 bits; distance marks 16 bits; headings 16 bits |
| `obstacle_avoid_t` | 19 facts in a flags word; 7 small values in 20 bits; plan in 13 bits and three 16-bit distances; distances 16 bits |
| `barcode_scan_t` | Widths 16 bits in 16 us steps (up to about 1 s), count and flags 1 byte each |
| `barcode_trigger_t` | Index, count and two flags in 1 byte |
| `terrain_stopped_t` | Speed 11, age 8 and off-level count 8 in one word; peak push 16 bits in 1/256 g |

**Why 16-bit distance marks are safe.** A mark is read as the value nearest the
current travel, which is exact within 32 m either way. The line tracker's past
marks are also aged every sample, held at 30 m old, because a mark left unused
for more than 32 m would otherwise wrap and read as recent. The bypass's marks
are only read inside the leg or bypass that set them.

**What is deliberately left full size:**

- Calculations and one-sample inputs (`avoid_in`, `terrain_hint_t`, PID floats).
- Kernel clock stamps and the sequence counters that guard shared copies. They
  are synchronisation rather than data, and a narrower counter could let a torn
  read through.
- The corridor sweep's `uint16_t` per echo: the team ruled sweep samples are
  working data. Echoes stay 15 bits rather than 11, because the bypass compares
  ranges beyond what 11 bits holds.
- Compile-time configuration, tests, and upstream kernel, lwIP and TinyUSB.
- The console text and its MQTT mirror, the only debug view without a cable.

**Measured effect** on the normal MQTT image: static RAM fell from 111,804 to
103,048 bytes (8,756 saved), for 4,760 bytes more code from the packing helpers.

| Variable | Before (bytes) | After (bytes) |
|---|---|---|
| Telemetry records | 9,216 | 1,536 |
| `scan` (barcode) | 1,016 | 508 |
| Barcode edge queue | 512 | 256 |
| `track` | 216 | 64 |
| `avoid` | 152 | 76 |
| Line request slots | 48 and 24 | 16 and 8 |
| `trigger` | 32 | 24 |
| Published geometry | 12 | 8 |

## 8. Known limits

- **Width accuracy.** The test-day target is within 5 cm. At the 150 mm stop the
  sensor's beam reaches past the edges of small obstacles: 16 and 21 cm obstacles
  have read about the same. Stopping at 250 mm was tried and was worse. Width is
  not yet reliable at this beam width.
- **Scan side.** `OBSTACLE_SERVO_MIRRORED` is set so the sensor should swing to
  the car's left first. This has not been confirmed on the car.
- **Squaring up.** The car does not yet square up to the obstacle's face before
  measuring. The heading-based straighten leg lagged and is switched off
  (`OBSTACLE_AVOID_STRAIGHTEN_MAX_DEG` is 0).
- **Collision rule.** Confirmed on two of three deliberate hits, with no evidence
  yet from a hump pass or a long drive for false counts.
- **Bypass-state reasons on telemetry.** The packed record carries the from and
  to states only. The reason text is on the console mirror.
- **Barcode text on telemetry.** Limited to 10 characters. Longer text still
  decodes and appears on the console.
- **Barcode map.** Keys are up to three Code 39 characters.
