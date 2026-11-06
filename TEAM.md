# Team and subsystems

Fill this in at the start of the project and keep it up to date. It is the
single place that says **who owns what**, so that five people can work in
one firmware image without clashing over pins, priorities, or kernel objects.

A **subsystem** is one student's part of the product, with its own folder
`app_program/<name>/`. It may contain several tasks and own several
peripherals (see section 3).

Rule: **only the owner edits a subsystem's folders.** To change another
subsystem, ask its owner or open a pull request for them to review.

## 1. Members and subsystems

| Subsystem | What it does | Student ID | GitHub username | Code | Tests |
|---|---|---|---|---|---|
| obstacle | Ultrasonic sensor on a servo scans for a clear path | | Kitsuneez | `app_program/obstacle/` | `testing/obstacle/` |
| motion | Two H-bridge DC motors and wheel encoders: speed loop, distance moves, turns and line following in `tMotion` | | felicia-chuaxy | `app_program/motion/` | `testing/motion/` |
| line_follow | Two digital IR sensors → target headings (turn while black) and searches for `tMotion` | | kaiwenteoo | `app_program/line_follow/` | `testing/line_follow/` |
| barcode | Third digital IR sensor → Code 39 text (uses the `tBarcode` slot reserved under line_follow) | | *kaiwenteoo, to confirm* | `app_program/barcode/` | `testing/barcode/` |
| terrain | GY-511 accelerometer, calibration and corrected readings; magnetometer as a compass | | ann | `app_program/terrain/` | `testing/terrain/` |
| communication | Wi-Fi connection status | | Jerome-4469 | `app_program/communication/` | `testing/communication/` |

Shared files (`app_program/app_main.c`, `app_program/app.h`,
`app_program/test_report.h`, `config/`, `device/`, `build_make/`, this file)
are changed only by agreement of the team. The exception is claiming an empty
row in this file (a pin, a peripheral, an interface ID), which anyone may do
(section 2).

## 2. Pins

One owner per pin. Claim a pin here **before** you wire or code it.

**Why:** two subsystems using the same pin is one of the most common and
hardest-to-find mistakes in a team project. The code builds without any
error, but the hardware misbehaves. Each subsystem can pass its own tests
and then fail once they are combined, and two outputs driving the same pin
against each other can damage it. Writing the claim down first means a clash
is spotted when someone edits this table, not hours later on the bench.

**No instructor check needed.** The team keeps this table itself. You do not
need the instructor, or the rest of the team, to check or approve a claim:
if the row is empty, fill it in and commit. If two people claim the same
pin, they agree between themselves who moves. Only the *reserved* rows are
fixed.

| Pin | Owner | Used for |
|---|---|---|
| GP23, GP24, GP25, GP29 | *reserved* | Pico W radio |
| GP0 | | (taken whenever the team builds with `CRASH_REPORT=1`) |
| GP1 | | (taken whenever the team builds with `CRASH_REPORT=1`) |
| GP2 | motion | right encoder phase A (input, PIO0 state machine 3) |
| GP3 | motion | right encoder phase B (input, PIO0 state machine 3) |
| GP4 | terrain | I2C0 SDA to the GY-511 (Robo Pico Grove 3), claimed by the kernel I2C driver at boot |
| GP5 | terrain | I2C0 SCL to the GY-511 (Robo Pico Grove 3), claimed by the kernel I2C driver at boot |
| GP6 | | |
| GP7 | | |
| GP8 | motion | left motor input A (PWM slice 4 A) |
| GP9 | motion | left motor input B (PWM slice 4 B) |
| GP10 | motion | right motor input A (PWM slice 5 A) |
| GP11 | motion | right motor input B (PWM slice 5 B) |
| GP12 | obstacle | servo signal (PWM slice 6 A) |
| GP13 | motion | left encoder phase A (input, PIO0 state machine 2; PWM slice 6 B is not used, so the servo is unaffected) |
| GP14 | motion | left encoder phase B (input, PIO0 state machine 2) |
| GP15 | | |
| GP16 | obstacle | ultrasonic trigger (output) |
| GP17 | obstacle | ultrasonic echo (input, edge interrupt) |
| GP18 | *planned: tMission (owner to confirm)* | Robo Pico RGB LED (WS2812); proposed on PIO1 (section 3) |
| GP19 | | |
| GP20 | | |
| GP21 | | |
| GP22 | | |
| GP26 | line_follow | left IR sensor DO (digital input; the kernel ADC driver leaves it alone) |
| GP27 | line_follow | right IR sensor DO (digital input; the kernel ADC driver leaves it alone) |
| GP28 | barcode | barcode IR sensor DO (digital input, edge interrupt) |

Application settings live in `app_program/obstacle/obstacle_config.h`.
Shared application payloads live in `app_program/robot_types.h`; their sweep
geometry is the decoding contract and must be agreed with telemetry users.
The `config/` folder remains kernel configuration. GPIO bank dispatch is
obstacle-owned; the wheel encoders do not use it, because PIO0 counts them
(section 3).

## 3. Peripherals

**Subsystems, tasks and peripherals are three different things:**

| Term | What it is | Example |
|---|---|---|
| **Subsystem** | one student's part of the product, one folder | "obstacle", "motion", "terrain" |
| **Task** | one thread of execution. A subsystem may have **as many tasks as it needs** | a sampling task and a filtering task, both in "terrain" |
| **Peripheral** | one hardware block on the RP2040 | I2C0, UART1, the ADC |

A driver is **not** a subsystem. It is code *inside* the subsystem that
needs that hardware: the "terrain" subsystem contains the IMU code, and owns
I2C0. Write it as `drv_<name>.c` / `drv_<name>.h` in that subsystem's
folder, never under `device/`, and call it only from the peripheral's owning
task. The kernel's own drivers in `device/` (serial, I2C, ADC) may be used
instead; switching one on in `config/config_device.h` changes shared
configuration, so record it in the table below first.

**The rule: each peripheral is driven by exactly one task, its *owning
task*.** That task can be in any subsystem, and one subsystem may own
several peripherals (one owning task each, or one task owning all of them).
Every other task, in the same subsystem or another one, asks the owning task
through a kernel object (section 5) and never touches that peripheral itself.

*Why:* this port runs tasks on both cores at the same time. If two tasks
drive the same I2C or UART block at once, their transfers get mixed up, and
the drivers have no locking to stop it.

| Peripheral | Owner subsystem | Owning task | Pins |
|---|---|---|---|
| UART1 | | | |
| I2C0 | terrain | `terrain_task` (kernel driver `"iica"`, `DEVCNF_USE_IIC` = 1) | GP4, GP5 |
| I2C1 | | | |
| SPI0 | | | |
| SPI1 | | | |
| ADC | | (kernel driver `"adca"` still built, no pin set up; the IR sensors are digital) | |
| PWM slices (list) | obstacle: slice 6. motion: slices 4, 5 | obstacle: `obstacle_scan_task`. motion: `tMotion` (`motion_start()` configures them braked, then hands them over) | GP12 (obstacle); GP8–GP11 (motion) |
| GPIO bank interrupt (`IO_IRQ_BANK0`, all GPIOs) | obstacle | dispatcher `obstacle/drv_gpio_irq.c`, core 0 only; each pin is attached from its own core-0 task | GP17 echo edges (`obstacle_scan_task`), GP28 barcode edges (`tBarcode`). Attach other pins through `drv_gpio_irq_attach()`, never `tk_def_int()`; ask the obstacle owner first |
| PIO0 | motion | `tMotion` (`motion_start()` loads the encoder program; only `tMotion` reads the counts, as `encoder_get_count()` empties the FIFO) | GP2, GP3, GP13, GP14 (inputs). The program fills instruction slots 0–23 and uses state machines 2 and 3; slots 24–31 and state machines 0 and 1 are free. 2 and 3, not 0 and 1, because the SDK allocator hands out state machine 0 first and places programs at the top slots, so a fallback to PIO0 would not collide. `encoder_init()` fails if any PIO0 state machine is already running |
| PIO1 | *kernel*; planned RGB LED | `cyw43_utk_task` (gSPI to the radio: 6 instructions, 1 state machine). Proposed: the WS2812 LED program (4 instructions) goes here too, leaving PIO0 to the encoders; the LED's owner must confirm | GP23–25, GP29; GP18 |
| TIMER (free-running 1 MHz counter) | *shared, read only* | any | — Read by obstacle and terrain; never write it |
| WiFi radio, TCP/IP, MQTT | *kernel* | `cyw43_utk_task` (built in) | GP23–25, GP29 |

The kernel's physical timers (`tk_sta_ptm` and friends) are built on the PWM
slices; do not use physical timers 4, 5 or 6.

The CYW43 radio driver takes its PIO through the Pico SDK allocator
(`pio_claim_free_sm_and_add_program()`), which tries PIO1 before PIO0, so
it lands on PIO1. Any other PIO program must also go on PIO1: the encoder
program on PIO0 has to sit at address 0, and the allocator cannot see it.
The comment in `kernel/sysdepend/pico_rp2040/hw_setting.c` that labels
PIO0 "CYW43439 PIO-SPI" is stale: that code only releases PIO0 and PIO1
from reset at boot, and only in Wi-Fi builds. `drv_encoder.c` releases
PIO0 itself for builds without Wi-Fi.

### WiFi and MQTT are special

The radio and the network stack belong to the kernel's built-in WiFi service
task, `cyw43_utk_task` (`lib/libwifi/sysdepend/pico_rp2040/`). **No student
task may call `cyw43_...`, lwIP or MQTT functions, not even from the
communication subsystem.** The calls are not safe from any other task, on
either core. The service joins the network in `config/wifi_secrets.h` on a
`NET=lwip` build, which is the default, and retries on its own.

What students *can* do:

- **Read the connection state** from any task, which is always safe:
  ```c
  #include "cyw43_utk.h"
  T_CYW43_UTK_STATUS st;
  cyw43_utk_get_status(&st);   /* st.ready, st.link_status, st.ip_addr, ... */
  ```
  The communication subsystem does this; other subsystems can call
  `connectivity_is_connected()` from `app_program/communication/connectivity.h`.
- **Send or receive data over the network** by adding code that *runs
  inside the service task*. The MQTT client is in
  `lib/liblwip/sysdepend/pico_rp2040/mqtt_utk.c`. It publishes a heartbeat
  today. A "cloud / reporting" subsystem would add a kernel object
  (e.g. a message buffer) that application tasks fill. The MQTT code then
  reads it with `TMO_POL` (never blocking) and publishes it.
  Because this changes shared library code, **agree it with the team and
  the instructor first**, and record it as an interface in section 5.
  It needs the `MQTT=1` build (README section 3).

## 4. Task priorities

1 is the highest. 1–4 are used by the kernel: the initial task 1 (it runs
the tests in `make TEST=1` builds), USB console 3 and Wi-Fi 4, both pinned
to core 0. Application tasks follow the order of the Week 6 design freeze,
each at its freeze priority + 3 so they sit below the kernel's tasks.
Change it only by team agreement. Write down every task you create.

| Subsystem | Priority | Tasks (name – priority – core) |
|---|---|---|
| *safety (unassigned)* | 5 | *tSafety – 5 (reserved)* |
| motion | 6 | `tMotion` – 6 – core 1 (`TP_PRC2`; nothing above it runs there, NFR1) |
| line_follow | 7 | `line_follow_task` (tLine) – 7 – any |
| barcode | 8 | `tBarcode` – 8 – core 0 (`TP_PRC1`; the GPIO interrupt is on core 0) |
| terrain | 9 | `terrain_task` (tImu) – 9 – any |
| *mission (unassigned)* | 10 | *tMission – 10 (reserved)* |
| obstacle | 11–12 | `obstacle_scan_task` (tScan) – 11 – core 0 (`TP_PRC1`; the echo interrupt is on core 0); its test helper task 12; with the bypass on, it also holds INHIBIT from start-up and sends motion commands (IF-06) |
| communication | 13 | `communication_task` (tComms) – 13 – any |

Core: `TP_PRC1` = core 0, `TP_PRC2` = core 1, or "any".

## 5. Interfaces between subsystems

Every arrow between two subsystems is listed here. The **provider** creates
the kernel object and publishes its ID in its own `<name>.h`; the **user**
only sends to it or waits on it. Agree on the data format before coding.

Rows marked *proposed* or *interim* are not agreed yet.

| ID | From → To | Mechanism | Object ID name | Data / meaning | Agreed by |
|---|---|---|---|---|---|
| IF-01 | tMission → motion | Message buffer | `motion_mbf_cmd` (MBF_MOTION_CMD) | one `uint32_t` command word, packed with the `robot_motion_cmd_` helpers in `robot_types.h` (22-bit layout, Tribunal 1; stop-at-line acquires fresh current visibility) | proposed by motion; tMission owner to agree |
| IF-02 | tSafety → motion | Event flag | `motion_flg_ctrl` (FLG_MOTION), bit `MOTION_FLG_INHIBIT` | set: brake at once and discard commands; clear: back to STOPPED | proposed by motion; tSafety owner to agree |
| IF-03 | motion → tMission | Event flag + status word | `motion_flg_event` (`MOTION_EVT_` bits), `motion_get_status()` | MOVE_DONE (reason in status), STALL, ENC_FAULT, REJECTED, FOLLOW_ENDED; mode 3 bits. The status is published before any event bit is set, so a reader that sees an event and then reads the status gets that command's done reason | interim until FLG_MISSION exists |
| IF-04 | line_follow ↔ motion | Latest-value slot under a mutex (`TMO_POL`, never waits); reads of IF-05 and one status byte | `motion_command_submit()`; `motion_get_odometry()`, `motion_line_control_active()`, `motion_get_status()` | `line_request_t` every 10 ms: mode (TRACK, SEARCH_LEFT/RIGHT, STOP), absolute target heading (odometry degrees), line seen (for stop-at-line), junction (a cross being taken). TRACK holds the heading at follow speed; SEARCH pivots to it. Older than 100 ms stops following | owners agreed (digital rewrite, 2026-11-05) |
| IF-05 | motion → terrain | Shared copy, sequence-checked (no lock, never waits) | `motion_get_odometry()` | `motion_shared_odometry_t`: update count, distance mm and heading deg (`int32`), speed and both wheel speeds mm/s (`int16`); + forward and left; updated every 20 ms | proposed by motion; ann to agree; moves into `telemetry_t` under MTX_TELEM in step 8 |
| IF-06 | obstacle → motion | Uses IF-01, IF-02, IF-03 | `motion_mbf_cmd`, `motion_flg_ctrl` (`MOTION_FLG_INHIBIT`), `motion_flg_event` | interim tMission/tSafety for obstacles, only while `OBSTACLE_AVOID_ENABLED`: INHIBIT from start-up until a confirmed clear path and on every stop; box-bypass commands; FOLLOW to resume; a latched FAULT keeps INHIBIT until reboot. The only task that sets or clears INHIBIT and the only consumer of MOVE_DONE (clears every `MOTION_EVT_` bit before each command); hands both to tSafety/tMission when they exist | proposed by obstacle; felicia-chuaxy to agree |
| IF-08 | barcode → line_follow | One word, latest value (no lock, never waits) | `line_follow_set_turn()` | `line_turn_t` for the next cross junction: STRAIGHT (default), LEFT, RIGHT, UTURN; set on each decoded barcode, used by one junction then STRAIGHT again. The barcode-to-turn mapping is still to be set (`barcode.c`, `turn_for()`) | *proposed* (2026-11-06) |
| IF-07 | obstacle → any | Event flag + shared copy, sequence-checked (no lock, never waits) | `obstacle_flg_event` (FLG_OBSTACLE), `obstacle_get_state()` | STOP, CRITICAL and FAULT levels; RESULT edge on a new result sequence (one consumer clears it); `obstacle_shared_state_t`: packed result, its geometry (mm from the servo pivot), stop request, bypass state and fault reason | proposed by obstacle |
| IF-09 | terrain → any | Shared copy under the terrain spin lock (held for a struct copy, never waits) | `terrain_compass_get()` (`app_program/terrain/terrain_compass.h`) | `terrain_compass_t`: valid flag, raw field on the car's X, Y, Z (counts), heading in tenths of a degree counter-clockwise from magnetic north (+ is left, like IF-05), sample count; updated every 10 ms by `terrain_task`. Read by line_follow for its trace only (not steering) | *proposed* (2026-11-06) |

Each interface is tested by at least one integration test
(`testing/integration/TEST_LOG.md`).

## 6. Kernel object budget

Limits are set in `config/config.h`. If the team needs more, raise the
limit there.

| Object | Limit | obstacle | motion | line_follow | barcode | terrain | communication |
|---|---|---|---|---|---|---|---|
| Task | 32 | 1 | 1 | 1 | 1 | 1 | 1 |
| Semaphore | 16 | | | | | | |
| Event flag | 16 | 1 | 2 | | | | |
| Mailbox | 12 | | | | | | |
| Mutex | 12 | | 1 | | | | |
| Message buffer | 12 | | 1 | | | | |
| Fixed memory pool | 12 | | | | | | |
| Variable memory pool | 12 | | | | | | |
| Cyclic handler | 12 | | 1 | | | | |
| Alarm handler | 12 | | | | | | |

(The kernel's USB and WiFi service tasks, and its I2C and ADC drivers, also
use a few of these.)

## TIMER tick initialization (cold-boot fix)

`usermain()` explicitly configures the watchdog tick to 1 MHz from the
12 MHz `CLK_REF` before it blocks or starts application subsystems. This
shared `app_main.c` change leaves kernel sources untouched. TIMER remains
read-only for application tasks after that one startup configuration.

On 2026-11-04, isolated diagnostic images on the Pico reproduced CYCLES=0
and about 23,672 TIMER counts over 1,010 ms after removing all power, for
both default and WIFI=none builds. After BOOTSEL flashing both had
CYCLES=12 and about 1,010,000 counts. See `testing/integration/TIMER_DIAGNOSTIC.md`
for source/firmware provenance and before/after evidence.

The initial task was observed on core 0 in these SMP runs. The radio task
is pinned to core 0; it starts using TIMER after the initial task blocks.
This placement must be revisited if startup affinity/scheduling changes.
Kernel consumers before `usermain()` (including SMP lock statistics) remain
outside this application's initialization guarantee. Motion, terrain and
communication measurements that relied on TIMER after a cold boot before
this fix need review; the test history must not be retroactively marked PASS.
