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
| motion | Two H-bridge DC motors, open-loop drive (held braked for now) | | felicia-chuaxy | `app_program/motion/` | `testing/motion/` |
| line_follow | Two IR sensors → line commands (dry run, no motors) | | kaiwenteoo | `app_program/line_follow/` | `testing/line_follow/` |
| terrain | GY-511 accelerometer, calibration and corrected readings | | ann | `app_program/terrain/` | `testing/terrain/` |
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
| GP2 | motion (reserved) | planned left encoder A; released by obstacle |
| GP3 | motion (reserved) | planned left encoder B; released by obstacle |
| GP4 | terrain | I2C0 SDA to the GY-511 (Robo Pico Grove 3), claimed by the kernel I2C driver at boot |
| GP5 | terrain | I2C0 SCL to the GY-511 (Robo Pico Grove 3), claimed by the kernel I2C driver at boot |
| GP6 | | |
| GP7 | | |
| GP8 | motion | left motor input A (PWM slice 4 A) |
| GP9 | motion | left motor input B (PWM slice 4 B) |
| GP10 | motion | right motor input A (PWM slice 5 A) |
| GP11 | motion | right motor input B (PWM slice 5 B) |
| GP12 | obstacle | servo signal (PWM slice 6 A) |
| GP13 | | (shares PWM slice 6 with the servo: 50 Hz only) |
| GP14 | | |
| GP15 | | |
| GP16 | obstacle | ultrasonic trigger (output) |
| GP17 | obstacle | ultrasonic echo (input, edge interrupt) |
| GP18 | | |
| GP19 | | |
| GP20 | | |
| GP21 | | |
| GP22 | | |
| GP26 | line_follow | left IR sensor (ADC0), set to analogue by the kernel ADC driver at boot |
| GP27 | line_follow | right IR sensor (ADC1), set to analogue by the kernel ADC driver at boot |
| GP28 | | (ADC2 capable) |

Application settings live in `app_program/obstacle/obstacle_config.h`.
Shared application payloads live in `app_program/robot_types.h`; their sweep
geometry is the decoding contract and must be agreed with telemetry users.
The `config/` folder remains kernel configuration. GPIO bank dispatch is
still obstacle-owned; encoder IRQ integration remains a coordinated change.

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
| ADC | line_follow | `line_follow_task` (kernel driver `"adca"`) | GP26, GP27 (GP28 free) |
| PWM slices (list) | obstacle: slice 6. motion: slices 4, 5 | obstacle: `obstacle_scan_task`. motion: none yet; `motion_start()` configures them and holds the motors braked | GP12 (obstacle); GP8–GP11 (motion) |
| GPIO bank interrupt (`IO_IRQ_BANK0`, all GPIOs) | obstacle | `obstacle_scan_task`, core 0 only | GP17 echo edges. Ask the obstacle owner before enabling any other GPIO interrupt |
| TIMER (free-running 1 MHz counter) | *shared, read only* | any | — Read by obstacle and terrain; never write it |
| WiFi radio, TCP/IP, MQTT | *kernel* | `cyw43_utk_task` (built in) | GP23–25, GP29 |

The kernel's physical timers (`tk_sta_ptm` and friends) are built on the PWM
slices; do not use physical timers 4, 5 or 6.

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

1 is the highest. 1–4 are used by the kernel (USB console 3, WiFi 4). Each subsystem gets its own band by default;
change it only by team agreement. Write down every task you create.

| Subsystem | Priority band | Tasks (name – priority – core) |
|---|---|---|
| obstacle | 12–13 | `obstacle_scan_task` – 12 – core 0 (`TP_PRC1`; the echo interrupt is on core 0) |
| motion | 14–15 | none yet |
| line_follow | 16–17 | `line_follow_task` – 16 – any |
| terrain | 18–19 | `terrain_task` – 18 – any |
| communication | 20–21 | `communication_task` – 20 – any |

Core: `TP_PRC1` = core 0, `TP_PRC2` = core 1, or "any".

## 5. Interfaces between subsystems

Every arrow between two subsystems is listed here. The **provider** creates
the kernel object and publishes its ID in its own `<name>.h`; the **user**
only sends to it or waits on it. Agree on the data format before coding.

The subsystems do not exchange data yet: each runs on its own. The obstacle
results, line commands and motor commands are still TODOs.

| ID | From → To | Mechanism | Object ID name | Data / meaning | Agreed by |
|---|---|---|---|---|---|
| IF-01 | | | | | |

Each interface is tested by at least one integration test
(`testing/integration/TEST_LOG.md`).

## 6. Kernel object budget

Limits are set in `config/config.h`. If the team needs more, raise the
limit there.

| Object | Limit | obstacle | motion | line_follow | terrain | communication |
|---|---|---|---|---|---|---|
| Task | 32 | 1 | | 1 | 1 | 1 |
| Semaphore | 16 | | | | | |
| Event flag | 16 | | | | | |
| Mailbox | 12 | | | | | |
| Mutex | 12 | | | | | |
| Message buffer | 12 | | | | | |
| Fixed memory pool | 12 | | | | | |
| Variable memory pool | 12 | | | | | |
| Cyclic handler | 12 | | | | | |
| Alarm handler | 12 | | | | | |

(The kernel's USB and WiFi service tasks, and its I2C and ADC drivers, also
use a few of these.)
