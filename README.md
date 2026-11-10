# INF2004 Vehicle Controller — micro T-Kernel 3.0 on Raspberry Pi Pico W

The team's vehicle controller, built on the course's micro T-Kernel 3.0
project template: the kernel runs on **both cores** of the Pico W's RP2040,
with a USB serial console and the WiFi driver built in.

The car follows a black line and reads the Code 39 barcodes beside it to
choose its turn at the next junction. It stops for obstacles, measures them
and drives around them, and counts humps and collisions. It reports to a web
dashboard over MQTT, which also starts and stops it.

## INF2004 at a glance

Six subsystems, one folder each in `app_program/` (owners, pins,
priorities, peripherals and interfaces are in **`TEAM.md`**):

| Subsystem | Feature | Hardware | Task (priority, core) |
|---|---|---|---|
| `obstacle/` | Stops for an obstacle, sweeps it for width, then drives the bypass around it and back to the line | ultrasonic trigger GP16, echo GP17; servo GP12 | `obstacle_scan_task` (11, core 0) |
| `motion/` | Closed-loop drive: a speed loop on each wheel, moves and turns by the encoders, odometry, and following a target heading | motors GP8–GP11; encoders GP2/GP3 and GP13/GP14 (PIO0) | `tMotion` (6, core 1) |
| `line_follow/` | Two digital IR edges become target headings, line searches and junction turns | IR sensor DO GP26/GP27 | `line_follow_task` (7, any) |
| `barcode/` | Code 39 barcodes beside the line, read in either direction | IR sensor DO GP28 (edge interrupt) | `tBarcode` (8, core 0) |
| `terrain/` | GY-511: calibration at each start, tilt, humps and their height, collisions, compass and a run summary | I2C0 GP4/GP5 (Grove 3) | `terrain_task` (9, any) |
| `communication/` | Wi-Fi status; with `MQTT=1`, status, telemetry, the console and dashboard commands | Pico W radio (kernel-owned) | `communication_task` (13, any) |

How they work together:

- `line_follow` sends `motion` a target heading every 10 ms; `motion` never
  reads the line sensors.
- `barcode` decodes each barcode, and the action the dashboard set for that
  text becomes the turn at the next junction. The car slows to 30 mm/s to
  read it.
- `obstacle` holds the motors when something is ahead, measures it, and
  sends `motion` the bypass moves.
- `terrain` reads the wheel speeds from `motion`. A hump slows following,
  and humps and collisions are counted.

Every interface is listed in `TEAM.md` section 5. For each subsystem's
logic, files and known limits, read
[`docs/architecture/subsystems.md`](docs/architecture/subsystems.md).

**Minimum-bit data.** Everything stored, shared between tasks or sent uses
only the bits its range needs, packed with masks and shifts and clamped at
the field limits. The layouts are in `app_program/robot_types.h`, and
`docs/architecture/subsystems.md` section 7 lists each one with its range.

Before the first build, create your local Wi-Fi settings (section 2.2). A
plain `make` builds with `NET=lwip`, so the Pico joins that network at
power-up.

Coding standard: [BARR-C:2018](https://barrgroup.com/sites/default/files/barr_c_coding_standard_2018.pdf)
and [`docs/coding_standard.md`](docs/coding_standard.md) for `app_program/`;
the kernel keeps its own upstream style. The Dev Container in `.devcontainer/`
supplies the compiler, Pico SDK, clang-format and pre-commit.

---

You need a **Raspberry Pi Pico W**, a USB cable, and a computer running
macOS, Linux, or Windows with **WSL (Ubuntu)**. On Windows, run every
command below inside the WSL Ubuntu terminal.

---

## 1. Set up the Pico C SDK and the compiler

You need two separate things:

| What | Why | Where it comes from |
|---|---|---|
| **Pico C SDK** | USB, WiFi and network stack source code | `git clone` (step 1.2) |
| **Arm GNU Toolchain** (`arm-none-eabi-gcc`) | the compiler | a separate download (step 1.3) |

> ⚠️ **The SDK does NOT include the compiler.** You must install the compiler
> on its own and put its `bin` folder on your `PATH` (step 1.4).

The Dev Container already has both; inside it, skip to section 2.

**1.1 Install the basic tools: git, make and a host C++ compiler**

```sh
# macOS
xcode-select --install

# Linux / WSL (Ubuntu)
sudo apt update && sudo apt install -y git make g++
```

**1.2 Download the Pico C SDK** (any folder works; this guide uses `~/pico`)

```sh
mkdir -p ~/pico && cd ~/pico
git clone --branch 2.2.0 https://github.com/raspberrypi/pico-sdk.git
cd pico-sdk
git submodule update --init lib/tinyusb lib/cyw43-driver lib/lwip
```

The `submodule` line is required: it downloads the USB, WiFi and network
(lwIP) drivers. (SDK 2.3.0 has also been tested and works.)

**1.3 Install the Arm GNU Toolchain (the compiler)**

- **macOS / Windows (WSL) / Linux:** download from
  <https://developer.arm.com/downloads/-/arm-gnu-toolchain-downloads>.
  Pick the **AArch32 bare-metal target (arm-none-eabi)** package for your
  computer. For WSL, choose the **Linux x86_64** `.tar.xz`.
  Extract it, e.g. to `~/tools/arm-gnu-toolchain-<version>/`.
- **Linux / WSL shortcut:** `sudo apt install -y gcc-arm-none-eabi`
  installs it straight onto your PATH. If you use this, skip the PATH line in 1.4.

Find the folder that contains `arm-none-eabi-gcc`. It is the toolchain's
**`bin`** folder, e.g. `~/tools/arm-gnu-toolchain-15.3.rel1/bin`.

**1.4 Tell your shell where the compiler and SDK are**

Add these two lines to the end of `~/.zshrc` (macOS) or `~/.bashrc` (Linux/WSL),
**changing both paths to match your computer**:

```sh
export PATH="$HOME/tools/arm-gnu-toolchain-15.3.rel1/bin:$PATH"   # folder that contains arm-none-eabi-gcc
export PICO_SDK_PATH="$HOME/pico/pico-sdk"                         # the SDK from step 1.2
```

Open a **new** terminal, then check both:

```sh
arm-none-eabi-gcc --version     # must print a version, not "command not found"
ls $PICO_SDK_PATH               # must list the SDK files
```

If `arm-none-eabi-gcc` is "not found", your `PATH` does not point to the
toolchain's `bin` folder. Fix that before going further.

---

## 2. Clone this repo and build the `.uf2` file

**2.1 Clone your project**

```sh
cd ~/pico
git clone <URL-of-this-repo> my-project
cd my-project
```

**2.2 Set your Wi-Fi network**

```sh
cp config/wifi_secrets.example.h config/wifi_secrets.h
```

Edit `config/wifi_secrets.h` and replace `your-network-name` and
`your-network-password` with your network's name and password. The file is
ignored by git: never commit it. The credentials are compiled into the
firmware, so do not share a `.uf2` built with private credentials. To build
without joining a network, use `make NET=none`.

The same file holds the MQTT settings used by `MQTT=1` builds (section 4):
set `MQTT_BROKER_IP` to the address of the computer that will run the
dashboard. Keep `MQTT_BROKER_PORT` at 1884, the dashboard's default.

**2.3 Build**

```sh
cd build_make
make -j8
```

A successful build ends with a line like
`tools/elf2uf2 mtk3pico_smp1_usb_cdc_wifi_lwip.elf mtk3pico_smp1_usb_cdc_wifi_lwip.uf2`.
Your firmware is **`build_make/mtk3pico_smp1_usb_cdc_wifi_lwip.uf2`**.

If `make` stops with an error, it tells you which one is wrong:
`arm-none-eabi-gcc not found` means fix `PATH`,
`PICO_SDK_PATH is not set` means fix `PICO_SDK_PATH` (step 1.4), and
`config/wifi_secrets.h not found` means do step 2.2.

To build a single-core, UART-only or test version, see section 3.

**2.4 Flash it onto the Pico W**

1. Hold the **BOOTSEL** button and plug the Pico W into USB, then release.
   A drive called **RPI-RP2** appears.
2. Copy the `.uf2` file onto that drive. On WSL, run `explorer.exe .` in
   `build_make` to open the folder in Windows. The Pico reboots by itself.

**2.5 See the output**

Open the Pico's USB serial port at 115200 baud:

```sh
screen /dev/tty.usbmodem* 115200      # macOS   (quit: Ctrl-A then K)
screen /dev/ttyACM0 115200            # Linux
```

On Windows, use PuTTY or the VS Code *Serial Monitor* on the new COM port.
You should see the kernel banner, one start-up line per subsystem and
`all subsystems started`, then `Wi-Fi connected` and the IMU calibration
result (`terrain: calibration VALID tilt …`). While the car runs, each
subsystem reports its events, for example `line JUNCTION …`,
`barcode: A dir=… elements=29 …`, `obstacle: width …` and
`terrain: hump LEVEL height … mm`.

> ⚠️ **A plain `make` image drives on its own.** It has no dashboard to
> wait for: the motors are released as soon as the ultrasonic sensor
> confirms the path ahead is clear. Put the car on the line, or lift it,
> before you power it up. An `MQTT=1` image waits for Start on the
> dashboard (section 4).

**2.6 Write your own code**

Your code lives in **`app_program/`**. Every `.c` file there, and in its
subfolders, is compiled automatically.

- **`app_main.c` → `usermain()` is where the program starts.** It first calls
  every subsystem's `<name>_init()` (create kernel objects), then every
  `<name>_start()` (start tasks). Read its comments first.
- **One folder per subsystem:** `app_program/obstacle/`, `motion/`,
  `line_follow/`, `barcode/`, `terrain/`, `communication/`.
  `console_mirror/` copies the console to MQTT in `MQTT=1` builds.
- `<name>.h` is that subsystem's public interface: the object IDs and
  data formats the other subsystems may use. `<name>.c` holds
  `<name>_init()` and `<name>_start()`; `<name>_test.c` holds the tests.
- `app.h` holds the definitions shared by the whole team, and
  `robot_types.h` every packed layout that crosses a subsystem boundary or
  goes over MQTT, with its masks, shifts and clamp helpers.
- **Drivers go in your subsystem folder too**, as a `drv_<name>.c` /
  `drv_<name>.h` pair, e.g. `obstacle/drv_ultrasonic.c`. They are compiled
  automatically. Do not add drivers under `device/`: that folder holds the
  kernel's own serial, I2C and ADC drivers, and adding to it means editing
  kernel and makefile files shared by the whole team. `terrain` (I2C)
  uses those kernel drivers through `tk_opn_dev()`.
- Start new files from `templates/` (see `docs/coding_standard.md`).

**Working as a team:** record owners, pins, priorities and the interfaces
between subsystems in **`TEAM.md`**, and record every test in
**`testing/`** (see `testing/README.md`). `make TEST=1` builds a test image
that runs all the subsystem tests at power-up. Short videos of tests and
demos are shared as unlisted YouTube links, listed in **`videos/`**.

**Host tests.** The pure logic also builds and runs on your computer, with
no Pico: the line tracker, barcode decoder, bypass state machine, hump,
collision and compass code, motion's command slot and line adapter,
dashboard commands and decoding, and the packed layouts. They need a host C
compiler with the address and undefined-behaviour sanitizer libraries
(`libasan` and `libubsan`), `python3`, and `PICO_SDK_PATH` for the lwIP
headers; the Dev Container has all of them. Each subsystem has its own
suite; run them all from the repository root before you open a PR:

```sh
for t in testing/*/host/run.sh; do sh "$t" || echo "FAILED: $t"; done
```

A host test checks the logic, not the sensors: changes in behaviour still
need a run on the car, recorded in that subsystem's `TEST_LOG.md`.

Pins: use **GP0–GP22 and GP26–GP28**. GP0/GP1 are free unless your team
builds with `CRASH_REPORT=1` (section 3), which needs them. Never use
GP23/GP24/GP25/GP29: they are wired to the Pico W radio. The Pico W's
on-board LED is also on the radio, so use an external LED instead.

After editing, run `make -j8` again in `build_make` and copy the new `.uf2`.

---

## 3. Build options: `SMP`, `CONSOLE`, `WIFI`, `NET`, `MQTT`, `TEST`, `CRASH_REPORT`

`make` on its own uses the defaults. To build differently, put options
after `make`. They can be combined in any order:

```sh
make -j8                          # defaults: SMP=1 CONSOLE=usb_cdc WIFI=cyw43 NET=lwip MQTT=0 TEST=0 CRASH_REPORT=0
make SMP=0 -j8                    # single-core build
make NET=none -j8                 # radio on, but no network stack: does not join Wi-Fi
make CONSOLE=uart WIFI=none -j8   # no USB console, radio off (no Pico SDK needed)
make MQTT=1 -j8                   # the dashboard build used on the course (section 4)
make TEST=1 -j8                   # firmware with the subsystem self-tests
make CRASH_REPORT=1 -j8           # also see crash reports, on GP0/GP1 (uses those pins)
```

To set several options at once, list them all, separated by spaces. For
example, to track down a crash in the test build while running on one core only:

```sh
make SMP=0 TEST=1 CRASH_REPORT=1 -j8
#  -> build_make/mtk3pico_smp0_usb_cdc_wifi_lwip_crash_test.uf2
```

Any option you leave out keeps its default (here `CONSOLE=usb_cdc`,
`WIFI=cyw43` and `NET=lwip`). Options are not remembered: give them on every
`make`, as the next plain `make` goes back to the defaults.

| Option | Values (**default**) | What it does | Image name | C macro your code can test |
|---|---|---|---|---|
| `SMP` | `0`, **`1`** | `1` runs the kernel on both RP2040 cores; `0` uses core 0 only | `smp1` / `smp0` | `TK_SUPPORT_SMP` (1 or 0) |
| `CONSOLE` | **`usb_cdc`**, `uart` | where `tm_printf` output goes. `usb_cdc` = USB cable; `uart` = UART0 only (GP0 TX / GP1 RX, needs a USB-serial adapter) | `_usb_cdc` / `_uart` | `TM_CONSOLE_USB_CDC` |
| `WIFI` | **`cyw43`**, `none` | `cyw43` powers up the Pico W radio and its driver task; `none` leaves the radio off | `_wifi` / *(nothing)* | `TM_WIFI_CYW43` |
| `NET` | **`lwip`** (with `WIFI=cyw43`), `none` | `lwip` adds the lwIP network stack to the radio task, which joins the network in `config/wifi_secrets.h` (needs that file). Defaults to `none` when `WIFI=none` | `_lwip` / *(nothing)* | `TM_NET_LWIP` |
| `MQTT` | **`0`**, `1` | `1` adds the MQTT client (needs `NET=lwip`): status, telemetry and the console go to the dashboard, and the car waits for its Start (section 4) | `_mqtt` / *(nothing)* | `TM_MQTT` |
| `TEST` | **`0`**, `1` | `1` compiles every `<name>_test.c` and runs the tests at power-up | `_test` / *(nothing)* | `APP_TEST` (1 or 0) |
| `CRASH_REPORT` | **`0`**, `1` | `1` also copies the console to UART0 on **GP0 (TX) / GP1 (RX)**, 115200 baud, so you can see the last messages and the kernel's crash report (see below). **Uses GP0/GP1.** `CONSOLE=uart` always does this. | `_crash` / *(nothing)* | `TM_CONSOLE_UART` (1 or 0) |

Two more options are for setting up the IR sensors:

| Option | Values (**default**) | What it does | Image name | C macro |
|---|---|---|---|---|
| `IR_CALIBRATION` | **`0`**, `1` | `1` runs only the line and barcode sensor tasks and the compass, with the motors off, while you set each sensor's DO threshold and do the compass calibration spin | `_ir_calibration` | `IR_CALIBRATION_ONLY` |
| `IR_MOTOR_TEST` | **`0`**, `1` | `1` runs only line following and motion, following the line with no time limit; needs `IR_CALIBRATION=0` and `TEST=0` | `_ir_motor_test` | `IR_MOTOR_TEST` |

**Why `CRASH_REPORT` exists.** USB output is not sent the moment you call
`tm_printf`: it waits in a buffer until the kernel's USB task sends it. If the
program crashes, no task runs again, so the last messages and the kernel's
crash report never reach your USB terminal. The terminal just goes quiet.
UART output *is* sent at once, even inside the crash handler. So when you
need to know why the Pico crashed or froze, build with `CRASH_REPORT=1`,
connect a USB-serial adapter (adapter RX → GP0, adapter GND → GND), and read
it at 115200 baud. Normal `tm_printf` output on USB works the same with
either setting. If a subsystem has claimed GP0/GP1 in `TEAM.md`, unplug that
hardware before using `CRASH_REPORT=1`.

The options are part of the image name, e.g.
`mtk3pico_smp1_usb_cdc_wifi_lwip_test.uf2`, so you can always tell which build
you are flashing. Switching options needs no `make clean`: the makefile
notices and recompiles everything.

Using an option in C code:

```c
#if TK_SUPPORT_SMP
    ctsk.tskatr |= TA_ASSPRC; /* Core affinity only exists on the dual-core build. */
#endif

#if APP_TEST
    /* Code that only exists in the test build. */
#endif
```

### Where the options live in the makefile

All of them are handled in **`build_make/pico_rp2040.mk`**. To find each piece, search the file for the
names in the left column (e.g. `grep -n "TEST" build_make/pico_rp2040.mk`).
Each option goes through the same steps:

| Step | What to look for | Example for `TEST` |
|---|---|---|
| 1. Default value | `NAME ?= value` near the top of the file | `TEST ?= 0` |
| 2. Allowed values | an `$(error Unknown ...)` check just below it | `$(error Unknown TEST '$(TEST)'; use 0 or 1)` |
| 3. Image-name suffix | `NAME_SUFFIX :=` and the `EXE_FILE :=` line | `TEST_SUFFIX := _test` |
| 4. C macro for the code | a `CFLAGS += -D...` line | `CFLAGS += -DAPP_TEST=$(TEST)` |
| 5. Rebuild on change | the `PROFILE_ID :=` line | `...-test$(TEST)` |
| 6. Extra source files (`CONSOLE`, `WIFI`, `NET`, `MQTT` only) | an `ifeq ($(CONSOLE),usb_cdc)`, `ifeq ($(WIFI),cyw43)`, `ifeq ($(NET),lwip)` or `ifeq ($(MQTT),1)` block that adds the USB/radio/network/MQTT files and SDK include paths | — |

- **Changing a default:** edit the `?=` line from step 1, e.g.
  `CONSOLE ?= uart`. Every `make` without that option then uses the new value.
- **Adding your own option** (say `DEBUG=1` to turn on extra prints): copy
  how `TEST` is done. Add steps 1 to 5 next to the `TEST` lines, then
  use `#if APP_DEBUG` in your code. **Step 5 matters:** without it, switching
  the option can quietly link files that were compiled with the old setting.
- **Kernel settings** such as the maximum number of tasks, semaphores or
  mailboxes are not make options. They are in `config/config.h` (`CNF_MAX_...`).
  Which kernel device drivers start at boot is in `config/config_device.h`.

---

## 4. Run the car with the dashboard (`MQTT=1`)

On the course the car runs an `MQTT=1` image and needs no USB cable:
everything goes over Wi-Fi to `tools/dashboard/car_dashboard.py`, which is
the car's MQTT broker, its log and a web page in one script. It needs only
`python3`.

**4.1 Build and flash**

Set `MQTT_BROKER_IP` in `config/wifi_secrets.h` to your computer's address
on the car's Wi-Fi network (section 2.2), then:

```sh
cd build_make
make MQTT=1 -j8      # -> mtk3pico_smp1_usb_cdc_wifi_lwip_mqtt.uf2
```

Flash it as in section 2.4.

**4.2 Start the dashboard, then power the car**

From the repository root, on the computer whose address you set:

```sh
python3 tools/dashboard/car_dashboard.py testing/system/logs/YYYY-MM-DD_<name>_mqtt.txt
```

Open <http://localhost:8080> on the same computer. When the car connects,
the page shows its status, and the log gains a `connect from …` line.

- **Barcode actions.** Set what each barcode does at the next junction
  (`STRAIGHT`, `LEFT`, `RIGHT` or `UTURN`; up to 8 barcodes of up to 3
  characters). The dashboard saves them in `tools/dashboard/barcode_map.json`
  and sends them again every time the car connects, because the car forgets
  them on reset. A barcode with no action goes straight on.
- **Start and Stop.** The car waits, motors held, until you press Start.
  Start is never sent by itself, so a car that resets stays still. Stop
  holds the motors, but it travels over Wi-Fi and is **not an emergency
  stop**.

The topics, each under `MQTT_TOPIC` from `config/wifi_secrets.h`:

| Topic | Direction | Content |
|---|---|---|
| `/status` | car → dashboard | Six packed words, twice a second: speeds, distance, heading, tilt and turn rate, modes, obstacle range and width, hump and collision counts |
| `/telemetry` | car → dashboard | Packed records for obstacle, bypass, line and barcode events |
| `/console` | car → dashboard | Every console line, with the uptime in ms |
| `/cmd` | dashboard → car | `start`, `stop`, `clear`, `map <barcode> <action>` |

The packed layouts are in `app_program/robot_types.h`. The dashboard
decodes them and writes readable lines to the log, so one text file holds the
whole run. Keep logs under `testing/<subsystem>/logs/` and name them after
the test.

Every `MQTT=1` image joins Wi-Fi, calibration images included. A
`TEST=1 MQTT=1` image waits up to 20 s for the broker before its tests
start, so their results reach the log.

---

*micro T-Kernel 3.0 © Ken Sakamura / TRON Forum, distributed under the
T-License 2.2. This dual-core RP2040 port is derived from the official
μT-Kernel 3.0 BSP (`tron-forum/mtk3_bsp`, branch `pico_rp2040`).*
