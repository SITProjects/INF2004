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
the field limits. The layouts are in `app_program/robot_types.h`. Section 5
gives the Tribunal 1 evidence: every field with its range and bits, RAM use,
message sizes, and the tests and car runs behind it.

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

## 5. Tribunal 1: minimum-bit data representation

**The rule.** Data that is stored, shared between tasks or sent uses only the
bits its range needs. Each field is packed with masks and shifts and saturates
at its limit, so an out-of-range value can never spill into the next field.
Calculations unpack first and work at full size. Every shared and sent layout
is in [`app_program/robot_types.h`](app_program/robot_types.h), with its pack
and unpack functions and the helpers they use: `robot_field_put()`,
`robot_field_get()`, `robot_field_from_signed()` and `robot_mark16()`.
[`docs/architecture/subsystems.md`](docs/architecture/subsystems.md)
section 7 explains why each range was chosen.

All figures are for the final implementation: the car image
(`make MQTT=1`) built from `main` on 2026-11-10.

### 5.1 Data representation table

A *word* is a `uint32_t`; bit 0 is the least significant. Signed fields are
two's complement within their bits.

**Motion command** (IF-01 and IF-06, obstacle → motion): one word, 22 bits,
bits 22–31 always zero.

| Field | Valid range | Bits | Representation |
|---|---|---|---|
| Type | STOP, MOVE, TURN, FOLLOW (0–3) | 2 | bits 0–1 |
| Stop at the line | 0 or 1 | 1 | bit 2 |
| Reverse (a move backward, a turn right) | 0 or 1 | 1 | bit 3 |
| Distance | 0–1023 mm | 10 | bits 4–13 |
| Angle | 0–180° | 8 | bits 14–21 |

**Motion status** (IF-03, motion → obstacle, line following, terrain and
status): one byte, 5 bits.

| Field | Valid range | Bits | Representation |
|---|---|---|---|
| Mode | 7 modes (0–6) | 3 | bits 0–2 |
| Why the last move ended | NONE, DIST, LINE (0–2) | 2 | bits 3–4 |

**Line request** (IF-04, line following → motion): one word, 30 bits.

| Field | Valid range | Bits | Representation |
|---|---|---|---|
| Mode | TRACK, SEARCH_L, SEARCH_R, STOP (0–3) | 2 | bits 0–1 |
| Line seen, junction, slow, valid | 0 or 1 each | 4 | bits 2–5 |
| Reverse distance | 0–255 mm (the barcode back-up is 170 mm) | 8 | bits 6–13 |
| Target heading | −32,768 to 32,767°, not wrapped (about 91 turns) | 16 signed | bits 14–29 |

**Odometry** (IF-05, motion → terrain, obstacle and line following): three
words, 81 bits.

| Field | Valid range | Bits | Representation |
|---|---|---|---|
| Distance | ±8,388,607 mm (about 8.4 km) | 24 signed | word 0, bits 0–23 |
| Update count | 1–255, wraps; 0 before the first | 8 | word 0, bits 24–31 |
| Heading | −32,768 to 32,767°, not wrapped | 16 signed | word 1, bits 0–15 |
| Mean speed | −1024 to 1023 mm/s (the car tops out at 620) | 11 signed | word 1, bits 16–26 |
| Left and right wheel speed | −1024 to 1023 mm/s | 11 signed each | word 2, bits 0–10 and 11–21 |

**Obstacle result** (IF-07, scanner → bypass and telemetry): 33 bits in a
6-byte structure.

| Field | Valid range | Bits | Representation |
|---|---|---|---|
| Status | SCANNING, CLEAR, PATH_FOUND, BLOCKED, UNKNOWN (0–4) | 3 | flags byte, bits 0–2 |
| Direction | NONE, LEFT, CENTRE, RIGHT (0–3) | 2 | flags byte, bits 3–4 |
| Left and right edge found | 0 or 1 each | 2 | flags byte, bits 5–6 |
| Critically close | 0 or 1 | 1 | flags byte, bit 7 |
| Left and right edge | sweep index 0–8 (50–130° in 10° steps), 15 none | 4 each | edges byte, bits 0–3 and 4–7 |
| Width | 0–511 mm, 0 unavailable | 9 | `uint16_t`, saturated at 511 |
| Sequence | 0–255, wraps | 8 | `uint8_t` |

**Obstacle geometry** (IF-07): one 64-bit word.

| Field | Valid range | Bits | Representation |
|---|---|---|---|
| Range to the face | 0–4095 mm (the sensor's 4 m limit) | 12 | bits 0–11 |
| Left and right edge, sideways | ±4,095 mm | 13 signed each | bits 12–24 and 25–37 |
| Range at each end of the sweep | 0–4095 mm | 12 each | bits 38–49 and 50–61 |
| Each end clear | 0 or 1 each | 1 each | bits 62 and 63 |

**Obstacle control** (obstacle → others): one half-word, 9 bits.

| Field | Valid range | Bits | Representation |
|---|---|---|---|
| Stop requested | 0 or 1 | 1 | bit 0 |
| Bypass state | 13 states (0–12) | 4 | bits 1–4 |
| Fault | 9 reasons (0–8) | 4 | bits 5–8 |

**Watch reading** (scanner → bypass, while passing the obstacle): one word,
24 bits.

| Field | Valid range | Bits | Representation |
|---|---|---|---|
| Echo | 0–32,767 µs, 0 none | 15 | bits 0–14 |
| Sequence | 0–255, wraps | 8 | bits 15–22 |
| Sensor held to the side | 0 or 1 | 1 | bit 23 |

**Barcode turn and pass phase** (IF-08, barcode → line following): one byte
each, each written by one task only, so neither needs a lock.

| Field | Valid range | Bits | Representation |
|---|---|---|---|
| Turn at the next junction | STRAIGHT, LEFT, RIGHT, UTURN (0–3) | 2 | `uint8_t` |
| Slow pass phase | NONE, BRAKE, BACK, CREEP (0–3) | 2 | `uint8_t` |

**Compass** (IF-09, terrain → any): two words, 57 bits.

| Field | Valid range | Bits | Representation |
|---|---|---|---|
| X, Y and Z field | −2048 to 2047 counts (the sensor's own range) | 12 signed each | word 0, bits 0–11 and 12–23; word 1, bits 0–11 |
| Samples read since start-up | 0–255, wraps | 8 | word 0, bits 24–31 |
| Heading | 0–3599 tenths of a degree | 12 | word 1, bits 12–23 |
| Valid | 0 or 1 | 1 | word 1, bit 24 |

**Barcode map entry** (IF-10, dashboard → car, 8 entries): one word, 20 bits,
so one write replaces an entry with no lock.

| Field | Valid range | Bits | Representation |
|---|---|---|---|
| Barcode text, up to 3 characters | Code 39 symbol + 1 (1–43); 0 ends the text | 6 each | bits 0–5, 6–11 and 12–17 |
| Turn | STRAIGHT, LEFT, RIGHT, UTURN (0–3) | 2 | bits 18–19 |

**Barcode edge** (GP28 interrupt → `tBarcode`, 64-entry queue): one word.

| Field | Valid range | Bits | Representation |
|---|---|---|---|
| Level after the edge | 0 or 1 | 1 | bit 0 |
| Time | the 1 MHz timer in 2 µs steps, 0 to 2³¹−1 | 31 | bits 1–31; differences still work across the timer's wrap |

**Terrain event** (terrain monitor → terrain task, 16-entry queue): one word.

| Field | Valid range | Bits | Representation |
|---|---|---|---|
| Type | 11 events (0–10) | 4 | bits 0–3 |
| Value (for example a hump's height in mm) | 0–4095 | 12 | bits 4–15 |
| Time | 0–65,535 ticks of 10 ms, wraps | 16 | bits 16–31 |

**Telemetry record** (MQTT `<topic>/telemetry`): three words, 12 bytes.

| Field | Valid range | Bits | Representation |
|---|---|---|---|
| Type | OBSTACLE, AVOID, LINE, BARCODE (1–4) | 4 | word 0, bits 0–3 |
| Sequence | 0–255, wraps | 8 | word 0, bits 4–11 |
| Capture time | 0–1,048,575 ticks of 10 ms (2.9 h), wraps | 20 | word 0, bits 12–31 |
| OBSTACLE: flags, edges, width | as the obstacle result | 8, 8, 9 | word 1, bits 0–7, 8–15 and 16–24 |
| AVOID: from and to state | 0–12 each | 4 each | word 1, bits 0–3 and 4–7 |
| LINE: mode, junction, turn | 0–3, 0 or 1, 0–3 | 2, 1, 2 | word 1, bits 0–1, 2 and 3–4 |
| LINE: distance | ±524,287 mm (about 524 m) | 20 signed | word 1, bits 5–24 |
| LINE: target heading and heading | −32,768 to 32,767° | 16 signed each | word 2, bits 0–15 and 16–31 |
| BARCODE: text, up to 10 characters | Code 39 symbol + 1 (1–43); 0 ends | 6 each | five per word, words 1 and 2 |
| BARCODE: read backwards | 0 or 1 | 1 | word 1, bit 30 |

**Car status** (MQTT `<topic>/status`, twice a second): six words, 174 bits,
24 bytes.

| Field | Valid range | Bits | Representation |
|---|---|---|---|
| Distance | ±8,388,607 mm | 24 signed | word 0, bits 0–23 |
| Humps counted | 0–255 | 8 | word 0, bits 24–31 |
| Speed, left and right wheel | −1024 to 1023 mm/s | 11 signed each | word 1, bits 0–10 and 11–21; word 2, bits 0–10 |
| Motion mode | 0–6 | 3 | word 1, bits 22–24 |
| Obstacle status | 0–4 | 3 | word 1, bits 25–27 |
| Bypass state | 0–12 | 4 | word 1, bits 28–31 |
| Obstacle width | 0–511 mm | 9 | word 2, bits 11–19 |
| Range to the obstacle | 0–4095 mm | 12 | word 2, bits 20–31 |
| Heading | −32,768 to 32,767° | 16 signed | word 3, bits 0–15 |
| Pitch and roll | ±204.7°, in tenths | 12 signed each | word 3, bits 16–27; word 4, bits 0–11 |
| IMU calibration state | 0–7 | 3 | word 3, bits 28–30 |
| Turn rate | −1024 to 1023 °/s | 11 signed | word 4, bits 12–22 |
| Line requests sent and taken | low 8 bits of each count | 8 each | word 4, bits 23–30; word 5, bits 8–15 |
| Collisions counted | 0–255 | 8 | word 5, bits 0–7 |

**State kept inside a subsystem** is packed the same way:

| Data | Representation |
|---|---|
| Line tracker (`line_track_t`, 64 bytes) | 14 yes/no facts in one flags word, 16 small values in 30 bits, distance marks and headings 16 bits each |
| Bypass (`obstacle_avoid_t`, 76 bytes) | 19 yes/no facts in one flags word, 7 small values in 20 bits, the plan in 13 bits, distances 16 bits |
| Barcode scan (`barcode_scan_t`) | Element widths 16 bits each, in 16 µs steps (up to about 1 s) |
| Barcode trigger | Index, count and two flags in one byte |
| Stopped-car check | Speed 11, age 8 and off-level count 8 in one word; the peak push 16 bits in 1/256 g |

A 16-bit distance mark is read as the value nearest the current travel, which
is exact within 32 m either way. The line tracker also ages its marks, so one
left unused never wraps round and reads as recent.

**Left full size on purpose:** calculations and single-sample inputs, the
kernel clock stamps and sequence counters that guard shared copies (a
narrower counter could let a torn read through), compile-time settings,
tests, the upstream kernel, lwIP and TinyUSB, and the console text and its
MQTT mirror, which is the only debug view without a cable.

### 5.2 RAM usage

Static RAM of the final car image, from `arm-none-eabi-size`:

| Section | Bytes |
|---|---|
| RAM vector table | 192 |
| `.data` | 1,196 |
| `.bss` | 102,856 |
| **Total** | **104,244 of 270,336 (38.6%)** |

The application's six task stacks, 2,048 bytes each, come from the kernel's
memory area above this.

By owner, from the symbol table (`arm-none-eabi-nm -S -l`):

| Owner | Bytes |
|---|---|
| lwIP network stack | 57,359 |
| Console mirror (debug copy of the console to MQTT) | 16,993 |
| micro T-Kernel | 15,932 |
| TinyUSB | 6,112 |
| CYW43 radio driver | 2,532 |
| Communication | 1,660 |
| Other library code | 1,240 |
| Barcode | 821 |
| Terrain | 486 |
| Motion | 440 |
| Obstacle | 407 |
| Line following | 92 |

The six subsystems use 3,906 bytes between them. Their largest buffers are
all packed:

| Buffer | Bytes |
|---|---|
| Telemetry queue, 128 records of 12 bytes | 1,536 |
| Barcode scan widths | 508 |
| Barcode edge queue, 64 edges of 4 bytes | 256 |
| Bypass state | 76 |
| Terrain event queue, 16 events of 4 bytes and a bookkeeping word | 68 |
| Line tracker | 64 |
| Barcode map, 8 entries of 4 bytes | 32 |
| Line request slots and motion's copy | 24 |
| Obstacle result, control and geometry | 16 |
| Odometry | 12 |

To measure it again:

```sh
cd build_make
make MQTT=1 -j8
arm-none-eabi-size -A -d mtk3pico_smp1_usb_cdc_wifi_lwip_mqtt.elf
arm-none-eabi-nm -S -l --size-sort -t d mtk3pico_smp1_usb_cdc_wifi_lwip_mqtt.elf
```

### 5.3 Message sizes

| Message | From → to | Transport | Bits used | Bytes per message | Rate and buffer |
|---|---|---|---|---|---|
| Motion command | obstacle → motion | kernel message buffer | 22 | 4 | Per bypass leg; 4 deep |
| Motion status | motion → others | status byte and event flag | 5 | 1 | On change |
| Line request | line following → motion | two latest-value slots | 30 | 4, plus a 4-byte receive time | 50 a second |
| Odometry | motion → terrain, obstacle, line following | sequence-checked copy | 81 | 12 | 50 a second |
| Obstacle result, control and geometry | obstacle → any | one sequence-checked copy | 33 + 9 + 64 | 16 | Per scan |
| Watch reading | scanner → bypass | one word | 24 | 4 | Per reading while passing |
| Barcode turn and phase | barcode → line following | one byte each | 2 + 2 | 1 + 1 | Per barcode |
| Compass | terrain → any | copy under a spin lock | 57 | 8 | Per sample |
| Barcode map entry | radio task → barcode and line following | one word each | 20 | 4 | Per dashboard change; 8 entries |
| Barcode edge | GP28 interrupt → `tBarcode` | queue | 32 | 4 | Per edge; 64 deep |
| Terrain event | terrain monitor → terrain task | queue | 32 | 4 | Per event; 16 deep |
| Telemetry record | any task → MQTT `/telemetry` | queue, then MQTT | 96 | 12 | Per event; 128 deep |
| Car status | → MQTT `/status` | built when sent | 174 | 24 | Twice a second |

Dashboard commands on `/cmd` arrive as short text (`start`, `stop`,
`clear`, `map <barcode> <action>`), and the car stores the result packed: one
flag for start and stop, and a 20-bit word per barcode.

### 5.4 Correctness and performance

**No loss of required functionality.**

- **Layouts.** Each field reads back at its minimum and maximum, saturates
  one past them, and leaves its neighbours alone (INT-18,
  `sh testing/integration/host/run.sh`, 67 checks).
- **Packed state.** The line tracker's marks are tested across the 16-bit
  wrap (`testing/line_follow/host`, 525 checks). The bypass's packed state is
  checked field by field (AS01–AS10 in `testing/obstacle/host`), and the
  barcode widths at their limits (`testing/barcode/host`, 33 checks).
- **Sent data.** The dashboard decodes telemetry and status records built by
  the firmware's own pack functions (`testing/communication/host`, 62 checks,
  W01–W07).
- **Same behaviour.** The packed line tracker, bypass and collision rule gave
  identical outputs to the unpacked code on 8 million, 6 million and
  20 million random inputs. Bugs planted in every layout were caught.
- **On the car**, with the minimum-bit firmware on 2026-11-09 and 2026-11-10:
  - line following with junction turns and a U-turn;
  - barcodes A, Z and F decoded and their turns taken;
  - humps counted;
  - obstacle bypasses finished and rejoined the line, six of them on
    2026-11-09 (INT-16);
  - the dashboard showed live status and barcode reads.

Run every host suite with:

```sh
for t in testing/*/host/run.sh; do sh "$t" || echo "FAILED: $t"; done
```

**No unacceptable loss of timing or performance.**

- **Cost of packing.** Compiled for the Cortex-M0+ at `-O2`, as the firmware
  is, packing and unpacking take at most these instructions:

  | Message | Pack | Unpack |
  |---|---|---|
  | Line request | 31 | 10 |
  | Odometry | 84 | 42 |
  | Obstacle geometry | 62 | 35 |
  | Watch reading | 15 | 10 |
  | Car status | 183 | (dashboard) |

  At 125 MHz each takes under 3 µs. The busiest work is the line request, sent
  50 times a second, and the odometry, written 50 times a second and read
  about 300 times a second. Together they cost under 20,000 instructions a
  second, about 0.03% of one core.
- **Rates on the car.** While following, motion received line requests at a
  steady 50 a second: 25 in every half-second status interval, with the 10th
  to 90th percentile at 25–26, over 343 intervals in the 2026-11-09 and
  2026-11-10 runs. Status records arrived every 0.5 s.
- **Memory.** The final image uses 38.6% of RAM, and its code
  (414,894 bytes) fits easily in the 2 MB of flash.

---

*micro T-Kernel 3.0 © Ken Sakamura / TRON Forum, distributed under the
T-License 2.2. This dual-core RP2040 port is derived from the official
μT-Kernel 3.0 BSP (`tron-forum/mtk3_bsp`, branch `pico_rp2040`).*
