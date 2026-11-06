# Architecture

The firmware is the course's micro T-Kernel 3.0 template for the Pico W:
the kernel runs on both RP2040 cores, with the USB console and the CYW43
radio service built in. The vehicle code lives in `app_program/`.

```text
app_program/
    app_main.c / app.h      usermain(): init then start every subsystem
    test_report.h           TEST_CHECK helpers for make TEST=1
    obstacle/               ultrasonic + servo scan      (task, core 0)
    motion/                 motors, encoders, moves      (tMotion task)
    line_follow/            IR sensors -> target heading (task)
    barcode/                IR sensor -> Code 39 text    (task, core 0)
    terrain/                GY-511 IMU, calibration      (task)
    communication/          Wi-Fi status                 (task)
kernel/ include/ lib/ device/ config/ etc/   micro T-Kernel (shared, upstream style)
build_make/                 makefile; build here with make
tools/elf2uf2/              host tool that converts the .elf to .uf2
```

Each subsystem folder holds:

- `<name>.c` / `<name>.h`: `<name>_init()` creates kernel objects,
  `<name>_start()` starts the tasks; the header is the public interface.
- `<name>_test.c`: on-target tests, compiled only with `make TEST=1`.
- `drv_<name>.c/.h`: drivers for the hardware the subsystem owns.
- Feature modules: `obstacle_scan`, `motion_drive`, `motion_move` and
  `motion_config.h`, `line_track`, `barcode_scan`, `barcode_code39`,
  `terrain_monitor`, `terrain_process`, `terrain_platform` and
  `terrain_config.h`, `connectivity`.

Start-up: the kernel calls `usermain()` in the initial task. It calls every
`<name>_init()`, then every `<name>_start()`, then (test build only) every
`<name>_test()`, and sleeps forever.

Dependencies point downwards only: subsystem tasks call their feature
modules; feature modules call their drivers; drivers call the kernel, the
board helpers in `bsp/libbsp.h`, the kernel device drivers, or registers.
Subsystems communicate only through kernel objects and the non-blocking
functions listed in `TEAM.md` section 5 and
[`../interfaces/README.md`](../interfaces/README.md); motion's are
proposed there and not yet agreed.

Hardware ownership (one owning task per peripheral) and task priorities are
in `TEAM.md` sections 3 and 4.

## Changes since the Week 6 design review

- **Wheel encoders are counted by PIO0, not a GPIO interrupt.** The review
  froze GPIO-ISR counting, but the GPIO bank interrupt has one handler
  shared by both cores, and the obstacle subsystem's echo handler already
  owns it, so a second handler would replace it. Polling is also not an
  option, because the 10 ms kernel tick misses edges. A PIO state
  machine per encoder (`motion/drv_encoder.c`) counts every edge in
  hardware with no CPU load, and leaves obstacle's code unchanged. The
  motion task's interface does not change: it still reads both counts
  every 20 ms.
- **Task priorities are the freeze priorities + 3, and tMotion runs on
  core 1.** The freeze numbered tSafety 2 and tMotion 3, but in this
  template the initial task uses 1, the kernel's USB console 3 and its
  Wi-Fi task 4. Every application task keeps the freeze order, 3 lower
  (`TEAM.md` section 4). The kernel's Wi-Fi task still outranks every
  application task, unlike the freeze's lowest-priority tComms, so tMotion
  is pinned to core 1: the USB, Wi-Fi and obstacle tasks are pinned to
  core 0, so none of them can delay its 20 ms update (NFR1, NFR6).
- **The IR sensors are digital, and line following steps the heading.**
  The analogue line follower weaved on straights. Each IR module's DO
  output (an LM393 comparator set by its potentiometer) is now read as a
  plain input: the two line sensors straddle the tape and are polled every
  10 ms, and a confirmed edge moves the target heading by a few degrees.
  `tMotion` holds that heading with a PI loop on the wheel odometry, so the
  car turns by each step in a short arc and then runs straight. A quick
  crossing, a held edge or a long white run starts a pivot search
  (`line_follow/line_track.c`).
- **The GPIO bank interrupt is shared through a dispatcher.** The barcode
  sensor (GP28) needs edge timestamps, which the 10 ms tick cannot give, so
  `obstacle/drv_gpio_irq.c` now owns the one `IO_IRQ_BANK0` handler and
  calls the ultrasonic echo and barcode handlers in turn. `tBarcode` decodes
  Code 39 from the edge widths in either direction.
