# Architecture

The firmware is the course's micro T-Kernel 3.0 template for the Pico W:
the kernel runs on both RP2040 cores, with the USB console and the CYW43
radio service built in. The vehicle code lives in `app_program/`.

```text
app_program/
    app_main.c / app.h      usermain(): init then start every subsystem
    test_report.h           TEST_CHECK helpers for make TEST=1
    obstacle/               ultrasonic + servo scan      (task, core 0)
    motion/                 motors, open-loop drive      (no task yet)
    line_follow/            IR sensors -> line commands  (task)
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
- Feature modules: `obstacle_scan`, `motion_drive` and `motion_config.h`,
  `line_command`, `terrain_monitor`, `terrain_process`, `terrain_platform`
  and `terrain_config.h`, `connectivity`.

Start-up: the kernel calls `usermain()` in the initial task. It calls every
`<name>_init()`, then every `<name>_start()`, then (test build only) every
`<name>_test()`, and sleeps forever.

Dependencies point downwards only: subsystem tasks call their feature
modules; feature modules call their drivers; drivers call the kernel, the
board helpers in `bsp/libbsp.h`, the kernel device drivers, or registers.
Subsystems do not call each other yet. When they do, they communicate
through kernel objects listed in `TEAM.md` section 5 and
[`../interfaces/README.md`](../interfaces/README.md).

Hardware ownership (one owning task per peripheral) and task priorities are
in `TEAM.md` sections 3 and 4.
