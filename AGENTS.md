# Repository guidance

These instructions apply throughout this repository. Check for more specific
`AGENTS.md` files when working in a subdirectory.

## Project and layout

INF2004 is an embedded C vehicle controller on the course's micro T-Kernel
3.0 template for the Raspberry Pi Pico W. The kernel runs on both RP2040
cores; the Pico SDK supplies only the USB, CYW43 radio and lwIP sources that
the makefile compiles in, not its runtime.

- `app_program/`: the application. `app_main.c` holds `usermain()`, which
  calls every subsystem's `<name>_init()` and then `<name>_start()`;
  `app.h` and `test_report.h` are shared team files.
- `app_program/<name>/`: one folder per subsystem (`obstacle`, `motion`,
  `line_follow`, `terrain`, `communication`) with `<name>.c/.h` (entry
  points and public interface), `<name>_test.c` (on-target tests),
  `drv_<name>.c/.h` drivers, and feature modules.
- `kernel/`, `include/`, `lib/`, `device/`, `config/`, `etc/`: micro
  T-Kernel, its libraries, device drivers and configuration (shared,
  upstream style).
- `build_make/`: the makefile (`pico_rp2040.mk` holds the build options).
- `tools/elf2uf2/`: host tool built on first use.
- `templates/`: starting points for subsystem, driver and module files.
- `docs/`: coding standard and architecture, hardware, and interface notes.
- `testing/`: test logs per subsystem plus integration and system logs.
- `videos/`: index of unlisted YouTube test and demo videos.
- `TEAM.md`: owners, pins, peripherals, task priorities, interfaces and the
  kernel object budget.
- `.devcontainer/`: reproducible tools (ARM GCC, clang-format, pre-commit,
  Pico SDK).

Keep a subsystem's tasks, drivers and feature logic inside its folder. Each
peripheral has exactly one owning task (`TEAM.md` section 3); other
subsystems reach it through kernel objects listed in `TEAM.md` section 5.
Application tasks must not call `cyw43_` or lwIP functions: the kernel's
radio service task owns them. Changes to `app_main.c`, `app.h`,
`test_report.h`, `config/`, `device/`, `build_make/` or `TEAM.md` affect the
whole team; keep them minimal and say so in the summary.

## Build and development

Prefer the supplied VS Code Dev Container. It supplies make, the ARM GCC
toolchain, clang-format, pre-commit, and the Pico SDK, with `PICO_SDK_PATH`
set to `/opt/pico-sdk`. Elsewhere, set `PICO_SDK_PATH` to an SDK with the
`lib/tinyusb`, `lib/cyw43-driver` and `lib/lwip` submodules.

The default build joins Wi-Fi (`NET=lwip`) and needs the gitignored
`config/wifi_secrets.h`; create it from `config/wifi_secrets.example.h`
(placeholder values are enough to compile). Never commit real credentials.

```sh
cd build_make
make -j8                 # -> mtk3pico_smp1_usb_cdc_wifi_lwip.uf2
make TEST=1 -j8          # firmware that runs every <name>_test() at boot
```

Other options (`SMP`, `CONSOLE`, `WIFI`, `NET`, `TEST`, `CRASH_REPORT`) are
described in README section 3. Changing options needs no `make clean`. Build
outputs stay in `build_make/` and are gitignored.

## C conventions

Follow `docs/coding_standard.md`, `.clang-format`, `.editorconfig`, and the
section order in `templates/`. These apply to `app_program/` and
`templates/`; the kernel sources keep their upstream conventions.

- Use four spaces, LF endings, a final newline, and an 80-column limit for C.
- Put braces on separate lines and use braces for all control-flow bodies.
- Pair each source with a same-named header and include that header first.
- Use header guards and expose only the necessary public API.
- Keep private helpers and module state `static`; use lowercase names with
  underscores and module-prefixed public symbols.
- Document public contracts with Doxygen, including units, return values,
  initialization requirements, ownership, and which task or core may call.
- Use the kernel API, `bsp/libbsp.h`, the kernel device drivers, or register
  access; Pico SDK runtime functions are not linked. Print with `tm_printf()`
  (no `%f`). Guard dual-core-only kernel calls with `TK_SUPPORT_SMP`.
- Follow the project's BARR-C:2018 guidance. Formatting checks cover only a
  subset of the standard, so review types and control flow as well.

## Validation and change scope

For C or build changes, build the firmware and check formatting:

```sh
find app_program templates -name '*.[ch]' -print0 |
    xargs -0 clang-format --dry-run --Werror
```

Build at least the default profile and the profiles a change touches; for
kernel-facing code also `make SMP=0` and `make TEST=1`. There is no host test
suite. For behaviour changes, exercise the relevant cases on hardware with
`make TEST=1` and the manual tests in `testing/<name>/TEST_LOG.md`. Report
exactly what was checked and any unavailable validation; a successful build
does not verify sensor behaviour. Documentation-only changes need a content
and diff review, not a firmware build.

Keep changes focused on the requested task and preserve unrelated work. Avoid
repository-wide formatting as part of a small fix, and never reformat kernel
sources. Keep generated firmware, build files, and credentials out of
commits. Before finishing, review `git diff --check` and `git status --short`,
and summarize the change and validation. The configured pre-commit hooks also
enforce formatting and file hygiene and prevent commits directly to `main`.
