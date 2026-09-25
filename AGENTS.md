# Repository guidance

These instructions apply throughout this repository. Check for more specific
`AGENTS.md` files when working in a subdirectory.

## Project and layout

INF2004 is an embedded C vehicle controller using the Raspberry Pi Pico SDK.
The application currently runs obstacle scanning with ultrasonic and servo
drivers; most other components remain placeholders.

- `app/`: application entry point, pin assignments, and vehicle coordination.
- `modules/`: motion, navigation, obstacle, telemetry, and terrain features.
- `drivers/`: barcode, GPIO, IMU, motor, servo, ultrasonic, and Wi-Fi drivers.
- `shared/`: common utilities without dependencies on higher layers.
- `templates/`: starting points for new C source and header pairs.
- `docs/`: coding standard and architecture, hardware, and interface notes.
- `.devcontainer/`: reproducible development tools and Pico SDK installation.
- `wokwi.toml` and `diagram.json`: simulator output paths and wiring.

Keep application decisions in `app/`, feature behavior in `modules/`, and
hardware access in `drivers/`. Features may depend on drivers and shared code;
drivers must not depend on features or application code. Keep dependencies and
include paths scoped to CMake targets. Register new sources and components in
the relevant `CMakeLists.txt` files.

## Build and development

Prefer the supplied VS Code Dev Container. It supplies CMake, the ARM GCC
toolchain, clang-format, pre-commit, and the Pico SDK. Inside it,
`PICO_SDK_PATH` is set to `/opt/pico-sdk`; elsewhere, set this environment
variable to an installed SDK with its required submodules.

Run from the repository root for the Pico board used by the current simulator:

```sh
cmake -S . -B build -DPICO_BOARD=pico
cmake --build build --parallel 2
```

Use the intended `PICO_BOARD` for other hardware. Use a separate build directory
when changing board or toolchain, and adjust simulator paths if needed.
`wokwi.toml` currently expects `build/vehicle_controller.uf2` and
`build/vehicle_controller.elf`. USB stdio is enabled; UART stdio is disabled.

Some documentation still describes an empty C99 skeleton and an optional
`BUILD_FIRMWARE` mode. The active root `CMakeLists.txt` requires the Pico SDK,
sets C11, and creates the firmware executable directly; it does not include
`app/CMakeLists.txt`. Do not assume the documented SDK-free skeleton build
exists. Check current source and CMake configuration when documentation differs.

## C conventions

Follow `docs/coding_standard.md`, `.clang-format`, `.editorconfig`, and the
section order in `templates/module.c` and `templates/module.h`.

- Use four spaces, LF endings, a final newline, and an 80-column limit for C.
- Put braces on separate lines and use braces for all control-flow bodies.
- Pair each source with a same-named header and include that header first.
- Use header guards and expose only the necessary public API in `include/`.
- Keep private helpers and module state `static`; use lowercase names with
  underscores and module-prefixed public symbols.
- Document public contracts with Doxygen, including units, return values,
  initialization requirements, and ownership where relevant.
- Follow the project's BARR-C:2018 guidance. Formatting checks cover only a
  subset of the standard, so review types and control flow as well.
- Preserve upstream conventions in SDK and other third-party code.

## Validation and change scope

For C or build changes, build the firmware and check formatting of affected C
and header files with `clang-format --dry-run --Werror <paths>`. To check all
project C files:

```sh
rg --files app modules drivers shared templates -g '*.c' -g '*.h' -0 |
    xargs -0 clang-format --dry-run --Werror
```

There is currently no repository test suite or CTest setup. For behavior changes,
exercise relevant cases on hardware or a suitable simulator, or use focused
host tests with hardware dependencies stubbed where practical. The checked-in
Wokwi diagram does not currently wire the ultrasonic sensor or servo, so it
cannot validate obstacle scanning as-is. Report exactly what was checked and
any unavailable validation; a successful build does not verify sensor behavior.
Documentation-only changes need a content and diff review, not a firmware build.

Keep changes focused on the requested task and preserve unrelated work. Avoid
repository-wide formatting as part of a small fix. Keep generated firmware,
build files, and downloaded SDK dependencies out of commits. Before finishing,
review `git diff --check` and `git status --short`, and summarize the change and
validation. The configured pre-commit hooks also enforce formatting and file
hygiene and prevent commits directly to `main`.
