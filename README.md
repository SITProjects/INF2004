# INF2004 Vehicle Controller

Structure-only embedded C project. Source files contain file comments, their
matching header include, and empty section markers. There are no functions,
API declarations, algorithms, or hardware operations yet.

## Repository layout

```text
app/
    CMakeLists.txt
    main.c / main.h
    vehicle_controller/
        vehicle_controller.c / vehicle_controller.h
modules/
    motion/
    navigation/
    obstacle/
    telemetry/
    terrain/
drivers/
    CMakeLists.txt
    barcode/
    gpio/
    imu/
    motor/
    ultrasonic/
    wifi/
shared/
    CMakeLists.txt
    include/shared.h
    src/shared.c
templates/
    module.c / module.h
docs/
    architecture/
    hardware/
    interfaces/
    coding_standard.md
```

Every feature module and individual driver contains `CMakeLists.txt`,
`include/<name>.h`, and `src/<name>.c`. Public include paths and dependencies
are scoped to CMake targets.

## Development environment

Open the repository in VS Code and select **Reopen in Container**. The supplied
Dev Container provides CMake, the ARM compiler, Pico SDK, clang-format, and
pre-commit. Work inside the container for consistent tooling.

Install the commit hooks if they are not already installed:

```sh
pre-commit install
```

## Check the skeleton

The default build compiles the empty modules and application objects using C99.
It requires a host C compiler and CMake; no Pico SDK is needed for this check.

```sh
cmake -S . -B build/skeleton
cmake --build build/skeleton
find app modules drivers shared templates -type f \
    \( -name '*.c' -o -name '*.h' \) -exec clang-format --dry-run --Werror {} +
```

No executable or firmware is produced because `main()` is intentionally absent.
After implementing the application, the reserved Pico firmware configuration is:

```sh
cmake -S . -B build/firmware -DBUILD_FIRMWARE=ON
cmake --build build/firmware
```

This requires `PICO_SDK_PATH` and the ARM toolchain. Select the intended board
with `-DPICO_BOARD=<board>` when configuring firmware. Linking firmware will
remain incomplete until the entry point is implemented. Wokwi paths refer to
the future output in `build/firmware/`.

## Coding standard

Use [BARR-C:2018](https://barrgroup.com/sites/default/files/barr_c_coding_standard_2018.pdf)
and the [project coding guide](docs/coding_standard.md). Start new C modules
from [the templates](templates/). Editor settings and the formatting hook cover
the supported style rules; future code still needs semantic review.
