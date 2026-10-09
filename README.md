# INF2004 Vehicle Controller

Embedded C vehicle controller using the Raspberry Pi Pico SDK. The application
runs obstacle scanning and Wi-Fi connection monitoring; other components remain
placeholders.

## Repository layout

```text
app/
    CMakeLists.txt
    main.c / main.h
    credentials.example.h
    vehicle_controller/
        vehicle_controller.c / vehicle_controller.h
modules/
    communication/
    motion/
    navigation/
    obstacle/
    terrain/
drivers/
    CMakeLists.txt
    barcode/
    gpio/
    imu/
    motor/
    servo/
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

## Wi-Fi credentials

Before building, copy the committed example header from the repository root:

```sh
cp app/credentials.example.h app/credentials.h
```

If you already have `app/credentials.h`, edit it instead of overwriting it.
Replace `your-network-name` and `your-network-password` with your Wi-Fi SSID
and password. The current driver uses WPA2 authentication.

`app/main.c` includes this local header and passes `WIFI_SSID` and
`WIFI_PASSWORD` to `connectivity_init()`. Keep the actual credentials in
`app/credentials.h`, which Git ignores; commit only the example header. A fresh
checkout needs this setup before compilation. Credentials are compiled into
firmware, so avoid sharing firmware containing private credentials.

## Build and check

The current firmware uses C11 and requires the Pico SDK and ARM toolchain.
The Dev Container sets `PICO_SDK_PATH` to `/opt/pico-sdk`; outside the container,
set it to your installed SDK, including its required submodules.

For the application with Wi-Fi enabled, use a Raspberry Pi Pico W:

```sh
cmake -S . -B build/pico_w -DPICO_BOARD=pico_w
cmake --build build/pico_w --parallel 2
```

Use a separate build directory when changing board or toolchain. The current
Wi-Fi driver requires a board with a supported Wi-Fi chip; the plain Pico does
not provide Wi-Fi. Outputs include `build/pico_w/vehicle_controller.uf2` and
`build/pico_w/vehicle_controller.elf`.

Check project C formatting with:

```sh
rg --files app modules drivers shared templates -g '*.c' -g '*.h' -0 |
    xargs -0 clang-format --dry-run --Werror
```

USB stdio is enabled and UART stdio is disabled. The checked-in `wokwi.toml`
expects firmware under `build/`; update its paths if using `build/pico_w/`.
The simulator diagram does not currently wire the ultrasonic sensor or servo,
so a successful build does not validate obstacle scanning or Wi-Fi behavior.

## Coding standard

Use [BARR-C:2018](https://barrgroup.com/sites/default/files/barr_c_coding_standard_2018.pdf)
and the [project coding guide](docs/coding_standard.md). Start new C modules
from [the templates](templates/). Editor settings and the formatting hook cover
the supported style rules; future code still needs semantic review.
