# Calibration host checks

Run from the repository root:

```sh
python3 testing/obstacle/host/test_calibration.py
```

Requires a host C compiler (`cc`), Python 3 and `nm`. An optional first
argument selects a different repository root. Compilation and execution use
fresh temporary directories; configuration in the repository stays unchanged.
The harness copies the actual calibration source, configuration and public
headers, replacing only kernel calls and hardware driver functions. It
selects each mode and signed trims in the temporary configuration and uses a
123 ms travel setting to exercise tick rounding. No Pico SDK is required.

Checks include all calibration modes, six numeric CSV fields, context and
move-to-trigger metadata, ranging burst size and reposition windows, all
trim angles, settled reference and all 70 settle trials, direct 60-degree
moves in both directions and ten-second angle holds. Mocks reject busy
polling, simulate trigger spacing and pending reads, and poison invalid echo
outputs. Both driver initialisation failures must stop measurements. The
mode-zero object must contain no symbols.

These checks validate software sequencing and logging only. They do not
measure servo position, physical travel, ultrasonic accuracy, electrical
fault behaviour or motor integration. Perform the documented stationary
hardware procedures separately.

Mode 3 also runs with its destination at 120 degrees for both signed trims,
checking the reverse move against the same reference/trial requirements.
Preprocessor checks cover provisional scale 1715, an alternate scale, signed
offsets -6/0/+6, and rejection of zero scale, an invalid threshold domain,
and an unsupported settle destination. These are software checks, not
calibration measurements.
