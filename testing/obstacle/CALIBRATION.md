# Obstacle calibration

Status: calibration firmware and procedures only. All measurements, updated
constants and hardware acceptance remain pending. Normal mode is 0. Do not
commit a calibration mode selection. Calibration replaces normal obstacle
scanning and does not provide motion stop requests; use a stationary car.
Stopping-distance validation belongs to integration.

## Bench build workflow

Configuration remains provisional until physical measurements support each
replacement. Keep the final configuration at `OBSTACLE_CAL_MODE_OFF`. Check
formatting, the host calibration harness, default/SMP=0/TEST=1 builds and all
four calibration branches before bench use. Building verifies software only.

For each calibration run, select the named mode in `obstacle_config.h`, then
build inside the dev container:

```sh
cd build_make
make WIFI=none -j8
```

Expect the calibration warning. From the repository root on the host, flash
with BOOTSEL and the mounted board (adjust the mount path when necessary):

```sh
cp build_make/mtk3pico_smp1_usb_cdc.uf2 /run/media/$USER/RPI-RP2/
sync
```

This is the WIFI=none filename; the default Wi-Fi image is
`mtk3pico_smp1_usb_cdc_wifi_lwip.uf2`. Start capture before allowing firmware
to run, or reconnect/reset after capture is ready, so the first calibration
record is `CALMETA,START,<mode>,<trim>` (other boot output may precede it).
Do not silently attach to the middle of a run. Restore mode OFF after use,
rebuild, check that the calibration warning is absent and flash the matching
normal image before normal-mode verification. Save mode and metadata
alongside each capture.

## Setup and capture

Mount the sensor and servo in their final positions on the car, power them
from the car power bank, and record room temperature, firmware commit, supply
arrangement, target, distance and mode for each run. Record the flashed UF2
SHA256 and exact configuration/build options alongside the commit: a local
mode or configuration edit may be uncommitted, so the commit alone cannot
identify the firmware. Measure distance from the transducer grilles. Use a
hard flat board square to the beam unless testing inclination. A wall must
cover the whole sweep and beam; a 20 cm board does not cover the complete
sweep at 300 mm. Keep motors stationary.

Equipment: millimetre ruler, protractor/set square, flat targets, scope or
logic analyser, slow-motion camera and serial capture. Save each capture
under a unique name in `testing/obstacle/logs/`; extract only lines starting
`CAL,` for a numeric spreadsheet and retain the full log with metadata.

For example, from the repository root:

```sh
while true; do
    if [ -e /dev/ttyACM0 ]; then cat /dev/ttyACM0; fi
    sleep 0.2
done | tee testing/obstacle/logs/YYYY-MM-DD_cal_ranging.txt
```

```sh
grep '^CAL,' testing/obstacle/logs/YYYY-MM-DD_cal_ranging.txt \
    > testing/obstacle/logs/YYYY-MM-DD_cal_ranging.csv
```

Numeric rows have exactly six fields:

```text
CAL,mode,accepted_trigger_time_ms,logical_angle,result,echo_us
```

Results: 1 VALID, 2 NO_ECHO, 3 FAULT. Echo is zero for non-VALID outcomes;
never treat these zeros as measured distances. Metadata schemas are:

```text
CALMETA,START,mode,trim
CALMETA,ERROR,initialisation_failed
CALMETA,CONTEXT,label,point,margin_ms,repetition
CALMETA,MOVE,move_time_ms,logical_angle,actual_servo_command,trim,requested_wait_ms
CALMETA,PING,accepted_trigger_time_ms,logical_angle,ping_number,move_to_trigger_elapsed_ms
```

CONTEXT labels are `burst` (one-based burst), `point` (angle in mode 2,
zero-based sequence index in mode 4), `reference` (selected target angle)
and `margin` (selected target angle, margin 300..0, repetition 1..10).
Unused context fields are zero. CONTEXT precedes its samples; each CAL
sample is followed by its MOVE and PING metadata. START identifies the build
mode and trim. ERROR records initialisation failure. Mode4 ping numbers are
always 1.

Metadata about a move or ping is emitted after measurement so serial output
does not add to the measured delay. Kernel timestamps have 10 ms resolution;
scheduling, logging and the driver's 60 ms trigger spacing can increase
actual waits. Preserve the full log as well as extracted numeric rows.

## Modes

| Mode | Operation |
|---|---|
| 0 | Normal scanner; calibration disabled |
| 1 | Hold logical 90 degrees; repeated 20-ping bursts with 5 s board-change gaps |
| 2 | Sweep logical 80 through 100 degrees, 1 degree steps, 1 s waits and 20 pings per point |
| 3 | Resting reference at selected 60 or 120 degree target; ten trials per margin 300, 250, ... 0 ms; fully settle at opposite angle before each trial |
| 4 | Repeated holds at 60, 120, 60, 90, 120 degrees, including direct travel in both directions; stationary pings during each hold of at least 10 s |

Mode4 hold timing starts at the command and includes initial settling; a
last attempted ping is allowed to finish, so spacing, transaction, polling
and console latency can overrun 10 s. Mode1 waits 5 s after each burst; the
first burst follows initial settling.

Before START and the first mode-specific move, firmware waits the configured
180-degree travel allowance plus settle margin because the initial physical
angle is unknown and servo initialisation commands raw centre. This is a
configured allowance, not a verified physical bound.

Logical angles exclude trim; actual command is logical angle plus signed
trim. Drivers initialise only from the owning scan task on core 0. Mode
changes require rebuilding and flashing; restore mode 0 afterwards.

## OBS-U12: start delay and open-space response

In mode 1, scope TRIG GP16 and ECHO GP17 for 20 open-space pings and 20 pings
at 300 mm. Record TRIG-falling-to-ECHO-rising delay and ECHO high time for
each sample, missing rises and driver results. The driver's start timeout
origin precedes the 2 us setup and 10 us TRIG pulse: add these 12 us when
converting scope delay to that origin (also account for measurement
uncertainty). Using the maximum delay from the driver's origin, choose
`ceil_to_100_us(max(2 * maximum, maximum + 500 us))` only after reviewing data.

Do not presume the module raises ECHO in open space. Current no-rise result
is FAULT; keep that policy until observations support an explicit decision.
If normal open space sometimes has no rise, propose NO_ECHO instead and
record that an unplugged sensor would also appear clear. A pulse at or above
23000 us is NO_ECHO; shorter pulses are VALID only if otherwise well formed.
They count as clear only when their value meets the configured detection
threshold. Save all 40 measurements and representative scope screenshots.

## OBS-U28: distance fit

Mode 1, board square at 50, 100, 150, 200, 300, 400, 500, 700, 1000 mm.
Record VALID/NO_ECHO/FAULT counts and the median and max-minus-min of VALID
echoes at each point. A burst with fewer than 18 VALID samples out of 20
must be repeated before fitting. Investigate remaining invalid samples;
never fit their zeros. Fit `D = k*t + b` to the nine medians with
spreadsheet SLOPE/INTERCEPT and retain all residuals. Require residuals
within ±max(5 mm, 2% of D), and VALID echo spread at most 35 us through
500 mm.

After measurements, propose `round(k * 10000)` as the integer scale, with
positive nonzero scale, and a signed rounded offset only if |b| exceeds
5 mm. The present scale stays unchanged until then. Threshold conversion is
`ceil((D-b)*10000/scale)`; ensure D>b, check intermediate range and use
signed/wider arithmetic before conversion to unsigned. Apply any offset to
runtime distance conversion too. Review the quantised fit, threshold ordering
and range before updating configuration; do not mix unsigned D with signed b
without controlling arithmetic. Keep conversion macros cast-free because
preprocessor checks use them. A guard comparing unsigned thresholds with a
negative offset is unsafe; first test `#if OBSTACLE_ECHO_OFFSET_MM >= 0`,
then check critical distance exceeds that offset. Verify the numerator fits
its selected integer width, including negative-offset magnitude. Recompute
residuals with the actual quantised scale S/10000 and the rounded offset
that will be deployed (zero if omitted); dropping a small fitted offset can
still break acceptance. Recheck threshold echoes using that actual model.
The software uses one provisional integer scale (1715) and signed offset
zero; the unused float factor is removed. These are the same unmeasured
conversion values as before. The cast-free macro promotes to signed 64-bit
arithmetic; its input domain is 0..400000 mm and strictly above the offset.
Configuration guards validate the scale, offset and threshold domains.
Replace values only after measurements; no measured fit is supplied here.

## OBS-U29: centre and side angles

Use a protractor and chassis centreline as the primary alignment reference.
The wall sweep is a coarse symmetry check: at 300 mm, ideal range changes
only about 0.046 mm for 1 degree, so ping minima cannot establish 1 degree
accuracy. Beam reflections, servo pivot-to-face displacement and wall size
also affect readings. Record actual angles and raw commands.

With `servo_set_angle(logical + trim)`, the new absolute trim is
`raw_command_that_centres_sensor - 90`. If a nonzero trim already exists,
use the logged actual command, not just the logical angle. A raw centre of
93 means trim +3. Recheck mechanical clearance throughout the corrected arc.
Use mode 4 to check 60/90/120 actual angles within ±3 degrees. Compare wall
readings only as a coarse check: the ideal 300/cos(30°) = 346 mm assumes a
fixed measurement origin and narrow beam. Investigate asymmetry before
altering servo pulse endpoints; a side error alone does not prove
nonlinearity. Driver endpoint changes are separate work.

## OBS-U30: travel and settling

Film at the highest available frame rate and measure command-to-settled
time for at least five direct 60 to 120 moves and five reverse moves. Include
command response latency, PWM update delay and uncertainty when relating
video to command metadata; movement-onset-to-stop alone underestimates the
wait. If command timing cannot be identified, record that limitation and do
not certify an upper bound. If using visible movement frame counts, convert
units explicitly: `movement_ms = frames * 1000 / fps`; the two-frame
uncertainty is `2000 / fps` milliseconds. One 50 Hz PWM period is 20 ms, but
this is not a guaranteed command-to-motion response bound: servo
electronics, load and scheduling may add latency. Any estimate using
movement +20 ms +2000/fps must retain that limitation and be checked against
actual command-to-settled behaviour. Round the largest supported bound up to
10 ms for the proposed travel-per-60-degrees value. Check whether smaller
moves also fit the proportional travel model before relying on it.

Then mode 3 uses the measured travel value. Set
`OBSTACLE_CAL_SETTLE_TARGET_DEG` to 60 (default) or 120, rebuild mode 3 and
position a board 300 mm away square to that selected beam. Each run takes a
resting 20-ping reference at the target and 70 trials returning from the
opposite angle, which is fully settled before each trial. Repeat with the
other selector value and reposition the board; choose bounds that pass both
runs. CONTEXT reference/margin point fields identify the selected target.
Use the resting VALID median as the reference.

Each margin has ten controlled moves to the selected target. All ten must be
VALID and within ±30 us of reference. Use the logged actual
command-to-trigger delay; requested delay alone is insufficient. Select the
smallest margin for which it and all tested larger margins pass, add 50 ms
safety and round up to 10 ms. If even the largest margin fails, investigate
target alignment, sensor noise, supply, reference quality and mechanical
motion; failure does not prove travel time alone is too short. Repeat or
extend the test after identifying the cause. Test both directions using the
selectable mode 3 target described above, and representative shorter moves;
normal-mode scanning alone is not a reverse-direction accuracy test. The
configured settle margin must remain at least OBSTACLE_TRANSACTION_MS (the
existing in-flight measurement drain constraint). No measured timing is
supplied here; current values remain provisional.

## OBS-U31: geometry

Measure axle-midpoint-to-front distance a, signed sensor-face setback s
(negative if protruding), and actual width W. The proposed model is
`dcrit = sqrt(a*a + (W/2)^2) - a + s + 20 mm`, with configured critical
`max(80, ceil(dcrit))`. It assumes an in-place turn about the drive axle,
a rectangular front envelope and a frontal obstacle; confirm these
assumptions against the actual turn and mounting. It does not establish a
braking or dynamic collision guarantee. Record a, s, W and the calculation
in the threshold comment; no unused dimension macros are needed. If
changing critical/release values, maintain critical < release < detection;
propose release = critical +20 mm when needed and review detection
compatibility. No dimensions are invented.

## OBS-U32: sensor limits

Mode 1: test square boards at 10,20,30,40,50,60,80 mm; a 300 mm board angled
0,15,30,45,60 degrees; a 20 mm pole, 50 mm box, and cloth/foam at 300 mm.
Record each outcome, echo, counts and VALID percentage. Document NO_ECHO
blind spots as risks because current policy treats NO_ECHO as clear. These
are characterisation results, not configuration fixes.

## Normal-mode verification and evidence

Restore mode 0, rebuild and flash. Test just below and above the configured
detection distance (300 mm currently: 290/310 mm), critical set below its
threshold, persistence within its hysteresis band and release above the
configured release distance after a full centre group (currently 75/95/110
mm). Derive new test distances if thresholds change. Record side-scan timing
against the configured bound, allowing for scheduling and logging resolution.

`CRITICAL set`/`CRITICAL cleared` messages observe flag changes on every
poll independently of result sequence. `side scan started` is emitted only
after centre confirmation, when the first side move is commanded.
`side scan completed after N ms` ends after both side measurements, before
return-to-centre; its wrap-safe elapsed time excludes centre confirmation.
A timeout emits `side scan timed out after N ms`; a state recovery during
scanning emits `side scan aborted for state recovery after N ms`. These are
aborts, not successful completions, and must not be used as passing duration
samples. Other recovery messages also do not establish a side-scan result.
Logging observes current decisions and does not change sequence or bit
layout.

Keep raw text, numeric CSV, spreadsheet, setup photos and scope captures in
logs; index unlisted video links in `videos/README.md`. Add a new Results
row to `TEST_LOG.md` for every run, with commit, temperature, tester and
evidence; never replace existing results. OBS-U20–U27 stay reserved for the
critical/freshness work.

Work sequence: firmware, OBS-U12 start timeout and no-rise policy, OBS-U28
distance fit, OBS-U29 alignment, OBS-U30 timing, OBS-U31 geometry, OBS-U32
limits, then normal-mode verification. Every step after firmware requires
hardware evidence.
