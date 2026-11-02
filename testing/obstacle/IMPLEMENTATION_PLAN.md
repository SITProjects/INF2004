# Obstacle test implementation and evidence plan

Updated 2026-10-31. Baseline: verified branch `47`, commit
`06181b16fe6cc23a3d18e0d78f641be2a354fd84`. A local `main` reference is not
an authoritative baseline. No hardware results or recovered October 28 logs
are available. OBS-U05–U09/U11 remain reserved; calibration U28–U32 stays
under its existing plan. This work does not change shared files or kernel code.

## A — Bounded scan-task sleep check

Replace U01 initial-task sampling/priority manipulation with a bounded worker
pinned to the scan task's core, lower priority than `OBSTACLE_PRI_SCAN`. The
initial task can migrate. Check every create/start/wait/delete result, save a
failure reason, and clean up on timeout. Do not assume priority orders tasks
on different cores. No external messages to teammates are authorized.

## B — Predicate regression cases

Implement U20–U22 after U19 using configured echo macros, strict equality
boundaries, invalid statuses and uint32 wrap. Test descriptions must describe
configured boundaries rather than fixed 80/100 mm assumptions.

## C — Synchronized APP_TEST support and watchdog cases

A scanner-owned request mailbox and locked snapshot replace unsynchronized
volatile cross-task access. Only the scan task mutates scan state. Snapshot
fields include hold acknowledgement, stop/state, specific stale/side-timeout
event counters and timestamps, UNKNOWN publication, and critical timing.
Test-only object use: one snapshot/mailbox mutex plus a transient U01 worker
task and completion event flag; confirm cleanup and available object slots.
U23/U24 await hold acknowledgement and assert the specific watchdog event,
UNKNOWN, stop, then fresh recovery; unrelated stop requests cannot pass them.
Use configured waits with bounded margins and always release holds. Await the trigger-hold/drain acknowledgement during U24; there is no
motion-stop ACK handshake implemented here. No scan-state helper may be read directly
from the migrating initial task. APP_TEST hooks compile out in normal builds.

## D — Software validation, then on-target automatic evidence

Complete A→B→C before D, then manual tests. Validate normal, SMP=0, TEST=1,
and TEST=1 SMP=0 profiles in an isolated copy; check formatting and diff
hygiene. Host checks with explicit fault injection are software evidence only.
Record commands, exit codes and saved logs, and identify staged changes by
baseline plus patch/source hashes; an uncommitted build must not claim to be
an exact committed firmware result. Commit only when separately authorized.

For actual evidence, first commit reviewed source, build and flash that exact
commit. Flashing disconnects/re-enumerates USB: flash first, wait for the
correct console port, open capture, then reset without BOOTSEL and confirm
`[TEST]` start through test-run-end markers. If reset disconnects USB, use a
reconnecting capture and rerun when start markers are missing. Do not capture
an old image or silently lose boot results. Record date, exact commit, tester,
PASS/FAIL and saved log per ID; unavailable cases are NOT RUN with reason.
All on-target automatic results currently remain NOT RUN.

## E — Manual bench evidence

Run U15→U12→U03→U04→U14→U13→U25→U26→U27 with a stationary car,
prepared board, scope and safe 3.3 V stimulus. U15 must establish pulse width,
nominal clear-path cadence, servo waveform and actual GPIO ownership/pinmux;
quiet encoder pins alone are insufficient. U12 may reuse calibration evidence
only when source/configuration, stimuli, outcomes and scope observations are
equivalent and traceable; physical samples remain pending. U04 uses a 150 mm
flat board rather than a hand at 30 mm. U13 records unsupported malformed-edge
subcases as NOT RUN and verifies recovery after reconnecting ECHO.

U25 needs three distinct held stages below critical, inside hysteresis band,
and above release, each observed through the appropriate centre group. Wait
through side scanning/servo return; a two-second hold is not a centre-group
guarantee. U26 distinguishes first-FAULT stop from group-exhaustion UNKNOWN;
confirm disconnected input level/bias, and do not invent a physical-unplug
timestamp or fixed three-fault limit when retry configuration differs.

U27 uses the confirmed software-response method. APP_TEST microsecond
instrumentation pairs centre reading consumption with critical flag setting.
The expectation is the same scanner update; save actual delta and timer
resolution, without asserting a physical latency bound.
Report consumed-reading→critical-set, trigger→event and physical arrival→event
as separate quantities; the latter two require their own matched timestamps.
Adjacent log timestamps cannot measure this. The roughly 95 ms expression
(`OBSTACLE_FORWARD_PERIOD_MS + OBSTACLE_TRANSACTION_MS + OBSTACLE_TICK_MS`)
is a healthy forward freshness budget, not a critical-response requirement.
Do not invent a pass threshold without a specified contract.

Save local logs/photos; no upload, video-index edit or external message is
authorized here. Add actual Results rows only after runs. Build success is
never hardware PASS. Review all diffs and preserve existing Results history.

## Bench command and evidence checklist

After the reviewed source is committed, save `git rev-parse HEAD` and build:

```sh
cd build_make
make -j8
make SMP=0 -j8
make TEST=1 -j8
make TEST=1 SMP=0 -j8
```

For the normal dual-core target, flash
`build_make/mtk3pico_smp1_usb_cdc_wifi_lwip.uf2`; the dual-core automatic image
is `build_make/mtk3pico_smp1_usb_cdc_wifi_lwip_test.uf2`. For an explicitly selected dual-core test artifact use
`make TEST=1 SMP=1 -j8`; make options otherwise use their defaults on each
invocation (they are not sticky).
After flashing/reconnecting, capture then reset:

```sh
cat /dev/ttyACM0 | tee testing/obstacle/logs/YYYY-MM-DD_unit.txt
```

Verify complete start/end markers and twelve obstacle checks:
U01, U02, U10, U16–U24. Expect 12 passed/0 failed only after the actual run;
this document records no such run. Bench prompts require a clear 400 mm path
for U23 and a 150 mm board for U24, removed when prompted.

For U27 use TEST=1 after the complete automatic run, then perform five
critical-set cycles. Each cycle starts with a completed above-release centre
group so critical has cleared; move below critical and hold until a new event.
Save `critical_count`, `critical_echo_us`, `critical_receipt_us`,
`critical_flag_us` and unsigned `critical_flag_us - critical_receipt_us` from
snapshot/log instrumentation, paired per event. Timer unit/resolution is 1 us;
record same-update processing and actual values without creating a numeric
physical-response deadline. Include source/config provenance and log.

Append actual evidence using this template (placeholders are not results):

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| YYYY-MM-DD | OBS-U23 | `<flashed commit>` | `<tester>` | PASS / FAIL / NOT RUN | `logs/<saved file>` | specific event, timestamp, recovery; reason if NOT RUN |

U26 and U27 use TEST=1 after the entire automatic run ends and all trigger
holds have been released. U26 records the APP_TEST centre FAULT timestamp,
retry-before-increment and attempt diagnostic with the UNKNOWN publication;
it is a software-consumption timestamp. U25 observes completed centre decisions
through the subsequent `side scan started` event while below detection, or
CLEAR publication above detection, alongside critical set/clear events.
