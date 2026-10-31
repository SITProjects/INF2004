#!/usr/bin/env python3
"""Compile actual calibration/config sources against deterministic host doubles.

Usage: python3 testing/obstacle/host/test_calibration.py [repository-root]
No target hardware or repository mutations. Requires host C compiler and nm.
"""
import os
import pathlib
import re
import shutil
import subprocess
import sys
import tempfile

ROOT = pathlib.Path(sys.argv[1]).resolve() if len(sys.argv) > 1 else pathlib.Path(__file__).resolve().parents[3]
HERE = pathlib.Path(__file__).resolve().parent

def run(*args):
    result = subprocess.run(args, text=True, capture_output=True,
                            env=dict(os.environ, CCACHE_DISABLE='1'))
    if result.returncode:
        raise RuntimeError(result.stderr or result.stdout)
    return result.stdout

def check(mode, trim, target=60):
    with tempfile.TemporaryDirectory(prefix='obstacle-cal-host-') as directory:
        base = pathlib.Path(directory)
        obstacle = base / 'obstacle'
        obstacle.mkdir()
        for name in ('obstacle_calibration.c', 'obstacle_calibration.h', 'obstacle_config.h', 'drv_servo.h', 'drv_ultrasonic.h'):
            shutil.copy(ROOT / 'app_program/obstacle' / name, obstacle)
        shutil.copy(ROOT / 'app_program/robot_types.h', base)
        config = obstacle / 'obstacle_config.h'
        source = config.read_text()
        for name, value in [('OBSTACLE_CALIBRATION_MODE', mode), ('OBSTACLE_SERVO_TRIM_DEG', trim), ('OBSTACLE_SERVO_TRAVEL_MS_PER_60_DEG', '123u'), ('OBSTACLE_CAL_SETTLE_TARGET_DEG', target)]:
            source, n = re.subn(r'(#define ' + name + r')\s+[^\n]+', rf'\g<1> {value}', source)
            assert n == 1, (name, n)
        config.write_text(source)
        (base / 'config.h').write_text('#define CNF_TIMER_PERIOD 10\n')
        (base / 'app.h').write_text('''#include <stdint.h>
typedef unsigned char UB;
typedef uint32_t RELTIM;
typedef struct { uint32_t lo; } SYSTIM;
#define TMO_FEVR (-1)
int tk_get_otm(SYSTIM *);
int tk_dly_tsk(RELTIM);
int tk_slp_tsk(int);
void tk_ext_tsk(void);
int tm_printf(UB *, ...);
''')
        flags = ['cc', *(os.environ.get('CAL_HOST_CFLAGS', '').split() if mode else []), '-std=c11', '-Wall', '-Wextra', '-Werror', '-Wno-error=cpp', '-I' + str(base)]
        obj = base / 'cal.o'
        run(*flags, '-c', str(obstacle / 'obstacle_calibration.c'), '-o', str(obj))
        if mode == 0:
            assert not run('nm', str(obj)).strip(), 'mode 0 retains symbols'
            return
        executable = base / 'calibration'
        run(*flags, str(HERE / 'calibration_harness.c'), str(obj), '-o', str(executable))
        for failure in (1, 2):
            out = run(str(executable), '1', str(failure))
            assert 'CALMETA,ERROR,initialisation_failed' in out and 'EXIT' in out
            assert 'TRIGGER,' not in out and 'SERVO,' not in out
        limits = {1: 41, 2: 421, 3: 91, 4: 850}
        lines = [line.split(',') for line in run(str(executable), str(limits[mode]), '0').splitlines()]
        rows = [list(map(int, row[1:])) for row in lines if row[0] == 'CAL']
        assert len(rows) == limits[mode]
        triggers = [int(row[1]) for row in lines if row[0] == 'TRIGGER']
        assert triggers == [row[1] for row in rows]
        assert all(len(row) == 5 for row in rows)
        for i, (actual_mode, time, angle, result, echo) in enumerate(rows):
            assert actual_mode == mode and time >= 0
            assert result == 1 + i % 3 and echo == (1749 if result == 1 else 0)
        moves = [list(map(int, row[2:])) for row in lines if row[:2] == ['CALMETA', 'MOVE']]
        pings = [list(map(int, row[2:])) for row in lines if row[:2] == ['CALMETA', 'PING']]
        assert len(moves) == len(pings) == len(rows)
        for row, move, ping in zip(rows, moves, pings):
            assert move[2] == move[1] + trim and move[3] == trim
            assert ping[0] == row[1] and ping[1] == row[2]
            assert ping[3] == row[1] - move[0] and ping[3] >= move[4]
        servo = [list(map(int, row[1:])) for row in lines if row[0] == 'SERVO']
        contexts = [row[2:] for row in lines if row[:2] == ['CALMETA', 'CONTEXT']]
        if mode == 1:
            assert {row[2] for row in rows} == {90}
            assert rows[20][1] - rows[19][1] >= 5000
            assert rows[40][1] - rows[39][1] >= 5000
            assert [row[1] for row in contexts] == ['1', '2', '3']
        elif mode == 2:
            assert [row[2] for row in rows[:420]] == [a for a in range(80, 101) for _ in range(20)]
            assert all(moves[i * 20][4] == 1000 for i in range(21))
        elif mode == 3:
            assert [row[2] for row in rows[:90]] == [target] * 90
            trials = [row for row in contexts if row[0] == 'margin'][:70]
            assert [(int(row[2]), int(row[3])) for row in trials] == [(m, r) for m in range(300, -1, -50) for r in range(1, 11)]
            assert all(moves[20 + i][4] == 123 + int(trials[i][2]) for i in range(70))
            for i in range(70):
                away, back = servo[1 + 2*i:3 + 2*i]
                assert [away[1], back[1]] == [180 - target + trim, target + trim]
                assert back[0] - away[0] >= 1123
        elif mode == 4:
            assert [row[1] - trim for row in servo[:5]] == [60, 120, 60, 90, 120]
            assert all(b[0] - a[0] >= 10000 for a, b in zip(servo, servo[1:]))
            assert {row[2] for row in rows} == {60, 90, 120}

for mode in range(5):
    for trim in ((0,) if mode == 0 else (-3, 4)):
        check(mode, trim)
        print(f'PASS mode={mode} trim={trim}')

for trim in (-3, 4):
    check(3, trim, target=120)
    print(f'PASS mode=3 trim={trim} target=120')

# Configuration conversions are used by #if: check unsigned call sites and
# signed offsets through the preprocessor, without duplicating C arithmetic.
with tempfile.TemporaryDirectory(prefix='obstacle-cal-conversion-') as directory:
    base = pathlib.Path(directory)
    obstacle = base / 'obstacle'
    obstacle.mkdir()
    shutil.copy(ROOT / 'app_program/robot_types.h', base)
    (base / 'config.h').write_text('#define CNF_TIMER_PERIOD 10\n')
    source = (ROOT / 'app_program/obstacle/obstacle_config.h').read_text()
    for scale, offset in ((1715, 0), (1715, -6), (1715, 6), (1702, -6)):
        variant = source
        for name, value in [('OBSTACLE_SOUND_MM_PER_10K_US', f'{scale}u'), ('OBSTACLE_ECHO_OFFSET_MM', offset)]:
            variant, count = re.subn(r'(#define ' + name + r')\s+[^\n]+', rf'\g<1> {value}', variant)
            assert count == 1, name
        (obstacle / 'obstacle_config.h').write_text(variant)
        assertions = []
        for distance in (80, 100, 300, 1000):
            expected = ((distance - offset) * 10000 + scale - 1) // scale
            assertions.append(f'#if OBSTACLE_MM_TO_ECHO_US({distance}u) != {expected}\n#error bad_conversion\n#endif\n')
        (base / 'check.c').write_text('#include "obstacle/obstacle_config.h"\n' + ''.join(assertions))
        run('cc', '-E', '-I' + str(base), str(base / 'check.c'))
        print(f'PASS conversion scale={scale} offset={offset}')
    for name, value in [('OBSTACLE_SOUND_MM_PER_10K_US', '0u'), ('OBSTACLE_SOUND_MM_PER_10K_US', '-1'), ('OBSTACLE_SOUND_MM_PER_10K_US', '4294967296LL'), ('OBSTACLE_ECHO_OFFSET_MM', -400001), ('OBSTACLE_ECHO_OFFSET_MM', 400001), ('OBSTACLE_DETECT_DISTANCE_MM', '400001u'), ('OBSTACLE_ECHO_OFFSET_MM', 80), ('OBSTACLE_CAL_SETTLE_TARGET_DEG', 90)]:
        variant, count = re.subn(r'(#define ' + name + r')\s+[^\n]+', rf'\g<1> {value}', source)
        assert count == 1, name
        (obstacle / 'obstacle_config.h').write_text(variant)
        (base / 'check.c').write_text('#include "obstacle/obstacle_config.h"\n')
        rejected = subprocess.run(['cc', '-E', '-I' + str(base), str(base / 'check.c')], text=True, capture_output=True, env=dict(os.environ, CCACHE_DISABLE='1'))
        assert rejected.returncode != 0 and '#error' in rejected.stderr, name
        print(f'PASS invalid configuration rejected: {name}={value}')
