#!/usr/bin/env python3
"""Fit the obstacle edge correction and range scale from OBS-U71 CALWIDE logs.

Usage: width_fit.py <log> --width 210 --stations 150 200 250
           [--ignore 250:55-60] [--radial 14]

--ignore drops readings of something that is not the obstacle (a wire, say) at
one station; --radial tries a value of OBSTACLE_SENSOR_RADIAL_OFFSET_MM.

Replays each pass through the car's rules (obstacle_width.c, obstacle_scan.c):
the 10 degree grid from 50 to 130, the flat-face hit limit, edges looked for
from the outside in, half a step out, then moved in by the correction. Keep
the constants below in step with obstacle_config.h. A pass may straddle a move
of the obstacle, so the first and last pass at each station are dropped.
"""
import argparse
import math
import re
import statistics
import sys

SCALE_MM_PER_US = 0.1700  # OBSTACLE_SOUND_MM_PER_10K_US / 10000
ECHO_OFFSET_MM = -18.0  # OBSTACLE_ECHO_OFFSET_MM
RADIAL_OFFSET_MM = 0.0  # OBSTACLE_SENSOR_RADIAL_OFFSET_MM
EDGE_JUMP_US = 466  # OBSTACLE_EDGE_JUMP_US
END_BEHIND_US = 100  # OBSTACLE_END_BEHIND_US
GRID_FIRST, GRID_LAST, GRID_STEP, CENTRE = 50, 130, 10, 90
MIN_VALID_PINGS = 3
STATION_TOLERANCE_MM = 40.0
# Negative: the echoes stop inside the true edge. The car accepts 0 to 15 only
# (OBSTACLE_EDGE_CORRECTION_MAX_DEG), so a negative best needs a code change.
CORRECTIONS = range(-10, 16)


def range_mm(echo_us):
    return echo_us * SCALE_MM_PER_US + ECHO_OFFSET_MM + RADIAL_OFFSET_MM


def read_passes(path):
    """Return {pass: {angle: (valid, nearest_us, farthest_us)}}, in log order."""
    passes = {}
    line_re = re.compile(r"CALWIDE,(\d+),(\d+),-?\d+,(\d+),(\d+),(\d+)")
    run = 0
    last_pass = 0
    with open(path, errors="replace") as log:
        for line in log:
            match = line_re.search(line)
            if match is None:
                continue
            number, angle, valid, nearest, farthest = map(int, match.groups())
            if number < last_pass:
                run += 1  # The car restarted: pass numbers begin again.
            last_pass = number
            passes.setdefault((run, number), {})[angle] = (
                valid,
                nearest,
                farthest,
            )
    return passes


def centre_echo(one_pass):
    reading = one_pass.get(CENTRE)
    if reading is None or reading[0] < MIN_VALID_PINGS:
        return None
    return reading[1]


def is_hit(one_pass, angle, centre_us):
    reading = one_pass.get(angle)
    if reading is None or reading[0] < MIN_VALID_PINGS:
        return False
    off = math.radians(abs(angle - CENTRE))
    return reading[1] < centre_us / math.cos(off) + EDGE_JUMP_US


def end_is_edge(one_pass, centre_us, end, other):
    """As obstacle_width_end_is_edge(): False for an end behind the face."""
    off = math.radians(abs(end - CENTRE))
    behind_us = centre_us / math.cos(off) + END_BEHIND_US
    return is_hit(one_pass, other, centre_us) or one_pass[end][1] < behind_us


def edge(one_pass, centre_us, side):
    """Return (hit angle, edge found) for side -1 (left) or +1 (right)."""
    end = GRID_FIRST if side < 0 else GRID_LAST
    other = GRID_LAST if side < 0 else GRID_FIRST
    for angle in range(end, CENTRE, -side * GRID_STEP):
        if is_hit(one_pass, angle, centre_us) and (
            angle != end or end_is_edge(one_pass, centre_us, end, other)
        ):
            return angle, angle != end
    return CENTRE, True


def width_mm(one_pass, centre_us, correction_deg):
    """Return (width, left found, right found), as obstacle_width_estimate."""
    left, left_found = edge(one_pass, centre_us, -1)
    right, right_found = edge(one_pass, centre_us, +1)
    left_range = range_mm(one_pass[left][1] if left != CENTRE else centre_us)
    right_range = range_mm(one_pass[right][1] if right != CENTRE else centre_us)
    left_deg = left - GRID_STEP / 2 if left_found else float(left)
    right_deg = right + GRID_STEP / 2 if right_found else float(right)
    bare_left, bare_right = left_deg, right_deg
    if left_found:
        left_deg += correction_deg
    if right_found:
        right_deg -= correction_deg
    if left_deg > right_deg:
        left_deg = right_deg = (bare_left + bare_right) / 2
    dx = left_range * math.cos(math.radians(left_deg)) - right_range * math.cos(
        math.radians(right_deg)
    )
    dy = left_range * math.sin(math.radians(left_deg)) - right_range * math.sin(
        math.radians(right_deg)
    )
    return math.hypot(dx, dy), left_found, right_found


def group_stations(passes, stations):
    """Assign passes to stations by centre range; drop those around a move."""
    runs = []
    for key in sorted(passes):
        centre_us = centre_echo(passes[key])
        station = None
        if centre_us is not None:
            nearest = min(stations, key=lambda d: abs(d - range_mm(centre_us)))
            if abs(nearest - range_mm(centre_us)) <= STATION_TOLERANCE_MM:
                station = nearest
        if runs and runs[-1][0] == station:
            runs[-1][1].append(key)
        else:
            runs.append((station, [key]))
    grouped = {d: [] for d in stations}
    empty = []
    for station, keys in runs:
        if station is None:
            empty.extend(keys)
        else:
            grouped[station].extend(keys[1:-1] if len(keys) >= 3 else keys)
    return grouped, empty


def main():
    global RADIAL_OFFSET_MM
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("log")
    parser.add_argument("--width", type=float, required=True, help="true, mm")
    parser.add_argument(
        "--stations", type=float, nargs="+", required=True, help="true d, mm"
    )
    parser.add_argument(
        "--ignore", nargs="*", default=[], help="station:first-last, degrees"
    )
    parser.add_argument("--radial", type=float, default=RADIAL_OFFSET_MM)
    args = parser.parse_args()
    RADIAL_OFFSET_MM = args.radial

    passes = read_passes(args.log)
    if not passes:
        sys.exit("no CALWIDE lines in " + args.log)
    grouped, empty = group_stations(passes, args.stations)
    for item in args.ignore:
        station, span = item.split(":")
        first, last = map(int, span.split("-"))
        for key in grouped[float(station)]:
            for angle in passes[key]:
                if first <= angle <= last:
                    passes[key][angle] = (0, 0, 0)
    print(f"{len(passes)} passes, {len(empty)} with nothing at the station "
          "ranges")

    near = {}
    for key in empty:
        for angle, (valid, nearest, _) in passes[key].items():
            if valid >= MIN_VALID_PINGS and nearest < 1500:
                near[angle] = min(near.get(angle, nearest), nearest)
    if near:
        print("echoes under 1500 us with no station in front (car body?):",
              ", ".join(f"{a} deg {near[a]} us" for a in sorted(near)))

    fit_points = []
    results = {}
    fitted = []  # Stations with both edges: a clipped one ignores the value.
    for station in args.stations:
        keys = grouped[station]
        if not keys:
            print(f"\n{station:.0f} mm: no usable passes")
            continue
        centres = [centre_echo(passes[k]) for k in keys]
        echo = statistics.median(centres)
        fit_points.append((echo, station))
        print(f"\n{station:.0f} mm: {len(keys)} passes, centre {echo:.0f} us, "
              f"reads {range_mm(echo):.0f} mm "
              f"({range_mm(echo) - station:+.0f} mm)")
        half = math.degrees(math.atan(args.width / 2 / station))
        print(f"  true edges at {CENTRE - half:.0f} and {CENTRE + half:.0f} "
              "deg")
        one = passes[keys[len(keys) // 2]]
        print("  echoes:", " ".join(
            f"{a}:{one[a][1] if one[a][0] >= MIN_VALID_PINGS else '-'}"
            for a in sorted(one)))
        results[station] = [
            [width_mm(passes[k], c_us, c) for k, c_us in zip(keys, centres)]
            for c in CORRECTIONS
        ]
        found = results[station][0]
        both = sum(1 for _, left, right in found if left and right)
        print(f"  both edges found in {both} of {len(found)} passes")
        if 2 * both > len(found):
            fitted.append(station)

    if not results:
        sys.exit("no station had usable passes")

    print("\ncorrection: median width per station (error), worst error")
    print("  fitted on", ", ".join(f"{d:.0f}" for d in fitted) or "nothing")
    best = None
    for index, correction in enumerate(CORRECTIONS):
        cells = []
        worst = 0.0
        for station, per_c in results.items():
            median = statistics.median(w for w, _, _ in per_c[index])
            error = median - args.width
            if station in fitted:
                worst = max(worst, abs(error))
            cells.append(f"{station:.0f}: {median:3.0f} ({error:+4.0f})")
        print(f"  {correction:2d} deg  " + "  ".join(cells) +
              f"  worst {worst:.0f}")
        if fitted and (best is None or worst < best[1]):
            best = (correction, worst)
    if best is None:
        sys.exit("no station found both edges in most passes: no fit")
    print(f"best correction {best[0]} deg, worst error {best[1]:.0f} mm "
          f"({100 * best[1] / args.width:.0f}% of the width)")

    if len(fit_points) >= 2:
        count = len(fit_points)
        mean_t = sum(t for t, _ in fit_points) / count
        mean_d = sum(d for _, d in fit_points) / count
        slope = sum((t - mean_t) * (d - mean_d) for t, d in fit_points) / sum(
            (t - mean_t) ** 2 for t, _ in fit_points
        )
        print(f"range fit: D = {slope:.4f} t {mean_d - slope * mean_t:+.1f} mm "
              f"(config {SCALE_MM_PER_US:.4f} t {ECHO_OFFSET_MM:+.1f})")


if __name__ == "__main__":
    main()
