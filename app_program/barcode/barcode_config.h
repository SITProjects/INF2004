/**
 * @file barcode_config.h
 * @brief Pin and tuning of the Code 39 barcode reader.
 *
 * The barcode sensor is a third IR module, mounted over the barcodes that
 * sit 2 cm beside the line. Its DO output flips at each bar edge; the
 * edge interrupt timestamps every flip, and widths are compared within a
 * character, so the speed only has to be steady across one character.
 */

#ifndef BARCODE_CONFIG_H
#define BARCODE_CONFIG_H

/* Public includes. */

/* Public types, constants, and macros. */

/** DO pin (TEAM.md section 2). */
#define BARCODE_PIN 28u

/** DO level over a black bar; as the line modules (line_follow_config.h). */
#define BARCODE_DO_BLACK_LEVEL 1u

/**
 * @brief An edge this soon after the last one, in us, is chatter: the
 * module has no hysteresis, so DO flips for a while as a bar edge crosses
 * its threshold (1 to 3 us apart, and flickers of 6 ms, 2026-11-06). 10 ms
 * is 0.8 mm at the 80 mm/s follow speed; the narrowest real bar, on the
 * smaller barcode, reads 25 ms.
 */
#define BARCODE_MIN_PULSE_US 10000u

/** Edges the interrupt can queue for the task; a power of two. */
#define BARCODE_EDGE_QUEUE_SIZE 64u

/** Elements one scan can hold: 25 characters of 10 elements. */
#define BARCODE_MAX_ELEMENTS 250u

/** Characters of decoded text, including the '*' start and stop. */
#define BARCODE_MAX_TEXT 25u

/**
 * @brief A white gap longer than this, in us, ends a scan (the quiet zone),
 * and so does a longer black, at the bar before it. 1070 ms is 32 mm at the
 * 30 mm/s barcode speed, so change it with MOTION_COMMAND_SLOW_SPEED_MM_S;
 * wide white gaps are 19 mm, 640 ms. It is fixed because a limit based on
 * the narrowest element ended scans early: one flicker at the first bar
 * shrank it below a normal gap (2026-11-06, the first 300 ms of a barcode
 * was lost on both sizes).
 */
#define BARCODE_QUIET_MAX_US 1070000u

/**
 * @brief This many edges within BARCODE_TRIGGER_WINDOW_US mean a barcode is
 * under the sensor at follow speed. In the 2026-11-06 logs 4 in half a second
 * caught 45 of 49 barcodes and 15 of 159 stray marks; a false alarm only
 * costs a short slow stretch.
 */
#define BARCODE_TRIGGER_EDGES 4u
#define BARCODE_TRIGGER_WINDOW_US 500000u

/**
 * @brief After a slow pass, edges this long, in us, cannot start another:
 * 160 mm at follow speed, so what is left of a barcode read early cannot
 * send the car back over it.
 */
#define BARCODE_TRIGGER_HOLDOFF_US 2000000u

/**
 * @brief On a hump the sensor sits too close to the floor to trust, so edges
 * must not slow the car: they cannot trigger until this long, in us, after the
 * hump ends. The hump state can end 0.4 s early at low speed.
 */
#define BARCODE_TRIGGER_HUMP_US 500000u

/**
 * @brief The most, in us, one hump can hold the trigger. The course's hump
 * takes 2.4 s at 80 mm/s. A car calibrated on a slope leans for good and reads
 * as on a hump, and that must not stop every barcode for the run.
 */
#define BARCODE_TRIGGER_HUMP_MAX_US 4000000u

/**
 * @brief A slow pass with no edge for this long, in us, is over: 150 mm at
 * the barcode speed, past the reverse. It was a false alarm.
 */
#define BARCODE_CREEP_QUIET_US 5000000u

/**
 * @brief In each character the narrowest wide bar must be at least this
 * many tenths of the widest narrow bar, and the same for spaces, or the
 * wide/narrow split is unclear.
 */
#define BARCODE_WIDE_RATIO_X10 14u

/**
 * @brief A failed scan prints up to this many element widths, in tenths
 * of a ms, to show why it failed. 0 prints none.
 */
#define BARCODE_TRACE_WIDTHS 64u

/**
 * @brief 1 prints the sensor level and edge counts twice a second, and
 * short scans with their widths. Always on in the IR_CALIBRATION image;
 * set it locally to see them while the car drives past a barcode.
 */
#ifndef BARCODE_EDGE_REPORT
#define BARCODE_EDGE_REPORT IR_CALIBRATION_ONLY
#endif

/** Barcode task period, in ms: one kernel tick. */
#define BARCODE_POLL_MS 10u

#endif /* BARCODE_CONFIG_H */
