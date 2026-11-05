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

/** Edges the interrupt can queue for the task; a power of two. */
#define BARCODE_EDGE_QUEUE_SIZE 64u

/** Elements one scan can hold: 25 characters of 10 elements. */
#define BARCODE_MAX_ELEMENTS 250u

/** Characters of decoded text, including the '*' start and stop. */
#define BARCODE_MAX_TEXT 25u

/**
 * @brief A white gap this many times the narrowest element so far ends a
 * scan (the quiet zone). Code 39 elements are at most about 3 narrow wide.
 */
#define BARCODE_QUIET_FACTOR 6u

/**
 * @brief Longest white gap, or black bar, inside a scan, in us; also the
 * quiet gap before the narrowest element is known. 400 ms is 48 mm at
 * 120 mm/s.
 */
#define BARCODE_QUIET_MAX_US 400000u

/**
 * @brief In each character the 3rd-widest element must be at least this
 * many tenths of the 4th-widest, or the wide/narrow split is unclear.
 */
#define BARCODE_WIDE_RATIO_X10 14u

/** Barcode task period, in ms: one kernel tick. */
#define BARCODE_POLL_MS 10u

#endif /* BARCODE_CONFIG_H */
