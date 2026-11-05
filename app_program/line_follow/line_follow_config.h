/**
 * @file line_follow_config.h
 * @brief Pins and tuning of digital line following.
 *
 * Two IR modules straddle the tape: both read white while the car is
 * centred, and one reads black as the car drifts towards that side. Each
 * module's DO output is a comparator set by its potentiometer, so the
 * level flips at the tape edge (calibrate with IR_CALIBRATION=1).
 */

#ifndef LINE_FOLLOW_CONFIG_H
#define LINE_FOLLOW_CONFIG_H

/* Public includes. */

/* Public types, constants, and macros. */

/** DO pins (TEAM.md section 2). GP26/27 are not ADC inputs any more. */
#define LINE_LEFT_PIN 26u
#define LINE_RIGHT_PIN 27u

/**
 * @brief DO level over black tape. LM393 modules pull DO low while they
 * see their own reflection (white), so black reads 1.
 * TODO(calibrate): confirm with the IR_CALIBRATION=1 image.
 */
#define LINE_DO_BLACK_LEVEL 1u

/** Line task period, in ms: one kernel tick (CNF_TIMER_PERIOD). */
#define LINE_SAMPLE_MS 10u

/** Equal samples in a row before a sensor's level counts. */
#define LINE_CONFIRM_SAMPLES 2u

/**
 * @brief Heading step, in degrees, for each confirmed edge. Motion turns
 * by it while driving, then holds the new heading.
 * TODO(calibrate) on a 2 m straight: small enough not to weave, large
 * enough that one or two steps bring the tape back between the sensors.
 */
#define LINE_STEP_DEG 4

/** Travel, in mm, after a step before the same edge may step again. */
#define LINE_COOLDOWN_MM 40

/** Steps towards one side, without the edge clearing, before a corner. */
#define LINE_STEP_MAX 3u

/**
 * @brief A sensor that reads black for less than this travel, in mm, and
 * then white again crossed the tape: a corner (or overshoot) to that side.
 * A drifting edge stays black for longer while the step takes effect.
 * TODO(calibrate): about 1.5 x the tape width.
 */
#define LINE_CROSS_MM 30

/**
 * @brief Both sensors white for this travel, in mm, counts as lost.
 * Straddling sensors read white while centred, so keep it long.
 */
#define LINE_LOST_MM 600

/** Largest pivot, in degrees, to either side while searching. */
#define LINE_SEARCH_MAX_DEG 120

/** A search pivot this close to its target heading, in degrees, ended. */
#define LINE_SEARCH_DONE_DEG 10

/** A search that does not end within this time, in ms, stops. */
#define LINE_SEARCH_TIMEOUT_MS 8000u

/** Both sensors black for this many samples: a junction or crossing. */
#define LINE_JUNCTION_SAMPLES 3u

#endif /* LINE_FOLLOW_CONFIG_H */
