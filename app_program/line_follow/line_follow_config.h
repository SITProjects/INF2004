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

/**
 * @brief While tracking under line control, print one trace line this
 * often, in ms: "LT t=target h=heading d=distance LR" (console, so MQTT
 * builds publish it too). 0 turns the trace off.
 */
#define LINE_TRACE_MS 200u

/** Equal samples in a row before a sensor's level counts. */
#define LINE_CONFIRM_SAMPLES 2u

/**
 * @brief While one sensor reads black, the target heading leads the car's
 * heading by this much, in degrees, towards that side, so motion keeps
 * turning (at up to MOTION_FOLLOW_MAX_DEG_S) until the sensor reads white;
 * then the car holds the heading it reached. With MOTION_FOLLOW_KP 4,
 * 25 degrees asks for 100 deg/s, so the turn runs at the cap.
 * TODO(calibrate) on the tightest curve, then a 2 m straight: lower it if
 * the car weaves on straights, raise the cap if it runs wide on curves.
 */
#define LINE_LEAD_DEG 25

/**
 * @brief Travel, in mm, over which the tracker averages how fast the
 * heading turns (degrees per mm, corrections included). While no single
 * sensor reads black the car keeps turning at that rate, so it follows a
 * bend between corrections instead of driving straight off it; on a
 * straight the average is about zero. Shorter adapts faster to a new bend
 * but weaves more after each correction.
 * TODO(calibrate) on the curve and a 2 m straight.
 */
#define LINE_CURVE_WINDOW_MM 100

/**
 * @brief Lead, in mm of travel, of the learned bend: the target heading is
 * the heading plus the average rate times this. The car then turns at
 * about MOTION_FOLLOW_KP x rate x this deg/s; MOTION_FOLLOW_SPEED_MM_S /
 * MOTION_FOLLOW_KP (120 / 4 = 30 mm) makes that the bend's own rate.
 */
#define LINE_CURVE_LEAD_MM 30

/**
 * @brief An edge that stays black for this travel, in mm, while the car
 * turns towards it is a bend too tight to drive round: pivot-search
 * towards it. At the turn cap the car turns about 1 degree per mm.
 */
#define LINE_EDGE_MAX_MM 200

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
