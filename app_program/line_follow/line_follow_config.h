/**
 * @file line_follow_config.h
 * @brief Pins and tuning of digital line following.
 *
 * Two IR modules straddle the tape: both read white while the car is
 * centred, and one reads black as the car drifts towards that side. Each
 * module's DO output is a comparator set by its potentiometer, so the
 * level flips at the tape edge (calibrate with IR_CALIBRATION=1).
 *
 * Geometry (user, 2026-11-06): the sensors are about a tape width apart,
 * so there is little white margin each side, and about 130 mm ahead of
 * the drive-wheel axle, so every degree the car turns swings them about
 * 2.3 mm sideways: a 7 degree correction moves them a whole tape width.
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
 * @brief While tracking under line control, print one trace line on every
 * raw sensor change and at least this often, in ms: "LT t=target
 * h=heading d=distance LR c=compass m=fieldX,fieldY n=readings" (console,
 * so MQTT builds publish it too; c is -1 without a compass reading). 0
 * turns the trace off.
 */
#define LINE_TRACE_MS 200u

/** Equal samples in a row before a sensor's level counts. */
#define LINE_CONFIRM_SAMPLES 2u

/**
 * @brief While one sensor reads black, the target heading leads the car's
 * heading by this much, in degrees, towards that side, so motion keeps
 * turning until the sensor reads white. With MOTION_FOLLOW_KP 4, 15
 * degrees asks for 60 deg/s.
 * With LINE_SEEK_MAX_DEG and LINE_SEEK_RAMP_MM, chosen on LT19's model
 * fitted to the 2026-11-06 car10 run. Not yet confirmed on the car.
 * TODO(calibrate) on the tightest curve, then a 2 m straight.
 */
#define LINE_LEAD_DEG 15

/**
 * @brief The same sensor touching again within this travel, in mm, means
 * a bend rather than drift, and turns the seek on. A touch of the other
 * sensor turns it off. On a straight touches alternate, or come far apart,
 * so the car holds its heading and barcodes read cleanly: always seeking
 * swung it +-8 degrees (2026-11-06 BAR-U06).
 */
#define LINE_CURVE_TOUCH_MM 80

/**
 * @brief In a bend, while both sensors read white, the target heading
 * leads the car by up to this much, in degrees, towards the last edge,
 * reached after LINE_SEEK_RAMP_MM of white. White on both can mean the
 * tape slipped out, so the car must keep turning.
 */
#define LINE_SEEK_MAX_DEG 12

/**
 * @brief White travel, in mm, over which the seek grows from nothing to
 * LINE_SEEK_MAX_DEG.
 */
#define LINE_SEEK_RAMP_MM 20

/**
 * @brief An edge that stays black for this travel, in mm, while the car
 * turns towards it is a bend too tight to drive round: pivot-search
 * towards it. At the turn cap the car turns about 1 degree per mm.
 */
#define LINE_EDGE_MAX_MM 200

/**
 * @brief The tape crossing both sensors and leaving on one side, with the
 * car turned at least LINE_BEND_TREND_DEG that way over the last
 * LINE_TREND_SLOTS * LINE_TREND_STEP_MM of travel, means the car has run
 * across a bend and the tape is outside on that side: seek towards it. It
 * used to hold its heading and run off: 13 of 16 such crossings on
 * 2026-11-06 left on the left, with a median 203 mm of white after them,
 * and 4 never found the tape. On a straight the car has not been turning,
 * so a crossing there changes nothing (41 on the same logs).
 */
#define LINE_BEND_TREND_DEG 15

/** Travel, in mm, between the headings kept for the turn above. */
#define LINE_TREND_STEP_MM 32

/**
 * @brief In a bend, both sensors white for this travel, in mm, counts as
 * lost: pivot search towards the last edge. The seek has by then turned
 * the car about 120 degrees that way, so a longer run would only circle.
 */
#define LINE_LOST_MM 250

/**
 * @brief The same outside a bend, where white is normal while centred, so
 * it must be long: the car has run 368 mm without a touch (car10).
 */
#define LINE_STRAIGHT_LOST_MM 600

/** Largest pivot, in degrees, to either side while searching. */
#define LINE_SEARCH_MAX_DEG 120

/** A search pivot this close to its target heading, in degrees, ended. */
#define LINE_SEARCH_DONE_DEG 10

/** A search that does not end within this time, in ms, stops. */
#define LINE_SEARCH_TIMEOUT_MS 8000u

/**
 * @brief How far the car backs up for a barcode, in mm. The reader needs
 * up to 40 mm of barcode at follow speed to recognise one, the car coasts on
 * a little, and a short reverse has fallen 15 to 25 mm short (MOT-U09), so
 * this leaves the sensor before the first bar.
 */
#define LINE_BARCODE_REVERSE_MM 100u

/** Line samples to brake for before reversing: about 200 ms on the car. */
#define LINE_BARCODE_BRAKE_SAMPLES 10u

/** Line samples after which a reverse is abandoned: about 3 s on the car. */
#define LINE_BARCODE_BACK_SAMPLES 150u

/**
 * @brief Longest slow pass, in mm: the reverse, the normal barcode's 290 mm,
 * and some margin. A backstop; the reader normally ends it first.
 */
#define LINE_BARCODE_CREEP_MM 450

/**
 * @brief LINE_CURVE_TOUCH_MM while creeping. A slower car touches the tape
 * again sooner, so this scales with speed: 80 mm at 80 mm/s, so 30 at
 * 30 mm/s. Left at 80, the model took a straight for a bend and swung +-9
 * degrees at 50 mm/s (LINE-U04 LT22, 2026-11-06).
 */
#define LINE_BARCODE_TOUCH_MM 30

/** Both sensors black for this many samples confirms a junction. */
#define LINE_JUNCTION_SAMPLES 3u

/**
 * @brief Both sensors turning black within this travel, in mm, of each
 * other is a crossing line: a cross (+) junction. A longer gap is the tape
 * crossing the sensors at an angle, and tracking carries on.
 * TODO(calibrate): the car meets a crossing up to about 18 degrees off
 * square with 30 mm between the sensors.
 */
#define LINE_JUNCTION_SKEW_MM 10

/**
 * @brief Going straight over a junction, hold the heading it was met on for
 * this travel, in mm, so the crossing tape cannot steer the car: on
 * 2026-11-07 it turned the car 19 degrees right and off the line. Only when
 * the car has not been turning: a false junction in a bend must not stop it
 * steering, so LINE_BEND_TREND_DEG of recent turn skips this.
 */
#define LINE_JUNCTION_HOLD_MM 45

/**
 * @brief Travel, in mm, from a junction's first black to the point where
 * the car turns: the sensors' distance ahead of the wheel axle (about 130
 * mm) plus half the tape width, so the car pivots over the crossing. A
 * turning junction drives this far on the heading it met the junction
 * with, ignoring the sensors; straight on keeps tracking.
 */
#define LINE_JUNCTION_CENTRE_MM 140

/** U-turns pivot left (1) or right (0). */
#define LINE_UTURN_LEFT 1

/**
 * @brief Turn at a junction no barcode has given an instruction for, and
 * for a barcode with no action set on the dashboard. Keep
 * LINE_TURN_STRAIGHT.
 */
#define LINE_JUNCTION_DEFAULT_TURN LINE_TURN_STRAIGHT

#endif /* LINE_FOLLOW_CONFIG_H */
