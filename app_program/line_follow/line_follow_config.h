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

/**
 * @brief In a bend, the other sensor touching while the car has turned
 * LINE_BEND_TREND_DEG the bend's way is the car a little ahead of the bend,
 * not a reason to leave it: for this travel, in mm, it steers back only
 * LINE_COUNTER_LEAD_DEG and the bend stays on. Steering back at full lead
 * jerked it off left bends: 15 to 28 degrees against the bend per touch on
 * 2026-11-07, with the right sensor slow to let go. The turn is judged as the
 * touch begins, because steering back soon takes it below the limit. The
 * travel covers a touch at the end of a U, 95 mm on 2026-11-07: at full lead
 * from 40 mm the car swung 16 degrees and the tape went out on the far side,
 * twice in two. Black for longer is a real turn the other way, so it steers as
 * usual. In the car model, stronger passes more S bends with healthy sensors,
 * but from 7 degrees a U fails with a sensor as slow as that one was (LT44).
 */
#define LINE_COUNTER_MM 100
#define LINE_COUNTER_LEAD_DEG 6

/**
 * @brief Such a touch that clears after at least this travel, in mm, was the
 * straight after the bend, so the bend ends and the car holds its heading.
 * Seeking back the bend's way would turn it further off: a 52 mm touch at the
 * end of a right turn left the bend on with this at 60, and the car spun past
 * the straight on white (2026-11-07). A touch in the middle of a U read 37 mm
 * on the same run, so this has little room to go lower. A shorter touch
 * leaves the bend on.
 */
#define LINE_COUNTER_END_MM 40

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
 * @brief How far the car backs up for a barcode, in mm. The reverse falls
 * about 26 mm short at 120 mm/s, counting the coasting before it, so 100
 * ended 74 mm back from the braking point. Most passes brake within 14 mm of
 * the first bar, but a late trigger braked about 125 mm in, and the creep
 * then began inside the barcode and read 23 of its 29 elements (2026-11-07).
 * 170 ends about 145 mm back.
 */
#define LINE_BARCODE_REVERSE_MM 170u

/** Line samples to brake for before reversing: about 200 ms on the car. */
#define LINE_BARCODE_BRAKE_SAMPLES 10u

/** Line samples after which a reverse is abandoned: about 3 s on the car. */
#define LINE_BARCODE_BACK_SAMPLES 150u

/**
 * @brief Longest slow pass, in mm: about 145 mm back to the barcode, the
 * normal barcode's 290 mm, and some margin. A backstop; the reader normally
 * ends it first.
 */
#define LINE_BARCODE_CREEP_MM 520

/**
 * @brief On a hump the car follows at the barcode speed, to spare the sensors
 * close to the floor there, for at most this many samples: 8 s, where the
 * hump takes about 6 s at that speed. A car calibrated on a slope leans for
 * good and reads as on a hump, and must not crawl for the whole run.
 */
#define LINE_HUMP_SLOW_SAMPLES 400u

/**
 * @brief LINE_CURVE_TOUCH_MM while creeping. A slower car touches the tape
 * again sooner, so this scales with speed: 80 mm at 80 mm/s, so 30 at
 * 30 mm/s. Left at 80, the model took a straight for a bend and swung +-9
 * degrees at 50 mm/s (LINE-U04 LT22, 2026-11-06).
 */
#define LINE_BARCODE_TOUCH_MM 30

/**
 * @brief Both sensors black for this many samples confirms a junction. Each
 * sensor's level is confirmed already, so one will do: at 120 mm/s a skewed
 * tape left a single sample of overlap on 2026-11-07.
 */
#define LINE_JUNCTION_SAMPLES 1u

/**
 * @brief Both sensors turning black within this travel, in mm, of each
 * other is a crossing line: a cross (+) junction. A longer gap is the tape
 * crossing the sensors at an angle, and tracking carries on. A sample is
 * 2.4 mm at 120 mm/s: at 10 a crossing read 11 and was missed (2026-11-07).
 * Replaying the logs, 12 and 14 add the same four crossings and nothing else;
 * 16 fires in a bend.
 */
#define LINE_JUNCTION_SKEW_MM 14

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
 * the car starts to stop for its turn: the sensors' distance ahead of the
 * wheel axle (about 130 mm) plus half the tape width, less the 10 mm it runs
 * on braking, so the car pivots over the crossing. At 140 it stopped about
 * 11 mm past the centre at 80 and 120 mm/s alike, which leaves the new line
 * off to one side: on 2026-11-07 the car missed it after a right turn and ran
 * parallel to it, through the curve beyond. A turning junction drives this
 * far on the heading it met the junction with, ignoring the sensors;
 * straight on keeps tracking.
 */
#define LINE_JUNCTION_CENTRE_MM 130

/**
 * @brief A left pivot starts with the sensors over the line carrying on past
 * the junction, so the right sensor leaving that tape shows where the line
 * runs: this many degrees behind that heading. Three left turns on 2026-11-06
 * and 07 that lost the line stopped 8 to 10 degrees short by the next crossing
 * tape, and this read the same to within about 2. It is half the sensors'
 * spacing and half the tape seen 130 mm from the axle, 11 degrees, plus the
 * debounce and lag at 90 degrees/s.
 *
 * The car's heading is no guide: it stopped up to 11 degrees right of the
 * line. Nor is the gap between the sensors on the way in: a sensor already on
 * the line reads as skew, and took 19 degrees off one left turn. Right pivots
 * are left at 90: they work, as a short one drifts onto the tape, and nothing
 * independent shows this reading is right for them.
 */
#define LINE_OLD_LINE_DEG 14

/**
 * @brief The sensor leaving the tape counts from this far into the pivot, in
 * degrees: before that the car is still rolling forward as it brakes.
 */
#define LINE_OLD_LINE_MIN_DEG 5

/**
 * @brief ...and up to this far. Logged sweeps ended by 24; later, the sensor
 * can only be over something else: at 97 it was the crossing tape.
 */
#define LINE_OLD_LINE_MAX_DEG 35

/** U-turns pivot left (1) or right (0). */
#define LINE_UTURN_LEFT 1

/**
 * @brief Turn at a junction no barcode has given an instruction for, and
 * for a barcode with no action set on the dashboard. Keep
 * LINE_TURN_STRAIGHT.
 */
#define LINE_JUNCTION_DEFAULT_TURN LINE_TURN_STRAIGHT

#endif /* LINE_FOLLOW_CONFIG_H */
