/**
 * @file obstacle_scan.h
 * @brief Nonblocking three-position sensing with packed obstacle results.
 */

#ifndef OBSTACLE_SCAN_H
#define OBSTACLE_SCAN_H

/* Public includes. */
#include "../robot_types.h"
#include <stdbool.h>
#include <stdint.h>

/* Public types, constants, and macros. */

/** Servo timing measured at the operating voltage and mechanical load. */
typedef struct
{
    /** Travel time in milliseconds per 60 degrees, excluding settling. */
    uint16_t travel_ms_per_60_degrees;
    /** Extra settling time in milliseconds after the estimated travel. */
    uint16_t settle_margin_ms;
} obstacle_servo_timing_t;

/* Public function declarations. */

/**
 * @brief Advance forward sensing and both side measurements without waiting.
 *
 * Call regularly from the initializing scan task on core 0. Centre triggers
 * use OBSTACLE_FORWARD_PERIOD_MS (nominal 60 ms); polling/driver delays can
 * lengthen this interval. Missed periods are not replayed. A first near ping
 * asserts the separate stop request immediately when consumed. It and the
 * next two usable pings confirm an obstacle with at least two near votes.
 * NO_ECHO and echoes at/beyond the threshold count as clear. While already
 * clear, each clear centre ping publishes a fresh result without grouping.
 *
 * Faults assert/retain the stop request and consume the group's two extra
 * attempts, never votes. Exhaustion publishes UNKNOWN, discards the group
 * and starts recovery. Only a full group with at least two clear votes can
 * release the stop request. Startup also requires this clear confirmation.
 *
 * Side positions retain the configured ping count and nonzero median:
 * zeros are ignored, two nonzero values are averaged, all-zero is NO_ECHO.
 * Exhausted side retries invalidate the position. Retry budgets reset per
 * side position or centre group, not per ping.
 *
 * A blocked centre starts left then right sensing; a completed side result
 * is published only after both positions. NO_ECHO ranks beyond finite clear
 * readings, finite readings use the configured detection threshold, and ties
 * select left. If neither side is clear, publish BLOCKED, or UNKNOWN if a
 * side was invalid. A clear side can still be selected when the other failed.
 *
 * Width is not measured: width_mm is always WIDTH_UNAVAILABLE, both edge
 * indices are EDGE_NONE and both edge-found flags are false. See robot_types.h
 * for the bit layout. Sequence wraps modulo 256 and advances on a
 * completed result, newly asserted stop request or state-recovery fault.
 * Consume each sequence once;
 * fewer than 256 unseen results must separate comparisons. Results persist
 * while returning to centre. The function requests no vehicle movement and
 * a selected direction does not establish vehicle clearance.
 * @return Packed snapshot; SCANNING also covers initial near confirmation.
 */
robot_obstacle_result_t obstacle_scan_update (void);

/**
 * @brief Read the latched obstacle stop request from the owning scan task.
 * @return True before initialization, during startup confirmation, after a
 * near ping/fault and throughout side scanning. Only a complete centre group
 * with two clear votes releases it; PATH_FOUND does not release it.
 * Call from the same task as obstacle_scan_update(), normally immediately
 * after each update. This output neither brakes nor starts motors. It is
 * separate from the packed telemetry layout and is not synchronized for
 * cross-task access; the motion transport remains an integration task.
 */
bool obstacle_scan_stop_requested (void);

/**
 * @brief Initialize the sensor and centre the servo before sensing.
 * @param trig_pin Ultrasonic trigger GPIO number.
 * @param echo_pin Ultrasonic echo GPIO number, distinct from trig_pin.
 * @param servo_pin Servo GPIO number, distinct from both ultrasonic pins.
 * @param timing Non-null timing configuration, copied during this call.
 * At least one timing field must be nonzero. Travel time is rounded up and
 * added to the settling margin. Startup allows up to 180 degrees of travel
 * because physical position is unknown. Existing calibration is unchanged.
 * @return 0 on success; 1 for invalid pins/timing, unsupported system clock
 * or initialization outside core 0. Initialize once from the scan task and
 * call obstacle_scan_update() only from that task on core 0.
 */
int obstacle_scan_init (int trig_pin, int echo_pin, int servo_pin,
                        const obstacle_servo_timing_t * timing);

/**
 * @brief Convert a decoded status to a printable name, without shared access.
 * @param status Decoded robot status from robot_obstacle_get_status().
 * @return Static string, never NULL; unknown enum values return "INVALID".
 * May be called on either core; no initialization is required.
 */
char const * obstacle_scan_status_name (robot_obstacle_status_t status);

#endif /* OBSTACLE_SCAN_H */
