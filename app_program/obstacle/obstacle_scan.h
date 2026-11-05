/**
 * @file obstacle_scan.h
 * @brief Nonblocking forward sensing and width sweep with packed results.
 */

#ifndef OBSTACLE_SCAN_H
#define OBSTACLE_SCAN_H

/* Public includes. */
#include "../robot_types.h"
#include "drv_ultrasonic.h"
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

/**
 * Geometry published with one result: zeros unless that result is a
 * finished sweep (PATH_FOUND or BLOCKED). Every range is from the servo
 * pivot (obstacle_width_range_mm()); lateral offsets are + left.
 */
typedef struct
{
    uint16_t centre_mm;    /**< Servo pivot to obstacle face, 0 if none. */
    int16_t left_x_mm;     /**< Left boundary, lateral mm from pivot. */
    int16_t right_x_mm;    /**< Right boundary, lateral mm from pivot. */
    bool b_left_clear;     /**< The 60 degree end was clear (free space). */
    bool b_right_clear;    /**< The 120 degree end was clear (free space). */
    uint16_t left_end_mm;  /**< Range at the 60 degree end, 0 if no echo. */
    uint16_t right_end_mm; /**< Range at the 120 degree end, 0 if none. */
} obstacle_scan_geometry_t;

/**
 * One stationary corridor sweep: every grid position from 60 to 120
 * degrees, index 0 at 60 degrees (left). Echoes are medians in
 * microseconds; 0 is no echo. A position whose faults exhausted its retries
 * has its valid bit clear; a timed-out or cancelled sweep has none set.
 */
typedef struct
{
    uint8_t sequence;    /**< Advances once per finished corridor sweep. */
    uint16_t valid_mask; /**< Bit i set: echo_us[i] is usable. */
    uint16_t echo_us[ROBOT_OBSTACLE_SWEEP_POINTS]; /**< Median, 0 = none. */
} obstacle_scan_corridor_t;

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
 * readings. A finite clear echo must reach both the detection threshold and
 * the confirming centre group's median echo + OBSTACLE_EDGE_JUMP_US (the
 * width classifier's MISS).
 * Ties select left. If neither side is clear, publish BLOCKED, or UNKNOWN if a
 * side was invalid. A clear side can still be selected when the other failed.
 *
 * Critical (flags bit 7): one VALID centre echo below
 * OBSTACLE_CRITICAL_ECHO_US (80 mm) sets it at once, without voting. It
 * clears after a complete centre group with no VALID echo below
 * OBSTACLE_CRITICAL_RELEASE_ECHO_US (100 mm), or on a clear centre ping
 * once the stop request is released. Side readings never change it. A
 * change to this bit alone does not advance the sequence, so read it on
 * every poll, like the stop request.
 *
 * Width (coarse then fine): the two side positions above are the coarse
 * scan. The confirming centre group's near votes give a reference echo; a
 * completed reading is a HIT when VALID and nearer than that echo plus
 * OBSTACLE_EDGE_JUMP_US, a MISS for NO_ECHO or a farther echo. An end that
 * is a HIT is that side's outermost hit and its edge is not found (the
 * obstacle continues past the sweep, so the width is a lower bound). From an
 * end that is a MISS, the scanner walks inward one 5 degree step at a time
 * with one ping per angle, right side first, and stops at the first HIT; if
 * none hits, the centre is the hit. Either way that edge is found. The
 * direction and the width are published together, once, after both walks.
 * Any coarse or fine position whose faults exhaust its retries makes the
 * width unavailable (width 0, edges EDGE_NONE, both flags false) while the
 * direction is still published. Edges are the first hit from outside, valid
 * for one contiguous obstacle in the 60..120 degree arc; a found edge is
 * accurate to about half a step before calibration. Results assume the car
 * is stationary throughout the sweep. See obstacle_width.h for the geometry
 * and robot_types.h for the bit layout.
 *
 * Sequence wraps modulo 256 and advances on a
 * completed result, newly asserted stop request, stale forward data,
 * side-scan timeout or state-recovery fault. Consume each sequence once;
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
 * @brief Read the geometry published with the latest result.
 * @return Geometry of the result last returned by obstacle_scan_update(),
 * with the same sequence. It is written only when a result is published, so
 * it stays valid after the sweep's working state is reset, until the next
 * publish of any kind. Call only from the scan task.
 */
obstacle_scan_geometry_t obstacle_scan_geometry (void);

/**
 * @brief Discard every reading so far and start forward sensing afresh.
 *
 * Use after the car has moved: readings taken while turning or driving
 * describe another heading. Cancels any side or corridor sweep (a corridor
 * sweep is published with no valid positions), discards pings and votes,
 * latches the stop request and publishes SCANNING with a new sequence, then
 * centres the servo and waits for it to settle. A full centre group must
 * then confirm a clear path before the stop request is released; a near
 * group starts a normal side scan. The critical flag is kept until a
 * centre group clears it.
 * @return Sequence of the SCANNING result just published. A result is
 * fresh only if its sequence is newer than this one.
 * Call only from the scan task.
 */
uint8_t obstacle_scan_rearm (void);

/**
 * @brief Start a corridor sweep for bypass verification.
 *
 * Latches the stop request, then measures every grid position from 60 to
 * 120 degrees with the side-position ping count, retries and median. When
 * done, or after OBSTACLE_CORRIDOR_TIMEOUT_MS, the sweep is published for
 * obstacle_scan_corridor() and obstacle_scan_rearm() runs. Assumes the car
 * is stationary throughout.
 * @return true if started; false while a side or corridor sweep is running
 * (try again on a later update).
 * Call only from the scan task.
 */
bool obstacle_scan_request_corridor (void);

/**
 * @brief Read the latest published corridor sweep.
 * @return Copy of the last finished sweep; sequence 0 and no valid
 * positions before the first. Call only from the scan task.
 */
obstacle_scan_corridor_t obstacle_scan_corridor (void);

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

#if APP_TEST
/** Coherent test evidence, published by the sensor-owning task only. */
typedef struct
{
    bool initialized;
    uint32_t heartbeat;
    bool stop_requested;
    bool scanning_sides;
    robot_obstacle_status_t status;
    bool hold_requested;
    bool hold_acknowledged;
    uint32_t last_usable_centre_ms;
    uint32_t side_scan_start_ms;
    uint32_t stale_count;
    uint32_t stale_ms;
    uint32_t timeout_count;
    uint32_t timeout_ms;
    uint32_t critical_count;
    uint32_t critical_receipt_us;
    uint32_t critical_flag_us;
    /** Event is always ULTRASONIC_VALID; captured centre pulse in us. */
    uint16_t critical_echo_us;
    robot_obstacle_result_t result;    /**< Latest published result. */
    obstacle_scan_geometry_t geometry; /**< Published with result. */
    uint8_t script_remaining;          /**< Scripted readings not yet used. */
    obstacle_scan_corridor_t corridor; /**< Latest corridor sweep. */
    uint8_t rearm_sequence;            /**< Returned by the last test rearm. */
    bool command_pending;              /**< A test command not yet run. */
    uint32_t corridor_refused;         /**< Corridor requests refused. */
} obstacle_scan_test_snapshot_t;

/** Largest number of scripted readings one request can queue. */
#define OBSTACLE_SCAN_TEST_SCRIPT_MAX 48u

/** Scan-task calls a test can ask for through the test mailbox. */
typedef enum
{
    OBSTACLE_SCAN_TEST_REARM,   /**< obstacle_scan_rearm(). */
    OBSTACLE_SCAN_TEST_CORRIDOR /**< obstacle_scan_request_corridor(). */
} obstacle_scan_test_command_t;

/** One scripted reading, used in place of the next real ultrasonic result. */
typedef struct
{
    ultrasonic_result_t measurement; /**< VALID, NO_ECHO or FAULT. */
    uint16_t echo_us;                /**< Echo in us, used when VALID. */
} obstacle_scan_test_reading_t;

/**
 * @brief Create mailbox mutex once before starting scanner or tests.
 * Initial task only, on either core.
 * @return 0 on success, 1 on failure.
 */
int obstacle_scan_test_init (void);
/**
 * @brief Request/release trigger hold after test_init, from any task/core.
 * @param hold True to freeze triggers; false to resume sensing.
 * @return 0 on success, 1 on synchronization failure. Acknowledgement means
 * all pending measurements were drained. Watchdogs continue during holds.
 */
int obstacle_scan_test_request (bool hold);
/**
 * @brief Copy evidence from any task/core after test_init.
 * @param snapshot Non-null output receiving the synchronized snapshot.
 * @return 0 on success, 1 on synchronization failure or null output.
 * Critical timestamps measure software receipt-to-flag only, in microseconds,
 * modulo 2^32; they do not measure physical arrival or interrupt latency.
 */
int obstacle_scan_test_snapshot (obstacle_scan_test_snapshot_t * snapshot);
/**
 * @brief Replace the next completed ultrasonic readings with a script.
 *
 * The real trigger and echo still run, so driver timing is unchanged; each
 * completed reading is then replaced by the next scripted entry, in order,
 * until the script is used up. A new request replaces any unused entries.
 * @param p_readings Readings to use; may be NULL only when count is 0, which
 *        cancels the script.
 * @param count Number of entries, at most OBSTACLE_SCAN_TEST_SCRIPT_MAX.
 * @return 0 on success, 1 for invalid arguments or synchronization failure.
 * Any task, after obstacle_scan_test_init().
 */
int obstacle_scan_test_script (obstacle_scan_test_reading_t const * p_readings,
                               uint8_t count);
/**
 * @brief Ask the scan task to run a command, then start a script.
 *
 * On its next update the scan task runs the command and only then replaces
 * the script with these readings, so none of them is used before the
 * command. A refused corridor request increments corridor_refused.
 * @param command Scan-task call to make.
 * @param p_readings Readings for after the command; NULL when count is 0.
 * @param count Number of entries, at most OBSTACLE_SCAN_TEST_SCRIPT_MAX.
 * @return 0 on success, 1 for invalid arguments, a command still pending,
 * or synchronization failure. Any task, after obstacle_scan_test_init().
 */
int obstacle_scan_test_command (obstacle_scan_test_command_t command,
                                obstacle_scan_test_reading_t const * p_readings,
                                uint8_t count);
#endif

#endif /* OBSTACLE_SCAN_H */
