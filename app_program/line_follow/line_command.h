/**
 * @file line_command.h
 * @brief Line-following decisions using two IR sensors.
 */

#ifndef LINE_COMMAND_H
#define LINE_COMMAND_H

#include "drv_ir_sensor.h"

/* Public types, constants, and macros. */

/** Mounted-sensor ADC measurements; recheck if height or surface changes.
 * Left: white 130-160, black 1689-1720; intermediate values are partial.
 * Right: white 160-200, edge 1300-1400, black 3374-3410.
 */
#define LINE_LEFT_WHITE_MAX 160u
#define LINE_RIGHT_WHITE_MAX 200u
/** Existing line-coverage detection levels, in ADC counts. Below a level
 * means no line detected, even if above the full-white calibration value.
 * Partial edge coverage is sufficient; neither sensor waits for full black.
 */
#define LINE_LEFT_DETECT_MIN 710u
#define LINE_RIGHT_DETECT_MIN 750u
/** Full-black references for junction confirmation and correction ramps. */
#define LINE_LEFT_BLACK_MIN 1689u
#define LINE_RIGHT_BLACK_MIN                                                   \
    1800u /* LOCAL TEST: was 3374; edge reads 1250-2200. */
/** Time without either edge before requesting SEARCH, in milliseconds. */
#define LINE_EDGE_PROBE_MS 250u /* LOCAL TEST: was 1000. */
/** Minimum slow reacquisition time; a fresh edge is needed to finish. */
#define LINE_REACQUIRE_CONFIRM_MS 1000u

/**
 * @brief Navigation requests; this module does not operate the motors.
 */
typedef enum
{
    /** No recent edge; request slow reacquisition toward the last edge side. */
    LINE_COMMAND_SEARCH = 0,
    /** No side decision, or pending full-black confirmation; move forward. */
    LINE_COMMAND_FORWARD,
    /** Left detects the line and right does not; steer left while moving. */
    LINE_COMMAND_STEER_LEFT,
    /** Right detects the line and left does not; steer right while moving. */
    LINE_COMMAND_STEER_RIGHT,
    /** Sustained both BLACK; report a junction candidate and drive forward. */
    LINE_COMMAND_JUNCTION,
    /** No line detected briefly after the left edge; keep a small left bias. */
    LINE_COMMAND_PROBE_LEFT,
    /** No line detected briefly after the right edge; keep a small right bias. */
    LINE_COMMAND_PROBE_RIGHT,
    /** Invalid input or explicit sensor failure; stop. */
    LINE_COMMAND_STOP,
    /** Confirmed both BLACK during SEARCH; turn left 90 degrees once. */
    LINE_COMMAND_TURN_LEFT,
    /** Confirmed both BLACK during SEARCH; turn right 90 degrees once. */
    LINE_COMMAND_TURN_RIGHT,
    /** Reacquiring the left edge; steer at search speed before following. */
    LINE_COMMAND_ALIGN_LEFT,
    /** Reacquiring the right edge; steer at search speed before following. */
    LINE_COMMAND_ALIGN_RIGHT
} line_command_t;

/* Public function declarations. */

/**
 * @brief Reset to SEARCH and clear consecutive junction confirmation.
 * @note State belongs to one sensor pair; calls must be serialized.
 */
void line_command_init (void);

/**
 * @brief Arm tracking when the caller knows the robot is placed on the line.
 * @note Resets junction confirmation. No motors are accessed. Call from the
 *       sensor-owning task only, serialized with update/init. Both-white
 *       subsequently probes for an edge, initially left if no side is known.
 *       This does not prove the line is present.
 */
void line_command_begin_tracking (void);

/**
 * @brief Follow line boundaries using either sensor's edge detection.
 * @pre Call line_command_init() before the first update.
 * @param reading Left and right ADC counts, each in the range 0 to 4095.
 * @param now_ms Monotonic milliseconds for this fresh reading. Calls must
 *        use chronological timestamps; unsigned wraparound is supported.
 * @note Left-only line coverage requests STEER_LEFT; right-only coverage
 *       requests STEER_RIGHT. Coverage uses each sensor's DETECT_MIN.
 *       During following, both detecting the line requests FORWARD, or
 *       JUNCTION after three consecutive full-black pairs. During SEARCH,
 *       full-black pairs retain SEARCH for the first two samples, then
 *       request TURN_LEFT/RIGHT towards the last singly detected side
 *       (left if unknown). No junction/forward request is issued for those
 *       search samples. Non-full-black resets confirmation. A single-side
 *       detection during SEARCH requests ALIGN_LEFT/RIGHT at search speed.
 *       ALIGN preserves a gentle remembered bias through brief white gaps;
 *       side changes update that direction without restarting the timer.
 *       After LINE_REACQUIRE_CONFIRM_MS, a fresh single-edge sample permits
 *       normal following. White gaps or both-side coverage never authorize
 *       acceleration. Only a full LINE_EDGE_PROBE_MS without either edge
 *       cancels alignment and returns SEARCH. Full-black recovery turns
 *       remain separate from this slow phase.
 *       While neither detects the line, PROBE_LEFT/RIGHT preserves the last
 *       singly detected side for LINE_EDGE_PROBE_MS after the last coverage.
 *       If no side is known, a manually armed test starts probing left.
 *       Expiry requests SEARCH followed by slow alignment on reacquisition;
 *       there is no search-failure timeout. This is a no-observation heuristic,
 *       not proof that the robot left the track. Sensors cannot establish
 *       heading or distinguish every centred/off-track WHITE/WHITE case.
 *       Initialization requests SEARCH immediately until acquisition; a caller
 *       with known on-track placement can use line_command_begin_tracking().
 *       Invalid counts stop and reset state. PD damping may briefly counter-
 *       steer independently of the side label. Call once per fresh pair.
 * @return Requested navigation command.
 */
line_command_t line_command_update (ir_sensor_reading_t reading,
                                    uint32_t now_ms);

/**
 * @brief Get a printable name for a line command.
 * @param command Decision to name.
 * @return Static, immutable string; UNKNOWN for an unrecognized command.
 *         The caller must not free the returned pointer.
 */
char const * line_command_name (line_command_t command);

#endif /* LINE_COMMAND_H */
