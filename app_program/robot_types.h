/**
 * @file robot_types.h
 * @brief Shared obstacle result encoding and motion command packing for
 * application tasks and telemetry.
 *
 * Sweep geometry is part of the wire contract: angle = first + step * index.
 * Changing it requires updating the MQTT decoder with the team.
 * The motion command layout (MBF_MOTION_CMD, Tribunal 1) is a contract
 * between tMission and tMotion; change it only with both owners.
 * Helpers operate only on their arguments, allocate no memory and may run
 * on either core. Publish shared results under the owning module's lock;
 * these helpers do not synchronize access to a shared result structure.
 */

#ifndef ROBOT_TYPES_H
#define ROBOT_TYPES_H

/* Public includes. */
#include <stdbool.h>
#include <stdint.h>

/* Public types, constants, and macros. */

/**
 * Motion command type, bits 0..1 of a motion command word. All-zero is
 * STOP, so a cleared word never drives the motors.
 */
typedef enum
{
    ROBOT_MOTION_CMD_STOP = 0,  /**< Brake; ends any move, turn or follow. */
    ROBOT_MOTION_CMD_MOVE = 1,  /**< Drive a distance, then brake. */
    ROBOT_MOTION_CMD_TURN = 2,  /**< Turn in place by an angle, then brake. */
    ROBOT_MOTION_CMD_FOLLOW = 3 /**< Follow the line until replaced. */
} robot_motion_cmd_type_t;

/**
 * Motion mode, as tMotion reports it (3 bits, Tribunal 1). Values 0..5
 * match the motion module's own states; INHIBITED is tMotion's.
 */
typedef enum
{
    ROBOT_MOTION_MODE_STOPPED = 0,   /**< Braked, no command. */
    ROBOT_MOTION_MODE_MOVE = 1,      /**< Distance move in progress. */
    ROBOT_MOTION_MODE_STALLED = 2,   /**< Braked after a stall; needs STOP. */
    ROBOT_MOTION_MODE_TURN = 3,      /**< Turn in place in progress. */
    ROBOT_MOTION_MODE_TIMED_OUT = 4, /**< Braked, target not reached in
                                          time (encoder fault); needs STOP. */
    ROBOT_MOTION_MODE_FOLLOW = 5,    /**< Following the line. */
    ROBOT_MOTION_MODE_INHIBITED = 6  /**< Braked by MOTOR_INHIBIT; commands
                                          are discarded until it clears. */
} robot_motion_mode_t;

/** Why the last move or turn ended (MOVE_DONE reason, design review 6.4). */
typedef enum
{
    ROBOT_MOTION_DONE_NONE = 0, /**< Not ended by itself. */
    ROBOT_MOTION_DONE_DIST = 1, /**< Its distance or angle was reached. */
    ROBOT_MOTION_DONE_LINE = 2  /**< Ended early as the line came into view. */
} robot_motion_done_t;

/*
 * Motion command word: 22 of 32 bits. Bits 0..1 type, 2 stop at line
 * (MOVE and TURN only), 3 reverse (MOVE: backward, TURN: right), 4..13
 * distance in mm, 14..21 angle in degrees; bits 22..31 must be zero.
 */
#define ROBOT_MOTION_CMD_TYPE_MASK 0x00000003u
#define ROBOT_MOTION_CMD_STOP_AT_LINE 0x00000004u
#define ROBOT_MOTION_CMD_REVERSE 0x00000008u
#define ROBOT_MOTION_CMD_DISTANCE_SHIFT 4u
#define ROBOT_MOTION_CMD_DISTANCE_MASK 0x000003FFu
#define ROBOT_MOTION_CMD_ANGLE_SHIFT 14u
#define ROBOT_MOTION_CMD_ANGLE_MASK 0x000000FFu
#define ROBOT_MOTION_CMD_RESERVED_MASK 0xFFC00000u

#define ROBOT_MOTION_CMD_DISTANCE_MAX_MM 1023u
#define ROBOT_MOTION_CMD_ANGLE_MAX_DEG 180u

/** Status encoded in bits 0..2 of the flags byte. */
typedef enum
{
    ROBOT_OBSTACLE_SCANNING = 0,
    ROBOT_OBSTACLE_CLEAR = 1,
    ROBOT_OBSTACLE_PATH_FOUND = 2,
    ROBOT_OBSTACLE_BLOCKED = 3,
    ROBOT_OBSTACLE_UNKNOWN = 4
} robot_obstacle_status_t;

/** Candidate direction codes; these are not commanded vehicle turns. */
typedef enum
{
    ROBOT_OBSTACLE_DIR_NONE = 0,
    ROBOT_OBSTACLE_DIR_LEFT_60 = 1,
    ROBOT_OBSTACLE_DIR_CENTRE_90 = 2,
    ROBOT_OBSTACLE_DIR_RIGHT_120 = 3
} robot_obstacle_direction_t;

/* Shared angular grid used to interpret packed edge indices: 30 to 150
 * degrees in 10 degree steps (13 points, index 0 at 30). The direction
 * codes keep their names; LEFT_60 and RIGHT_120 now mean the 30 and 150
 * degree ends. */
#define ROBOT_OBSTACLE_SWEEP_FIRST_DEG 30u
#define ROBOT_OBSTACLE_SWEEP_LAST_DEG 150u
#define ROBOT_OBSTACLE_SWEEP_STEP_DEG 10u
#define ROBOT_OBSTACLE_SWEEP_POINTS                                            \
    (((ROBOT_OBSTACLE_SWEEP_LAST_DEG - ROBOT_OBSTACLE_SWEEP_FIRST_DEG) /       \
      ROBOT_OBSTACLE_SWEEP_STEP_DEG) +                                         \
     1u)

#define ROBOT_OBSTACLE_STATUS_MASK 0x07u
#define ROBOT_OBSTACLE_DIRECTION_MASK 0x18u
#define ROBOT_OBSTACLE_DIRECTION_SHIFT 3u
#define ROBOT_OBSTACLE_LEFT_EDGE_FOUND 0x20u
#define ROBOT_OBSTACLE_RIGHT_EDGE_FOUND 0x40u
#define ROBOT_OBSTACLE_CRITICAL 0x80u

#define ROBOT_OBSTACLE_EDGE_MAX_INDEX (ROBOT_OBSTACLE_SWEEP_POINTS - 1u)
#define ROBOT_OBSTACLE_EDGE_NONE 15u
#define ROBOT_OBSTACLE_EDGE_MASK 0x0Fu
#define ROBOT_OBSTACLE_RIGHT_EDGE_SHIFT 4u

/* Width uses nine meaningful bits; zero means unavailable. */
#define ROBOT_OBSTACLE_WIDTH_MAX_MM 511u
#define ROBOT_OBSTACLE_WIDTH_UNAVAILABLE 0u

#if ROBOT_OBSTACLE_SWEEP_STEP_DEG == 0u
#error "Shared sweep step must be nonzero"
#else
#if ROBOT_OBSTACLE_SWEEP_LAST_DEG <= ROBOT_OBSTACLE_SWEEP_FIRST_DEG
#error "Shared sweep limits must be increasing"
#endif
#if ((ROBOT_OBSTACLE_SWEEP_LAST_DEG - ROBOT_OBSTACLE_SWEEP_FIRST_DEG) %        \
     ROBOT_OBSTACLE_SWEEP_STEP_DEG) != 0u
#error "Shared sweep range must contain a whole number of steps"
#endif
#if ROBOT_OBSTACLE_SWEEP_POINTS > ROBOT_OBSTACLE_EDGE_NONE
#error "Edge indices must fit below the unavailable sentinel"
#endif
#endif

/**
 * @brief Application snapshot; serialize fields explicitly for MQTT.
 *
 * The C structure may contain padding. Width is a saturated observed value,
 * not proof of a full obstacle width unless the profile establishes both
 * edges. A sequence comparison assumes fewer than 256 unseen publications.
 */
typedef struct
{
    /** Status 0..2, direction 3..4, left/right edge flags 5/6, critical 7. */
    uint8_t flags;
    /** Left index in bits 0..3, right in 4..7; 15 means unavailable. */
    uint8_t edges;
    /** Width in mm, 0 unavailable, 511 the saturated maximum. */
    uint16_t width_mm;
    /** Publication counter, wrapping modulo 256. */
    uint8_t sequence;
} robot_obstacle_result_t;

/* Public inline function definitions. */

/**
 * @brief Encode a flags byte with the critical bit 7 clear.
 * Set bit 7 afterwards with robot_obstacle_set_critical().
 * @param status Result status; unsupported values become UNKNOWN.
 * @param direction Candidate direction; unsupported values become NONE.
 * @param left_edge_found Whether the left object edge was established.
 * @param right_edge_found Whether the right object edge was established.
 * @return Encoded flags byte. No initialization or shared access required.
 */
static inline uint8_t
robot_obstacle_pack_flags (robot_obstacle_status_t status,
                           robot_obstacle_direction_t direction,
                           bool left_edge_found, bool right_edge_found)
{
    uint8_t status_value = ((unsigned int)status <= ROBOT_OBSTACLE_UNKNOWN)
                               ? (uint8_t)status
                               : (uint8_t)ROBOT_OBSTACLE_UNKNOWN;
    uint8_t direction_value =
        ((unsigned int)direction <= ROBOT_OBSTACLE_DIR_RIGHT_120)
            ? (uint8_t)direction
            : (uint8_t)ROBOT_OBSTACLE_DIR_NONE;

    return (uint8_t)(status_value |
                     (direction_value << ROBOT_OBSTACLE_DIRECTION_SHIFT) |
                     (left_edge_found ? ROBOT_OBSTACLE_LEFT_EDGE_FOUND : 0u) |
                     (right_edge_found ? ROBOT_OBSTACLE_RIGHT_EDGE_FOUND : 0u));
}

/**
 * @brief Decode the result status from a value copied from a snapshot.
 * @param flags Encoded flags byte; other fields are ignored.
 * @return Typed status; reserved status codes become UNKNOWN.
 */
static inline robot_obstacle_status_t
robot_obstacle_get_status (uint8_t flags)
{
    uint8_t value = flags & ROBOT_OBSTACLE_STATUS_MASK;
    return (value <= ROBOT_OBSTACLE_UNKNOWN) ? (robot_obstacle_status_t)value
                                             : ROBOT_OBSTACLE_UNKNOWN;
}

/**
 * @brief Decode the candidate direction from a copied flags byte.
 * @param flags Encoded flags byte; other fields are ignored.
 * @return Typed direction code; all four two-bit patterns are defined.
 */
static inline robot_obstacle_direction_t
robot_obstacle_get_direction (uint8_t flags)
{
    return (
        robot_obstacle_direction_t)((flags & ROBOT_OBSTACLE_DIRECTION_MASK) >>
                                    ROBOT_OBSTACLE_DIRECTION_SHIFT);
}

/**
 * @brief Encode the left and right sweep indices into one byte.
 * @param left Left edge index on the shared angular grid.
 * @param right Right edge index on the shared angular grid.
 * @return Packed indices; out-of-grid inputs become EDGE_NONE before shifts.
 */
static inline uint8_t
robot_obstacle_pack_edges (unsigned int left, unsigned int right)
{
    uint8_t left_value = (left <= ROBOT_OBSTACLE_EDGE_MAX_INDEX)
                             ? (uint8_t)left
                             : ROBOT_OBSTACLE_EDGE_NONE;
    uint8_t right_value = (right <= ROBOT_OBSTACLE_EDGE_MAX_INDEX)
                              ? (uint8_t)right
                              : ROBOT_OBSTACLE_EDGE_NONE;
    return (uint8_t)(left_value |
                     (right_value << ROBOT_OBSTACLE_RIGHT_EDGE_SHIFT));
}

/**
 * @brief Decode the left edge index from a copied edge byte.
 * @param edges Encoded left/right edge byte.
 * @return Left index, or EDGE_NONE for sentinel or reserved values.
 */
static inline uint8_t
robot_obstacle_get_left_edge (uint8_t edges)
{
    uint8_t value = edges & ROBOT_OBSTACLE_EDGE_MASK;
    return (value <= ROBOT_OBSTACLE_EDGE_MAX_INDEX) ? value
                                                    : ROBOT_OBSTACLE_EDGE_NONE;
}

/**
 * @brief Decode the right edge index from a copied edge byte.
 * @param edges Encoded left/right edge byte.
 * @return Right index, or EDGE_NONE for sentinel or reserved values.
 */
static inline uint8_t
robot_obstacle_get_right_edge (uint8_t edges)
{
    uint8_t value = edges >> ROBOT_OBSTACLE_RIGHT_EDGE_SHIFT;
    return (value <= ROBOT_OBSTACLE_EDGE_MAX_INDEX) ? value
                                                    : ROBOT_OBSTACLE_EDGE_NONE;
}

/**
 * @brief Saturate an unsigned width before storing the nine-bit value.
 * @param width_mm Nonnegative width in mm; zero remains unavailable.
 * @return Width in mm, limited to WIDTH_MAX_MM without wrapping.
 */
static inline uint16_t
robot_obstacle_clamp_width (uint32_t width_mm)
{
    return (width_mm > ROBOT_OBSTACLE_WIDTH_MAX_MM)
               ? ROBOT_OBSTACLE_WIDTH_MAX_MM
               : (uint16_t)width_mm;
}

/**
 * @brief Set or clear the critical flag in a copied flags byte.
 * @param flags Encoded flags byte; every other bit is returned unchanged.
 * @param b_critical True to set the critical flag, false to clear it.
 * @return Updated flags byte with the critical bit set or cleared.
 */
static inline uint8_t
robot_obstacle_set_critical (uint8_t flags, bool b_critical)
{
    return b_critical ? (uint8_t)(flags | ROBOT_OBSTACLE_CRITICAL)
                      : (uint8_t)(flags & (uint8_t)~ROBOT_OBSTACLE_CRITICAL);
}

/**
 * @brief Get the critical flag from a copied flags byte.
 * @param flags Encoded flags byte; other fields are ignored.
 * @return true if the critical flag is set, false otherwise.
 */
static inline bool
robot_obstacle_get_critical (uint8_t flags)
{
    return (flags & ROBOT_OBSTACLE_CRITICAL) != 0u;
}

/**
 * @brief Encode a distance move.
 * @param distance_mm Distance in mm; above DISTANCE_MAX_MM it saturates
 * (check longer moves before packing: split them). 0 is refused by
 * tMotion.
 * @param b_backward True to reverse, false to drive forward.
 * @param b_stop_at_line True to end early at a fresh confirmed line,
 * including one already visible when this queued command is armed.
 * @return Command word for MBF_MOTION_CMD.
 */
static inline uint32_t
robot_motion_cmd_move (uint32_t distance_mm, bool b_backward,
                       bool b_stop_at_line)
{
    uint32_t distance = (distance_mm > ROBOT_MOTION_CMD_DISTANCE_MAX_MM)
                            ? ROBOT_MOTION_CMD_DISTANCE_MAX_MM
                            : distance_mm;

    return (uint32_t)ROBOT_MOTION_CMD_MOVE |
           (b_stop_at_line ? ROBOT_MOTION_CMD_STOP_AT_LINE : 0u) |
           (b_backward ? ROBOT_MOTION_CMD_REVERSE : 0u) |
           (distance << ROBOT_MOTION_CMD_DISTANCE_SHIFT);
}

/**
 * @brief Encode a turn in place.
 * @param angle_deg Angle in degrees; above ANGLE_MAX_DEG it saturates.
 * 0 is refused by tMotion.
 * @param b_right True to turn right (clockwise), false left.
 * @param b_stop_at_line True to end early at a fresh confirmed line,
 * including one already visible when this queued command is armed.
 * @return Command word for MBF_MOTION_CMD.
 */
static inline uint32_t
robot_motion_cmd_turn (uint32_t angle_deg, bool b_right, bool b_stop_at_line)
{
    uint32_t angle = (angle_deg > ROBOT_MOTION_CMD_ANGLE_MAX_DEG)
                         ? ROBOT_MOTION_CMD_ANGLE_MAX_DEG
                         : angle_deg;

    return (uint32_t)ROBOT_MOTION_CMD_TURN |
           (b_stop_at_line ? ROBOT_MOTION_CMD_STOP_AT_LINE : 0u) |
           (b_right ? ROBOT_MOTION_CMD_REVERSE : 0u) |
           (angle << ROBOT_MOTION_CMD_ANGLE_SHIFT);
}

/**
 * @brief Encode line following at the motion module's fixed follow speed.
 * @return Command word for MBF_MOTION_CMD.
 */
static inline uint32_t
robot_motion_cmd_follow (void)
{
    return (uint32_t)ROBOT_MOTION_CMD_FOLLOW;
}

/**
 * @brief Encode a stop.
 * @return Command word for MBF_MOTION_CMD.
 */
static inline uint32_t
robot_motion_cmd_stop (void)
{
    return (uint32_t)ROBOT_MOTION_CMD_STOP;
}

/**
 * @brief Decode the command type.
 * @param cmd Command word.
 * @return Type; all four two-bit patterns are defined.
 */
static inline robot_motion_cmd_type_t
robot_motion_cmd_get_type (uint32_t cmd)
{
    return (robot_motion_cmd_type_t)(cmd & ROBOT_MOTION_CMD_TYPE_MASK);
}

/**
 * @brief Decode the distance of a MOVE.
 * @param cmd Command word.
 * @return Distance in mm, 0 to DISTANCE_MAX_MM.
 */
static inline uint16_t
robot_motion_cmd_get_distance (uint32_t cmd)
{
    return (uint16_t)((cmd >> ROBOT_MOTION_CMD_DISTANCE_SHIFT) &
                      ROBOT_MOTION_CMD_DISTANCE_MASK);
}

/**
 * @brief Decode the angle of a TURN.
 * @param cmd Command word.
 * @return Angle in degrees, 0 to 255; above ANGLE_MAX_DEG is invalid.
 */
static inline uint16_t
robot_motion_cmd_get_angle (uint32_t cmd)
{
    return (uint16_t)((cmd >> ROBOT_MOTION_CMD_ANGLE_SHIFT) &
                      ROBOT_MOTION_CMD_ANGLE_MASK);
}

/**
 * @brief Decode the reverse bit (MOVE backward, TURN right).
 * @param cmd Command word.
 * @return true if set.
 */
static inline bool
robot_motion_cmd_get_reverse (uint32_t cmd)
{
    return (cmd & ROBOT_MOTION_CMD_REVERSE) != 0u;
}

/**
 * @brief Decode the stop-at-line bit.
 * @param cmd Command word.
 * @return true if set.
 */
static inline bool
robot_motion_cmd_get_stop_at_line (uint32_t cmd)
{
    return (cmd & ROBOT_MOTION_CMD_STOP_AT_LINE) != 0u;
}

#endif /* ROBOT_TYPES_H */
