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

/* Shared angular grid used to interpret packed edge indices: 50 to 130
 * degrees in 10 degree steps (9 points, index 0 at 50). The direction
 * codes keep their names; LEFT_60 and RIGHT_120 now mean the 50 and 130
 * degree ends. */
#define ROBOT_OBSTACLE_SWEEP_FIRST_DEG 50u
#define ROBOT_OBSTACLE_SWEEP_LAST_DEG 130u
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

/*
 * Minimum-bit layouts for data stored or shared between tasks. A field
 * takes only the bits its range needs, and robot_field_put() saturates at
 * its limit so a value cannot reach the next field. Calculations unpack to
 * full-size variables first.
 */

/* Motion status byte: bits 0..2 mode, 3..4 done reason. */
#define ROBOT_MOTION_STATUS_MODE_SHIFT 0u
#define ROBOT_MOTION_STATUS_MODE_BITS 3u
#define ROBOT_MOTION_STATUS_DONE_SHIFT 3u
#define ROBOT_MOTION_STATUS_DONE_BITS 2u

/*
 * Line request word (IF-04), 30 of 32 bits: bits 0..1 mode, 2 line seen,
 * 3 junction, 4 slow, 5 valid, 6..13 reverse distance in mm (0 none),
 * 14..29 target heading in degrees, not wrapped (about 91 turns each way).
 */
#define ROBOT_LINE_REQ_MODE_SHIFT 0u
#define ROBOT_LINE_REQ_MODE_BITS 2u
#define ROBOT_LINE_REQ_SEEN 0x00000004u
#define ROBOT_LINE_REQ_JUNCTION 0x00000008u
#define ROBOT_LINE_REQ_SLOW 0x00000010u
#define ROBOT_LINE_REQ_VALID 0x00000020u
#define ROBOT_LINE_REQ_REVERSE_SHIFT 6u
#define ROBOT_LINE_REQ_REVERSE_BITS 8u
#define ROBOT_LINE_REQ_REVERSE_MAX_MM 255u
#define ROBOT_LINE_REQ_HEADING_SHIFT 14u
#define ROBOT_LINE_REQ_HEADING_BITS 16u

/*
 * Odometry (IF-05), three words. Word 0: bits 0..23 distance in mm (about
 * 8.3 km each way), 24..31 update count. Word 1: bits 0..15 heading in
 * degrees, not wrapped, 16..26 mean speed. Word 2: bits 0..10 left speed,
 * 11..21 right speed. Speeds in mm/s, limited to 1023; the car tops out
 * at 620.
 */
#define ROBOT_ODOMETRY_WORDS 3u
#define ROBOT_ODOMETRY_DISTANCE_SHIFT 0u
#define ROBOT_ODOMETRY_DISTANCE_BITS 24u
#define ROBOT_ODOMETRY_UPDATES_SHIFT 24u
#define ROBOT_ODOMETRY_UPDATES_BITS 8u
#define ROBOT_ODOMETRY_HEADING_SHIFT 0u
#define ROBOT_ODOMETRY_HEADING_BITS 16u
#define ROBOT_ODOMETRY_SPEED_SHIFT 16u
#define ROBOT_ODOMETRY_LEFT_SHIFT 0u
#define ROBOT_ODOMETRY_RIGHT_SHIFT 11u
#define ROBOT_ODOMETRY_SPEED_BITS 11u

/** Odometry as tMotion publishes it (IF-05), unpacked for the caller. */
typedef struct
{
    uint8_t updates;          /**< Advances per tMotion update, 255 to 1;
                                   0 until the first update. Stops rising
                                   when tMotion stops. */
    int32_t distance_mm;      /**< Mean of both wheels' travel. */
    int32_t heading_deg;      /**< Total turn, not wrapped. */
    int16_t speed_mm_s;       /**< Mean of the two wheel speeds. */
    int16_t left_speed_mm_s;  /**< Left wheel speed. */
    int16_t right_speed_mm_s; /**< Right wheel speed. */
} robot_odometry_t;

/*
 * Obstacle geometry (IF-07), 64 bits, least significant first: centre 12,
 * left and right lateral offsets 13 each (signed), left and right end
 * ranges 12 each, left and right clear 1 each. Ranges in mm, to 4095.
 */
#define ROBOT_GEOMETRY_RANGE_BITS 12u
#define ROBOT_GEOMETRY_LATERAL_BITS 13u
#define ROBOT_GEOMETRY_CENTRE_SHIFT 0u
#define ROBOT_GEOMETRY_LEFT_X_SHIFT 12u
#define ROBOT_GEOMETRY_RIGHT_X_SHIFT 25u
#define ROBOT_GEOMETRY_LEFT_END_SHIFT 38u
#define ROBOT_GEOMETRY_RIGHT_END_SHIFT 50u
#define ROBOT_GEOMETRY_LEFT_CLEAR_SHIFT 62u
#define ROBOT_GEOMETRY_RIGHT_CLEAR_SHIFT 63u
#define ROBOT_GEOMETRY_FLAG_BITS 1u

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
    bool b_left_clear;     /**< The first (30 degree) end was clear. */
    bool b_right_clear;    /**< The last (150 degree) end was clear. */
    uint16_t left_end_mm;  /**< Range at the 30 degree end, 0 if none. */
    uint16_t right_end_mm; /**< Range at the 150 degree end, 0 if none. */
} robot_obstacle_geometry_t;

/* Obstacle control half-word: bit 0 stop, 1..4 avoid state, 5..8 fault. */
#define ROBOT_OBSTACLE_CONTROL_STOP 0x0001u
#define ROBOT_OBSTACLE_CONTROL_STATE_SHIFT 1u
#define ROBOT_OBSTACLE_CONTROL_STATE_BITS 4u
#define ROBOT_OBSTACLE_CONTROL_FAULT_SHIFT 5u
#define ROBOT_OBSTACLE_CONTROL_FAULT_BITS 4u

/* Watch reading word, 24 of 32 bits: echo 0..14, sequence 15..22, active 23. */
#define ROBOT_WATCH_ECHO_SHIFT 0u
#define ROBOT_WATCH_ECHO_BITS 15u
#define ROBOT_WATCH_SEQUENCE_SHIFT 15u
#define ROBOT_WATCH_SEQUENCE_BITS 8u
#define ROBOT_WATCH_ACTIVE 0x00800000u

/** Latest reading while the sensor is held at a watch angle. */
typedef struct
{
    uint8_t sequence; /**< Advances once per reading taken while watching. */
    uint16_t echo_us; /**< Echo of that reading; 0 = none or unusable. */
    bool b_active;    /**< The sensor is held at a watch angle. */
} robot_obstacle_watch_t;

/*
 * Compass (IF-09), two words. Word 0: bits 0..11 X, 12..23 Y, 24..31 sample
 * count. Word 1: bits 0..11 Z, 12..23 heading in tenths of a degree, 24
 * valid. Axes are the sensor's signed 12-bit counts.
 */
#define ROBOT_COMPASS_WORDS 2u
#define ROBOT_COMPASS_AXIS_BITS 12u
#define ROBOT_COMPASS_X_SHIFT 0u
#define ROBOT_COMPASS_Y_SHIFT 12u
#define ROBOT_COMPASS_SAMPLES_SHIFT 24u
#define ROBOT_COMPASS_SAMPLES_BITS 8u
#define ROBOT_COMPASS_Z_SHIFT 0u
#define ROBOT_COMPASS_HEADING_SHIFT 12u
#define ROBOT_COMPASS_HEADING_BITS 12u
#define ROBOT_COMPASS_VALID 0x01000000u

/*
 * Barcode map entry (IF-10), 20 of 32 bits: three 6-bit characters from
 * the bottom, then the 2-bit turn. A character is its Code 39 code plus 1;
 * 0 ends the text, so an all-zero entry is free. One word, so the radio
 * task replaces an entry in one write and needs no lock.
 */
#define ROBOT_BARCODE_MAP_CHAR_BITS 6u
#define ROBOT_BARCODE_MAP_CHARS 3u
#define ROBOT_BARCODE_MAP_TEXT_MASK 0x0003FFFFu
#define ROBOT_BARCODE_MAP_TURN_SHIFT 18u
#define ROBOT_BARCODE_MAP_TURN_BITS 2u

/*
 * Barcode edge word: bit 0 level after the edge, bits 1..31 the 1 MHz
 * TIMER in 2 us steps. The time keeps its top bit, so unsigned differences
 * still work across the clock's wrap.
 */
#define ROBOT_BARCODE_EDGE_BLACK 0x00000001u
#define ROBOT_BARCODE_EDGE_TIME_MASK 0xFFFFFFFEu

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

/**
 * @brief Read an unsigned field of a packed word.
 * @param word Packed word; other fields are ignored.
 * @param shift Field position, bits.
 * @param bits Field width, 1 to 31.
 * @return Field value.
 */
static inline uint32_t
robot_field_get (uint32_t word, uint32_t shift, uint32_t bits)
{
    return (word >> shift) & ((1u << bits) - 1u);
}

/**
 * @brief Write an unsigned field of a packed word.
 * @param word Packed word; other fields are returned unchanged.
 * @param shift Field position, bits.
 * @param bits Field width, 1 to 31.
 * @param value Value; above the field's limit it saturates.
 * @return Updated word.
 */
static inline uint32_t
robot_field_put (uint32_t word, uint32_t shift, uint32_t bits, uint32_t value)
{
    uint32_t const max = (1u << bits) - 1u;
    uint32_t const kept = (value > max) ? max : value;

    return (word & ~(max << shift)) | (kept << shift);
}

/**
 * @brief Saturate a signed value to a field and encode it.
 * @param value Signed value.
 * @param bits Field width, 2 to 31.
 * @return Two's complement field value for robot_field_put().
 */
static inline uint32_t
robot_field_from_signed (int32_t value, uint32_t bits)
{
    int32_t const max = (int32_t)((1u << (bits - 1u)) - 1u);
    int32_t const min = -max - 1;
    int32_t const kept = (value > max) ? max : ((value < min) ? min : value);

    return (uint32_t)kept & ((1u << bits) - 1u);
}

/**
 * @brief Decode a two's complement field.
 * @param field Field value from robot_field_get().
 * @param bits Field width, 2 to 31.
 * @return Signed value.
 */
static inline int32_t
robot_field_to_signed (uint32_t field, uint32_t bits)
{
    uint32_t const sign = 1u << (bits - 1u);

    return (int32_t)(field ^ sign) - (int32_t)sign;
}

/**
 * @brief Read an unsigned field of a 64-bit layout.
 * @param word Packed value; other fields are ignored.
 * @param shift Field position, bits, 0 to 63.
 * @param bits Field width, 1 to 31.
 * @return Field value.
 */
static inline uint32_t
robot_field64_get (uint64_t word, uint32_t shift, uint32_t bits)
{
    return (uint32_t)(word >> shift) & ((1u << bits) - 1u);
}

/**
 * @brief Write an unsigned field of a 64-bit layout.
 * @param word Packed value; other fields are returned unchanged.
 * @param shift Field position, bits, 0 to 63.
 * @param bits Field width, 1 to 31.
 * @param value Value; above the field's limit it saturates.
 * @return Updated value.
 */
static inline uint64_t
robot_field64_put (uint64_t word, uint32_t shift, uint32_t bits, uint32_t value)
{
    uint32_t const max = (1u << bits) - 1u;
    uint32_t const kept = (value > max) ? max : value;

    return (word & ~((uint64_t)max << shift)) | ((uint64_t)kept << shift);
}

/**
 * @brief Saturate a value to 16 signed bits.
 * @param value Full-size value.
 * @return Value limited to INT16_MIN..INT16_MAX.
 */
static inline int16_t
robot_clamp_i16 (int32_t value)
{
    return (value > INT16_MAX)
               ? INT16_MAX
               : ((value < INT16_MIN) ? INT16_MIN : (int16_t)value);
}

/**
 * @brief Saturate a value to 16 unsigned bits.
 * @param value Full-size value.
 * @return Value limited to UINT16_MAX.
 */
static inline uint16_t
robot_clamp_u16 (uint32_t value)
{
    return (value > UINT16_MAX) ? UINT16_MAX : (uint16_t)value;
}

/**
 * @brief Keep the low 16 bits of a running distance as a mark.
 * @param value Running distance in mm.
 * @return Mark for robot_mark16_near().
 */
static inline int16_t
robot_mark16 (int32_t value)
{
    return (int16_t)(uint16_t)(uint32_t)value;
}

/**
 * @brief The running distance a 16-bit mark stands for.
 * @param now Current running distance in mm.
 * @param mark Mark from robot_mark16().
 * @return The value nearest @p now; marks more than 32 m away do not read
 * correctly, so age any mark that can stay unused for that long.
 */
static inline int32_t
robot_mark16_near (int32_t now, int16_t mark)
{
    uint32_t const low = (uint32_t)(int32_t)mark - (uint32_t)now;

    return now + (int32_t)(int16_t)(uint16_t)low;
}

/**
 * @brief Encode a line request (IF-04).
 * @param mode line_mode_t value, 0 to 3.
 * @param flags ROBOT_LINE_REQ_SEEN, JUNCTION, SLOW and VALID, ORed.
 * @param reverse_mm Reverse distance in mm; saturates at REVERSE_MAX_MM.
 * @param heading_deg Target heading, not wrapped; saturates at 16 bits.
 * @return Packed word.
 */
static inline uint32_t
robot_line_request_pack (uint32_t mode, uint32_t flags, uint32_t reverse_mm,
                         int32_t heading_deg)
{
    uint32_t word = flags & (ROBOT_LINE_REQ_SEEN | ROBOT_LINE_REQ_JUNCTION |
                             ROBOT_LINE_REQ_SLOW | ROBOT_LINE_REQ_VALID);

    word = robot_field_put(word, ROBOT_LINE_REQ_MODE_SHIFT,
                           ROBOT_LINE_REQ_MODE_BITS, mode);
    word = robot_field_put(word, ROBOT_LINE_REQ_REVERSE_SHIFT,
                           ROBOT_LINE_REQ_REVERSE_BITS, reverse_mm);
    return robot_field_put(
        word, ROBOT_LINE_REQ_HEADING_SHIFT, ROBOT_LINE_REQ_HEADING_BITS,
        robot_field_from_signed(heading_deg, ROBOT_LINE_REQ_HEADING_BITS));
}

/**
 * @brief Decode the mode of a line request.
 * @param word Packed word.
 * @return line_mode_t value; all four patterns are defined.
 */
static inline uint32_t
robot_line_request_get_mode (uint32_t word)
{
    return robot_field_get(word, ROBOT_LINE_REQ_MODE_SHIFT,
                           ROBOT_LINE_REQ_MODE_BITS);
}

/**
 * @brief Decode the reverse distance of a line request.
 * @param word Packed word.
 * @return Distance in mm, 0 to REVERSE_MAX_MM.
 */
static inline uint16_t
robot_line_request_get_reverse (uint32_t word)
{
    return (uint16_t)robot_field_get(word, ROBOT_LINE_REQ_REVERSE_SHIFT,
                                     ROBOT_LINE_REQ_REVERSE_BITS);
}

/**
 * @brief Decode the target heading of a line request.
 * @param word Packed word.
 * @return Heading in degrees, not wrapped.
 */
static inline int32_t
robot_line_request_get_heading (uint32_t word)
{
    return robot_field_to_signed(robot_field_get(word,
                                                 ROBOT_LINE_REQ_HEADING_SHIFT,
                                                 ROBOT_LINE_REQ_HEADING_BITS),
                                 ROBOT_LINE_REQ_HEADING_BITS);
}

/**
 * @brief Encode odometry (IF-05).
 * @param p_in Unpacked odometry; values saturate at their field limits.
 * @param p_words Destination, ROBOT_ODOMETRY_WORDS long.
 */
static inline void
robot_odometry_pack (robot_odometry_t const * p_in, uint32_t * p_words)
{
    uint32_t word = robot_field_put(
        0u, ROBOT_ODOMETRY_DISTANCE_SHIFT, ROBOT_ODOMETRY_DISTANCE_BITS,
        robot_field_from_signed(p_in->distance_mm,
                                ROBOT_ODOMETRY_DISTANCE_BITS));
    p_words[0] = robot_field_put(word, ROBOT_ODOMETRY_UPDATES_SHIFT,
                                 ROBOT_ODOMETRY_UPDATES_BITS, p_in->updates);
    word = robot_field_put(0u, ROBOT_ODOMETRY_HEADING_SHIFT,
                           ROBOT_ODOMETRY_HEADING_BITS,
                           robot_field_from_signed(
                               p_in->heading_deg, ROBOT_ODOMETRY_HEADING_BITS));
    p_words[1] = robot_field_put(
        word, ROBOT_ODOMETRY_SPEED_SHIFT, ROBOT_ODOMETRY_SPEED_BITS,
        robot_field_from_signed(p_in->speed_mm_s, ROBOT_ODOMETRY_SPEED_BITS));
    word = robot_field_put(0u, ROBOT_ODOMETRY_LEFT_SHIFT,
                           ROBOT_ODOMETRY_SPEED_BITS,
                           robot_field_from_signed(p_in->left_speed_mm_s,
                                                   ROBOT_ODOMETRY_SPEED_BITS));
    p_words[2] = robot_field_put(
        word, ROBOT_ODOMETRY_RIGHT_SHIFT, ROBOT_ODOMETRY_SPEED_BITS,
        robot_field_from_signed(p_in->right_speed_mm_s,
                                ROBOT_ODOMETRY_SPEED_BITS));
}

/**
 * @brief Decode odometry (IF-05).
 * @param p_words Source, ROBOT_ODOMETRY_WORDS long.
 * @return Unpacked odometry.
 */
static inline robot_odometry_t
robot_odometry_unpack (uint32_t const * p_words)
{
    robot_odometry_t out;

    out.updates = (uint8_t)robot_field_get(
        p_words[0], ROBOT_ODOMETRY_UPDATES_SHIFT, ROBOT_ODOMETRY_UPDATES_BITS);
    out.distance_mm = robot_field_to_signed(
        robot_field_get(p_words[0], ROBOT_ODOMETRY_DISTANCE_SHIFT,
                        ROBOT_ODOMETRY_DISTANCE_BITS),
        ROBOT_ODOMETRY_DISTANCE_BITS);
    out.heading_deg = robot_field_to_signed(
        robot_field_get(p_words[1], ROBOT_ODOMETRY_HEADING_SHIFT,
                        ROBOT_ODOMETRY_HEADING_BITS),
        ROBOT_ODOMETRY_HEADING_BITS);
    out.speed_mm_s = (int16_t)robot_field_to_signed(
        robot_field_get(p_words[1], ROBOT_ODOMETRY_SPEED_SHIFT,
                        ROBOT_ODOMETRY_SPEED_BITS),
        ROBOT_ODOMETRY_SPEED_BITS);
    out.left_speed_mm_s = (int16_t)robot_field_to_signed(
        robot_field_get(p_words[2], ROBOT_ODOMETRY_LEFT_SHIFT,
                        ROBOT_ODOMETRY_SPEED_BITS),
        ROBOT_ODOMETRY_SPEED_BITS);
    out.right_speed_mm_s = (int16_t)robot_field_to_signed(
        robot_field_get(p_words[2], ROBOT_ODOMETRY_RIGHT_SHIFT,
                        ROBOT_ODOMETRY_SPEED_BITS),
        ROBOT_ODOMETRY_SPEED_BITS);
    return out;
}

/**
 * @brief Encode obstacle geometry (IF-07).
 * @param p_in Unpacked geometry; values saturate at their field limits.
 * @return Packed value.
 */
static inline uint64_t
robot_obstacle_geometry_pack (robot_obstacle_geometry_t const * p_in)
{
    uint64_t word =
        robot_field64_put(0u, ROBOT_GEOMETRY_CENTRE_SHIFT,
                          ROBOT_GEOMETRY_RANGE_BITS, p_in->centre_mm);

    word = robot_field64_put(
        word, ROBOT_GEOMETRY_LEFT_X_SHIFT, ROBOT_GEOMETRY_LATERAL_BITS,
        robot_field_from_signed(p_in->left_x_mm, ROBOT_GEOMETRY_LATERAL_BITS));
    word = robot_field64_put(
        word, ROBOT_GEOMETRY_RIGHT_X_SHIFT, ROBOT_GEOMETRY_LATERAL_BITS,
        robot_field_from_signed(p_in->right_x_mm, ROBOT_GEOMETRY_LATERAL_BITS));
    word = robot_field64_put(word, ROBOT_GEOMETRY_LEFT_END_SHIFT,
                             ROBOT_GEOMETRY_RANGE_BITS, p_in->left_end_mm);
    word = robot_field64_put(word, ROBOT_GEOMETRY_RIGHT_END_SHIFT,
                             ROBOT_GEOMETRY_RANGE_BITS, p_in->right_end_mm);
    word = robot_field64_put(word, ROBOT_GEOMETRY_LEFT_CLEAR_SHIFT,
                             ROBOT_GEOMETRY_FLAG_BITS,
                             p_in->b_left_clear ? 1u : 0u);
    return robot_field64_put(word, ROBOT_GEOMETRY_RIGHT_CLEAR_SHIFT,
                             ROBOT_GEOMETRY_FLAG_BITS,
                             p_in->b_right_clear ? 1u : 0u);
}

/**
 * @brief Decode obstacle geometry (IF-07).
 * @param word Packed value.
 * @return Unpacked geometry.
 */
static inline robot_obstacle_geometry_t
robot_obstacle_geometry_unpack (uint64_t word)
{
    robot_obstacle_geometry_t out;

    out.centre_mm = (uint16_t)robot_field64_get(
        word, ROBOT_GEOMETRY_CENTRE_SHIFT, ROBOT_GEOMETRY_RANGE_BITS);
    out.left_x_mm = (int16_t)robot_field_to_signed(
        robot_field64_get(word, ROBOT_GEOMETRY_LEFT_X_SHIFT,
                          ROBOT_GEOMETRY_LATERAL_BITS),
        ROBOT_GEOMETRY_LATERAL_BITS);
    out.right_x_mm = (int16_t)robot_field_to_signed(
        robot_field64_get(word, ROBOT_GEOMETRY_RIGHT_X_SHIFT,
                          ROBOT_GEOMETRY_LATERAL_BITS),
        ROBOT_GEOMETRY_LATERAL_BITS);
    out.left_end_mm = (uint16_t)robot_field64_get(
        word, ROBOT_GEOMETRY_LEFT_END_SHIFT, ROBOT_GEOMETRY_RANGE_BITS);
    out.right_end_mm = (uint16_t)robot_field64_get(
        word, ROBOT_GEOMETRY_RIGHT_END_SHIFT, ROBOT_GEOMETRY_RANGE_BITS);
    out.b_left_clear = robot_field64_get(word, ROBOT_GEOMETRY_LEFT_CLEAR_SHIFT,
                                         ROBOT_GEOMETRY_FLAG_BITS) != 0u;
    out.b_right_clear =
        robot_field64_get(word, ROBOT_GEOMETRY_RIGHT_CLEAR_SHIFT,
                          ROBOT_GEOMETRY_FLAG_BITS) != 0u;
    return out;
}

/**
 * @brief Encode the obstacle control half-word.
 * @param b_stop Latched stop request.
 * @param avoid_state obstacle_avoid_state_t value, saturating at 15.
 * @param fault obstacle_avoid_fault_t value, saturating at 15.
 * @return Packed half-word.
 */
static inline uint16_t
robot_obstacle_control_pack (bool b_stop, uint32_t avoid_state, uint32_t fault)
{
    uint32_t word = b_stop ? ROBOT_OBSTACLE_CONTROL_STOP : 0u;

    word = robot_field_put(word, ROBOT_OBSTACLE_CONTROL_STATE_SHIFT,
                           ROBOT_OBSTACLE_CONTROL_STATE_BITS, avoid_state);
    return (uint16_t)robot_field_put(word, ROBOT_OBSTACLE_CONTROL_FAULT_SHIFT,
                                     ROBOT_OBSTACLE_CONTROL_FAULT_BITS, fault);
}

/**
 * @brief Encode a watch reading.
 * @param p_in Unpacked reading; the echo saturates at 15 bits.
 * @return Packed word.
 */
static inline uint32_t
robot_obstacle_watch_pack (robot_obstacle_watch_t const * p_in)
{
    uint32_t word = p_in->b_active ? ROBOT_WATCH_ACTIVE : 0u;

    word = robot_field_put(word, ROBOT_WATCH_ECHO_SHIFT, ROBOT_WATCH_ECHO_BITS,
                           p_in->echo_us);
    return robot_field_put(word, ROBOT_WATCH_SEQUENCE_SHIFT,
                           ROBOT_WATCH_SEQUENCE_BITS, p_in->sequence);
}

/**
 * @brief Decode a watch reading.
 * @param word Packed word.
 * @return Unpacked reading.
 */
static inline robot_obstacle_watch_t
robot_obstacle_watch_unpack (uint32_t word)
{
    robot_obstacle_watch_t out;

    out.sequence = (uint8_t)robot_field_get(word, ROBOT_WATCH_SEQUENCE_SHIFT,
                                            ROBOT_WATCH_SEQUENCE_BITS);
    out.echo_us = (uint16_t)robot_field_get(word, ROBOT_WATCH_ECHO_SHIFT,
                                            ROBOT_WATCH_ECHO_BITS);
    out.b_active = (word & ROBOT_WATCH_ACTIVE) != 0u;
    return out;
}

#endif /* ROBOT_TYPES_H */
