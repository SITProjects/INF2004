/**
 * @file robot_types.h
 * @brief Shared obstacle result encoding for application tasks and telemetry.
 *
 * Sweep geometry is part of the wire contract: angle = first + step * index.
 * Changing it requires updating the MQTT decoder with the team.
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

/* Shared angular grid used to interpret packed edge indices. */
#define ROBOT_OBSTACLE_SWEEP_FIRST_DEG 60u
#define ROBOT_OBSTACLE_SWEEP_LAST_DEG 120u
#define ROBOT_OBSTACLE_SWEEP_STEP_DEG 5u
#define ROBOT_OBSTACLE_SWEEP_POINTS                                            \
    (((ROBOT_OBSTACLE_SWEEP_LAST_DEG - ROBOT_OBSTACLE_SWEEP_FIRST_DEG) /       \
      ROBOT_OBSTACLE_SWEEP_STEP_DEG) +                                         \
     1u)

#define ROBOT_OBSTACLE_STATUS_MASK 0x07u
#define ROBOT_OBSTACLE_DIRECTION_MASK 0x18u
#define ROBOT_OBSTACLE_DIRECTION_SHIFT 3u
#define ROBOT_OBSTACLE_LEFT_EDGE_FOUND 0x20u
#define ROBOT_OBSTACLE_RIGHT_EDGE_FOUND 0x40u

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
    /** Status 0..2, direction 3..4, left/right edge flags 5/6; bit 7 zero. */
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
 * @brief Encode a flags byte, leaving reserved bit 7 zero.
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

#endif /* ROBOT_TYPES_H */
