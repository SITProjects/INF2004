/**
 * @file line_track.h
 * @brief Digital line tracking decisions: heading steps, corners, search.
 *
 * Two sensors straddle the tape. While both read white the car holds its
 * heading. A confirmed black edge moves the target heading one step
 * towards that side, and motion turns by it while driving, then drives
 * straight again. A sensor that crosses the tape quickly (a corner), an
 * edge that stays black through LINE_STEP_MAX steps, or a long white run
 * starts a pivot search towards the last edge.
 *
 * The module is pure: no kernel calls, so host tests can drive it. Only
 * the line task calls it.
 */

#ifndef LINE_TRACK_H
#define LINE_TRACK_H

/* Public includes. */
#include <stdbool.h>
#include <stdint.h>

/* Public types, constants, and macros. */

/** @brief What the line task asks motion to do. Fits 2 bits. */
typedef enum
{
    LINE_MODE_TRACK = 0,    /**< Drive at follow speed on the target. */
    LINE_MODE_SEARCH_LEFT,  /**< Pivot left towards the target heading. */
    LINE_MODE_SEARCH_RIGHT, /**< Pivot right towards the target heading. */
    LINE_MODE_STOP          /**< Line not found: brake and wait. */
} line_mode_t;

/** @brief One decision for motion (IF-04). */
typedef struct
{
    line_mode_t mode;           /**< See line_mode_t. */
    int32_t target_heading_deg; /**< Odometry heading to hold or pivot
                                     to; not wrapped, + is left. */
    bool b_seen;                /**< Either sensor black in this sample,
                                     for stop-at-line (motion debounces). */
    bool b_junction;            /**< Both sensors black, confirmed. */
} line_request_t;

/** @brief One sample and the motion state it was taken in. */
typedef struct
{
    bool b_left_black;   /**< Left sensor over the tape. */
    bool b_right_black;  /**< Right sensor over the tape. */
    int32_t distance_mm; /**< Odometry travel, + forward. */
    int32_t heading_deg; /**< Odometry heading, not wrapped, + left. */
    bool b_control;      /**< Motion follows line requests (not a queued
                              command, inhibit or bypass). */
    bool b_turning;      /**< Motion is pivoting (mode TURN). */
} line_track_input_t;

/** @brief Tracking state. Callers allocate it; fields are private. */
typedef struct
{
    bool b_black[2];          /* Confirmed levels: [0] left, [1] right. */
    uint32_t change_count[2]; /* Samples disagreeing with b_black. */
    line_mode_t mode;
    int32_t target_deg;
    int32_t black_start_mm[2]; /* Travel when each edge turned black. */
    int32_t step_mm;           /* Travel at the last step. */
    uint32_t steps;            /* Steps in the current edge episode. */
    uint32_t step_side;        /* Side of that episode; 2 = none. */
    uint32_t last_side;        /* Side of the last edge, for searches. */
    int32_t white_start_mm;    /* Travel when both last read white. */
    uint32_t junction_count;
    bool b_crossed_both; /* Both read black since both last read white. */
    int32_t search_start_deg;
    uint32_t search_side;  /* Side the current pivot turns towards. */
    uint32_t search_phase; /* 0 first side, 1 the other side. */
    uint32_t search_samples;
    bool b_search_hit; /* Search-side sensor has read black. */
} line_track_t;

/* Public function declarations. */

/**
 * @brief Start tracking on the current heading.
 * @param p_track State to reset.
 * @param p_in Current sample, for the heading and travel.
 */
void line_track_init (line_track_t * p_track, line_track_input_t const * p_in);

/**
 * @brief Take one sample (every LINE_SAMPLE_MS) and decide.
 * @param p_track State from line_track_init().
 * @param p_in This sample. While b_control is false, tracking restarts on
 *        the current heading every sample, so a bypass or queued command
 *        hands back cleanly.
 * @return The request to submit to motion.
 */
line_request_t line_track_update (line_track_t * p_track,
                                  line_track_input_t const * p_in);

/** @brief Short name of @p mode for logs ("TRACK", "SEARCH_L", ...). */
char const * line_track_mode_name (line_mode_t mode);

#endif /* LINE_TRACK_H */
