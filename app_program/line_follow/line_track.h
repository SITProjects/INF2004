/**
 * @file line_track.h
 * @brief Digital line tracking decisions: turn while black, corners,
 *        search.
 *
 * Two sensors straddle the tape. While one reads black the target heading
 * leads the car by LINE_LEAD_DEG towards it, so motion keeps turning
 * while driving. Once it clears the car holds the heading it reached, so
 * it runs straight and barcodes read cleanly. The same sensor touching
 * again within LINE_CURVE_TOUCH_MM means a bend: then, while both read
 * white, the car keeps turning towards the last edge (up to
 * LINE_SEEK_MAX_DEG of lead after LINE_SEEK_RAMP_MM), so a tape that
 * slipped out is caught again, until the other sensor touches. So does the
 * tape crossing both sensors and leaving on the side the car has been
 * turning towards (LINE_BEND_TREND_DEG). A sensor that
 * reads black only briefly is a correction, never a corner: turning
 * towards it clears it quickly (2026-11-06). An edge that stays black for
 * LINE_EDGE_MAX_MM, or a long white run after an edge (LINE_LOST_MM in a
 * bend, LINE_STRAIGHT_LOST_MM otherwise), starts a pivot search towards
 * the last edge.
 *
 * Both sensors turning black together (within LINE_JUNCTION_SKEW_MM) is a
 * cross junction. Straight on, the car holds its heading over the crossing
 * tape (LINE_JUNCTION_HOLD_MM), then tracking carries on; for a turn the
 * car drives LINE_JUNCTION_CENTRE_MM on, pivots exactly 90 or 180 degrees
 * and drives on along that heading. Both
 * black one after the other is the tape crossing the sensors at an angle,
 * usually a correction that turned a little too far: tracking carries on.
 *
 * When the barcode reader sees a barcode coming, the car brakes, backs up
 * LINE_BARCODE_REVERSE_MM to before it, and tracks on at the slow speed until
 * the reader is done. A bend, junction or search refuses this.
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

/** Headings kept to see how far the car has turned lately. */
#define LINE_TREND_SLOTS 3u

/** @brief What the line task asks motion to do. Fits 2 bits. */
typedef enum
{
    LINE_MODE_TRACK = 0,    /**< Drive at follow speed on the target. */
    LINE_MODE_SEARCH_LEFT,  /**< Pivot left towards the target heading. */
    LINE_MODE_SEARCH_RIGHT, /**< Pivot right towards the target heading. */
    LINE_MODE_STOP          /**< Line not found: brake and wait. */
} line_mode_t;

/** @brief What to do at the next cross junction (from a barcode). */
typedef enum
{
    LINE_TURN_STRAIGHT = 0, /**< Drive across (the default). */
    LINE_TURN_LEFT,         /**< Pivot left onto the crossing line. */
    LINE_TURN_RIGHT,        /**< Pivot right onto the crossing line. */
    LINE_TURN_UTURN         /**< Pivot round onto the route behind. */
} line_turn_t;

/** @brief Where a slow pass over a barcode has got to. */
typedef enum
{
    LINE_BARCODE_NONE = 0, /**< Normal speed. */
    LINE_BARCODE_BRAKE,    /**< Stopping. */
    LINE_BARCODE_BACK,     /**< Backing up to before the barcode. */
    LINE_BARCODE_CREEP     /**< Tracking slowly over it. */
} line_barcode_phase_t;

/** @brief One decision for motion (IF-04). */
typedef struct
{
    line_mode_t mode;           /**< See line_mode_t. */
    int32_t target_heading_deg; /**< Odometry heading to hold or pivot
                                     to; not wrapped, + is left. */
    bool b_seen;                /**< Either sensor black in this sample,
                                     for stop-at-line (motion debounces). */
    bool b_junction;            /**< A cross junction is being taken: from
                                     its detection until both sensors
                                     clear (straight on) or tracking
                                     resumes on the new line (a turn). */
    uint16_t reverse_mm;        /**< Above 0: drive this far backward, once,
                                     in place of the mode. */
    bool b_slow;                /**< Track at the barcode speed. */
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
    bool b_moving;       /**< Motion is on a distance move (mode MOVE). */
    line_turn_t turn;    /**< Instruction for the next junction. */
    bool b_barcode;      /**< A barcode has just come into view. */
    bool b_barcode_done; /**< The reader has finished with it. */
    bool b_hump;         /**< The terrain task reports a hump. */
} line_track_input_t;

/** @brief Tracking state. Callers allocate it; fields are private. */
typedef struct
{
    bool b_black[2];          /* Confirmed levels: [0] left, [1] right. */
    uint32_t change_count[2]; /* Samples disagreeing with b_black. */
    line_mode_t mode;
    int32_t target_deg;
    int32_t black_start_mm[2]; /* Travel when each edge turned black. */
    uint32_t touch_side;       /* Side of the last single-side touch. */
    int32_t touch_mm;          /* Travel when it began. */
    bool b_curve;              /* That side touched again soon: a bend. */
    uint32_t counter_side;     /* Side steering back against a bend. */
    int32_t counter_mm;        /* Travel when that touch began. */
    bool b_cross_hold;         /* Holding the heading over a crossing. */
    int32_t trend_deg[LINE_TREND_SLOTS]; /* Recent headings, oldest at next. */
    uint32_t trend_next;
    int32_t trend_mm; /* Travel at the latest of them. */
    /* Dark episode: from a sensor turning black until both read white. */
    uint32_t entry_side;     /* First side black; 2 = both at once. */
    int32_t entry_mm;        /* Travel when the episode started. */
    int32_t entry_deg;       /* Heading then. */
    int32_t entry_skew_mm;   /* Travel from then until both were black. */
    bool b_both_seen;        /* Both black during this episode. */
    bool b_skew_small;       /* ... within LINE_JUNCTION_SKEW_MM. */
    uint32_t junction_phase; /* 0 none, 1 driving to the centre, 2 pivot,
                                3 crossing straight on. */
    line_turn_t junction_turn;
    int32_t junction_mm;       /* Travel when the junction began. */
    int32_t junction_deg;      /* Heading the junction was met on. */
    int32_t junction_skew_deg; /* The line is this far left of the heading. */
    uint32_t last_side;        /* Side of the last edge, for searches. */
    int32_t white_start_mm;    /* Travel when both last read white. */
    uint32_t junction_count;
    int32_t search_start_deg;
    uint32_t search_side;  /* Side the current pivot turns towards. */
    uint32_t search_phase; /* 0 first side, 1 the other side. */
    uint32_t search_samples;
    bool b_search_hit; /* Search-side sensor has read black. */
    line_barcode_phase_t barcode_phase;
    bool b_hump_slow;         /* Following slowly over a hump. */
    bool b_hump_spent;        /* That ran its limit: wait for level ground. */
    uint32_t hump_samples;    /* Samples spent slow on this hump. */
    uint32_t barcode_samples; /* Samples in this phase. */
    bool b_back_started;      /* The reverse has begun. */
    int32_t barcode_mm;       /* Travel when the creep began. */
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

/**
 * @brief Where a slow pass over a barcode has got to.
 * @param p_track State from line_track_init().
 * @return The current phase.
 */
line_barcode_phase_t line_track_barcode_phase (line_track_t const * p_track);

/**
 * @brief Whether the car is following slowly over a hump.
 * @param p_track State.
 * @return True while it is.
 */
bool line_track_hump_slow (line_track_t const * p_track);

/** @brief Short name of @p phase for logs ("BRAKE", "BACK", ...). */
char const * line_track_barcode_name (line_barcode_phase_t phase);

/** @brief Short name of @p mode for logs ("TRACK", "SEARCH_L", ...). */
char const * line_track_mode_name (line_mode_t mode);

/** @brief Short name of @p turn for logs ("STRAIGHT", "LEFT", ...). */
char const * line_track_turn_name (line_turn_t turn);

#endif /* LINE_TRACK_H */
