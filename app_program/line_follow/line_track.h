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
 * the reader is done, then steers the same way up to the crossing. A bend,
 * junction or search refuses this.
 *
 * The module is pure: no kernel calls, so host tests can drive it. Only
 * the line task calls it.
 */

#ifndef LINE_TRACK_H
#define LINE_TRACK_H

/* Public includes. */
#include <stdbool.h>
#include <stdint.h>
#include "../robot_types.h"

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
                                     for stop-at-line (motion debounces).
                                     On a distance move, only the sensor on
                                     the side the car last pivoted to. */
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

/* line_track_t.flags, one bit per fact. */
#define LINE_TRACK_F_BLACK_LEFT 0x0001u    /* Confirmed level, left. */
#define LINE_TRACK_F_BLACK_RIGHT 0x0002u   /* Confirmed level, right. */
#define LINE_TRACK_F_CURVE 0x0004u         /* A side touched again soon. */
#define LINE_TRACK_F_CROSS_HOLD 0x0008u    /* Heading held over a crossing. */
#define LINE_TRACK_F_BOTH_SEEN 0x0010u     /* Both black this dark episode. */
#define LINE_TRACK_F_SKEW_SMALL 0x0020u    /* ... within the junction skew. */
#define LINE_TRACK_F_OLD_LINE_SEEN 0x0040u /* The pivot swept the old line. */
#define LINE_TRACK_F_REJOIN 0x0080u        /* After a bypass: find the line. */
#define LINE_TRACK_F_SEARCH_HIT 0x0100u    /* Search-side sensor read black. */
#define LINE_TRACK_F_HUMP_SLOW 0x0200u     /* Following slowly over a hump. */
#define LINE_TRACK_F_HUMP_SPENT 0x0400u    /* Ran its limit: await level. */
#define LINE_TRACK_F_BACK_STARTED 0x0800u  /* The reverse has begun. */
#define LINE_TRACK_F_RETRACE 0x1000u /* Going back past it after U-turn. */
#define LINE_TRACK_F_SETTLE 0x2000u  /* From a read to its crossing. */

/* line_track_t.fields, each "shift, bits": 30 of 32 bits. */
#define LINE_TRACK_FIELD_MODE 0u, 2u
#define LINE_TRACK_FIELD_TOUCH_SIDE 2u, 2u   /* Last single-side touch. */
#define LINE_TRACK_FIELD_COUNTER_SIDE 4u, 2u /* Steering back on a bend. */
#define LINE_TRACK_FIELD_ENTRY_SIDE 6u, 2u   /* First black; 2 = both. */
#define LINE_TRACK_FIELD_PIVOT_SIDE 8u, 2u   /* Way it last pivoted. */
#define LINE_TRACK_FIELD_BARS_SIDE 10u, 2u   /* On a barcode's bars. */
#define LINE_TRACK_FIELD_LAST_SIDE 12u, 2u   /* Side of the last edge. */
#define LINE_TRACK_FIELD_SEARCH_SIDE 14u, 2u /* Current pivot's side. */
#define LINE_TRACK_FIELD_JUNCTION_PHASE                                        \
    16u, 2u /* 0 none, 1 to the centre,
                                                   2 pivot, 3 straight on. */
#define LINE_TRACK_FIELD_JUNCTION_TURN 18u, 2u
#define LINE_TRACK_FIELD_SEARCH_PHASE 20u, 1u /* 0 first side, 1 the other. */
#define LINE_TRACK_FIELD_BARCODE_PHASE 21u, 2u
#define LINE_TRACK_FIELD_TREND_NEXT 23u, 2u
#define LINE_TRACK_FIELD_JUNCTION_COUNT 25u, 1u
#define LINE_TRACK_CHANGE_SHIFT 26u /* Samples disagreeing, per side. */
#define LINE_TRACK_CHANGE_BITS 2u

/**
 * @brief Tracking state. Callers allocate it; fields are private.
 *
 * Distance marks keep the low 16 bits of the travel: compare them through
 * the module's wrap-safe helpers. Headings are odometry degrees, which
 * IF-05 limits to 16 bits.
 */
typedef struct
{
    uint32_t fields; /* Small values, LINE_TRACK_FIELD_*. */
    uint16_t flags;  /* LINE_TRACK_F_*. */
    int16_t target_deg;
    int16_t black_start_mm[2]; /* Travel when each edge turned black. */
    int16_t touch_mm;          /* Travel when the last touch began. */
    int16_t counter_mm;        /* Travel when the counter touch began. */
    int16_t trend_deg[LINE_TREND_SLOTS]; /* Recent headings, oldest at next. */
    int16_t trend_mm;                    /* Travel at the latest of them. */
    /* Dark episode: from a sensor turning black until both read white. */
    int16_t entry_mm;         /* Travel when the episode started. */
    int16_t entry_deg;        /* Heading then. */
    int16_t junction_mm;      /* Travel when the junction began. */
    int16_t junction_deg;     /* Heading the junction was met on. */
    int16_t pivot_start_deg;  /* Heading as a junction's pivot began. */
    int16_t last_heading_deg; /* Heading in the last sample. */
    int16_t rejoin_mm;        /* Travel when it last set out to rejoin. */
    int16_t lost_start_mm;    /* Travel when it last saw the line itself. */
    int16_t loop_mm;          /* Travel at the start of this loop check. */
    int16_t loop_deg;         /* Heading then. */
    int16_t white_start_mm;   /* Travel when both last read white. */
    int16_t search_start_deg;
    uint16_t search_samples;
    uint16_t hump_samples;   /* Samples spent slow on this hump. */
    uint8_t barcode_samples; /* Samples in this phase. */
    int16_t barcode_mm;      /* Travel when the creep began. */
    int16_t line_deg;        /* The line's heading, held in a creep. */
    int16_t brake_mm;        /* Travel when a barcode last braked the car. */
    int16_t retrace_end_mm;  /* Travel at which the retrace ends. */
    int16_t settle_end_mm;   /* Travel at which the settle ends. */
} line_track_t;

/**
 * @brief Read one LINE_TRACK_F_* fact.
 * @param p_track Tracking state.
 * @param flag One LINE_TRACK_F_* bit.
 * @return true if that fact holds.
 */
static inline bool
line_track_flag (line_track_t const * p_track, uint32_t flag)
{
    return (p_track->flags & flag) != 0u;
}

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

/**
 * @brief Whether the car is driving back past a barcode after its U-turn.
 * @param p_track State.
 * @return True while it is.
 */
bool line_track_retrace (line_track_t const * p_track);

/** @brief Short name of @p phase for logs ("BRAKE", "BACK", ...). */
char const * line_track_barcode_name (line_barcode_phase_t phase);

/** @brief Short name of @p mode for logs ("TRACK", "SEARCH_L", ...). */
char const * line_track_mode_name (line_mode_t mode);

/** @brief Short name of @p turn for logs ("STRAIGHT", "LEFT", ...). */
char const * line_track_turn_name (line_turn_t turn);

#endif /* LINE_TRACK_H */
