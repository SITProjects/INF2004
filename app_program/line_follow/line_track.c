/**
 * @file line_track.c
 * @brief Digital line tracking decisions: turn while black, corners,
 *        search.
 *
 * Sides are indexed 0 (left) and 1 (right). A black left sensor means the
 * tape has moved under it, so the car drifted right and must turn left:
 * turns and searches go towards the side that read black.
 *
 * While pivoting towards a side, the tape sweeps across that side's
 * sensor first and then sits between the sensors: black then white on the
 * search side is the line found. If the opposite sensor reads black
 * first, the tape was already between the sensors, so tracking resumes on
 * the heading the search started from.
 */

#include "line_track.h"
#include <stddef.h>
#include "line_follow_config.h"

/* Private types, constants, and macros. */
#define SIDE_LEFT 0u
#define SIDE_RIGHT 1u
#define SIDE_NONE 2u
#define SIDE_COUNT 2u

#define SEARCH_TIMEOUT_SAMPLES (LINE_SEARCH_TIMEOUT_MS / LINE_SAMPLE_MS)

_Static_assert(LINE_LEAD_DEG > 0, "Invalid lead");
_Static_assert(LINE_EDGE_MAX_MM > LINE_CROSS_MM,
               "A held edge must outlast a crossing");
_Static_assert(LINE_CONFIRM_SAMPLES > 0u, "Invalid debounce");
_Static_assert(LINE_SEARCH_DONE_DEG < LINE_SEARCH_MAX_DEG,
               "Search must turn further than its end band");
_Static_assert(LINE_MODE_STOP <= 3, "Line mode must fit two bits");

/* Private function declarations. */
static void debounce (line_track_t * p_track, line_track_input_t const * p_in);
static void restart_tracking (line_track_t * p_track,
                              line_track_input_t const * p_in,
                              int32_t target_deg);
static void track (line_track_t * p_track, line_track_input_t const * p_in);
static void track_edge (line_track_t * p_track, line_track_input_t const * p_in,
                        uint32_t side, bool b_was_black);
static void start_search (line_track_t * p_track,
                          line_track_input_t const * p_in, uint32_t side);
static void search (line_track_t * p_track, line_track_input_t const * p_in);
static void aim_search (line_track_t * p_track, uint32_t side);
static int32_t side_sign (uint32_t side);
static int32_t abs_deg (int32_t value);

/* Public function definitions. */

void
line_track_init (line_track_t * p_track, line_track_input_t const * p_in)
{
    for (uint32_t side = 0u; side < SIDE_COUNT; side++)
    {
        p_track->b_black[side] = false;
        p_track->change_count[side] = 0u;
    }
    p_track->last_side = SIDE_NONE;
    p_track->junction_count = 0u;
    restart_tracking(p_track, p_in, p_in->heading_deg);
}

line_request_t
line_track_update (line_track_t * p_track, line_track_input_t const * p_in)
{
    bool const was_black[SIDE_COUNT] = {p_track->b_black[SIDE_LEFT],
                                        p_track->b_black[SIDE_RIGHT]};
    debounce(p_track, p_in);

    bool const b_both =
        p_track->b_black[SIDE_LEFT] && p_track->b_black[SIDE_RIGHT];
    if (!b_both)
    {
        p_track->junction_count = 0u;
    }
    else if (p_track->junction_count < LINE_JUNCTION_SAMPLES)
    {
        p_track->junction_count++;
    }

    if (!p_in->b_control)
    {
        /* Someone else drives: start again from wherever it leaves us. */
        restart_tracking(p_track, p_in, p_in->heading_deg);
    }
    else if (LINE_MODE_TRACK == p_track->mode)
    {
        if (!p_track->b_black[SIDE_LEFT] && !p_track->b_black[SIDE_RIGHT])
        {
            if (was_black[SIDE_LEFT] || was_black[SIDE_RIGHT])
            {
                p_track->white_start_mm = p_in->distance_mm;
            }
        }
        if (b_both)
        {
            /* A crossing line: hold the heading until it passes, and do
             * not take its trailing edge for a corner. */
            if (!p_track->b_crossed_both)
            {
                p_track->target_deg = p_in->heading_deg;
            }
            p_track->b_crossed_both = true;
        }
        else
        {
            track_edge(p_track, p_in, SIDE_LEFT, was_black[SIDE_LEFT]);
            if (LINE_MODE_TRACK == p_track->mode)
            {
                track_edge(p_track, p_in, SIDE_RIGHT, was_black[SIDE_RIGHT]);
            }
            if (LINE_MODE_TRACK == p_track->mode)
            {
                track(p_track, p_in);
            }
        }
    }
    else if (LINE_MODE_STOP == p_track->mode)
    {
        /* Put back on the line by hand: track again. */
        if (p_track->b_black[SIDE_LEFT] || p_track->b_black[SIDE_RIGHT])
        {
            restart_tracking(p_track, p_in, p_in->heading_deg);
        }
    }
    else
    {
        search(p_track, p_in);
    }

    line_request_t const request = {
        .mode = p_track->mode,
        .target_heading_deg = p_track->target_deg,
        .b_seen = p_in->b_left_black || p_in->b_right_black,
        .b_junction = (p_track->junction_count >= LINE_JUNCTION_SAMPLES)};
    return request;
}

char const *
line_track_mode_name (line_mode_t mode)
{
    switch (mode)
    {
        case LINE_MODE_TRACK:
            return "TRACK";
        case LINE_MODE_SEARCH_LEFT:
            return "SEARCH_L";
        case LINE_MODE_SEARCH_RIGHT:
            return "SEARCH_R";
        case LINE_MODE_STOP:
            return "STOP";
        default:
            return "?";
    }
}

/* Private function definitions. */

/* A sensor's level changes after LINE_CONFIRM_SAMPLES samples agree. */
static void
debounce (line_track_t * p_track, line_track_input_t const * p_in)
{
    bool const raw[SIDE_COUNT] = {p_in->b_left_black, p_in->b_right_black};
    for (uint32_t side = 0u; side < SIDE_COUNT; side++)
    {
        if (raw[side] == p_track->b_black[side])
        {
            p_track->change_count[side] = 0u;
        }
        else if (++p_track->change_count[side] >= LINE_CONFIRM_SAMPLES)
        {
            p_track->b_black[side] = raw[side];
            p_track->change_count[side] = 0u;
        }
        else
        {
            /* Not confirmed yet. */
        }
    }
}

/* Track on @p target_deg with no edge episode in progress. */
static void
restart_tracking (line_track_t * p_track, line_track_input_t const * p_in,
                  int32_t target_deg)
{
    p_track->mode = LINE_MODE_TRACK;
    p_track->target_deg = target_deg;
    p_track->track_start_mm = p_in->distance_mm;
    p_track->white_start_mm = p_in->distance_mm;
    for (uint32_t side = 0u; side < SIDE_COUNT; side++)
    {
        p_track->black_start_mm[side] = p_in->distance_mm;
    }
    p_track->search_samples = 0u;
    p_track->b_search_hit = false;
    p_track->b_crossed_both = false;
}

/* Both white for too long: the line went away; search for it. */
static void
track (line_track_t * p_track, line_track_input_t const * p_in)
{
    bool const b_white =
        !p_track->b_black[SIDE_LEFT] && !p_track->b_black[SIDE_RIGHT];
    if (b_white)
    {
        p_track->b_crossed_both = false;
    }
    if (b_white && (p_track->last_side != SIDE_NONE) &&
        ((p_in->distance_mm - p_track->white_start_mm) >= LINE_LOST_MM))
    {
        start_search(p_track, p_in, p_track->last_side);
    }
}

/* One side's edge while tracking: turn towards it, or spot a corner. */
static void
track_edge (line_track_t * p_track, line_track_input_t const * p_in,
            uint32_t side, bool b_was_black)
{
    bool const b_black = p_track->b_black[side];
    if (b_black && !b_was_black)
    {
        p_track->black_start_mm[side] = p_in->distance_mm;
        p_track->last_side = side;
    }
    if (b_black)
    {
        if ((p_in->distance_mm - p_track->black_start_mm[side]) >=
            LINE_EDGE_MAX_MM)
        {
            /* Turning is not bringing the tape back: a tight bend. */
            start_search(p_track, p_in, side);
        }
        else
        {
            p_track->target_deg =
                p_in->heading_deg + (side_sign(side) * LINE_LEAD_DEG);
        }
    }
    else if (b_was_black)
    {
        bool const b_quick =
            (p_in->distance_mm - p_track->black_start_mm[side]) < LINE_CROSS_MM;
        bool const b_moved =
            (p_in->distance_mm - p_track->track_start_mm) >= LINE_CROSS_MM;
        if (!p_track->b_crossed_both && b_quick && b_moved)
        {
            /* Crossed the tape: it turns away towards this side. */
            start_search(p_track, p_in, side);
        }
        else
        {
            /* The tape is back between the sensors: hold this heading. */
            p_track->target_deg = p_in->heading_deg;
        }
    }
    else
    {
        /* White and was white. */
    }
}

static void
start_search (line_track_t * p_track, line_track_input_t const * p_in,
              uint32_t side)
{
    p_track->search_start_deg = p_in->heading_deg;
    p_track->search_phase = 0u;
    p_track->search_samples = 0u;
    p_track->last_side = side;
    aim_search(p_track, side);
}

/* Pivot towards @p side, LINE_SEARCH_MAX_DEG past the search start. */
static void
aim_search (line_track_t * p_track, uint32_t side)
{
    p_track->search_side = side;
    p_track->b_search_hit = false;
    p_track->mode =
        (SIDE_LEFT == side) ? LINE_MODE_SEARCH_LEFT : LINE_MODE_SEARCH_RIGHT;
    p_track->target_deg =
        p_track->search_start_deg + (side_sign(side) * LINE_SEARCH_MAX_DEG);
}

static void
search (line_track_t * p_track, line_track_input_t const * p_in)
{
    uint32_t const side = p_track->search_side;
    uint32_t const other = (SIDE_LEFT == side) ? SIDE_RIGHT : SIDE_LEFT;

    p_track->search_samples++;
    if (p_track->b_black[side])
    {
        p_track->b_search_hit = true;
    }
    else if (p_track->b_search_hit && !p_track->b_black[other])
    {
        /* The tape swept across this side's sensor and is between them. */
        restart_tracking(p_track, p_in, p_in->heading_deg);
        p_track->last_side = side;
        return;
    }
    else
    {
        /* Not found yet. */
    }
    if (p_track->b_black[other] && !p_track->b_search_hit)
    {
        /* The tape was between the sensors all along: go back to it. */
        restart_tracking(p_track, p_in, p_track->search_start_deg);
        return;
    }

    bool const b_reached =
        !p_in->b_turning && (abs_deg(p_track->target_deg - p_in->heading_deg) <=
                             LINE_SEARCH_DONE_DEG);
    if (p_track->search_samples >= SEARCH_TIMEOUT_SAMPLES)
    {
        p_track->mode = LINE_MODE_STOP;
        p_track->target_deg = p_in->heading_deg;
    }
    else if (b_reached && (0u == p_track->search_phase))
    {
        p_track->search_phase = 1u;
        aim_search(p_track, other);
    }
    else if (b_reached)
    {
        p_track->mode = LINE_MODE_STOP;
        p_track->target_deg = p_in->heading_deg;
    }
    else
    {
        /* Still pivoting. */
    }
}

/* Heading change of a step towards @p side: + is left. */
static int32_t
side_sign (uint32_t side)
{
    return (SIDE_LEFT == side) ? 1 : -1;
}

static int32_t
abs_deg (int32_t value)
{
    return (value < 0) ? -value : value;
}
