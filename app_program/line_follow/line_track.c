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
 * the heading the search started from. A junction pivot ignores the
 * sensors until it nears the new line, so the line it leaves is not taken
 * for it.
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

#define JUNCTION_NONE 0u
#define JUNCTION_DRIVE 1u /* Holding the heading to the turning point. */
#define JUNCTION_PIVOT 2u /* Pivoting onto the new line. */
#define JUNCTION_CROSS 3u /* Straight on: tracking while crossing it. */
#define JUNCTION_TURN_DEG 90
#define JUNCTION_UTURN_DEG 180

_Static_assert(LINE_LEAD_DEG > 0, "Invalid lead");
_Static_assert(LINE_EDGE_MAX_MM > 0, "Invalid held-edge limit");
_Static_assert((LINE_SEEK_MAX_DEG > 0) && (LINE_SEEK_MAX_DEG < LINE_LEAD_DEG) &&
                   (LINE_SEEK_RAMP_MM > 0),
               "The seek must turn more gently than a black edge");
_Static_assert(LINE_LOST_MM > LINE_SEEK_RAMP_MM,
               "Seek at full rate before searching");
_Static_assert(LINE_CONFIRM_SAMPLES > 0u, "Invalid debounce");
_Static_assert(LINE_SEARCH_DONE_DEG < LINE_SEARCH_MAX_DEG,
               "Search must turn further than its end band");
_Static_assert(LINE_MODE_STOP <= 3, "Line mode must fit two bits");
_Static_assert((LINE_JUNCTION_LOOK_DEG > 0) &&
                   (LINE_JUNCTION_LOOK_DEG < JUNCTION_TURN_DEG),
               "A junction pivot must look near the new line only");
_Static_assert((LINE_JUNCTION_SKEW_MM >= 0) && (LINE_JUNCTION_CENTRE_MM > 0),
               "Invalid junction geometry");

/* Private function declarations. */
static void debounce (line_track_t * p_track, line_track_input_t const * p_in);
static void restart_tracking (line_track_t * p_track,
                              line_track_input_t const * p_in,
                              int32_t target_deg);
static void track (line_track_t * p_track, line_track_input_t const * p_in);
static void follow_episode (line_track_t * p_track,
                            line_track_input_t const * p_in,
                            bool const * p_was_black);
static void start_episode (line_track_t * p_track,
                           line_track_input_t const * p_in);
static void start_junction (line_track_t * p_track,
                            line_track_input_t const * p_in);
static void drive_junction (line_track_t * p_track,
                            line_track_input_t const * p_in);
static int32_t seek_target (line_track_t const * p_track,
                            line_track_input_t const * p_in);
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
    for (uint32_t side = 0u; side < SIDE_COUNT; side++)
    {
        /* Here, not in track_edge(): an edge that turns black in the same
         * sample as the other sensor must restart its held-edge travel.
         * Only one side black on its own counts as held: the tape can sit
         * under both for a long way (2026-11-06 car9, 143 mm). */
        if (b_both || (p_track->b_black[side] && !was_black[side]))
        {
            p_track->black_start_mm[side] = p_in->distance_mm;
        }
    }
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
    else if (JUNCTION_DRIVE == p_track->junction_phase)
    {
        drive_junction(p_track, p_in);
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
        follow_episode(p_track, p_in, was_black);
        if ((LINE_MODE_TRACK != p_track->mode) ||
            (JUNCTION_DRIVE == p_track->junction_phase))
        {
            /* A junction or a search took over. */
        }
        else if (b_both)
        {
            /* The tape across both sensors (they are about a tape width
             * apart): centred, hold the heading until one clears. */
            p_track->target_deg = p_in->heading_deg;
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
        .b_junction = (p_track->junction_phase != JUNCTION_NONE)};
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

char const *
line_track_turn_name (line_turn_t turn)
{
    switch (turn)
    {
        case LINE_TURN_STRAIGHT:
            return "STRAIGHT";
        case LINE_TURN_LEFT:
            return "LEFT";
        case LINE_TURN_RIGHT:
            return "RIGHT";
        case LINE_TURN_UTURN:
            return "UTURN";
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
    p_track->white_start_mm = p_in->distance_mm;
    for (uint32_t side = 0u; side < SIDE_COUNT; side++)
    {
        p_track->black_start_mm[side] = p_in->distance_mm;
    }
    p_track->search_samples = 0u;
    p_track->b_search_hit = false;
    p_track->search_ignore_deg = 0;
    p_track->junction_phase = JUNCTION_NONE;
    start_episode(p_track, p_in);
}

/*
 * Follow one dark episode, from a sensor turning black until both read
 * white again, and spot a junction. The tape crossing from one sensor to
 * the other is not a reason to search: with the sensors about a tape
 * width apart, a correction that turns a little too far does that, and
 * the tape is then back between them (2026-11-06 car7: a search towards
 * the side that cleared last pivoted 72 degrees away from the tape).
 */
static void
follow_episode (line_track_t * p_track, line_track_input_t const * p_in,
                bool const * p_was_black)
{
    bool const b_left = p_track->b_black[SIDE_LEFT];
    bool const b_right = p_track->b_black[SIDE_RIGHT];
    bool const b_any = b_left || b_right;
    bool const b_was_any = p_was_black[SIDE_LEFT] || p_was_black[SIDE_RIGHT];

    if (b_any && !b_was_any)
    {
        start_episode(p_track, p_in);
    }
    if (!b_any && (JUNCTION_CROSS == p_track->junction_phase))
    {
        /* Over the crossing line. */
        p_track->junction_phase = JUNCTION_NONE;
    }
    if (b_left && b_right)
    {
        if (!p_track->b_both_seen)
        {
            p_track->b_both_seen = true;
            p_track->b_skew_small = (SIDE_NONE == p_track->entry_side) ||
                                    ((p_in->distance_mm - p_track->entry_mm) <=
                                     LINE_JUNCTION_SKEW_MM);
        }
        if (p_track->b_skew_small &&
            (JUNCTION_NONE == p_track->junction_phase) &&
            (p_track->junction_count >= LINE_JUNCTION_SAMPLES))
        {
            start_junction(p_track, p_in);
        }
    }
}

/* Start a dark episode from the levels now. */
static void
start_episode (line_track_t * p_track, line_track_input_t const * p_in)
{
    bool const b_left = p_track->b_black[SIDE_LEFT];
    bool const b_right = p_track->b_black[SIDE_RIGHT];
    if (b_left == b_right)
    {
        p_track->entry_side = SIDE_NONE;
    }
    else
    {
        p_track->entry_side = b_left ? SIDE_LEFT : SIDE_RIGHT;
    }
    p_track->entry_mm = p_in->distance_mm;
    p_track->entry_deg = p_in->heading_deg;
    p_track->b_both_seen = false;
    p_track->b_skew_small = false;
}

/*
 * A cross junction. Straight on, tracking carries on: with the sensors
 * about a tape width apart, a plain line under both reads the same, and
 * holding a heading would leave a bend (2026-11-06). A turn holds the
 * heading the junction was met on to the turning point.
 */
static void
start_junction (line_track_t * p_track, line_track_input_t const * p_in)
{
    p_track->junction_turn = p_in->turn;
    if ((LINE_TURN_LEFT != p_in->turn) && (LINE_TURN_RIGHT != p_in->turn) &&
        (LINE_TURN_UTURN != p_in->turn))
    {
        p_track->junction_phase = JUNCTION_CROSS;
        return;
    }
    p_track->junction_phase = JUNCTION_DRIVE;
    p_track->junction_turn = p_in->turn;
    p_track->junction_mm = p_track->entry_mm;
    p_track->junction_deg = p_track->entry_deg;
    p_track->target_deg = p_track->entry_deg;
}

/*
 * Drive over the junction on its heading, sensors ignored; at the turning
 * point go straight on, or pivot onto the new line.
 */
static void
drive_junction (line_track_t * p_track, line_track_input_t const * p_in)
{
    p_track->target_deg = p_track->junction_deg;
    if ((p_in->distance_mm - p_track->junction_mm) < LINE_JUNCTION_CENTRE_MM)
    {
        return;
    }
    int32_t turn_deg = JUNCTION_TURN_DEG;
    uint32_t side = SIDE_LEFT;
    switch (p_track->junction_turn)
    {
        case LINE_TURN_LEFT:
            break;
        case LINE_TURN_RIGHT:
            side = SIDE_RIGHT;
            break;
        case LINE_TURN_UTURN:
            turn_deg = JUNCTION_UTURN_DEG;
            side = (LINE_UTURN_LEFT != 0) ? SIDE_LEFT : SIDE_RIGHT;
            break;
        default:
            /* Not reached: straight on never drives to the centre. */
            restart_tracking(p_track, p_in, p_track->junction_deg);
            return;
    }
    p_track->junction_phase = JUNCTION_PIVOT;
    p_track->search_start_deg = p_in->heading_deg;
    p_track->search_phase = 1u; /* Not found: stop, no other side. */
    p_track->search_samples = 0u;
    p_track->search_ignore_deg = turn_deg - LINE_JUNCTION_LOOK_DEG;
    p_track->last_side = side;
    p_track->search_side = side;
    p_track->b_search_hit = false;
    p_track->mode =
        (SIDE_LEFT == side) ? LINE_MODE_SEARCH_LEFT : LINE_MODE_SEARCH_RIGHT;
    p_track->target_deg =
        p_track->search_start_deg +
        (side_sign(side) * (turn_deg + LINE_JUNCTION_LOOK_DEG));
}

/* Both white: seek towards the last edge; for too long, search. */
static void
track (line_track_t * p_track, line_track_input_t const * p_in)
{
    bool const b_white =
        !p_track->b_black[SIDE_LEFT] && !p_track->b_black[SIDE_RIGHT];
    if (b_white)
    {
        p_track->target_deg = seek_target(p_track, p_in);
    }
    if (b_white && (p_track->last_side != SIDE_NONE) &&
        ((p_in->distance_mm - p_track->white_start_mm) >= LINE_LOST_MM))
    {
        start_search(p_track, p_in, p_track->last_side);
    }
}

/*
 * The heading to aim for while both sensors read white. White on both
 * means the tape is centred or has just slipped out past the last edge,
 * and the two look the same (2026-11-06 car10: the left sensor touched a
 * left curve for 59 mm, then nothing saw the tape again). Keep turning
 * towards the last edge, harder the longer both stay white: a centred
 * tape soon reaches the other sensor and turns the car back; a slipped
 * one is caught again instead of the car driving straight off.
 */
static int32_t
seek_target (line_track_t const * p_track, line_track_input_t const * p_in)
{
    if (SIDE_NONE == p_track->last_side)
    {
        return p_in->heading_deg;
    }
    int32_t lead =
        ((p_in->distance_mm - p_track->white_start_mm) * LINE_SEEK_MAX_DEG) /
        LINE_SEEK_RAMP_MM;
    if (lead > LINE_SEEK_MAX_DEG)
    {
        lead = LINE_SEEK_MAX_DEG;
    }
    else if (lead < 0)
    {
        lead = 0;
    }
    else
    {
        /* Ramping up. */
    }
    return p_in->heading_deg + (side_sign(p_track->last_side) * lead);
}

/* One side's edge while tracking: turn towards it, or a tight bend. */
static void
track_edge (line_track_t * p_track, line_track_input_t const * p_in,
            uint32_t side, bool b_was_black)
{
    bool const b_black = p_track->b_black[side];
    if (b_black)
    {
        p_track->last_side = side;
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
        /* However briefly it was black, this was a correction, not a
         * corner: seek from here. */
        p_track->target_deg = seek_target(p_track, p_in);
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
    p_track->search_ignore_deg = 0;
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
    bool const b_looking =
        abs_deg(p_in->heading_deg - p_track->search_start_deg) >=
        p_track->search_ignore_deg;
    if (!b_looking)
    {
        /* A junction pivot still crossing the line it is leaving. */
    }
    else if (p_track->b_black[side])
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
    if (b_looking && (0 == p_track->search_ignore_deg) &&
        p_track->b_black[other] && !p_track->b_search_hit)
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
        p_track->junction_phase = JUNCTION_NONE;
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
        p_track->junction_phase = JUNCTION_NONE;
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
