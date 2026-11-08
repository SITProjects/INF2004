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
 * sensors and turns exactly 90 or 180 degrees.
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
_Static_assert((LINE_STRAIGHT_LOST_MM >= LINE_LOST_MM) &&
                   (LINE_LOST_MM > LINE_SEEK_RAMP_MM),
               "Seek at full rate before searching");
_Static_assert(LINE_CURVE_TOUCH_MM > 0, "Invalid bend window");
_Static_assert(LINE_CONFIRM_SAMPLES > 0u, "Invalid debounce");
_Static_assert(LINE_SEARCH_DONE_DEG < LINE_SEARCH_MAX_DEG,
               "Search must turn further than its end band");
_Static_assert(LINE_MODE_STOP <= 3, "Line mode must fit two bits");
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
static bool hold_crossing (line_track_t * p_track,
                           line_track_input_t const * p_in);
static void back_up (line_track_t * p_track, line_track_input_t const * p_in);
static void watch_barcode (line_track_t * p_track,
                           line_track_input_t const * p_in);
static void watch_hump (line_track_t * p_track,
                        line_track_input_t const * p_in);
static bool slow_pass (line_track_t const * p_track);
static void note_bars (line_track_t * p_track, bool const * p_was_black);
static int32_t creep_lean (line_track_t const * p_track, uint32_t bars_side);
static int32_t line_heading (line_track_t const * p_track,
                             line_track_input_t const * p_in);
static void watch_loop (line_track_t * p_track,
                        line_track_input_t const * p_in);
static void note_pivot (line_track_t * p_track,
                        line_track_input_t const * p_in);
static bool line_seen (line_track_t const * p_track,
                       line_track_input_t const * p_in);
static void aim_by_old_line (line_track_t * p_track,
                             line_track_input_t const * p_in,
                             bool const * p_was_black);
static void pivot_junction (line_track_t * p_track,
                            line_track_input_t const * p_in,
                            bool const * p_was_black);
static int32_t seek_target (line_track_t const * p_track,
                            line_track_input_t const * p_in);
static void track_edge (line_track_t * p_track, line_track_input_t const * p_in,
                        uint32_t side, bool b_was_black);
static void note_heading (line_track_t * p_track,
                          line_track_input_t const * p_in);
static int32_t recent_turn (line_track_t const * p_track,
                            line_track_input_t const * p_in);
static void note_counter (line_track_t * p_track,
                          line_track_input_t const * p_in, uint32_t side);
static bool counters_bend (line_track_t const * p_track,
                           line_track_input_t const * p_in, uint32_t side);
static bool crossed_out (line_track_t const * p_track,
                         line_track_input_t const * p_in, uint32_t side);
static void note_touch (line_track_t * p_track, line_track_input_t const * p_in,
                        uint32_t side);
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
    p_track->pivot_side = SIDE_NONE;
    p_track->last_heading_deg = p_in->heading_deg;
    p_track->b_rejoin = false;
    p_track->bars_side = SIDE_NONE;
    p_track->rejoin_mm = p_in->distance_mm - LINE_REJOIN_MM;
    p_track->junction_count = 0u;
    p_track->barcode_phase = LINE_BARCODE_NONE;
    p_track->line_deg = p_in->heading_deg;
    p_track->b_hump_slow = false;
    p_track->b_hump_spent = false;
    p_track->hump_samples = 0u;
    restart_tracking(p_track, p_in, p_in->heading_deg);
}

line_request_t
line_track_update (line_track_t * p_track, line_track_input_t const * p_in)
{
    /* As the last update saw them, with the bars white. */
    bool was_black[SIDE_COUNT] = {p_track->b_black[SIDE_LEFT],
                                  p_track->b_black[SIDE_RIGHT]};
    if (p_track->bars_side != SIDE_NONE)
    {
        was_black[p_track->bars_side] = false;
    }
    debounce(p_track, p_in);
    note_bars(p_track, was_black);
    uint32_t const bars_side = p_track->bars_side;
    if (bars_side != SIDE_NONE)
    {
        /* Until the end of this update; debounce needs the real level. */
        p_track->b_black[bars_side] = false;
    }
    note_pivot(p_track, p_in);
    note_heading(p_track, p_in);

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
        if (p_track->b_black[side] && !was_black[side])
        {
            note_counter(p_track, p_in, side);
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
        p_track->barcode_phase = LINE_BARCODE_NONE;
        p_track->b_rejoin = p_track->b_rejoin || p_in->b_turning;
        restart_tracking(p_track, p_in, p_in->heading_deg);
    }
    else if (p_track->b_rejoin)
    {
        /* The last pivot turned the car back along the line, away from it
         * if it stopped short. */
        p_track->b_rejoin = false;
        p_track->rejoin_mm = p_in->distance_mm;
        start_search(p_track, p_in,
                     (SIDE_LEFT == p_track->pivot_side) ? SIDE_RIGHT
                                                        : SIDE_LEFT);
    }
    else if ((LINE_BARCODE_BRAKE == p_track->barcode_phase) ||
             (LINE_BARCODE_BACK == p_track->barcode_phase))
    {
        back_up(p_track, p_in);
    }
    else if (JUNCTION_DRIVE == p_track->junction_phase)
    {
        drive_junction(p_track, p_in);
    }
    else if (JUNCTION_PIVOT == p_track->junction_phase)
    {
        pivot_junction(p_track, p_in, was_black);
    }
    else if (LINE_MODE_TRACK == p_track->mode)
    {
        if (!p_track->b_black[SIDE_LEFT] && !p_track->b_black[SIDE_RIGHT])
        {
            if (was_black[SIDE_LEFT] || was_black[SIDE_RIGHT])
            {
                p_track->white_start_mm = p_in->distance_mm;
                if (p_track->junction_phase != JUNCTION_CROSS)
                {
                    /* A crossing tape is no sign of being on the line. */
                    p_track->lost_start_mm = p_in->distance_mm;
                }
            }
        }
        follow_episode(p_track, p_in, was_black);
        if ((LINE_MODE_TRACK != p_track->mode) ||
            (JUNCTION_DRIVE == p_track->junction_phase))
        {
            /* A junction or a search took over. */
        }
        else if (hold_crossing(p_track, p_in))
        {
            /* Over the crossing tape: the sensors say nothing useful. */
        }
        else if (b_both)
        {
            /* The tape across both sensors: centred, hold until one
             * clears. In a bend, stop turning without losing the heading. */
            if (p_track->b_curve)
            {
                p_track->target_deg = p_in->heading_deg;
            }
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
        uint32_t const tape_side =
            (SIDE_LEFT == bars_side) ? SIDE_RIGHT : SIDE_LEFT;
        if ((LINE_MODE_TRACK != p_track->mode) ||
            (JUNCTION_DRIVE == p_track->junction_phase))
        {
            /* A junction or a search took over. */
        }
        else if (LINE_BARCODE_CREEP == p_track->barcode_phase)
        {
            p_track->target_deg =
                p_track->line_deg +
                (p_track->b_cross_hold ? 0 : creep_lean(p_track, bars_side));
        }
        else if ((JUNCTION_NONE == p_track->junction_phase) &&
                 (bars_side != SIDE_NONE) && !p_track->b_black[tape_side])
        {
            /* Only the bars are black: the tape is the other way. */
            p_track->target_deg =
                p_in->heading_deg + (side_sign(tape_side) * LINE_LEAD_DEG);
        }
        else
        {
            /* As tracked. */
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

    watch_barcode(p_track, p_in);
    watch_hump(p_track, p_in);
    line_barcode_phase_t const phase = p_track->barcode_phase;
    line_request_t const request = {
        .mode = (LINE_BARCODE_BRAKE == phase) ? LINE_MODE_STOP : p_track->mode,
        .target_heading_deg = p_track->target_deg,
        .b_seen = line_seen(p_track, p_in),
        .b_junction = (p_track->junction_phase != JUNCTION_NONE),
        .reverse_mm =
            (LINE_BARCODE_BACK == phase) ? LINE_BARCODE_REVERSE_MM : 0u,
        .b_slow = (LINE_BARCODE_CREEP == phase) || p_track->b_hump_slow};
    if (bars_side != SIDE_NONE)
    {
        /* Noted only while black. */
        p_track->b_black[bars_side] = true;
    }
    return request;
}

bool
line_track_hump_slow (line_track_t const * p_track)
{
    return p_track->b_hump_slow;
}

line_barcode_phase_t
line_track_barcode_phase (line_track_t const * p_track)
{
    return p_track->barcode_phase;
}

char const *
line_track_barcode_name (line_barcode_phase_t phase)
{
    switch (phase)
    {
        case LINE_BARCODE_BRAKE:
            return "BRAKE";
        case LINE_BARCODE_BACK:
            return "BACK";
        case LINE_BARCODE_CREEP:
            return "CREEP";
        default:
            return "DONE";
    }
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

/*
 * Creeping over a barcode or a hump. These start on straights, and a slow car
 * touches the tape often: seeking a bend here swung the car up to 18 degrees,
 * spoilt barcodes and carried on as it sped up, off the line (2026-11-07).
 */
static bool
slow_pass (line_track_t const * p_track)
{
    return (LINE_BARCODE_CREEP == p_track->barcode_phase) ||
           p_track->b_hump_slow;
}

/*
 * Creeping over a barcode, a line sensor can read the bars (2026-11-08: 108 mm
 * on the left, taken for junctions), and barcodes lie on both sides of the
 * line. Black that begins under one sensor while the other is already on the
 * tape is the bars: ignore it until it clears, even if the creep ends first.
 * Black with the other white, or both at once, is the tape or a crossing. A
 * crossing met with one sensor on the tape is lost too, but a creep ends on a
 * read barcode, before its junction: one missed after a failed read was
 * STRAIGHT anyway.
 */
static void
note_bars (line_track_t * p_track, bool const * p_was_black)
{
    if ((p_track->bars_side != SIDE_NONE) &&
        !p_track->b_black[p_track->bars_side])
    {
        p_track->bars_side = SIDE_NONE;
    }
    if ((SIDE_NONE == p_track->bars_side) &&
        (LINE_BARCODE_CREEP == p_track->barcode_phase))
    {
        for (uint32_t side = 0u; side < SIDE_COUNT; side++)
        {
            uint32_t const other = (SIDE_LEFT == side) ? SIDE_RIGHT : SIDE_LEFT;
            if (p_track->b_black[side] && !p_was_black[side] &&
                p_was_black[other] && p_track->b_black[other])
            {
                p_track->bars_side = side;
            }
        }
    }
}

/*
 * A creep's lean off the line's heading: towards the one sensor black, or away
 * from bars alone, else none. A lean, not a lead: the heading cannot run on
 * while a sensor stays black, as it did on bars taken for the tape.
 */
static int32_t
creep_lean (line_track_t const * p_track, uint32_t bars_side)
{
    bool const b_left = p_track->b_black[SIDE_LEFT];
    bool const b_right = p_track->b_black[SIDE_RIGHT];
    uint32_t side = SIDE_NONE;
    if (b_left != b_right)
    {
        side = b_left ? SIDE_LEFT : SIDE_RIGHT;
    }
    else if (!b_left && (bars_side != SIDE_NONE))
    {
        side = (SIDE_LEFT == bars_side) ? SIDE_RIGHT : SIDE_LEFT;
    }
    else
    {
        /* Both white, or both black: along the line. */
    }
    return (SIDE_NONE == side) ? 0 : (side_sign(side) * LINE_CREEP_LEAD_DEG);
}

/*
 * The line's heading on a straight: the car's, averaged with the recent
 * headings, which evens out its weaving round the line.
 */
static int32_t
line_heading (line_track_t const * p_track, line_track_input_t const * p_in)
{
    int32_t sum = p_in->heading_deg;
    for (uint32_t slot = 0u; slot < LINE_TREND_SLOTS; slot++)
    {
        sum += p_track->trend_deg[slot];
    }
    return sum / (int32_t)(LINE_TREND_SLOTS + 1u);
}

/* Remember which way the car is pivoting. */
static void
note_pivot (line_track_t * p_track, line_track_input_t const * p_in)
{
    if (p_in->b_turning && (p_in->heading_deg != p_track->last_heading_deg))
    {
        p_track->pivot_side = (p_in->heading_deg > p_track->last_heading_deg)
                                  ? SIDE_LEFT
                                  : SIDE_RIGHT;
    }
    p_track->last_heading_deg = p_in->heading_deg;
}

/*
 * What stops a bypass coming back to the line. It comes back at an angle, so
 * the sensor on the side it turned to meets the line first, and the other one
 * meets a crossing tape first. On 2026-11-07 the far sensor on a crossing tape
 * ended a return 70 mm short, and the car ran alongside the line.
 */
static bool
line_seen (line_track_t const * p_track, line_track_input_t const * p_in)
{
    if (p_in->b_moving && (SIDE_LEFT == p_track->pivot_side))
    {
        return p_in->b_left_black;
    }
    if (p_in->b_moving && (SIDE_RIGHT == p_track->pivot_side))
    {
        return p_in->b_right_black;
    }
    return p_in->b_left_black || p_in->b_right_black;
}

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
    p_track->touch_side = SIDE_NONE;
    p_track->touch_mm = p_in->distance_mm;
    p_track->b_curve = false;
    p_track->counter_side = SIDE_NONE;
    p_track->b_cross_hold = false;
    for (uint32_t slot = 0u; slot < LINE_TREND_SLOTS; slot++)
    {
        p_track->trend_deg[slot] = p_in->heading_deg;
    }
    p_track->trend_next = 0u;
    p_track->trend_mm = p_in->distance_mm;
    p_track->white_start_mm = p_in->distance_mm;
    p_track->lost_start_mm = p_in->distance_mm;
    p_track->loop_mm = p_in->distance_mm;
    p_track->loop_deg = p_in->heading_deg;
    for (uint32_t side = 0u; side < SIDE_COUNT; side++)
    {
        p_track->black_start_mm[side] = p_in->distance_mm;
    }
    p_track->search_samples = 0u;
    p_track->b_search_hit = false;
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
        p_track->junction_mm = p_in->distance_mm;
        p_track->junction_deg = p_track->entry_deg;
        p_track->b_cross_hold =
            abs_deg(recent_turn(p_track, p_in)) < LINE_BEND_TREND_DEG;
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
    p_track->search_samples = 0u;
    p_track->last_side = side;
    p_track->mode =
        (SIDE_LEFT == side) ? LINE_MODE_SEARCH_LEFT : LINE_MODE_SEARCH_RIGHT;
    p_track->target_deg = p_in->heading_deg + (side_sign(side) * turn_deg);
    p_track->pivot_start_deg = p_in->heading_deg;
    p_track->b_old_line_seen = false;
}

/* Keep the heading the junction was met on until past the crossing tape. */
static bool
hold_crossing (line_track_t * p_track, line_track_input_t const * p_in)
{
    if (p_track->b_cross_hold && (JUNCTION_CROSS == p_track->junction_phase) &&
        ((p_in->distance_mm - p_track->junction_mm) < LINE_JUNCTION_HOLD_MM))
    {
        p_track->target_deg = p_track->junction_deg;
        return true;
    }
    p_track->b_cross_hold = false;
    return false;
}

/* Stop, then one reverse with the sensors ignored, then creep. */
static void
back_up (line_track_t * p_track, line_track_input_t const * p_in)
{
    p_track->barcode_samples++;
    if (LINE_BARCODE_BRAKE == p_track->barcode_phase)
    {
        if (p_track->barcode_samples >= LINE_BARCODE_BRAKE_SAMPLES)
        {
            p_track->barcode_phase = LINE_BARCODE_BACK;
            p_track->barcode_samples = 0u;
            p_track->b_back_started = false;
        }
        return;
    }
    p_track->b_back_started = p_track->b_back_started || p_in->b_moving;
    if ((p_track->b_back_started && !p_in->b_moving) ||
        (p_track->barcode_samples >= LINE_BARCODE_BACK_SAMPLES))
    {
        p_track->barcode_phase = LINE_BARCODE_CREEP;
        p_track->barcode_mm = p_in->distance_mm;
        restart_tracking(p_track, p_in, p_in->heading_deg);
    }
}

/*
 * Follow slowly over a hump. It starts only on a straight: most false leans
 * are steering in bends. Once started it holds until level ground, as the car
 * wobbles on the hump, and a stall ends the hump, so a car stalled on the ramp
 * tries again at full speed.
 */
static void
watch_hump (line_track_t * p_track, line_track_input_t const * p_in)
{
    if (!p_in->b_hump || !p_in->b_control)
    {
        p_track->b_hump_slow = false;
        p_track->b_hump_spent = false;
    }
    else if (p_track->b_hump_slow)
    {
        p_track->hump_samples++;
        if (p_track->hump_samples >= LINE_HUMP_SLOW_SAMPLES)
        {
            p_track->b_hump_slow = false;
            p_track->b_hump_spent = true;
        }
    }
    else if (!p_track->b_hump_spent && !p_track->b_curve &&
             (abs_deg(recent_turn(p_track, p_in)) < LINE_BEND_TREND_DEG) &&
             (LINE_MODE_TRACK == p_track->mode) &&
             (JUNCTION_NONE == p_track->junction_phase) &&
             (LINE_BARCODE_NONE == p_track->barcode_phase))
    {
        p_track->b_hump_slow = true;
        p_track->hump_samples = 0u;
    }
    else
    {
        /* Not now: a bend, a junction, a barcode, or the limit reached. */
    }
}

/*
 * Start a slow pass, only on plain tracking, and end one. A car still
 * turning is in a bend, and the barcode sensor crossing the tape looks like
 * a barcode: on 2026-11-06 no pass begun after turning LINE_BEND_TREND_DEG
 * read anything (6 of 6), and all 8 clean reads began within 5 degrees.
 */
static void
watch_barcode (line_track_t * p_track, line_track_input_t const * p_in)
{
    if (LINE_BARCODE_NONE == p_track->barcode_phase)
    {
        if (p_in->b_barcode && p_in->b_control && !p_track->b_curve &&
            (abs_deg(recent_turn(p_track, p_in)) < LINE_BEND_TREND_DEG) &&
            (LINE_MODE_TRACK == p_track->mode) &&
            (JUNCTION_NONE == p_track->junction_phase))
        {
            p_track->barcode_phase = LINE_BARCODE_BRAKE;
            p_track->barcode_samples = 0u;
            p_track->line_deg = line_heading(p_track, p_in);
        }
    }
    else if ((LINE_BARCODE_CREEP == p_track->barcode_phase) &&
             (p_in->b_barcode_done ||
              ((p_in->distance_mm - p_track->barcode_mm) >=
               LINE_BARCODE_CREEP_MM)))
    {
        p_track->barcode_phase = LINE_BARCODE_NONE;
    }
    else
    {
        /* Braking, backing up, or still creeping. */
    }
}

/*
 * Pivot to the new heading with the sensors ignored, then carry straight
 * on along it: the sensors take over when they next see tape. A pivot that
 * does not end stops.
 */
static void
pivot_junction (line_track_t * p_track, line_track_input_t const * p_in,
                bool const * p_was_black)
{
    p_track->search_samples++;
    aim_by_old_line(p_track, p_in, p_was_black);
    if (!p_in->b_turning && (abs_deg(p_track->target_deg - p_in->heading_deg) <=
                             LINE_SEARCH_DONE_DEG))
    {
        restart_tracking(p_track, p_in, p_track->target_deg);
    }
    else if (p_track->search_samples >= SEARCH_TIMEOUT_SAMPLES)
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

/*
 * Early in a pivot the far sensor leaves the line carrying on past the
 * junction: move the target by how far that line is off the car's heading.
 * Motion takes the new target on without stopping.
 */
static void
aim_by_old_line (line_track_t * p_track, line_track_input_t const * p_in,
                 bool const * p_was_black)
{
    uint32_t const far =
        (SIDE_LEFT == p_track->last_side) ? SIDE_RIGHT : SIDE_LEFT;
    int32_t const sign = side_sign(p_track->last_side);
    int32_t const swept = sign * (p_in->heading_deg - p_track->pivot_start_deg);
    if (p_track->b_old_line_seen || !p_was_black[far] ||
        p_track->b_black[far] || (swept < LINE_OLD_LINE_MIN_DEG) ||
        (swept > LINE_OLD_LINE_MAX_DEG))
    {
        return;
    }
    p_track->b_old_line_seen = true;
    p_track->target_deg += sign * (swept - LINE_OLD_LINE_DEG);
}

/* Both white: in a bend, seek the last edge; for too long, search. */
static void
track (line_track_t * p_track, line_track_input_t const * p_in)
{
    bool const b_white =
        !p_track->b_black[SIDE_LEFT] && !p_track->b_black[SIDE_RIGHT];
    if (b_white && p_track->b_curve)
    {
        p_track->target_deg = seek_target(p_track, p_in);
    }
    if (b_white && (p_track->last_side != SIDE_NONE) &&
        ((p_in->distance_mm - p_track->lost_start_mm) >=
         (p_track->b_curve ? LINE_LOST_MM : LINE_STRAIGHT_LOST_MM)))
    {
        start_search(p_track, p_in, p_track->last_side);
    }
    watch_loop(p_track, p_in);
}

/* Going round in circles: look for the line, and take the first stretch of it
 * as straight, or the same touches start the circle again. */
static void
watch_loop (line_track_t * p_track, line_track_input_t const * p_in)
{
    if ((p_in->distance_mm - p_track->loop_mm) >= LINE_LOOP_MM)
    {
        p_track->loop_mm = p_in->distance_mm;
        p_track->loop_deg = p_in->heading_deg;
    }
    else if (abs_deg(p_in->heading_deg - p_track->loop_deg) >= LINE_LOOP_DEG)
    {
        p_track->rejoin_mm = p_in->distance_mm;
        start_search(p_track, p_in, p_track->last_side);
    }
    else
    {
        /* Nothing unusual. */
    }
}

/*
 * The heading to aim for in a bend while both sensors read white. White
 * on both means the tape is centred or has just slipped out past the edge,
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
    bool const b_counter =
        b_black && (side == p_track->counter_side) &&
        ((p_in->distance_mm - p_track->counter_mm) < LINE_COUNTER_MM);
    if (b_black && !b_was_black && !b_counter)
    {
        note_touch(p_track, p_in, side);
    }
    if (b_counter)
    {
        /* Keep last_side and the bend: the tape is still curving that way. */
        p_track->target_deg =
            p_in->heading_deg + (side_sign(side) * LINE_COUNTER_LEAD_DEG);
    }
    else if (b_black)
    {
        if (p_track->touch_side != side)
        {
            /* Black too long to be the car ahead of a bend. */
            p_track->b_curve = false;
            p_track->touch_side = side;
        }
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
         * corner: hold what it reached, or seek in a bend. */
        if (side == p_track->counter_side)
        {
            if ((p_in->distance_mm - p_track->counter_mm) >=
                LINE_COUNTER_END_MM)
            {
                /* The tape was last on this side, not the bend's. */
                p_track->b_curve = false;
                p_track->last_side = side;
            }
            p_track->counter_side = SIDE_NONE;
        }
        if (!slow_pass(p_track) && crossed_out(p_track, p_in, side))
        {
            /* A bend on this side: the other sensor still ends it. */
            p_track->b_curve = true;
            p_track->touch_side = side;
            p_track->touch_mm = p_in->distance_mm;
        }
        p_track->target_deg =
            p_track->b_curve ? seek_target(p_track, p_in) : p_in->heading_deg;
    }
    else
    {
        /* White and was white. */
    }
}

/* Keep the heading every LINE_TREND_STEP_MM of travel. */
static void
note_heading (line_track_t * p_track, line_track_input_t const * p_in)
{
    if ((p_in->distance_mm - p_track->trend_mm) >= LINE_TREND_STEP_MM)
    {
        p_track->trend_deg[p_track->trend_next] = p_in->heading_deg;
        p_track->trend_next = (p_track->trend_next + 1u) % LINE_TREND_SLOTS;
        p_track->trend_mm = p_in->distance_mm;
    }
}

/* Degrees turned over the last stretch of travel, + is left. */
static int32_t
recent_turn (line_track_t const * p_track, line_track_input_t const * p_in)
{
    return p_in->heading_deg - p_track->trend_deg[p_track->trend_next];
}

/* A touch begins: judge now whether it steers back against a bend. */
static void
note_counter (line_track_t * p_track, line_track_input_t const * p_in,
              uint32_t side)
{
    if (counters_bend(p_track, p_in, side))
    {
        p_track->counter_side = side;
        p_track->counter_mm = p_in->distance_mm;
    }
    else if (side == p_track->counter_side)
    {
        p_track->counter_side = SIDE_NONE;
    }
    else
    {
        /* The other side's touch carries on. */
    }
}

/*
 * Black on @p side in a bend the other way. Turning alone is not enough:
 * after a correction too far, the sensor it swung towards must still steer
 * at full lead (LT17).
 */
static bool
counters_bend (line_track_t const * p_track, line_track_input_t const * p_in,
               uint32_t side)
{
    return p_track->b_curve && (side != p_track->touch_side) &&
           ((-side_sign(side) * recent_turn(p_track, p_in)) >=
            LINE_BEND_TREND_DEG);
}

/*
 * The tape has crossed from the other sensor and just left @p side, the
 * side the car has been turning towards: it ran across a bend, and the tape
 * is outside on that side. Both white alone cannot show this: a centred
 * tape reads the same. The seek is safe if this is wrong: a tape that is
 * centred after all soon reaches the other sensor and turns the car back.
 */
static bool
crossed_out (line_track_t const * p_track, line_track_input_t const * p_in,
             uint32_t side)
{
    uint32_t const other = (SIDE_LEFT == side) ? SIDE_RIGHT : SIDE_LEFT;
    return !p_track->b_black[other] && p_track->b_both_seen &&
           (p_track->entry_side == other) &&
           (JUNCTION_NONE == p_track->junction_phase) &&
           ((side_sign(side) * recent_turn(p_track, p_in)) >=
            LINE_BEND_TREND_DEG);
}

/* A single-side touch begins: the same side again soon is a bend. */
static void
note_touch (line_track_t * p_track, line_track_input_t const * p_in,
            uint32_t side)
{
    if (side != p_track->touch_side)
    {
        p_track->b_curve = false;
    }
    else if (!slow_pass(p_track) &&
             ((p_in->distance_mm - p_track->rejoin_mm) >= LINE_REJOIN_MM) &&
             ((p_in->distance_mm - p_track->touch_mm) <= LINE_CURVE_TOUCH_MM))
    {
        p_track->b_curve = true;
    }
    else
    {
        /* Too long ago to be the same bend. */
    }
    p_track->touch_side = side;
    p_track->touch_mm = p_in->distance_mm;
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
