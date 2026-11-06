/* Drive the real line_track module with simulated digital samples. */
#include "../../../app_program/line_follow/line_track.h"
#include "../../../app_program/line_follow/line_follow_config.h"
#include "../../../app_program/motion/motion_config.h"
#include <math.h>
#include <stdio.h>

/* 120 mm/s sampled every 10 ms. */
#define MM_PER_SAMPLE_X10 12

static line_track_t track;
static line_track_input_t in;
static line_request_t out;
static int travel_x10;
static unsigned int failures;

static void
check (char const * p_name, bool ok)
{
    printf("%s %s\n", ok ? "PASS" : "FAIL", p_name);
    failures += ok ? 0u : 1u;
}

/* Restart on heading 0 at 0 mm, both white, under line control. */
static void
reset (void)
{
    travel_x10 = 0;
    in = (line_track_input_t){.b_control = true};
    line_track_init(&track, &in);
    out = line_track_update(&track, &in);
}

/* @p samples samples of (@p left, @p right), driving forward. */
static void
drive (bool left, bool right, unsigned int samples)
{
    in.b_left_black = left;
    in.b_right_black = right;
    for (unsigned int i = 0u; i < samples; i++)
    {
        travel_x10 += MM_PER_SAMPLE_X10;
        in.distance_mm = travel_x10 / 10;
        out = line_track_update(&track, &in);
    }
}

/* Samples to cover @p mm of travel. */
static unsigned int
samples_for (int mm)
{
    return (unsigned int)((mm * 10 + MM_PER_SAMPLE_X10 - 1) /
                          MM_PER_SAMPLE_X10);
}

static void
test_debounce (void)
{
    reset();
    drive(true, false, 1u);
    bool const b_one = (0 == out.target_heading_deg) && out.b_seen;
    drive(false, false, 1u);
    drive(true, false, LINE_CONFIRM_SAMPLES);
    check("LT01 one black sample is ignored; confirmed black leads left",
          b_one && (LINE_LEAD_DEG == out.target_heading_deg) &&
              (LINE_MODE_TRACK == out.mode));
}

static void
test_right_lead (void)
{
    reset();
    drive(false, true, LINE_CONFIRM_SAMPLES);
    bool const b_first = (-LINE_LEAD_DEG == out.target_heading_deg);
    in.heading_deg = -10;
    drive(false, true, 1u);
    check("LT02 right edge leads right, and keeps leading as the car turns",
          b_first && (-10 - LINE_LEAD_DEG == out.target_heading_deg));
}

static void
test_edge_clears (void)
{
    reset();
    drive(false, false, samples_for(100));
    drive(true, false, LINE_CONFIRM_SAMPLES);
    drive(true, false, samples_for(30));
    bool const b_leading = (LINE_LEAD_DEG == out.target_heading_deg);
    drive(false, false, LINE_CONFIRM_SAMPLES);
    bool const b_held =
        (LINE_MODE_TRACK == out.mode) && (0 == out.target_heading_deg);
    drive(false, false, samples_for(100));
    check("LT03 an edge leads until white; on a straight the car then "
          "holds its heading",
          b_leading && b_held && (LINE_MODE_TRACK == out.mode) &&
              (0 == out.target_heading_deg));
}

static void
test_held_edge (void)
{
    reset();
    drive(true, false, LINE_CONFIRM_SAMPLES);
    in.heading_deg = 70;
    drive(true, false, samples_for(LINE_EDGE_MAX_MM) - 3u);
    bool const b_turning = (LINE_MODE_TRACK == out.mode) &&
                           (70 + LINE_LEAD_DEG == out.target_heading_deg);
    drive(true, false, 3u);
    check("LT04 an edge black for LINE_EDGE_MAX_MM is a tight bend: search",
          b_turning && (LINE_MODE_SEARCH_LEFT == out.mode) &&
              (70 + LINE_SEARCH_MAX_DEG == out.target_heading_deg));
}

/* The 2026-11-06 run: the left sensor touched the tape, the turn cleared
 * it within 20 mm, and that used to start a search (six in 650 mm). */
static void
test_brief_touch (void)
{
    reset();
    drive(false, false, samples_for(200));
    drive(true, false, LINE_CONFIRM_SAMPLES);
    drive(true, false, samples_for(15));
    drive(false, false, LINE_CONFIRM_SAMPLES);
    bool const b_tracking = (LINE_MODE_TRACK == out.mode);
    drive(false, true, LINE_CONFIRM_SAMPLES);
    drive(false, true, samples_for(10));
    drive(false, false, LINE_CONFIRM_SAMPLES + samples_for(100));
    check("LT05 a brief touch of either sensor is a correction, not a search",
          b_tracking && (LINE_MODE_TRACK == out.mode) &&
              (0 == out.target_heading_deg));
}

/* A cross met square: both sensors black together. */
static void
cross_junction (line_turn_t turn)
{
    reset();
    in.turn = turn;
    in.heading_deg = 4;
    drive(false, false, samples_for(100));
    drive(true, true, LINE_CONFIRM_SAMPLES + LINE_JUNCTION_SAMPLES - 1u);
}

static void
test_junction (void)
{
    cross_junction(LINE_TURN_STRAIGHT);
    bool const b_junction = out.b_junction && (4 == out.target_heading_deg) &&
                            (LINE_MODE_TRACK == out.mode);
    in.heading_deg = 6;
    drive(true, true, samples_for(15));
    /* Still tracking: the target follows the heading, no 4 degree hold. */
    bool const b_tracking = out.b_junction && (6 == out.target_heading_deg);
    drive(false, true, LINE_CONFIRM_SAMPLES + 2u);
    bool const b_edge = out.b_junction &&
                        (6 - LINE_LEAD_DEG == out.target_heading_deg) &&
                        (LINE_MODE_TRACK == out.mode);
    drive(false, false, LINE_CONFIRM_SAMPLES);
    check("LT06 a cross straight on (the default) is reported as a junction "
          "but tracking carries on over it",
          b_junction && b_tracking && b_edge && !out.b_junction &&
              (LINE_MODE_TRACK == out.mode));
}

/* 2026-11-06 car5: both sensors turned black in one sample, so the left
 * edge never started its held-edge travel; a short black after a long
 * white run then looked held and started a search. */
static void
test_both_rise_together (void)
{
    reset();
    drive(false, false, samples_for(300));
    drive(true, true, LINE_CONFIRM_SAMPLES + 1u);
    drive(true, false, LINE_CONFIRM_SAMPLES + 2u);
    check("LT18 an edge that turns black with the other sensor is a fresh "
          "correction, not a held edge",
          (LINE_MODE_TRACK == out.mode) &&
              (LINE_LEAD_DEG == out.target_heading_deg));
}

static void
test_junction_left (void)
{
    cross_junction(LINE_TURN_LEFT);
    drive(true, true, samples_for(15));
    drive(false, false, samples_for(LINE_JUNCTION_CENTRE_MM));
    bool const b_pivot =
        (LINE_MODE_SEARCH_LEFT == out.mode) &&
        (4 + 90 + LINE_JUNCTION_LOOK_DEG == out.target_heading_deg) &&
        out.b_junction;
    in.b_turning = true;
    /* The right sensor sweeps over the route being left: ignored. */
    in.heading_deg = 20;
    drive(false, true, LINE_CONFIRM_SAMPLES);
    in.heading_deg = 25;
    drive(false, false, LINE_CONFIRM_SAMPLES);
    bool const b_ignored = (LINE_MODE_SEARCH_LEFT == out.mode);
    in.heading_deg = 85;
    drive(true, false, LINE_CONFIRM_SAMPLES);
    in.heading_deg = 92;
    drive(false, false, LINE_CONFIRM_SAMPLES);
    check("LT15 a LEFT junction drives to the centre, pivots left past the "
          "route it leaves, and tracks the crossing line",
          b_pivot && b_ignored && (LINE_MODE_TRACK == out.mode) &&
              !out.b_junction && (92 == out.target_heading_deg));
}

static void
test_junction_uturn (void)
{
    cross_junction(LINE_TURN_UTURN);
    drive(false, false, samples_for(LINE_JUNCTION_CENTRE_MM));
    in.b_turning = true;
    bool const b_pivot =
        (LINE_MODE_SEARCH_LEFT == out.mode) &&
        (4 + 180 + LINE_JUNCTION_LOOK_DEG == out.target_heading_deg);
    /* The left branch passes under the left sensor at 90: ignored. */
    in.heading_deg = 94;
    drive(true, false, LINE_CONFIRM_SAMPLES);
    in.heading_deg = 100;
    drive(false, false, LINE_CONFIRM_SAMPLES);
    bool const b_ignored = (LINE_MODE_SEARCH_LEFT == out.mode);
    in.heading_deg = 178;
    drive(true, false, LINE_CONFIRM_SAMPLES);
    in.heading_deg = 184;
    drive(false, false, LINE_CONFIRM_SAMPLES);
    bool const b_found =
        (LINE_MODE_TRACK == out.mode) && (184 == out.target_heading_deg);
    cross_junction(LINE_TURN_RIGHT);
    drive(false, false, samples_for(LINE_JUNCTION_CENTRE_MM));
    in.b_turning = false;
    in.heading_deg = 4 - 90 - LINE_JUNCTION_LOOK_DEG;
    drive(false, false, 1u);
    check("LT16 a U-turn pivots 180 past the branches; a RIGHT pivot that "
          "finds nothing stops",
          b_pivot && b_ignored && b_found && (LINE_MODE_STOP == out.mode) &&
              !out.b_junction);
}

/* 2026-11-06 car7 d=823..906: the right sensor black, both 34 mm later
 * as the right turn went a little too far, then only the left, then
 * white. The tape is back between the sensors: no search. */
static void
test_overshoot_across (void)
{
    reset();
    in.heading_deg = 0;
    drive(false, false, samples_for(100));
    drive(false, true, LINE_CONFIRM_SAMPLES + samples_for(30));
    in.heading_deg = -23;
    drive(true, true, LINE_CONFIRM_SAMPLES + LINE_JUNCTION_SAMPLES + 5u);
    bool const b_no_junction = !out.b_junction;
    in.heading_deg = -31;
    drive(true, false, LINE_CONFIRM_SAMPLES + 5u);
    bool const b_turns_back = (-31 + LINE_LEAD_DEG == out.target_heading_deg);
    in.heading_deg = -22;
    drive(false, false, LINE_CONFIRM_SAMPLES + 2u);
    check("LT17 the tape crossing from one sensor to the other (a "
          "correction too far) is no junction and starts no search",
          b_no_junction && b_turns_back && (LINE_MODE_TRACK == out.mode));
}

/* Drive @p mm with both sensors white while the heading turns at
 * @p mdeg_per_mm (the car following a bend). */
static void
drive_bend (int mm, int mdeg_per_mm)
{
    static int heading_mdeg;
    heading_mdeg = in.heading_deg * 1000;
    in.b_left_black = false;
    in.b_right_black = false;
    for (unsigned int i = 0u; i < samples_for(mm); i++)
    {
        travel_x10 += MM_PER_SAMPLE_X10;
        heading_mdeg += (mdeg_per_mm * MM_PER_SAMPLE_X10) / 10;
        in.distance_mm = travel_x10 / 10;
        in.heading_deg = heading_mdeg / 1000;
        out = line_track_update(&track, &in);
    }
}

/* The 2026-11-06 curve: about 200 mm radius, 0.29 deg/mm. */
static void
test_bend (void)
{
    reset();
    int const expected = (290 * LINE_CURVE_LEAD_MM) / 1000;
    drive_bend(300, 290);
    int const lead = out.target_heading_deg - in.heading_deg;
    bool const b_bend = (lead >= expected - 1) && (lead <= expected + 1);
    drive_bend(400, 0);
    int const straight = out.target_heading_deg - in.heading_deg;
    drive_bend(300, -290);
    int const right = out.target_heading_deg - in.heading_deg;
    printf("  bend lead %d, straight %d, right %d deg\n", lead, straight,
           right);
    check("LT14 between corrections the car keeps to the bend it has been "
          "turning (0.29 deg/mm x LINE_CURVE_LEAD_MM); straight again after",
          b_bend && (0 == straight) && (right <= 1 - expected) &&
              (right >= -1 - expected) && (LINE_MODE_TRACK == out.mode));
}

static void
test_lost (void)
{
    reset();
    drive(false, false, samples_for(LINE_LOST_MM) + 5u);
    bool const b_no_edge = (LINE_MODE_TRACK == out.mode);
    drive(true, false, LINE_CONFIRM_SAMPLES);
    drive(true, false, samples_for(30));
    drive(false, false, LINE_CONFIRM_SAMPLES);
    drive(false, false, samples_for(LINE_LOST_MM) - 5u);
    bool const b_early = (LINE_MODE_TRACK == out.mode);
    drive(false, false, 5u);
    check("LT07 long white after an edge searches towards it",
          b_no_edge && b_early && (LINE_MODE_SEARCH_LEFT == out.mode));
}

/* Enter SEARCH_LEFT from heading @p heading: a left edge, then the line
 * lost. */
static void
start_left_search (int32_t heading)
{
    reset();
    in.heading_deg = heading;
    drive(false, false, samples_for(100));
    drive(true, false, LINE_CONFIRM_SAMPLES);
    drive(false, false, LINE_CONFIRM_SAMPLES + samples_for(LINE_LOST_MM));
    in.b_turning = true;
}

static void
test_search_found (void)
{
    start_left_search(0);
    bool const b_search = (LINE_MODE_SEARCH_LEFT == out.mode);
    in.heading_deg = 40;
    drive(true, false, LINE_CONFIRM_SAMPLES);
    bool const b_still = (LINE_MODE_SEARCH_LEFT == out.mode);
    in.heading_deg = 47;
    drive(false, false, LINE_CONFIRM_SAMPLES);
    check("LT08 search ends when the tape passes the search-side sensor",
          b_search && b_still && (LINE_MODE_TRACK == out.mode) &&
              (47 == out.target_heading_deg));
}

static void
test_search_between (void)
{
    start_left_search(5);
    in.heading_deg = 20;
    drive(false, true, LINE_CONFIRM_SAMPLES);
    check("LT09 opposite sensor first: back to the search start heading",
          (LINE_MODE_TRACK == out.mode) && (5 == out.target_heading_deg));
}

static void
test_search_both_sides (void)
{
    start_left_search(0);
    in.heading_deg = LINE_SEARCH_MAX_DEG;
    drive(false, false, 1u);
    bool const b_turning = (LINE_MODE_SEARCH_LEFT == out.mode);
    in.b_turning = false;
    drive(false, false, 1u);
    bool const b_other = (LINE_MODE_SEARCH_RIGHT == out.mode) &&
                         (-LINE_SEARCH_MAX_DEG == out.target_heading_deg);
    in.heading_deg = -LINE_SEARCH_MAX_DEG + LINE_SEARCH_DONE_DEG;
    drive(false, false, 1u);
    bool const b_stop = (LINE_MODE_STOP == out.mode);
    drive(false, true, LINE_CONFIRM_SAMPLES);
    check("LT10 search turns one way, then the other, then stops; a line "
          "under a sensor restarts tracking",
          b_turning && b_other && b_stop && (LINE_MODE_TRACK == out.mode) &&
              (in.heading_deg == out.target_heading_deg));
}

static void
test_search_timeout (void)
{
    start_left_search(0);
    drive(false, false, (LINE_SEARCH_TIMEOUT_MS / LINE_SAMPLE_MS) + 1u);
    check("LT11 a search that never ends stops", LINE_MODE_STOP == out.mode);
}

static void
test_control_off (void)
{
    reset();
    in.b_control = false;
    in.heading_deg = 33;
    drive(true, false, LINE_CONFIRM_SAMPLES + 2u);
    bool const b_anchored = (LINE_MODE_TRACK == out.mode) &&
                            (33 == out.target_heading_deg) && out.b_seen;
    in.b_control = true;
    drive(true, false, 1u);
    check("LT12 without line control the target follows the heading; an "
          "edge held at hand-back turns at once",
          b_anchored && (33 + LINE_LEAD_DEG == out.target_heading_deg));
}

/* The 2026-11-06 curve: a pivot found the tape, overshot, and the other
 * sensor's brief black looked like a quick crossing the other way. */
static void
test_pivot_overshoot (void)
{
    start_left_search(0);
    in.heading_deg = 30;
    drive(true, false, LINE_CONFIRM_SAMPLES);
    in.heading_deg = 36;
    drive(false, false, LINE_CONFIRM_SAMPLES);
    bool const b_found = (LINE_MODE_TRACK == out.mode);
    in.b_turning = false;
    in.heading_deg = 40;
    in.b_left_black = false;
    in.b_right_black = true;
    for (unsigned int i = 0u; i < LINE_CONFIRM_SAMPLES; i++)
    {
        out = line_track_update(&track, &in); /* Still pivoting: no travel. */
    }
    bool const b_turns_back = (40 - LINE_LEAD_DEG == out.target_heading_deg);
    in.heading_deg = 38;
    drive(false, false, LINE_CONFIRM_SAMPLES);
    check("LT13 the overshoot after a found search turns back, no search",
          b_found && b_turns_back && (LINE_MODE_TRACK == out.mode) &&
              (38 == out.target_heading_deg));
}

/*
 * LT19: the 2026-11-06 car8 failure as a model. Sensors 130 mm ahead of
 * the axle, 20 mm apart over 18 mm tape; the car meets a left quarter
 * circle of 200 mm radius turned 8 degrees right, as car8 did. The line
 * task samples every 20 ms (a 10 ms delay rounds up to two ticks); motion
 * acts on a request two samples late at MOTION_FOLLOW_KP times the error,
 * capped, and the car's turn rate lags that by 0.25 s (the car kept
 * turning about 0.3 s after a request ended). With a 25 degree lead, or
 * at 120 mm/s, the model loses the curve the way the car did.
 */
#define SIM_AHEAD_MM 130.0
#define SIM_HALF_GAP_MM 10.0
#define SIM_HALF_TAPE_MM 9.0
#define SIM_RADIUS_MM 200.0
#define SIM_STEP_S 0.02
#define SIM_LAG_S 0.25
#define SIM_DELAY_SAMPLES 2u
#define SIM_PI 3.14159265358979

/* Distance from (x, y) to the tape: +x up to the origin, the arc about
 * (0, R), then +y from (R, R). */
static double
tape_distance (double x, double y)
{
    if (x <= 0.0)
    {
        return fabs(y);
    }
    if (y >= SIM_RADIUS_MM)
    {
        return fabs(x - SIM_RADIUS_MM);
    }
    return fabs(sqrt((x * x) + ((y - SIM_RADIUS_MM) * (y - SIM_RADIUS_MM))) -
                SIM_RADIUS_MM);
}

/* Run the model with the sensors 2 * @p half_gap mm apart. */
static bool
curve_model (double half_gap)
{
    reset();
    double const speed = (double)MOTION_FOLLOW_SPEED_MM_S;
    double const step_mm = speed * SIM_STEP_S;
    /* Motion's cap: the follow limit, or where the inner wheel stops. */
    double const cap =
        fmin((double)MOTION_FOLLOW_MAX_DEG_S,
             (2.0 * speed) / ((double)MOTION_TRACK_WIDTH_MM * SIM_PI / 180.0));
    double x = -300.0 - SIM_AHEAD_MM; /* Sensors 300 mm before the arc. */
    double heading = -8.0;            /* Degrees, + is left. */
    /* Sensors on the tape, the car angled across it. */
    double y = -SIM_AHEAD_MM * sin(heading * SIM_PI / 180.0);
    double rate = 0.0;
    double travel = 0.0;
    double worst = 0.0;
    double requests[SIM_DELAY_SAMPLES + 1u];
    bool b_searched = false;
    for (unsigned int k = 0u; k <= SIM_DELAY_SAMPLES; k++)
    {
        requests[k] = heading;
    }
    unsigned int const steps = (unsigned int)(1400.0 / step_mm);
    for (unsigned int i = 0u; i < steps; i++)
    {
        double const rad = heading * SIM_PI / 180.0;
        double const sx = x + (SIM_AHEAD_MM * cos(rad));
        double const sy = y + (SIM_AHEAD_MM * sin(rad));
        double const off = tape_distance(sx, sy);
        worst = (off > worst) ? off : worst;
        in.b_left_black =
            tape_distance(sx - (half_gap * sin(rad)),
                          sy + (half_gap * cos(rad))) <= SIM_HALF_TAPE_MM;
        in.b_right_black =
            tape_distance(sx + (half_gap * sin(rad)),
                          sy - (half_gap * cos(rad))) <= SIM_HALF_TAPE_MM;
        in.heading_deg = (int32_t)lround(heading);
        in.distance_mm = (int32_t)travel;
        out = line_track_update(&track, &in);
        b_searched = b_searched || (LINE_MODE_TRACK != out.mode);

        for (unsigned int k = SIM_DELAY_SAMPLES; k > 0u; k--)
        {
            requests[k] = requests[k - 1u];
        }
        requests[0] = (double)out.target_heading_deg;
        double asked =
            MOTION_FOLLOW_KP * (requests[SIM_DELAY_SAMPLES] - heading);
        asked = fmin(fmax(asked, -cap), cap);
        rate += (asked - rate) * SIM_STEP_S / SIM_LAG_S;
        heading += rate * SIM_STEP_S;
        x += step_mm * cos(heading * SIM_PI / 180.0);
        y += step_mm * sin(heading * SIM_PI / 180.0);
        travel += step_mm;
    }
    printf("  curve model, gap %.0f mm: sensors at most %.0f mm off the "
           "tape, heading %.0f deg, %.0f mm up the exit straight\n",
           2.0 * half_gap, worst, heading, y - SIM_RADIUS_MM);
    return (worst < 30.0) && (heading > 70.0) && (heading < 110.0) &&
           (y > 600.0) && !b_searched;
}

static void
test_curve_model (void)
{
    /* Wider than the 18 mm tape (white when centred) and narrower (black
     * when centred): car7 and car9 started in one state each. */
    bool const b_wide = curve_model(SIM_HALF_GAP_MM);
    bool const b_narrow = curve_model(8.0);
    check("LT19 a 200 mm radius left curve, met 8 degrees off with the car's "
          "turn lag, is followed onto the exit straight without a search, "
          "with the sensors wider or narrower than the tape",
          b_wide && b_narrow);
}

/* 2026-11-06 car9: the tape under both sensors for 143 mm. Longer than
 * LINE_EDGE_MAX_MM, the first single black looked like a held edge. */
static void
test_long_both_black (void)
{
    reset();
    drive(true, true, samples_for(LINE_EDGE_MAX_MM + 100));
    drive(true, false, LINE_CONFIRM_SAMPLES + 1u);
    check("LT20 travel with the tape under both sensors is no held edge: "
          "the first single black is a correction",
          (LINE_MODE_TRACK == out.mode) &&
              (LINE_LEAD_DEG == out.target_heading_deg));
}

/* 2026-11-06 car9: with no sensor black, the learned bend fed on the turn
 * it caused and spun the car through 235 degrees. */
static void
test_bend_fades (void)
{
    reset();
    drive_bend(300, 300);
    double const speed = (double)MOTION_FOLLOW_SPEED_MM_S;
    double const step_mm = speed * SIM_STEP_S;
    double heading = (double)in.heading_deg;
    double rate = 0.3 * speed;
    double travel = (double)in.distance_mm;
    double heading_at = heading;
    unsigned int const steps = (unsigned int)(500.0 / step_mm);
    unsigned int const last = (unsigned int)(100.0 / step_mm);
    for (unsigned int i = 0u; i < steps; i++)
    {
        in.heading_deg = (int32_t)lround(heading);
        in.distance_mm = (int32_t)travel;
        out = line_track_update(&track, &in);
        double const asked =
            MOTION_FOLLOW_KP * ((double)out.target_heading_deg - heading);
        rate += (asked - rate) * SIM_STEP_S / SIM_LAG_S;
        heading += rate * SIM_STEP_S;
        travel += step_mm;
        if (i == (steps - last))
        {
            heading_at = heading;
        }
    }
    double const per_mm = (heading - heading_at) / 100.0;
    printf("  bend after 500 mm unconfirmed: %.2f deg/mm (was 0.30)\n", per_mm);
    check("LT21 a learned bend no sensor confirms fades instead of growing",
          (per_mm < 0.15) && (LINE_MODE_TRACK == out.mode));
}

int
main (void)
{
    test_debounce();
    test_right_lead();
    test_edge_clears();
    test_held_edge();
    test_brief_touch();
    test_junction();
    test_lost();
    test_search_found();
    test_search_between();
    test_search_both_sides();
    test_search_timeout();
    test_control_off();
    test_pivot_overshoot();
    test_bend();
    test_junction_left();
    test_both_rise_together();
    test_curve_model();
    test_long_both_black();
    test_bend_fades();
    test_junction_uturn();
    test_overshoot_across();
    printf("%u failure(s)\n", failures);
    return (0u == failures) ? 0 : 1;
}
