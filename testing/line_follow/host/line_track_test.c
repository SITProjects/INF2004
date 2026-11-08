/* Drive the real line_track module with simulated digital samples. */
#include "../../../app_program/line_follow/line_track.h"
#include "../../../app_program/line_follow/line_follow_config.h"
#include "../../../app_program/motion/motion_command_config.h"
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
    check("LT03 an edge leads until white; then the car holds its heading, "
          "so it runs straight past a barcode",
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
    /* Held on the heading it was met on, 4 degrees in cross_junction(). */
    bool const b_junction = out.b_junction && (4 == out.target_heading_deg) &&
                            (LINE_MODE_TRACK == out.mode);
    in.heading_deg = 6;
    drive(true, true, samples_for(15));
    /* The crossing tape cannot steer it: the target is the held heading. */
    bool const b_tracking = out.b_junction && (4 == out.target_heading_deg);
    /* Past the crossing, tracking carries on, not a blind drive. */
    drive(false, true, samples_for(LINE_JUNCTION_HOLD_MM));
    bool const b_edge = out.b_junction &&
                        (6 - LINE_LEAD_DEG == out.target_heading_deg) &&
                        (LINE_MODE_TRACK == out.mode);
    drive(false, false, LINE_CONFIRM_SAMPLES);
    check("LT06 a cross straight on (the default) is reported as a junction, "
          "held over the crossing tape, and tracking carries on after it",
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
    drive(true, false, samples_for(LINE_JUNCTION_HOLD_MM));
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
    bool const b_pivot = (LINE_MODE_SEARCH_LEFT == out.mode) &&
                         (4 + 90 == out.target_heading_deg) && out.b_junction;
    in.b_turning = true;
    /* Sensors sweep over tape on the way round: ignored. */
    in.heading_deg = 20;
    drive(false, true, LINE_CONFIRM_SAMPLES);
    in.heading_deg = 60;
    drive(true, false, LINE_CONFIRM_SAMPLES);
    drive(false, false, LINE_CONFIRM_SAMPLES);
    bool const b_ignored = (LINE_MODE_SEARCH_LEFT == out.mode) &&
                           (4 + 90 == out.target_heading_deg);
    in.heading_deg = 92;
    drive(false, false, 1u);
    bool const b_turning = (LINE_MODE_SEARCH_LEFT == out.mode);
    in.b_turning = false;
    drive(false, false, 1u);
    bool const b_on = (LINE_MODE_TRACK == out.mode) && !out.b_junction &&
                      (4 + 90 == out.target_heading_deg);
    drive(false, false, samples_for(300));
    check("LT15 a LEFT junction drives to the centre, pivots exactly 90 "
          "degrees past any tape, and carries straight on with no line seen",
          b_pivot && b_ignored && b_turning && b_on &&
              (LINE_MODE_TRACK == out.mode) &&
              (4 + 90 == out.target_heading_deg));
}

static void
test_junction_uturn (void)
{
    cross_junction(LINE_TURN_UTURN);
    drive(false, false, samples_for(LINE_JUNCTION_CENTRE_MM));
    in.b_turning = true;
    bool const b_pivot = (LINE_MODE_SEARCH_LEFT == out.mode) &&
                         (4 + 180 == out.target_heading_deg);
    in.heading_deg = 94;
    drive(true, false, LINE_CONFIRM_SAMPLES);
    drive(false, false, LINE_CONFIRM_SAMPLES);
    bool const b_ignored = (LINE_MODE_SEARCH_LEFT == out.mode);
    in.heading_deg = 182;
    in.b_turning = false;
    drive(false, false, 1u);
    bool const b_found =
        (LINE_MODE_TRACK == out.mode) && (184 == out.target_heading_deg);

    cross_junction(LINE_TURN_RIGHT);
    drive(false, false, samples_for(LINE_JUNCTION_CENTRE_MM));
    bool const b_right = (LINE_MODE_SEARCH_RIGHT == out.mode) &&
                         (4 - 90 == out.target_heading_deg);
    in.heading_deg = 4 - 90;
    drive(false, false, 1u);
    bool const b_right_on = (LINE_MODE_TRACK == out.mode) && !out.b_junction &&
                            (4 - 90 == out.target_heading_deg);

    cross_junction(LINE_TURN_LEFT);
    drive(false, false, samples_for(LINE_JUNCTION_CENTRE_MM));
    in.b_turning = true;
    drive(false, false, (LINE_SEARCH_TIMEOUT_MS / LINE_SAMPLE_MS) + 1u);
    in.b_turning = false;
    check("LT16 a U-turn pivots 180 and a RIGHT 90 and carry on; a pivot that "
          "never ends stops",
          b_pivot && b_ignored && b_found && b_right && b_right_on &&
              (LINE_MODE_STOP == out.mode) && !out.b_junction);
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

/* One touch: hold. The same sensor again soon: a bend, so seek, and look
 * for the line sooner. The other sensor ends the bend. */
static void
test_bend_seek (void)
{
    int const gap_mm = LINE_CURVE_TOUCH_MM / 2;
    reset();
    drive(true, false, LINE_CONFIRM_SAMPLES);
    drive(false, false, LINE_CONFIRM_SAMPLES + samples_for(gap_mm));
    bool const b_one = (0 == out.target_heading_deg);
    drive(true, false, LINE_CONFIRM_SAMPLES);
    drive(false, false, LINE_CONFIRM_SAMPLES + samples_for(LINE_SEEK_RAMP_MM));
    bool const b_seek = (LINE_SEEK_MAX_DEG == out.target_heading_deg);
    drive(false, true, LINE_CONFIRM_SAMPLES);
    bool const b_edge = (-LINE_LEAD_DEG == out.target_heading_deg);
    drive(false, false, LINE_CONFIRM_SAMPLES + samples_for(LINE_SEEK_RAMP_MM));
    check("LT14 a second touch of one sensor seeks that way; the other "
          "sensor ends it",
          b_one && b_seek && b_edge && (0 == out.target_heading_deg));

    reset();
    drive(true, false, LINE_CONFIRM_SAMPLES);
    drive(false, false, samples_for(LINE_CURVE_TOUCH_MM + 20));
    drive(true, false, LINE_CONFIRM_SAMPLES);
    drive(false, false, LINE_CONFIRM_SAMPLES + samples_for(LINE_SEEK_RAMP_MM));
    bool const b_far_apart = (0 == out.target_heading_deg);

    reset();
    drive(true, false, LINE_CONFIRM_SAMPLES);
    drive(false, false, LINE_CONFIRM_SAMPLES + samples_for(gap_mm));
    drive(true, false, LINE_CONFIRM_SAMPLES);
    drive(false, false, samples_for(LINE_LOST_MM) + LINE_CONFIRM_SAMPLES + 5u);
    check("LT21 touches far apart are no bend; in a bend the line counts as "
          "lost sooner",
          b_far_apart && (LINE_MODE_SEARCH_LEFT == out.mode));
}

static void
test_lost (void)
{
    reset();
    drive(false, false, samples_for(LINE_STRAIGHT_LOST_MM) + 5u);
    bool const b_no_edge = (LINE_MODE_TRACK == out.mode);
    drive(true, false, LINE_CONFIRM_SAMPLES);
    drive(true, false, samples_for(30));
    drive(false, false, LINE_CONFIRM_SAMPLES);
    drive(false, false, samples_for(LINE_STRAIGHT_LOST_MM) - 5u);
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
    drive(false, false,
          LINE_CONFIRM_SAMPLES + samples_for(LINE_STRAIGHT_LOST_MM));
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
 * LT19, LT22: a model of the car, fitted to the 2026-11-06 car10 run.
 * Sensors 130 mm ahead of the axle over 18 mm tape, about 20 mm of white
 * between the touches of the two sensors (car11), so a gap near 36 mm. The
 * line task samples every 20 ms (a 10 ms delay rounds up to two ticks);
 * motion acts on a request two samples late at MOTION_FOLLOW_KP times the
 * error, capped, and the car's turn rate lags that. A pivot search stops
 * the car going forward. It does not hold with the sensors closer than the
 * tape is wide: white then means off the tape.
 */
#define SIM_AHEAD_MM 130.0
#define SIM_HALF_TAPE_MM 9.0
#define SIM_STEP_S 0.02
#define SIM_DELAY_SAMPLES 2u
#define SIM_PI 3.14159265358979

static double sim_radius_mm;
static bool sim_u_turn;         /* The arc goes on round, back along y = 2R. */
static double sim_right_hold_s; /* The right sensor stays black this long. */
static double sim_right_white_s;

/* Distance from (x, y) to the tape: +x up to the origin, the arc about
 * (0, R), then +y from (R, R). */
static double
tape_distance (double x, double y)
{
    if (sim_u_turn)
    {
        return (x <= 0.0) ? fmin(fabs(y), fabs(y - (2.0 * sim_radius_mm)))
                          : fabs(hypot(x, y - sim_radius_mm) - sim_radius_mm);
    }
    if (x <= 0.0)
    {
        return fabs(y);
    }
    if (y >= sim_radius_mm)
    {
        return fabs(x - sim_radius_mm);
    }
    return fabs(sqrt((x * x) + ((y - sim_radius_mm) * (y - sim_radius_mm))) -
                sim_radius_mm);
}

/* The modelled car. */
typedef struct
{
    double x;
    double y;
    double heading; /* Degrees, + is left. */
    double rate;
    double travel;
    double requests[SIM_DELAY_SAMPLES + 1u];
    double half_gap;
    double lag_s;
    double start_deg; /* Heading the odometry calls 0. */
} sim_car_t;

static double sim_speed_mm_s = (double)MOTION_FOLLOW_SPEED_MM_S;

/* Sensors on the tape, the car angled across it, @p back mm before the
 * tape's bend. */
static sim_car_t
sim_start (double back, double half_gap, double lag_s, double entry_deg)
{
    sim_car_t car = {.x = -back - SIM_AHEAD_MM,
                     .y = -SIM_AHEAD_MM * sin(entry_deg * SIM_PI / 180.0),
                     .heading = entry_deg,
                     .half_gap = half_gap,
                     .lag_s = lag_s};
    for (unsigned int k = 0u; k <= SIM_DELAY_SAMPLES; k++)
    {
        car.requests[k] = entry_deg;
    }
    return car;
}

/* One line task sample; returns how far the sensors are off the tape. */
static double
sim_step (sim_car_t * p_car)
{
    double const speed = sim_speed_mm_s;
    double const step_mm = speed * SIM_STEP_S;
    /* Motion's cap: the follow limit, or where the inner wheel stops. */
    double const cap =
        fmin((double)MOTION_FOLLOW_MAX_DEG_S,
             (2.0 * speed) / ((double)MOTION_TRACK_WIDTH_MM * SIM_PI / 180.0));
    double const rad = p_car->heading * SIM_PI / 180.0;
    double const sx = p_car->x + (SIM_AHEAD_MM * cos(rad));
    double const sy = p_car->y + (SIM_AHEAD_MM * sin(rad));
    double const half_gap = p_car->half_gap;
    in.b_left_black =
        tape_distance(sx - (half_gap * sin(rad)), sy + (half_gap * cos(rad))) <=
        SIM_HALF_TAPE_MM;
    in.b_right_black =
        tape_distance(sx + (half_gap * sin(rad)), sy - (half_gap * cos(rad))) <=
        SIM_HALF_TAPE_MM;
    sim_right_white_s = in.b_right_black ? 0.0 : sim_right_white_s + SIM_STEP_S;
    in.b_right_black =
        in.b_right_black || (sim_right_white_s < sim_right_hold_s);
    in.heading_deg = (int32_t)lround(p_car->heading - p_car->start_deg);
    in.distance_mm = (int32_t)p_car->travel;
    out = line_track_update(&track, &in);

    for (unsigned int k = SIM_DELAY_SAMPLES; k > 0u; k--)
    {
        p_car->requests[k] = p_car->requests[k - 1u];
    }
    p_car->requests[0] = (double)out.target_heading_deg;
    /* Motion scales the gain with speed (set_follow_gains()). */
    double const asked =
        MOTION_FOLLOW_KP * (speed / (double)MOTION_FOLLOW_SPEED_MM_S) *
        (p_car->requests[SIM_DELAY_SAMPLES] + p_car->start_deg -
         p_car->heading);
    p_car->rate += (fmin(fmax(asked, -cap), cap) - p_car->rate) * SIM_STEP_S /
                   p_car->lag_s;
    p_car->heading += p_car->rate * SIM_STEP_S;
    if (LINE_MODE_TRACK == out.mode)
    {
        p_car->x += step_mm * cos(p_car->heading * SIM_PI / 180.0);
        p_car->y += step_mm * sin(p_car->heading * SIM_PI / 180.0);
        p_car->travel += step_mm;
    }
    return tape_distance(sx, sy);
}

/* Follow a curve of @p radius mm with the sensors 2 * @p half_gap mm
 * apart, the car's turn rate lagging by @p lag_s and starting @p entry_deg
 * off the tape's direction. */
static bool
curve_model (double radius, double half_gap, double lag_s, double entry_deg)
{
    reset();
    sim_radius_mm = radius;
    sim_car_t car = sim_start(300.0, half_gap, lag_s, entry_deg);
    double worst = 0.0;
    unsigned int const steps =
        (unsigned int)(1400.0 /
                       ((double)MOTION_FOLLOW_SPEED_MM_S * SIM_STEP_S));
    for (unsigned int i = 0u; i < steps; i++)
    {
        worst = fmax(worst, sim_step(&car));
    }
    printf("  curve model, radius %.0f, gap %.0f, lag %.2f: sensors at most "
           "%.0f mm off the tape, heading %.0f deg, %.0f mm up the exit\n",
           radius, 2.0 * half_gap, lag_s, worst, car.heading, car.y - radius);
    return (worst < 40.0) && (car.heading > 70.0) && (car.heading < 110.0) &&
           (car.y > radius + 300.0);
}

static void
test_curve_model (void)
{
    /* The fit to car10, then wider curves: the car's 30 mm gap, and more.
     * 20 mm, once allowed for, loses both curves at 120 mm/s. */
    bool const b_tight = curve_model(100.0, 15.0, 0.5, 0.0);
    bool const b_gap = curve_model(100.0, 18.0, 0.5, 0.0);
    bool const b_wide = curve_model(200.0, 15.0, 0.25, -8.0);
    bool const b_entry = curve_model(200.0, 18.0, 0.25, -8.0);
    check("LT19 the car model follows a left quarter circle onto the exit "
          "straight: the 100 mm radius fitted to car10, and 200 mm met 8 "
          "degrees off, with the sensors 30 and 36 mm apart",
          b_tight && b_gap && b_wide && b_entry);
}

/* A left U at @p speed, the right sensor @p hold_s slow to let go: true if
 * the car comes out on the straight and stays on it. */
static bool
u_exit_model (double speed, double hold_s, double half_gap, double lag_s)
{
    reset();
    sim_speed_mm_s = speed;
    sim_radius_mm = 165.0;
    sim_u_turn = true;
    sim_right_hold_s = hold_s;
    sim_right_white_s = hold_s;
    sim_car_t car = sim_start(400.0, half_gap, lag_s, 0.0);
    double worst = 0.0;
    unsigned int const steps = (unsigned int)(2400.0 / (speed * SIM_STEP_S));
    for (unsigned int i = 0u; i < steps; i++)
    {
        double const off = sim_step(&car);
        if ((car.x < -250.0) && (car.heading > 90.0))
        {
            worst = fmax(worst, off);
        }
    }
    sim_u_turn = false;
    sim_right_hold_s = 0.0;
    sim_speed_mm_s = (double)MOTION_FOLLOW_SPEED_MM_S;
    printf("  U exit model at %.0f mm/s: sensors at most %.0f mm off the exit "
           "straight, heading %.0f deg, %.0f mm along it\n",
           speed, worst, car.heading, -car.x);
    return (worst < 24.0) && (car.x < -500.0) && (LINE_MODE_TRACK == out.mode);
}

static void
test_u_exit_model (void)
{
    /* The sensor as it was on 2026-11-07: at 120 mm/s no gain or lag gets
     * round with it, so this guards the steering back at the old speed. */
    check("LT44 the car model comes out of a left U onto the straight at "
          "80 mm/s with the right sensor 0.34 s slow to let go",
          u_exit_model(80.0, 0.34, 18.0, 0.35));
    /* Fails from 0.35 s of turn lag, and from 0.1 s of sensor delay. */
    check("LT48 and at the follow speed, with the sensors as they are now",
          u_exit_model((double)MOTION_FOLLOW_SPEED_MM_S, 0.05, 15.0, 0.25));
}

/* Heading swing on a long straight, met 3 degrees off, once settled. */
static void
test_straight_model (void)
{
    reset();
    sim_radius_mm = 1.0e6;
    sim_car_t car = sim_start(5000.0, 18.0, 0.25, 3.0);
    car.start_deg = 3.0; /* The car does not know it is crooked. */
    for (unsigned int k = 0u; k <= SIM_DELAY_SAMPLES; k++)
    {
        car.requests[k] = 0.0;
    }
    double swing = 0.0;
    double travel_at = 0.0;
    unsigned int touches = 0u;
    bool b_was_touching = false;
    while (car.travel < 2000.0)
    {
        (void)sim_step(&car);
        bool const b_touching = in.b_left_black || in.b_right_black;
        if (car.travel > 300.0)
        {
            swing = fmax(swing, fabs(car.heading));
            touches += (b_touching && !b_was_touching) ? 1u : 0u;
            travel_at = car.travel;
        }
        b_was_touching = b_touching;
    }
    double const per_m = (double)touches * 1000.0 / (travel_at - 300.0);
    printf("  straight model: heading within +-%.1f deg, %.1f touches per m\n",
           swing, per_m);
    check("LT22 the car model holds a straight: swing under 4 degrees and "
          "under 12 touches per metre",
          (swing < 4.0) && (per_m < 12.0) && (LINE_MODE_TRACK == out.mode));
}

/* The slow pass, across the worst of a real one: met 3 degrees off. */
static void
test_creep_model (void)
{
    reset();
    sim_radius_mm = 1.0e6;
    sim_speed_mm_s = (double)MOTION_COMMAND_SLOW_SPEED_MM_S;
    sim_car_t car = sim_start(5000.0, 18.0, 0.25, 3.0);
    car.start_deg = 3.0;
    for (unsigned int k = 0u; k <= SIM_DELAY_SAMPLES; k++)
    {
        car.requests[k] = 0.0;
    }
    double swing = 0.0;
    while (car.travel < (double)LINE_BARCODE_CREEP_MM)
    {
        track.barcode_phase = LINE_BARCODE_CREEP;
        track.barcode_mm = (int32_t)car.travel;
        (void)sim_step(&car);
        if (car.travel > 100.0)
        {
            swing = fmax(swing, fabs(car.heading));
        }
    }
    sim_speed_mm_s = (double)MOTION_FOLLOW_SPEED_MM_S;
    printf("  creep model: heading within +-%.1f deg\n", swing);
    check("LT27 the car model creeps straight at the barcode speed: swing "
          "under 4 degrees and no search",
          (swing < 4.0) && (LINE_MODE_TRACK == out.mode));
}

/* Step the tracker through a stop for a barcode, up to the creep. */
static void
start_creep (void)
{
    in.b_barcode = true;
    drive(false, false, 1u);
    in.b_barcode = false;
    drive(false, false, LINE_BARCODE_BRAKE_SAMPLES);
    in.b_moving = true;
    drive(false, false, 5u);
    in.b_moving = false;
    drive(false, false, 1u);
}

static void
test_barcode_stop (void)
{
    reset();
    drive(false, false, 5u);
    bool const b_quiet =
        (0u == out.reverse_mm) && !out.b_slow && (LINE_MODE_TRACK == out.mode);
    in.heading_deg = 7;
    in.b_barcode = true;
    drive(false, false, 1u);
    in.b_barcode = false;
    bool const b_brake = (LINE_MODE_STOP == out.mode) &&
                         (LINE_BARCODE_BRAKE == line_track_barcode_phase(&track));
    drive(true, true, LINE_BARCODE_BRAKE_SAMPLES);
    /* The sensors are ignored all the way back. */
    bool const b_back = (LINE_BARCODE_REVERSE_MM == out.reverse_mm) &&
                        (LINE_MODE_TRACK == out.mode) &&
                        !track.b_both_seen;
    in.b_moving = true;
    drive(true, false, 5u);
    bool const b_still_back = (LINE_BARCODE_REVERSE_MM == out.reverse_mm);
    in.b_moving = false;
    drive(false, false, 1u);
    check("LT23 a barcode brakes the car, backs it up once, then creeps on "
          "the same heading",
          b_quiet && b_brake && b_back && b_still_back && out.b_slow &&
              (0u == out.reverse_mm) && (LINE_MODE_TRACK == out.mode) &&
              (7 == out.target_heading_deg));

    in.b_barcode_done = true;
    drive(false, false, 1u);
    in.b_barcode_done = false;
    bool const b_done = !out.b_slow &&
                        (LINE_BARCODE_NONE == line_track_barcode_phase(&track));
    reset();
    start_creep();
    bool const b_creeping = out.b_slow;
    drive(false, false, samples_for(LINE_BARCODE_CREEP_MM - 10));
    bool const b_before = out.b_slow;
    drive(false, false, samples_for(20));
    check("LT24 the reader ends the creep, or else the travel limit does",
          b_done && b_creeping && b_before && !out.b_slow);

    /* A reverse that never starts must not strand the car. */
    reset();
    in.b_barcode = true;
    drive(false, false, 1u);
    in.b_barcode = false;
    drive(false, false, LINE_BARCODE_BRAKE_SAMPLES);
    bool const b_waiting = (LINE_BARCODE_REVERSE_MM == out.reverse_mm);
    drive(false, false, LINE_BARCODE_BACK_SAMPLES);
    check("LT25 a reverse that never starts gives up and creeps anyway",
          b_waiting && out.b_slow && (0u == out.reverse_mm));

    /* Only plain tracking takes it. */
    reset();
    drive(true, false, 3u);
    drive(false, false, 3u);
    drive(true, false, 3u);
    bool const b_bend = track.b_curve;
    in.b_barcode = true;
    drive(true, false, 1u);
    in.b_barcode = false;
    bool const b_bend_refused =
        b_bend && (LINE_BARCODE_NONE == line_track_barcode_phase(&track));
    reset();
    in.b_barcode = true;
    in.b_control = false;
    drive(false, false, 1u);
    in.b_barcode = false;
    bool const b_bypass_refused =
        LINE_BARCODE_NONE == line_track_barcode_phase(&track);
    /* The bypass or Stop takes over partway: no slow pass survives it. */
    in.b_control = true;
    drive(false, false, 1u);
    start_creep();
    bool const b_started = out.b_slow;
    in.b_control = false;
    drive(false, false, 1u);
    in.b_control = true;
    drive(false, false, 1u);
    check("LT26 a bend or a bypass refuses a slow pass, and losing control "
          "ends one",
          b_bend_refused && b_bypass_refused && b_started && !out.b_slow &&
              (0u == out.reverse_mm));

    /* Turning means a bend, and the barcode sensor crossing the tape. */
    bool b_turning_refused = true;
    for (int sign = -1; sign <= 1; sign += 2)
    {
        reset();
        drive(false, false, samples_for(60));
        unsigned int const samples = samples_for(96);
        for (unsigned int i = 1u; i <= samples; i++)
        {
            in.heading_deg = (sign * 40 * (int)i) / (int)samples;
            drive(false, false, 1u);
        }
        in.b_barcode = true;
        drive(false, false, 1u);
        in.b_barcode = false;
        b_turning_refused =
            b_turning_refused &&
            (LINE_BARCODE_NONE == line_track_barcode_phase(&track));
    }
    drive(false, false, samples_for(120));
    in.b_barcode = true;
    drive(false, false, 1u);
    in.b_barcode = false;
    check("LT34 a car still turning takes no slow pass; once straight it does",
          b_turning_refused &&
              (LINE_BARCODE_BRAKE == line_track_barcode_phase(&track)));

    /* A slow pass spots a bend at the distance scaled for its speed. */
    reset();
    start_creep();
    drive(true, false, 3u);
    drive(false, false, samples_for(20));
    drive(true, false, 3u);
    bool const b_straight = !track.b_curve;
    reset();
    drive(true, false, 3u);
    drive(false, false, samples_for(20));
    drive(true, false, 3u);
    check("LT28 touches that are a bend at full speed are a straight in a "
          "slow pass",
          b_straight && track.b_curve);

    /* 2026-11-07: both sensors white, and the car still turning. */
    reset();
    start_creep();
    drive(true, false, 3u);
    drive(false, false, samples_for(20));
    drive(true, false, 3u);
    drive(false, false, LINE_CONFIRM_SAMPLES + samples_for(40));
    bool const b_holds = (out.target_heading_deg == in.heading_deg);
    in.b_barcode_done = true;
    drive(false, false, 1u);
    in.b_barcode_done = false;
    drive(false, false, samples_for(40));
    check("LT91 a creep holds its heading between touches, and still does as "
          "it speeds up",
          b_holds && !out.b_slow &&
              (out.target_heading_deg == in.heading_deg));

    reset();
    in.b_hump = true;
    drive(false, false, 1u);
    bool const b_on_hump = out.b_slow;
    drive(true, false, 3u);
    drive(false, false, samples_for(20));
    drive(true, false, 3u);
    check("LT92 so does a car slowed for a hump", b_on_hump && !track.b_curve);

    /* The tape crossing out of a bend, as cross_bend() does, in a creep: in
     * on the right too long before the left to be a junction. */
    reset();
    start_creep();
    for (int deg = 0; deg <= 50; deg += 5)
    {
        in.heading_deg = deg;
        drive(false, false, samples_for(12));
    }
    drive(false, true, LINE_CONFIRM_SAMPLES + samples_for(20));
    drive(true, true, LINE_CONFIRM_SAMPLES);
    drive(true, false, LINE_CONFIRM_SAMPLES);
    drive(false, false, LINE_CONFIRM_SAMPLES + samples_for(10));
    bool const b_creep_straight = !track.b_curve;
    in.heading_deg = 0;
    check("LT93 nor does the tape leaving a sensor read as a bend in a creep",
          out.b_slow && b_creep_straight);
}

/* @p samples samples of both black; true if any reported a junction. */
static bool
drive_both_junction (unsigned int samples)
{
    bool b_junction = false;
    for (unsigned int i = 0u; i < samples; i++)
    {
        drive(true, true, 1u);
        b_junction = b_junction || out.b_junction;
    }
    return b_junction;
}

/* 2026-11-08: creeping left of the line, the right sensor on the tape and the
 * left on the barcode's bars (LINE_BARCODE_ON_LEFT) for 108 mm. */
static void
test_creep_bars (void)
{
    reset();
    start_creep();
    drive(false, true, LINE_CONFIRM_SAMPLES + 2u);
    bool const b_bars_junction = drive_both_junction(samples_for(108));
    check("LT94 in a creep, bars under the left with the right on the tape are "
          "no junction: the car steers right, to the tape",
          !b_bars_junction && out.b_slow && (LINE_MODE_TRACK == out.mode) &&
              (-LINE_LEAD_DEG == out.target_heading_deg));

    drive(true, false, LINE_CONFIRM_SAMPLES + samples_for(20));
    bool const b_seeking = (-LINE_LEAD_DEG == out.target_heading_deg);
    drive(false, false, LINE_CONFIRM_SAMPLES + samples_for(20));
    check("LT95 the right off the tape first: the car is too far left, and "
          "turns right until the right finds it",
          b_seeking && (LINE_MODE_TRACK == out.mode) &&
              (-LINE_LEAD_DEG == out.target_heading_deg));

    reset();
    start_creep();
    drive(false, true, LINE_CONFIRM_SAMPLES + 2u);
    (void)drive_both_junction(samples_for(40));
    in.b_barcode_done = true;
    drive(true, true, 1u);
    in.b_barcode_done = false;
    bool const b_ended = !out.b_slow;
    drive(true, false, LINE_CONFIRM_SAMPLES + 2u);
    bool const b_to_tape = (-LINE_LEAD_DEG == out.target_heading_deg);
    bool const b_after_junction = drive_both_junction(samples_for(40));
    drive(false, true, LINE_CONFIRM_SAMPLES + samples_for(10));
    drive(false, false, LINE_CONFIRM_SAMPLES + samples_for(10));
    drive(true, false, LINE_CONFIRM_SAMPLES);
    check("LT96 the bars stay ignored past the creep's end until they clear, "
          "as the right comes and goes; black there later is the tape again",
          b_ended && b_to_tape && !b_after_junction &&
              (LINE_LEAD_DEG == out.target_heading_deg));

    reset();
    start_creep();
    drive(true, false, LINE_CONFIRM_SAMPLES);
    check("LT97 in a creep, the left black with the right white is the tape",
          out.b_slow && (LINE_LEAD_DEG == out.target_heading_deg));

    reset();
    start_creep();
    bool const b_square = drive_both_junction(LINE_CONFIRM_SAMPLES);
    reset();
    drive(false, true, LINE_CONFIRM_SAMPLES + 2u);
    bool const b_following = drive_both_junction(LINE_CONFIRM_SAMPLES);
    check("LT98 both black at once in a creep, or the same order as the bars "
          "outside one, are still junctions",
          b_square && b_following);
}

/* 2026-11-06 20:32:07: 7 to 57 degrees in the 120 mm before the crossing. */
#define BEND_DEG 50

/* Turn @p deg over the last @p mm (+ is left), then the tape crosses from
 * the @p b_from_right sensor, leaves the other, and both read white for
 * @p white_mm. 2026-11-06 20:32:07 read 01 11 10 00. */
static void
cross_bend (int deg, int mm, bool b_from_right, int white_mm)
{
    reset();
    drive(false, false, samples_for(60));
    unsigned int const samples = samples_for(mm);
    for (unsigned int i = 1u; i <= samples; i++)
    {
        in.heading_deg = (deg * (int)i) / (int)samples;
        drive(false, false, 1u);
    }
    drive(!b_from_right, b_from_right, samples_for(15));
    drive(true, true, samples_for(8));
    drive(b_from_right, !b_from_right, samples_for(4));
    drive(false, false, samples_for(white_mm));
}

/* In a left bend (right if @p sign is -1): two touches of the bend's sensor,
 * the car turning that way, seeking on white. */
static void
in_bend (int sign)
{
    bool const b_left = sign > 0;
    reset();
    drive(false, false, samples_for(100));
    drive(b_left, !b_left, samples_for(8));
    drive(false, false, samples_for(30));
    drive(b_left, !b_left, samples_for(8));
    unsigned int const samples = samples_for(100);
    for (unsigned int i = 1u; i <= samples; i++)
    {
        in.heading_deg = (sign * 40 * (int)i) / (int)samples;
        drive(false, false, 1u);
    }
}

static void
test_counter_touch (void)
{
    bool b_gentle = true;
    bool b_seeks_on = true;
    for (int sign = -1; sign <= 1; sign += 2)
    {
        in_bend(sign);
        bool const b_in_bend = track.b_curve;
        drive(sign < 0, sign > 0, samples_for(10));
        b_gentle = b_gentle && b_in_bend &&
                   ((in.heading_deg - (sign * LINE_COUNTER_LEAD_DEG)) ==
                    out.target_heading_deg);
        drive(false, false, samples_for(30));
        b_seeks_on = b_seeks_on && track.b_curve &&
                     ((sign * (out.target_heading_deg - in.heading_deg)) >=
                      (LINE_SEEK_MAX_DEG - 1));
    }
    check("LT39 the other sensor touching in a bend, left or right, steers "
          "back gently and the bend carries on",
          b_gentle && b_seeks_on);

    /* Still turning the bend's way all the while, so only the limit ends it. */
    in_bend(1);
    unsigned int const long_touch = samples_for(LINE_COUNTER_MM + 5);
    for (unsigned int i = 1u; i <= long_touch; i++)
    {
        in.heading_deg = 40 + ((20 * (int)i) / (int)long_touch);
        drive(false, true, 1u);
    }
    check("LT40 black for longer than that is a real turn: full lead, bend off",
          ((in.heading_deg - LINE_LEAD_DEG) == out.target_heading_deg) &&
              !track.b_curve);

    /* The car straightens: the turn falls away while the touch goes on. */
    bool b_stays = true;
    bool b_ends = true;
    for (int sign = -1; sign <= 1; sign += 2)
    {
        in_bend(sign);
        drive(sign < 0, sign > 0, samples_for(10));
        in.heading_deg = sign * 30;
        drive(sign < 0, sign > 0, samples_for(LINE_COUNTER_MM - 20));
        b_stays = b_stays && track.b_curve &&
                  ((in.heading_deg - (sign * LINE_COUNTER_LEAD_DEG)) ==
                   out.target_heading_deg);
        drive(false, false, samples_for(30));
        b_ends = b_ends && !track.b_curve &&
                 (in.heading_deg == out.target_heading_deg);
    }
    check("LT42 a touch begun against a bend stays gentle as the car "
          "straightens, left or right",
          b_stays);
    check("LT43 such a touch clearing after LINE_COUNTER_END_MM ends the bend: "
          "the car holds its heading, no seek",
          b_ends);

    /* The end of the right turn on 2026-11-07, and a touch in mid U. */
    bool b_exit_ends = true;
    bool b_mid_stays = true;
    for (int sign = -1; sign <= 1; sign += 2)
    {
        in_bend(sign);
        drive(sign < 0, sign > 0, samples_for(52));
        drive(false, false, samples_for(30));
        b_exit_ends = b_exit_ends && !track.b_curve &&
                      (in.heading_deg == out.target_heading_deg) &&
                      (track.last_side == ((sign < 0) ? 0u : 1u));
        in_bend(sign);
        drive(sign < 0, sign > 0, samples_for(37));
        drive(false, false, samples_for(30));
        b_mid_stays = b_mid_stays && track.b_curve;
    }
    check("LT46 the 52 mm touch that ended a right turn ends the bend, and "
          "the tape counts as last seen on that side",
          b_exit_ends);
    check("LT47 the 37 mm touch in the middle of a U leaves the bend on",
          b_mid_stays);

    /* Someone else drives for a moment, the sensor still black. */
    in_bend(1);
    drive(false, true, samples_for(10));
    in.b_control = false;
    drive(false, true, 1u);
    in.b_control = true;
    drive(false, true, 1u);
    check("LT45 a restart forgets a touch steering back: full lead again",
          (in.heading_deg - LINE_LEAD_DEG) == out.target_heading_deg);

    /* Turning, but no bend: a correction too far gets full lead back. */
    reset();
    drive(false, false, samples_for(60));
    unsigned int const samples = samples_for(90);
    for (unsigned int i = 1u; i <= samples; i++)
    {
        in.heading_deg = (-30 * (int)i) / (int)samples;
        drive(false, false, 1u);
    }
    drive(true, false, samples_for(10));
    check("LT41 with no bend, a sensor always steers at full lead",
          (in.heading_deg + LINE_LEAD_DEG) == out.target_heading_deg);
}

static void
test_hump_slow (void)
{
    reset();
    drive(false, false, samples_for(150));
    check("LT49 no hump, full speed", !out.b_slow);
    in.b_hump = true;
    drive(false, false, 2u);
    check("LT50 a hump on a straight slows the car at once", out.b_slow);

    /* The real hump: up to 15 degrees of wobble, touches on both sides. */
    bool b_held = true;
    for (int i = 0; i < 6; i++)
    {
        in.heading_deg = ((i % 2) == 0) ? 15 : -15;
        drive((i % 2) == 0, (i % 2) != 0, samples_for(30));
        b_held = b_held && out.b_slow;
    }
    /* Two touches on one side, close together: a bend, as far as it knows. */
    drive(true, false, samples_for(8));
    drive(false, false, samples_for(10));
    drive(true, false, samples_for(8));
    check("LT51 wobbling on it does not end the slow pass",
          b_held && out.b_slow);
    in.heading_deg = 0;
    in.b_hump = false;
    drive(false, false, 2u);
    check("LT52 level ground ends it", !out.b_slow);

    in_bend(1);
    in.b_hump = true;
    drive(false, false, 2u);
    check("LT53 a lean in a bend does not slow the car", !out.b_slow);

    /* Leaning for good, as if calibrated on a slope. */
    reset();
    drive(false, false, samples_for(150));
    in.b_hump = true;
    drive(false, false, LINE_HUMP_SLOW_SAMPLES + 2u);
    bool const b_ended = !out.b_slow;
    drive(false, false, 50u);
    check("LT54 a hump that never ends cannot hold the car slow",
          b_ended && !out.b_slow);
    in.b_hump = false;
    drive(false, false, 2u);
    in.b_hump = true;
    drive(false, false, 2u);
    check("LT55 after level ground the next hump slows it again", out.b_slow);

    in.b_control = false;
    drive(false, false, 1u);
    check("LT56 losing control ends it, the hump or not", !out.b_slow);
    in.b_control = true;
    in.b_hump = false;

    reset();
    drive(false, false, samples_for(150));
    in.b_barcode = true;
    drive(false, false, 1u);
    in.b_barcode = false;
    in.b_hump = true;
    drive(false, false, 2u);
    check("LT57 a lean during a barcode stop is left alone", !out.b_slow);
    in.b_hump = false;

    reset();
    in.turn = LINE_TURN_LEFT;
    drive(false, false, samples_for(150));
    drive(true, true, samples_for(12));
    in.b_hump = true;
    drive(false, false, 2u);
    bool const b_junction = out.b_junction;
    check("LT58 a lean while taking a junction is left alone",
          b_junction && !out.b_slow);
    in.b_hump = false;
    in.turn = LINE_TURN_STRAIGHT;

    /* A slow car touches again sooner: the same rule as a barcode creep. */
    reset();
    drive(false, false, samples_for(150));
    in.b_hump = true;
    drive(false, false, 2u);
    drive(true, false, samples_for(8));
    drive(false, false, samples_for(50));
    drive(true, false, samples_for(8));
    check("LT59 touches 50 mm apart are no bend at the slow speed",
          out.b_slow && !track.b_curve);
    in.b_hump = false;
}

/* Cross a junction and drive to its centre, for @p turn. The right sensor
 * meets the tape @p skew_mm before the left. */
static void
reach_pivot (line_turn_t turn, int skew_mm)
{
    reset();
    in.turn = turn;
    drive(false, false, samples_for(150));
    if (skew_mm > 0)
    {
        drive(false, true, samples_for(skew_mm));
    }
    drive(true, true, samples_for(12));
    drive(false, false, samples_for(LINE_JUNCTION_CENTRE_MM));
    in.turn = LINE_TURN_STRAIGHT;
}

/* The pivot, 2 degrees a sample up to @p last_deg off the start, with the
 * sensor on the far side, or the near one, black from @p black_deg until
 * @p white_deg; below 0 is before the car starts to turn. Its level changes one
 * sample late, so a white_deg of 20 counts at 22. */
static int32_t
pivot_sensor (line_turn_t turn, bool b_far, int black_deg, int white_deg,
              int last_deg)
{
    reach_pivot(turn, 0);
    bool const b_left = (LINE_MODE_SEARCH_LEFT == out.mode);
    in.b_turning = true;
    for (int deg = -4; deg <= last_deg; deg += 2)
    {
        int const turned = (deg > 0) ? deg : 0;
        bool const b_black = (deg >= black_deg) && (deg < white_deg);
        in.heading_deg = b_left ? turned : -turned;
        in.b_left_black = b_black && (b_left != b_far);
        in.b_right_black = b_black && (b_left == b_far);
        out = line_track_update(&track, &in);
    }
    in.b_turning = false;
    in.heading_deg = 0;
    return out.target_heading_deg;
}

static int32_t
pivot_target (line_turn_t turn, int black_deg, int white_deg, int last_deg)
{
    return pivot_sensor(turn, true, black_deg, white_deg, last_deg);
}

/* A bypass: a pivot of @p deg under motion's own control, then a move. */
static void
pivot_then_move (int deg)
{
    reset();
    in.b_control = false;
    in.b_turning = true;
    in.heading_deg = deg / 2;
    out = line_track_update(&track, &in);
    in.heading_deg = deg;
    out = line_track_update(&track, &in);
    in.b_turning = false;
    in.b_moving = true;
}

/* Coming back to the line, as on 2026-11-07. */
static void
test_return_sensor (void)
{
    pivot_then_move(60);
    drive(false, true, 1u);
    bool const b_far = out.b_seen;
    drive(true, false, 1u);
    check("LT74 back from the right, only the left sensor finds the line",
          !b_far && out.b_seen);

    pivot_then_move(-60);
    drive(true, false, 1u);
    bool const b_far_left = out.b_seen;
    drive(false, true, 1u);
    check("LT75 back from the left, only the right sensor finds it",
          !b_far_left && out.b_seen);

    reset();
    in.b_control = false;
    in.b_moving = true;
    drive(false, true, 1u);
    bool const b_no_pivot = out.b_seen;
    pivot_then_move(60);
    in.b_moving = false;
    drive(false, true, 1u);
    check("LT76 otherwise either sensor counts", b_no_pivot && out.b_seen);

    pivot_then_move(60);
    in.heading_deg = 58;
    drive(false, true, 1u);
    check("LT77 the heading drifting on the move changes nothing", !out.b_seen);

    /* Out to the right, then back left: a bypass's own pivots. */
    pivot_then_move(60);
    in.b_moving = false;
    in.b_turning = true;
    in.heading_deg = 40;
    out = line_track_update(&track, &in);
    in.heading_deg = 20;
    out = line_track_update(&track, &in);
    in.b_turning = false;
    in.b_moving = true;
    drive(true, false, 1u);
    bool const b_older = out.b_seen;
    drive(false, true, 1u);
    check("LT78 the last pivot decides", !b_older && out.b_seen);
}

/* A bypass ending with a pivot of @p deg, and control back. */
static void
end_of_bypass (int deg)
{
    reset();
    in.b_control = false;
    in.b_turning = true;
    in.heading_deg = deg;
    out = line_track_update(&track, &in);
    in.heading_deg = 0;
    out = line_track_update(&track, &in);
    in.b_turning = false;
    in.b_control = true;
    in.heading_deg = 0;
    drive(false, false, 1u);
}

/* Touch the right sensor twice, close together. */
static void
touch_twice (void)
{
    drive(false, true, LINE_CONFIRM_SAMPLES + samples_for(5));
    drive(false, false, LINE_CONFIRM_SAMPLES + samples_for(20));
    drive(false, true, LINE_CONFIRM_SAMPLES + samples_for(5));
}

/* Finding the line again, as on 2026-11-07. */
static void
test_rejoin (void)
{
    end_of_bypass(60); /* A pivot right, back along the line. */
    bool const b_left = (LINE_MODE_SEARCH_LEFT == out.mode);
    end_of_bypass(-60);
    bool const b_right = (LINE_MODE_SEARCH_RIGHT == out.mode);
    check("LT79 a bypass ends by looking for the line, towards where it came "
          "from",
          b_left && b_right);

    end_of_bypass(60);
    drive(false, true, LINE_CONFIRM_SAMPLES);
    check("LT80 with the tape between the sensors, it carries on along it",
          (LINE_MODE_TRACK == out.mode) && (0 == out.target_heading_deg));

    reset();
    in.b_control = false;
    in.b_moving = true;
    drive(false, false, 2u);
    in.b_moving = false;
    in.b_control = true;
    drive(false, false, 1u);
    check("LT81 a move with no pivot, the Reverse button, needs no search",
          LINE_MODE_TRACK == out.mode);

    end_of_bypass(60);
    drive(false, true, LINE_CONFIRM_SAMPLES);
    touch_twice();
    drive(false, false, LINE_CONFIRM_SAMPLES + samples_for(40));
    bool const b_held = (out.target_heading_deg == in.heading_deg) ||
                        (0 == out.target_heading_deg);
    drive(false, false, samples_for(LINE_REJOIN_MM));
    touch_twice();
    drive(false, false, LINE_CONFIRM_SAMPLES + samples_for(40));
    check("LT82 just after rejoining two touches are no bend; further on they "
          "are",
          b_held && (out.target_heading_deg ==
                     (in.heading_deg - LINE_SEEK_MAX_DEG)));

    /* A lap of 730 mm, touching tape every 100 mm. */
    reset();
    drive(true, false, LINE_CONFIRM_SAMPLES);
    drive(false, false, LINE_CONFIRM_SAMPLES);
    bool b_search = false;
    int mm = 0;
    for (; (mm < (2 * LINE_LOOP_MM)) && !b_search; mm += 10)
    {
        in.heading_deg = (mm * 360) / 730;
        drive((mm % 100) < 20, false, samples_for(10));
        b_search = LINE_MODE_TRACK != out.mode;
    }
    printf("  circling: search after %d mm\n", mm);
    check("LT83 going round in circles over tape ends in a search",
          b_search && (mm < (2 * LINE_LOOP_MM)));

    /* That search finds the tape: the same touches must not start it again.
     * It pivots left, so the left sensor sweeps over it. */
    drive(true, false, LINE_CONFIRM_SAMPLES);
    drive(false, false, LINE_CONFIRM_SAMPLES);
    bool const b_found = (LINE_MODE_TRACK == out.mode);
    touch_twice();
    drive(false, false, LINE_CONFIRM_SAMPLES + samples_for(40));
    check("LT86 and the first stretch after it is taken as straight",
          b_found &&
              (out.target_heading_deg != (in.heading_deg - LINE_SEEK_MAX_DEG)));

    /* A U-turn at a junction, then a bend the same way. */
    reset();
    in.b_control = false;
    drive(false, false, 1u);
    in.heading_deg = 180;
    drive(false, false, 1u);
    in.b_control = true;
    drive(true, false, LINE_CONFIRM_SAMPLES);
    drive(false, false, LINE_CONFIRM_SAMPLES);
    bool b_tracks = true;
    for (mm = 0; mm < 400; mm += 10)
    {
        in.heading_deg = 180 + ((mm * 130) / 400);
        drive((mm % 60) < 20, false, samples_for(10));
        b_tracks = b_tracks && (LINE_MODE_TRACK == out.mode);
    }
    check("LT87 a pivot does not count towards circling", b_tracks);

    /* A real U-turn: 180 degrees in 500 mm. */
    reset();
    drive(true, false, LINE_CONFIRM_SAMPLES);
    drive(false, false, LINE_CONFIRM_SAMPLES);
    bool b_stays = true;
    for (mm = 0; mm < 500; mm += 10)
    {
        in.heading_deg = (mm * 180) / 500;
        drive((mm % 60) < 20, false, samples_for(10));
        b_stays = b_stays && (LINE_MODE_TRACK == out.mode);
    }
    check("LT84 a U-turn in the line is not circling", b_stays);

    /* Running alongside the line, over crossing tapes only. */
    reset();
    drive(true, false, LINE_CONFIRM_SAMPLES);
    drive(false, false, LINE_CONFIRM_SAMPLES);
    drive(false, false, samples_for(LINE_STRAIGHT_LOST_MM - 150));
    drive(true, true, LINE_CONFIRM_SAMPLES + samples_for(15));
    drive(false, false, LINE_CONFIRM_SAMPLES);
    bool const b_on = (LINE_MODE_TRACK == out.mode);
    drive(false, false, samples_for(150));
    check("LT85 a crossing tape does not count as seeing the line",
          b_on && (LINE_MODE_SEARCH_LEFT == out.mode));
}

/* Left black, then right @p gap_mm later as left clears: the least overlap
 * the debounce lets through. */
static bool
thin_crossing (int gap_mm)
{
    reset();
    drive(false, false, samples_for(150));
    drive(true, false, samples_for(gap_mm));
    drive(true, true, 1u);
    drive(false, true, 1u);
    return out.b_junction;
}

/* The logged misses of 2026-11-07, and the sweeps that must stay out. */
static void
test_thin_crossing (void)
{
    check("LT66 a crossing read 11 mm askew with one sample of overlap is a "
          "junction",
          thin_crossing(11));
    check("LT67 the tape sweeping across the sensors, 17 mm apart, is not",
          !thin_crossing(17));

    /* Straight and clean for so long that the car looks for the line, just
     * as it reaches a crossing. */
    reset();
    drive(true, false, LINE_CONFIRM_SAMPLES + samples_for(5));
    drive(false, false, LINE_CONFIRM_SAMPLES + 1u +
                            samples_for(LINE_STRAIGHT_LOST_MM));
    bool const b_search = (LINE_MODE_SEARCH_LEFT == out.mode);
    drive(false, true, LINE_CONFIRM_SAMPLES);
    bool const b_back = (LINE_MODE_TRACK == out.mode);
    drive(false, true, samples_for(8));
    drive(true, true, LINE_CONFIRM_SAMPLES);
    check("LT68 a crossing met as a search finds the tape is still a junction",
          b_search && b_back && out.b_junction);
}

static void
test_pivot_aim (void)
{
    /* A square car's right sensor leaves at 14: at 22 it is 8 off. */
    check("LT60 no old line swept: a left turn is 90 degrees",
          90 == pivot_target(LINE_TURN_LEFT, 200, 200, 60));
    check("LT61 the car right of the line: a left turn goes further",
          98 == pivot_target(LINE_TURN_LEFT, 12, 20, 60));
    check("LT62 the car left of the line: a right turn goes further",
          -98 == pivot_target(LINE_TURN_RIGHT, 12, 20, 60));
    check("LT63 the car left of the line: a left turn goes less",
          84 == pivot_target(LINE_TURN_LEFT, 0, 6, 60));
    check("LT64 a square car turns exactly 90",
          90 == pivot_target(LINE_TURN_LEFT, 6, 12, 60));
    check("LT65 a U-turn, pivoting left, allows for it too",
          188 == pivot_target(LINE_TURN_UTURN, 12, 20, 60));
    check("LT69 a sensor leaving late in the pivot is something else",
          90 == pivot_target(LINE_TURN_LEFT, 90, 97, 120));
    check("LT70 and so is one leaving as the car still rolls forward",
          90 == pivot_target(LINE_TURN_LEFT, -4, 2, 60));
    check("LT71 a sensor black from the start still counts as it leaves",
          92 == pivot_target(LINE_TURN_LEFT, -4, 14, 60));
    check("LT88 the car right of the line: a right turn goes less",
          -84 == pivot_target(LINE_TURN_RIGHT, 0, 6, 60));
    check("LT89 2026-11-07: leaving at 32, a right turn goes 18 further",
          -108 == pivot_target(LINE_TURN_RIGHT, 22, 30, 60));
    check("LT90 the near sensor leaving a tape changes nothing, either way",
          (90 == pivot_sensor(LINE_TURN_LEFT, false, 12, 20, 60)) &&
              (-90 == pivot_sensor(LINE_TURN_RIGHT, false, 12, 20, 60)));

    /* Sweeping on over something else after the line. */
    reach_pivot(LINE_TURN_LEFT, 0);
    in.b_turning = true;
    for (int deg = 0; deg <= 60; deg += 2)
    {
        in.heading_deg = deg;
        in.b_right_black = ((deg >= 12) && (deg < 20)) ||
                           ((deg >= 24) && (deg < 30));
        out = line_track_update(&track, &in);
    }
    in.b_turning = false;
    in.heading_deg = 0;
    check("LT73 only the first tape swept counts",
          98 == out.target_heading_deg);

    /* 2026-11-07: right black 10 mm before the left, already on the line. */
    reach_pivot(LINE_TURN_LEFT, 10);
    check("LT72 a sensor on the line as the car meets the tape changes nothing",
          90 == out.target_heading_deg);
}

static void
test_crossing_hold (void)
{
    /* 2026-11-07 run 2: 11, then 01, 11, 01 over the crossing tape. */
    reset();
    in.heading_deg = -23;
    drive(false, false, samples_for(150));
    drive(true, true, samples_for(10));
    bool const b_seen = out.b_junction;
    drive(false, true, samples_for(12));
    drive(true, true, samples_for(5));
    drive(false, true, samples_for(8));
    check("LT36 straight over a junction, the crossing tape cannot steer it",
          b_seen && (-23 == out.target_heading_deg));

    drive(false, true, samples_for(LINE_JUNCTION_HOLD_MM));
    check("LT37 past the crossing, a sensor still black steers again",
          (-23 - LINE_LEAD_DEG) == out.target_heading_deg);

    /* A false junction in a bend: both black at once while turning. */
    reset();
    drive(false, false, samples_for(60));
    unsigned int const samples = samples_for(96);
    for (unsigned int i = 1u; i <= samples; i++)
    {
        in.heading_deg = (40 * (int)i) / (int)samples;
        drive(false, false, 1u);
    }
    drive(true, true, samples_for(10));
    bool const b_junction = out.b_junction;
    drive(true, false, samples_for(6));
    check("LT38 a false junction while turning keeps steering",
          b_junction && ((in.heading_deg + LINE_LEAD_DEG) ==
                         out.target_heading_deg));
}

static void
test_bend_crossing (void)
{
    cross_bend(BEND_DEG, 120, true, 25);
    bool const b_left = (out.target_heading_deg - in.heading_deg) >=
                        (LINE_SEEK_MAX_DEG - 1);
    cross_bend(-BEND_DEG, 120, false, 25);
    bool const b_right = (in.heading_deg - out.target_heading_deg) >=
                         (LINE_SEEK_MAX_DEG - 1);
    check("LT29 tape crossing out of a bend: the car keeps turning that way, "
          "left or right",
          b_left && b_right);

    cross_bend(0, 120, true, 25);
    bool const b_straight = out.target_heading_deg == in.heading_deg;
    cross_bend(BEND_DEG, 120, false, 25);
    bool const b_wrong_way = out.target_heading_deg == in.heading_deg;
    cross_bend(LINE_BEND_TREND_DEG - 3, 120, true, 25);
    bool const b_slight = out.target_heading_deg == in.heading_deg;
    check("LT30 no seek on a straight, against the turn, or after a slight "
          "turn",
          b_straight && b_wrong_way && b_slight);

    /* 2026-11-07 run 2 and the first run's swing: a correction that sweeps
     * the tape across on a straight is not a bend. Seeking there fed the next
     * crossing and the car wobbled +-11 degrees all along the straight. */
    reset();
    drive(false, false, samples_for(150));
    unsigned int const swung = samples_for(25);
    for (unsigned int i = 1u; i <= swung; i++)
    {
        in.heading_deg = (-15 * (int)i) / (int)swung;
        drive(false, true, 1u);
    }
    drive(true, true, samples_for(6));
    drive(true, false, samples_for(5));
    drive(false, false, samples_for(30));
    check("LT35 a correction sweeping the tape across a straight is no bend",
          (out.target_heading_deg == in.heading_deg) && !track.b_curve);

    /* The seek is not forever: a tape that never returns is searched for. */
    cross_bend(BEND_DEG, 120, true, LINE_LOST_MM + 10);
    check("LT31 a crossing that finds no tape searches towards it, soon",
          LINE_MODE_SEARCH_LEFT == out.mode);

    /* The tape reaching the other sensor ends it, as for any bend. */
    cross_bend(BEND_DEG, 120, true, 25);
    drive(false, true, samples_for(4));
    drive(false, false, samples_for(10));
    check("LT32 a tape that was centred after all turns the car back",
          out.target_heading_deg == in.heading_deg);

    /* A pivot, or anything else turning the car, is no bend. */
    reset();
    drive(false, false, samples_for(120));
    in.b_control = false;
    in.heading_deg = 90;
    drive(false, false, 1u);
    in.b_control = true;
    drive(false, false, samples_for(10));
    drive(false, true, samples_for(15));
    drive(true, true, samples_for(8));
    drive(true, false, samples_for(4));
    drive(false, false, samples_for(25));
    check("LT33 turning before tracking resumed does not count as a bend",
          out.target_heading_deg == in.heading_deg);
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

int
main (void)
{
    test_debounce();
    test_right_lead();
    test_edge_clears();
    test_held_edge();
    test_brief_touch();
    test_junction();
    test_thin_crossing();
    test_lost();
    test_search_found();
    test_search_between();
    test_search_both_sides();
    test_search_timeout();
    test_control_off();
    test_pivot_overshoot();
    test_bend_seek();
    test_junction_left();
    test_both_rise_together();
    test_curve_model();
    test_u_exit_model();
    test_straight_model();
    test_long_both_black();
    test_bend_crossing();
    test_crossing_hold();
    test_counter_touch();
    test_hump_slow();
    test_pivot_aim();
    test_return_sensor();
    test_rejoin();
    test_barcode_stop();
    test_creep_model();
    test_creep_bars();
    test_junction_uturn();
    test_overshoot_across();
    printf("%u failure(s)\n", failures);
    return (0u == failures) ? 0 : 1;
}
