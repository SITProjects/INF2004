/* The real hump machine and height code on the course's hump. */
#include "../../../app_program/terrain/terrain_detect.h"
#include <math.h>
#include <stdio.h>
#include "../../../app_program/terrain/terrain_config.h"
#include "../../../app_program/terrain/terrain_height.h"
#include "../../../app_program/terrain/terrain_process.h"

#define HUMP_LENGTH_MM 192.0f
#define HUMP_HEIGHT_MM 10.0f
#define WHEELS_APART_MM 107.5f
#define RAD_TO_DEG 57.29578f
#define LEFT_LINE_MM 200.0f /* Past the hump before following again. */
#define FOLLOW_MM_S 120.0f
#define SLOW_MM_S 30.0f
/* As logged: the brakes take 120 mm/s almost to a stop in 0.2 s. */
#define BRAKE_STEP_MM_S 5.5f
#define BRAKE_LOW_MM_S 10.0f
#define RECOVER_STEP_MM_S 1.0f
#define PULL_AWAY_STEP_MM_S 10.0f
#define START_MM (-300.0f)
#define PAST_MM 1000.0f
/* A steering lean: 3.5 degrees for 0.3 s, once the car is at speed. */
#define LEAN_DEG 3.5f
#define LEAN_START_S 2.5f
#define LEAN_END_S 2.8f

/* One pass as the line task drives it. */
typedef struct
{
    unsigned int climbs;
    unsigned int humps; /* Long enough to count. */
    int height;         /* Of the last hump, mm. */
    int32_t travel_mm;  /* Climb to level, counted or not. */
    float fastest_mm_s; /* From the end of braking until level. */
} pass_t;

static terrain_hump_limits_t const limits = {
    TERRAIN_HUMP_UP_DEG,      TERRAIN_HUMP_DROP_DEG,
    TERRAIN_HUMP_LEVEL_DEG,   TERRAIN_HUMP_ENTER_SAMPLES,
    TERRAIN_HUMP_TOP_SAMPLES, TERRAIN_HUMP_LEVEL_SAMPLES};
static terrain_height_limits_t const height_limits = {
    TERRAIN_TRACK_WIDTH_MM, TERRAIN_HUMP_LEVEL_DEG,
    TERRAIN_PROCESS_LOWPASS_ALPHA(TERRAIN_HUMP_HEIGHT_TAU_S,
                                  TERRAIN_SAMPLE_PERIOD_S)};

static unsigned int failures;

static void
check (char const * p_name, bool ok)
{
    printf("%s %s\n", ok ? "PASS" : "FAIL", p_name);
    failures += ok ? 0u : 1u;
}

/* Triangular: 10 mm at the middle of 192 mm. */
static float
lift_mm (float x_mm)
{
    if ((x_mm <= 0.0f) || (x_mm >= HUMP_LENGTH_MM))
    {
        return 0.0f;
    }
    return HUMP_HEIGHT_MM *
           (1.0f - fabsf((2.0f * x_mm / HUMP_LENGTH_MM) - 1.0f));
}

/* Over the hump at @p speed; from @p clear_mm, if not negative, the car is off
 * the line until past it. */
static void
drive (float speed, float clear_mm, unsigned int * p_climbs,
       unsigned int * p_humps, int * p_height)
{
    float const alpha = TERRAIN_PROCESS_LOWPASS_ALPHA(TERRAIN_TILT_TAU_S,
                                                      TERRAIN_SAMPLE_PERIOD_S);
    terrain_height_t height;
    terrain_height_reset(&height);
    uint32_t word = 0u;
    float tilt = 0.0f;
    *p_climbs = 0u;
    *p_humps = 0u;
    *p_height = -1;
    for (float x = -300.0f; x < (HUMP_LENGTH_MM + 800.0f);
         x += speed * TERRAIN_SAMPLE_PERIOD_S)
    {
        float const roll = asinf(lift_mm(x) / WHEELS_APART_MM) * RAD_TO_DEG;
        tilt += alpha * (roll - tilt);
        if ((clear_mm >= 0.0f) && (x >= clear_mm) &&
            (x < (HUMP_LENGTH_MM + LEFT_LINE_MM)))
        {
            /* As terrain_monitor.c does off the line, then back on it. */
            word = terrain_detect_hump_clear(word);
            terrain_height_clear_hump(&height);
            continue;
        }
        terrain_height_step(&height, terrain_detect_hump_state(word), roll,
                            &height_limits);
        terrain_event_type_t const event =
            terrain_detect_hump_step(&word, tilt, &limits);
        *p_climbs += (TERRAIN_EVENT_ASCENDING == event) ? 1u : 0u;
        if (TERRAIN_EVENT_LEVEL == event)
        {
            (*p_humps)++;
            *p_height = (int)terrain_height_mm(&height);
        }
    }
}

static float
hump_roll (float x_mm, float t_s)
{
    (void)t_s;
    return asinf(lift_mm(x_mm) / WHEELS_APART_MM) * RAD_TO_DEG;
}

static float
steering_roll (float x_mm, float t_s)
{
    (void)x_mm;
    return ((t_s >= LEAN_START_S) && (t_s < LEAN_END_S)) ? LEAN_DEG : 0.0f;
}

/*
 * The closed loop of the car and the machine: 120 mm/s, braking almost to a
 * stop and then 30 mm/s for as long as the machine says hump, as watch_hump()
 * asks. Samples go through the same gate as terrain_monitor.c.
 */
static pass_t
drive_slowing (float (*p_roll)(float x_mm, float t_s))
{
    float const alpha = TERRAIN_PROCESS_LOWPASS_ALPHA(TERRAIN_TILT_TAU_S,
                                                      TERRAIN_SAMPLE_PERIOD_S);
    pass_t pass = {0u, 0u, -1, 0, 0.0f};
    terrain_height_t height;
    terrain_height_reset(&height);
    uint32_t word = 0u;
    float tilt = 0.0f;
    float speed = FOLLOW_MM_S;
    float start_mm = 0.0f;
    bool b_braked = false;
    float t_s = 0.0f;
    for (float x = START_MM; x < PAST_MM; t_s += TERRAIN_SAMPLE_PERIOD_S)
    {
        if (TERRAIN_HUMP_LEVEL == terrain_detect_hump_state(word))
        {
            b_braked = false;
            speed = fminf(FOLLOW_MM_S, speed + PULL_AWAY_STEP_MM_S);
        }
        else if (!b_braked)
        {
            speed -= BRAKE_STEP_MM_S;
            b_braked = speed <= BRAKE_LOW_MM_S;
        }
        else
        {
            speed = fminf(SLOW_MM_S, speed + RECOVER_STEP_MM_S);
            pass.fastest_mm_s = fmaxf(pass.fastest_mm_s, speed);
        }
        x += speed * TERRAIN_SAMPLE_PERIOD_S;

        float const roll = p_roll(x, t_s);
        tilt += alpha * (roll - tilt);
        terrain_hump_sample_t const sample = terrain_detect_hump_sample(
            true, false, (int32_t)speed, TERRAIN_HUMP_MIN_SPEED_MM_S);
        if (TERRAIN_HUMP_SAMPLE_DROP == sample)
        {
            word = terrain_detect_hump_clear(word);
            terrain_height_clear_hump(&height);
        }
        if (sample != TERRAIN_HUMP_SAMPLE_USE)
        {
            continue;
        }

        terrain_height_step(&height, terrain_detect_hump_state(word), roll,
                            &height_limits);
        terrain_event_type_t const event =
            terrain_detect_hump_step(&word, tilt, &limits);
        if (TERRAIN_EVENT_ASCENDING == event)
        {
            pass.climbs++;
            start_mm = x;
        }
        else if (TERRAIN_EVENT_LEVEL == event)
        {
            pass.travel_mm = (int32_t)(x - start_mm);
            if (terrain_detect_hump_lasted(
                    (int32_t)start_mm, (int32_t)x,
                    terrain_detect_hump_min_mm(
                        (int32_t)speed, TERRAIN_HUMP_SLOW_MM_S,
                        TERRAIN_HUMP_SLOW_MIN_MM, TERRAIN_HUMP_MIN_MM)))
            {
                pass.humps++;
                pass.height = (int)terrain_height_mm(&height);
            }
            terrain_height_clear_hump(&height);
        }
        else
        {
            /* Nothing new. */
        }
    }
    return pass;
}

static terrain_stopped_limits_t const stop_limits = {
    TERRAIN_COLLISION_STOP_FROM_MM_S, TERRAIN_COLLISION_STOP_WINDOW_SAMPLES,
    TERRAIN_COLLISION_STOP_GRACE_SAMPLES};

/* @p samples at @p speed; returns the falls reported. */
static unsigned int
hold_speed (terrain_stopped_t * p_state, int speed, unsigned int samples,
            bool b_driving, terrain_hump_state_t hump)
{
    unsigned int falls = 0u;
    for (unsigned int i = 0u; i < samples; i++)
    {
        falls += terrain_detect_stopped_dead(p_state, speed, 0.1f, b_driving,
                                             hump, &stop_limits)
                     ? 1u
                     : 0u;
    }
    return falls;
}

/* From @p from, @p step mm/s less each sample, down to 0. */
static unsigned int
fall_speed (terrain_stopped_t * p_state, int from, int step,
            terrain_hump_state_t hump)
{
    unsigned int falls = 0u;
    for (int speed = from; speed >= 0; speed -= step)
    {
        falls += hold_speed(p_state, speed, 1u, true, hump);
    }
    return falls;
}

/* 2026-11-09: three hits on a box read 120 mm/s to under 10 in under 0.5 s,
 * with pushes under 0.3, under 0.3 and 0.66 g. */
static void
test_stopped_dead (void)
{
    terrain_hump_state_t const level = TERRAIN_HUMP_LEVEL;
    terrain_hump_state_t const climb = TERRAIN_HUMP_ASCENDING;

    terrain_stopped_t state = {0};
    unsigned int falls = hold_speed(&state, 120, 20u, true, level);
    falls += fall_speed(&state, 120, 10, level);
    bool const b_once = 1u == falls;
    bool const b_stays = 0u == hold_speed(&state, 0, 50u, true, level);
    (void)hold_speed(&state, 120, 20u, true, level);
    bool const b_again = 1u == fall_speed(&state, 120, 10, level);
    check("TH16 the wheels stopping dead while driving is one hit, and another "
          "only after driving again",
          b_once && b_stays && b_again);

    state = (terrain_stopped_t){0};
    (void)hold_speed(&state, 120, 20u, true, level);
    check("TH17 falling to exactly half speed is enough",
          1u == hold_speed(&state, 60, 1u, true, level));

    state = (terrain_stopped_t){0};
    (void)hold_speed(&state, 120, 20u, true, level);
    check("TH18 a gradual slow-down is not a hit",
          0u == fall_speed(&state, 120, 1, level));

    state = (terrain_stopped_t){0};
    (void)hold_speed(&state, 120, 20u, true, level);
    falls = hold_speed(&state, 120, 6u, true, climb);
    falls += fall_speed(&state, 120, 10, climb);
    check("TH19 slowing for a hump, the lean comes first, is not a hit",
          0u == falls);

    state = (terrain_stopped_t){0};
    (void)hold_speed(&state, 120, 20u, true, level);
    falls = hold_speed(&state, 120, 2u, true, climb);
    falls += hold_speed(&state, 30, 1u, true, climb);
    bool const b_grace = 1u == falls;
    state = (terrain_stopped_t){0};
    (void)hold_speed(&state, 120, 20u, true, climb);
    (void)hold_speed(&state, 120, 20u, true, level);
    bool const b_after = 1u == fall_speed(&state, 120, 10, level);
    check("TH20 a lean from the hit itself does not hide it, nor does an "
          "earlier hump",
          b_grace && b_after);

    state = (terrain_stopped_t){0};
    (void)hold_speed(&state, 120, 20u, false, level);
    falls = hold_speed(&state, 0, 5u, false, level);
    state = (terrain_stopped_t){0};
    (void)hold_speed(&state, 120, 20u, true, level);
    falls += hold_speed(&state, 120, 1u, false, level);
    falls += hold_speed(&state, 0, 5u, true, level);
    check("TH21 stopping on purpose, no longer following, is not a hit",
          0u == falls);

    state = (terrain_stopped_t){0};
    (void)hold_speed(&state, 80, 20u, true, level);
    check("TH22 a car under the driving speed has nothing to fall from",
          0u == fall_speed(&state, 80, 10, level));

    uint32_t word = 0u;
    uint32_t value = 0u;
    terrain_collision_limits_t const limits = {1.0f, 2.0f, 40.0f, 12u, 50u};
    terrain_collision_input_t in = {
        0.1f, TERRAIN_HUMP_LEVEL, true, 0, true, 0.66f};
    bool const b_counts =
        TERRAIN_EVENT_COLLISION ==
        terrain_detect_collision_step(&word, &in, &limits, &value);
    bool const b_size = 7u == value;
    bool const b_cooldown =
        TERRAIN_EVENT_NONE ==
        terrain_detect_collision_step(&word, &in, &limits, &value);
    check("TH23 that is a collision, sized by its push, and only one",
          b_counts && b_size && b_cooldown);

    state = (terrain_stopped_t){0};
    (void)terrain_detect_stopped_dead(&state, 120, 0.66f, true, level,
                                      &stop_limits);
    (void)hold_speed(&state, 120, 8u, true, level);
    check("TH24 the push is remembered for the moments before the speed falls",
          state.peak_g > 0.4f);
}

int
main (void)
{
    float const speeds[] = {30.0f, 120.0f};
    bool b_one = true;
    bool b_height = true;
    for (unsigned int i = 0u; i < 2u; i++)
    {
        unsigned int climbs = 0u;
        unsigned int humps = 0u;
        int height = 0;
        drive(speeds[i], -1.0f, &climbs, &humps, &height);
        printf("  %.0f mm/s: %u climb, %u hump, height %d mm\n", speeds[i],
               climbs, humps, height);
        b_one = b_one && (1u == climbs) && (1u == humps);
        b_height = b_height && (height >= 9) && (height <= 11);
    }
    check("TH01 the course's hump counts once, at 30 and 120 mm/s", b_one);
    check("TH02 and reads 9 to 11 mm high", b_height);

    /* Stopping or leaving the line half way up is no hump. */
    unsigned int climbs = 0u;
    unsigned int humps = 0u;
    int height = 0;
    drive(120.0f, 90.0f, &climbs, &humps, &height);
    check("TH03 a hump dropped half way never counts, back on the line after",
          (1u == climbs) && (0u == humps));

    /* The logged run: travel from climb to level. */
    check("TH04 the real hump, 188 mm, counts",
          terrain_detect_hump_lasted(3482, 3670, TERRAIN_HUMP_MIN_MM));
    check("TH05 the leans of steering, 34, 67 and 122 mm, do not",
          !terrain_detect_hump_lasted(7719, 7753, TERRAIN_HUMP_MIN_MM) &&
              !terrain_detect_hump_lasted(9821, 9888, TERRAIN_HUMP_MIN_MM) &&
              !terrain_detect_hump_lasted(8934, 9056, TERRAIN_HUMP_MIN_MM));
    check("TH06 reversing never counts, and the distance may wrap",
          !terrain_detect_hump_lasted(1000, 700, TERRAIN_HUMP_MIN_MM) &&
              terrain_detect_hump_lasted(INT32_MAX - 50, INT32_MIN + 150,
                                         TERRAIN_HUMP_MIN_MM));

    /* The five false pushes of 2026-11-07: g, speed, lean. */
    float const limit = TERRAIN_COLLISION_MAX_LEAN_DEG;
    int32_t const driving = TERRAIN_COLLISION_DRIVING_MM_S;
    check("TH07 pushes while stopped, creeping or reversing are ignored",
          (0.0f == terrain_detect_collision_push(1.3f, 0, 0.5f, driving,
                                                 limit)) &&
              (0.0f == terrain_detect_collision_push(1.2f, 33, 0.5f, driving,
                                                     limit)) &&
              (0.0f == terrain_detect_collision_push(1.9f, -236, 0.5f, driving,
                                                     limit)));
    check("TH08 so is a push with the car lifted",
          0.0f == terrain_detect_collision_push(3.0f, 129, 23.8f, driving,
                                                limit));
    check("TH09 a push driving forward is judged",
          1.5f == terrain_detect_collision_push(1.5f, 120, 1.0f, driving,
                                                limit));

    /* Slowing for the hump, as the car does. */
    pass_t const slowed = drive_slowing(hump_roll);
    printf("  slowed: %u climb, %u hump, %d mm travel, height %d mm, "
           "fastest %.0f mm/s\n",
           slowed.climbs, slowed.humps, (int)slowed.travel_mm, slowed.height,
           (double)slowed.fastest_mm_s);
    check("TH10 braking for the hump does not end it: it counts once",
          (1u == slowed.climbs) && (1u == slowed.humps));
    check("TH11 and the car stays slow until level, reading 9 to 11 mm",
          (slowed.fastest_mm_s <= SLOW_MM_S) && (slowed.height >= 9) &&
              (slowed.height <= 11));
    pass_t const steering = drive_slowing(steering_roll);
    printf("  steering lean: %u climb, %u hump, %d mm travel\n",
           steering.climbs, steering.humps, (int)steering.travel_mm);
    check("TH12 a steering lean, braked the same way, does not count",
          (1u == steering.climbs) && (0u == steering.humps));
    check("TH13 too slow or a held roll pauses, off the line drops",
          (TERRAIN_HUMP_SAMPLE_USE ==
           terrain_detect_hump_sample(true, false, 30, 20)) &&
              (TERRAIN_HUMP_SAMPLE_USE ==
               terrain_detect_hump_sample(true, false, 20, 20)) &&
              (TERRAIN_HUMP_SAMPLE_PAUSE ==
               terrain_detect_hump_sample(true, false, 19, 20)) &&
              (TERRAIN_HUMP_SAMPLE_PAUSE ==
               terrain_detect_hump_sample(true, true, 120, 20)) &&
              (TERRAIN_HUMP_SAMPLE_DROP ==
               terrain_detect_hump_sample(false, false, 120, 20)) &&
              (TERRAIN_HUMP_SAMPLE_DROP ==
               terrain_detect_hump_sample(false, true, 0, 20)));
    /* Slowed passes logged to 2026-11-09, travel from climb to level. */
    int32_t const slowed_min = terrain_detect_hump_min_mm(
        30, TERRAIN_HUMP_SLOW_MM_S, TERRAIN_HUMP_SLOW_MIN_MM,
        TERRAIN_HUMP_MIN_MM);
    check("TH15 slowed, the real hump's 66 to 116 mm counts, leans of up to "
          "55 mm do not",
          terrain_detect_hump_lasted(0, 66, slowed_min) &&
              terrain_detect_hump_lasted(0, 116, slowed_min) &&
              !terrain_detect_hump_lasted(0, 55, slowed_min));
    check("TH14 a car that slowed needs less travel than one at speed",
          (90 == terrain_detect_hump_min_mm(30, 60, 90, 130)) &&
              (90 == terrain_detect_hump_min_mm(60, 60, 90, 130)) &&
              (130 == terrain_detect_hump_min_mm(61, 60, 90, 130)) &&
              (130 == terrain_detect_hump_min_mm(120, 60, 90, 130)));

    test_stopped_dead();

    printf("%u failure(s)\n", failures);
    return (0u == failures) ? 0 : 1;
}
