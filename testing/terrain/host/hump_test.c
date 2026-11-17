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

    printf("%u failure(s)\n", failures);
    return (0u == failures) ? 0 : 1;
}
