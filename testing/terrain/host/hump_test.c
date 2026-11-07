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
    terrain_hump_limits_t const limits = {
        TERRAIN_HUMP_UP_DEG,      TERRAIN_HUMP_DROP_DEG,
        TERRAIN_HUMP_LEVEL_DEG,   TERRAIN_HUMP_ENTER_SAMPLES,
        TERRAIN_HUMP_TOP_SAMPLES, TERRAIN_HUMP_LEVEL_SAMPLES};
    terrain_height_limits_t const height_limits = {
        TERRAIN_TRACK_WIDTH_MM, TERRAIN_HUMP_LEVEL_DEG,
        TERRAIN_PROCESS_LOWPASS_ALPHA(TERRAIN_HUMP_HEIGHT_TAU_S,
                                      TERRAIN_SAMPLE_PERIOD_S)};
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

    printf("%u failure(s)\n", failures);
    return (0u == failures) ? 0 : 1;
}
