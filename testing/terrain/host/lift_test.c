/* The real lift and overturn check on the 2026-11-11 captures (TER-U34). */
#include "../../../app_program/terrain/terrain_detect.h"
#include <stdio.h>
#include "../../../app_program/terrain/terrain_config.h"

#define AT_LEAST_ONE (-1)
#define ANY (-2)
#define MM_PER_M 1000.0f
#define CM_PER_M 100.0f
#define CENTI 100.0f
#define LEVEL_MPS2 10.0f
#define EDGE_MPS2 13.0f
#define UP_MPS2 14.0f

/* One stretch of a capture and what it must give. */
typedef struct
{
    char const * p_name;
    int16_t const (*p_samples)[3];
    unsigned int count;
    int level_mm_s2[3]; /* The calibration reading of that boot. */
    int lifted;         /* Events wanted, or AT_LEAST_ONE or ANY. */
    int overturned;
} lift_segment_t;

#include "lift_fixture.h"

#define SEGMENTS (sizeof(segments) / sizeof(segments[0]))

static unsigned int failures;

static void
check (char const * p_name, bool ok)
{
    printf("%s %s\n", ok ? "PASS" : "FAIL", p_name);
    if (!ok)
    {
        failures++;
    }
}

static terrain_lift_limits_t
limits_with (float up_ratio)
{
    terrain_lift_limits_t const limits = {
        up_ratio, (float)TERRAIN_LIFT_OVER_DEG, TERRAIN_LIFT_OVER_SAMPLES,
        TERRAIN_LIFT_COOLDOWN_SAMPLES};
    return limits;
}

/* Run one segment from a fresh state; count each kind of event. */
static void
replay (lift_segment_t const * p_seg, terrain_lift_limits_t const * p_limits,
        unsigned int * p_lifted, unsigned int * p_over)
{
    terrain_vec3_t const level = {p_seg->level_mm_s2[0] / MM_PER_M,
                                  p_seg->level_mm_s2[1] / MM_PER_M,
                                  p_seg->level_mm_s2[2] / MM_PER_M};
    terrain_lift_t lift = {0};
    *p_lifted = 0u;
    *p_over = 0u;
    for (unsigned int i = 0u; i < p_seg->count; i++)
    {
        terrain_vec3_t const sample = {p_seg->p_samples[i][0] / CM_PER_M,
                                       p_seg->p_samples[i][1] / CM_PER_M,
                                       p_seg->p_samples[i][2] / CM_PER_M};
        uint32_t value = 0u;
        terrain_event_type_t const type =
            terrain_detect_lift_step(&lift, &sample, &level, p_limits, &value);
        *p_lifted += (TERRAIN_EVENT_LIFTED == type) ? 1u : 0u;
        *p_over += (TERRAIN_EVENT_OVERTURNED == type) ? 1u : 0u;
    }
}

static bool
count_ok (int want, unsigned int got)
{
    return (ANY == want) || ((AT_LEAST_ONE == want) && (got > 0u)) ||
           ((want >= 0) && (got == (unsigned int)want));
}

/* Every segment named with @p prefix gives what it should. */
static bool
segments_ok (char const * p_prefix, terrain_lift_limits_t const * p_limits)
{
    bool ok = true;
    unsigned int matched = 0u;
    for (unsigned int s = 0u; s < SEGMENTS; s++)
    {
        lift_segment_t const * const p_seg = &segments[s];
        unsigned int n = 0u;
        while ((p_prefix[n] != '\0') && (p_prefix[n] == p_seg->p_name[n]))
        {
            n++;
        }
        if (p_prefix[n] != '\0')
        {
            continue;
        }
        unsigned int lifted = 0u;
        unsigned int over = 0u;
        replay(p_seg, p_limits, &lifted, &over);
        printf("  %-12s %4u samples: %u lifted, %u overturned\n", p_seg->p_name,
               p_seg->count, lifted, over);
        ok = ok && count_ok(p_seg->lifted, lifted) &&
             count_ok(p_seg->overturned, over);
        matched++;
    }
    return ok && (matched > 0u);
}

/* Feed one reading @p samples times; count events of @p type. */
static unsigned int
hold (terrain_lift_t * p_lift, terrain_vec3_t sample, unsigned int samples,
      terrain_event_type_t type, uint32_t * p_value)
{
    terrain_vec3_t const level = {0.0f, 0.0f, LEVEL_MPS2};
    terrain_lift_limits_t const limits =
        limits_with(TERRAIN_LIFT_UP_CG / CENTI);
    unsigned int count = 0u;
    for (unsigned int i = 0u; i < samples; i++)
    {
        if (terrain_detect_lift_step(p_lift, &sample, &level, &limits,
                                     p_value) == type)
        {
            count++;
        }
    }
    return count;
}

static void
test_made_up (void)
{
    /* A level reading of 10 m/s^2 makes 13 exactly 1.30 of it. */
    terrain_vec3_t const level = {0.0f, 0.0f, LEVEL_MPS2};
    terrain_vec3_t const edge = {0.0f, 0.0f, EDGE_MPS2};
    terrain_vec3_t const up = {0.0f, 0.0f, UP_MPS2};
    terrain_vec3_t const side = {LEVEL_MPS2, 0.0f, 0.0f};
    terrain_vec3_t const below = {0.0f, 0.0f, -LEVEL_MPS2};
    terrain_lift_t lift = {0};
    uint32_t value = 0u;

    check("LF07 a full window at level, and at exactly 1.30 g up, is nothing",
          (0u == hold(&lift, level, 20u, TERRAIN_EVENT_LIFTED, &value)) &&
              (0u == hold(&lift, edge, 20u, TERRAIN_EVENT_LIFTED, &value)));
    bool const b_first =
        (1u == hold(&lift, up, 1u, TERRAIN_EVENT_LIFTED, &value)) &&
        (131u == value);
    bool const b_cooldown =
        (0u == hold(&lift, up, 10u, TERRAIN_EVENT_LIFTED, &value)) &&
        (0u == hold(&lift, level, 100u, TERRAIN_EVENT_LIFTED, &value));
    bool const b_again =
        1u == hold(&lift, up, 10u, TERRAIN_EVENT_LIFTED, &value);
    check("LF08 one brisk sample past 1.30 g lifts, sized 131; once per 1 s, "
          "then again",
          b_first && b_cooldown && b_again);

    (void)hold(&lift, level, 100u, TERRAIN_EVENT_LIFTED, &value);
    bool const b_side =
        (0u == hold(&lift, side, 35u, TERRAIN_EVENT_OVERTURNED, &value)) &&
        (1u == hold(&lift, side, 1u, TERRAIN_EVENT_OVERTURNED, &value)) &&
        (value >= 60u) &&
        (0u == hold(&lift, side, 300u, TERRAIN_EVENT_OVERTURNED, &value));
    (void)hold(&lift, level, 20u, TERRAIN_EVENT_OVERTURNED, &value);
    bool const b_below =
        (1u == hold(&lift, below, 40u, TERRAIN_EVENT_OVERTURNED, &value)) &&
        (value > 170u);
    check("LF09 on its side: overturned on the 36th sample, once while there; "
          "upside down again after righting",
          b_side && b_below);

    terrain_vec3_t const zero = {0.0f, 0.0f, 0.0f};
    terrain_lift_limits_t const limits = limits_with(1.3f);
    terrain_lift_t fresh = {0};
    terrain_lift_t empty = {0};
    check("LF11 nothing is decided until the 10-sample window is full",
          (0u == hold(&empty, side, 38u, TERRAIN_EVENT_OVERTURNED, &value)) &&
              (1u == hold(&empty, side, 1u, TERRAIN_EVENT_OVERTURNED, &value)));

    value = 7u;
    check(
        "LF10 NULL or a zero level reading is refused and changes nothing",
        (TERRAIN_EVENT_NONE ==
         terrain_detect_lift_step(NULL, &up, &level, &limits, &value)) &&
            (TERRAIN_EVENT_NONE ==
             terrain_detect_lift_step(&fresh, NULL, &level, &limits, &value)) &&
            (TERRAIN_EVENT_NONE ==
             terrain_detect_lift_step(&fresh, &up, &zero, &limits, &value)) &&
            (TERRAIN_EVENT_NONE ==
             terrain_detect_lift_step(&fresh, &up, &level, NULL, &value)) &&
            (TERRAIN_EVENT_NONE ==
             terrain_detect_lift_step(&fresh, &up, &level, &limits, NULL)) &&
            (0u == fresh.word) && (7u == value));
}

int
main (void)
{
    terrain_lift_limits_t const limits =
        limits_with(TERRAIN_LIFT_UP_CG / CENTI);
    check("LF01 43 s of floor driving, with starts, stops and bumps, gives "
          "nothing",
          segments_ok("floor", &limits));
    check("LF02 each of five lifts of the moving car is seen",
          segments_ok("moving_", &limits));
    check("LF03 each of three lifts of the stopped car is seen",
          segments_ok("stopped_", &limits));
    check("LF04 upside down and on its side are overturned",
          segments_ok("upside_down", &limits) &&
              segments_ok("on_side", &limits));
    check("LF05 a slow, gentle lift is not seen (the known limit)",
          segments_ok("gentle", &limits));

    /* The margins either side of 1.30: the floor stays quiet at 1.18, and
     * every lift is still seen at 1.41. */
    terrain_lift_limits_t const low = limits_with(1.18f);
    terrain_lift_limits_t const high = limits_with(1.41f);
    check("LF06 margins: floor quiet down to 1.18, lifts seen up to 1.41",
          segments_ok("floor", &low) && segments_ok("moving_", &high) &&
              segments_ok("stopped_", &high));

    test_made_up();

    printf("%u failure(s)\n", failures);
    return (0u == failures) ? 0 : 1;
}
