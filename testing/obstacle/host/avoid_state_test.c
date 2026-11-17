/* Check the bypass context's packed state directly. */
#include "../../../app_program/obstacle/obstacle_avoid.c"
#include <stdio.h>

static unsigned int failures;

int
tm_printf (char const * p_format, ...)
{
    (void)p_format;
    return 0;
}

static void
check (char const * p_name, bool b_pass)
{
    printf("%s %s\n", b_pass ? "PASS" : "FAIL", p_name);
    failures += b_pass ? 0u : 1u;
}

typedef struct
{
    uint32_t shift;
    uint32_t bits;
} field_t;

static void
test_fields (void)
{
    static field_t const fields[] = {
        {OBSTACLE_AVOID_FIELD_STATE},      {OBSTACLE_AVOID_FIELD_FAULT},
        {OBSTACLE_AVOID_FIELD_LEG},        {OBSTACLE_AVOID_FIELD_SIDESTEPS},
        {OBSTACLE_AVOID_FIELD_BACKOFFS},   {OBSTACLE_AVOID_FIELD_TREND_NEXT},
        {OBSTACLE_AVOID_FIELD_TREND_COUNT}};
    uint32_t const count = sizeof(fields) / sizeof(fields[0]);
    bool b_neighbours = true;
    bool b_saturates = true;
    obstacle_avoid_t ctx = {0};

    for (uint32_t i = 0u; i < count; i++)
    {
        ctx.fields = 0xFFFFFFFFu;
        field_set(&ctx, fields[i].shift, fields[i].bits, 0u);
        b_neighbours =
            b_neighbours &&
            (0u == obstacle_avoid_field(&ctx, fields[i].shift, fields[i].bits));
        for (uint32_t other = 0u; other < count; other++)
        {
            if (other != i)
            {
                b_neighbours = b_neighbours &&
                               (((1u << fields[other].bits) - 1u) ==
                                obstacle_avoid_field(&ctx, fields[other].shift,
                                                     fields[other].bits));
            }
        }
        uint32_t const max = (1u << fields[i].bits) - 1u;
        ctx.fields = 0u;
        field_set(&ctx, fields[i].shift, fields[i].bits, max + 1u);
        b_saturates = b_saturates &&
                      (max == obstacle_avoid_field(&ctx, fields[i].shift,
                                                   fields[i].bits));
    }
    check("AS01 no field touches another", b_neighbours);
    check("AS02 one past a field's limit saturates", b_saturates);

    static uint32_t const flags[] = {
        OBSTACLE_AVOID_F_PLAN_SEQUENCE,    OBSTACLE_AVOID_F_LEG_STARTED,
        OBSTACLE_AVOID_F_TO_FOLLOW,        OBSTACLE_AVOID_F_NEED_PREPARE,
        OBSTACLE_AVOID_F_RETRY_USED,       OBSTACLE_AVOID_F_BACKOFF,
        OBSTACLE_AVOID_F_REPLAN_PENDING,   OBSTACLE_AVOID_F_WATCHING,
        OBSTACLE_AVOID_F_WATCH_SEEN,       OBSTACLE_AVOID_F_WATCH_DONE,
        OBSTACLE_AVOID_F_WATCH_WIDENED,    OBSTACLE_AVOID_F_SHIFT,
        OBSTACLE_AVOID_F_BLOCKED,          OBSTACLE_AVOID_F_REARM_KNOWN,
        OBSTACLE_AVOID_F_CORRIDOR_STARTED, OBSTACLE_AVOID_F_ORIGIN_VALID,
        OBSTACLE_AVOID_F_HOME_RIGHT,       OBSTACLE_AVOID_F_STRAIGHTEN_DUE,
        OBSTACLE_AVOID_F_MEASURE_AGAIN};
    uint32_t all = 0u;
    bool b_single = true;
    for (uint32_t i = 0u; i < (sizeof(flags) / sizeof(flags[0])); i++)
    {
        b_single = b_single && (0u == (all & flags[i])) &&
                   (0u == (flags[i] & (flags[i] - 1u)));
        all |= flags[i];
    }
    check("AS03 every flag is its own bit", b_single);
    check("AS04 every enum fits its field",
          ((uint32_t)OBSTACLE_AVOID_FAULT <= 15u) &&
              ((uint32_t)OBSTACLE_AVOID_FAULT_RETRY_USED <= 15u) &&
              (OBSTACLE_AVOID_MAX_SIDESTEPS <= 3u) &&
              (OBSTACLE_AVOID_MAX_BACKOFFS <= 3u) &&
              (OBSTACLE_AVOID_TREND_SLOTS <= 3u));
}

static void
test_plan (void)
{
    obstacle_avoid_t ctx = {0};
    obstacle_avoid_plan_t plan = {.status = OBSTACLE_AVOID_PLAN_NO_GEOMETRY,
                                  .b_gap_right = true,
                                  .b_edge_unknown = false,
                                  .b_limited = true,
                                  .b_no_fit = false,
                                  .sidestep_mm = 65535u,
                                  .pass_mm = 1u,
                                  .angle_deg = 127u,
                                  .reverse_mm = 200u};
    plan_store(&ctx, plan);
    check("AS05 a plan reads back at the field limits",
          (OBSTACLE_AVOID_PLAN_NO_GEOMETRY ==
           obstacle_avoid_plan_status_of(&ctx)) &&
              (127u == obstacle_avoid_plan_angle(&ctx)) &&
              obstacle_avoid_plan_flag(&ctx, OBSTACLE_AVOID_PLAN_F_GAP_RIGHT) &&
              !obstacle_avoid_plan_flag(&ctx,
                                        OBSTACLE_AVOID_PLAN_F_EDGE_UNKNOWN) &&
              obstacle_avoid_plan_flag(&ctx, OBSTACLE_AVOID_PLAN_F_LIMITED) &&
              !obstacle_avoid_plan_flag(&ctx, OBSTACLE_AVOID_PLAN_F_NO_FIT) &&
              (65535u == ctx.plan_sidestep_mm) && (1u == ctx.plan_pass_mm) &&
              (200u == ctx.plan_reverse_mm));

    plan.angle_deg = 128u;
    plan.status = OBSTACLE_AVOID_PLAN_OK;
    plan.b_gap_right = false;
    plan.b_limited = false;
    plan.b_no_fit = true;
    plan_store(&ctx, plan);
    check("AS06 one past the angle's field saturates and leaves the flags",
          (127u == obstacle_avoid_plan_angle(&ctx)) &&
              (OBSTACLE_AVOID_PLAN_OK == obstacle_avoid_plan_status_of(&ctx)) &&
              !obstacle_avoid_plan_flag(&ctx, OBSTACLE_AVOID_PLAN_F_GAP_RIGHT) &&
              obstacle_avoid_plan_flag(&ctx, OBSTACLE_AVOID_PLAN_F_NO_FIT));
    plan_flag_set(&ctx, OBSTACLE_AVOID_PLAN_F_GAP_RIGHT, true);
    check("AS07 setting one plan flag leaves the others",
          obstacle_avoid_plan_flag(&ctx, OBSTACLE_AVOID_PLAN_F_GAP_RIGHT) &&
              obstacle_avoid_plan_flag(&ctx, OBSTACLE_AVOID_PLAN_F_NO_FIT) &&
              (127u == obstacle_avoid_plan_angle(&ctx)));
}

static void
test_marks (void)
{
    int16_t const mark = robot_mark16(32760);
    check("AS08 a mark reads back across the 16-bit limit",
          (32760 == robot_mark16_near(32800, mark)) &&
              (32760 == robot_mark16_near(32700, mark)));
    check("AS09 and across the full wrap",
          65530 == robot_mark16_near(65600, robot_mark16(65530)));
    check("AS10 a clamp holds 16 bits",
          (65535u == robot_clamp_u16(65536u)) &&
              (65535u == robot_clamp_u16(65535u)) &&
              (0u == robot_clamp_u16(0u)) &&
              (INT16_MAX == robot_clamp_i16(32768)) &&
              (INT16_MIN == robot_clamp_i16(-32769)));
}

int
main (void)
{
    test_fields();
    test_plan();
    test_marks();
    printf("%u failure(s)\n", failures);
    return (0u == failures) ? 0 : 1;
}
