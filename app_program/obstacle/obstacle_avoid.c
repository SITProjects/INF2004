/**
 * @file obstacle_avoid.c
 * @brief Hardware-free bypass planning and corridor checks.
 */

#include "obstacle_avoid.h"
#include "obstacle_config.h"
#include <math.h>
#include <stddef.h>

/* Private types, constants, and macros. */

#define DEG_TO_RAD (3.14159265f / 180.0f)

/* Turn angles are whole multiples of this, up to a quarter turn. */
#define ANGLE_STEP_DEG 5u
#define ANGLE_MAX_DEG 90u

/* Looked both ways in vain: reverse this much to look again, up to the
 * total, then stay and re-sweep until the way clears. */
#define ESCAPE_STEP_MM 100u
#define ESCAPE_MAX_MM 300u

/* Watch readings farther than this (from the pivot) are not the obstacle. */
#define WATCH_RANGE_MM 400.0f

/* Smallest widening for a watch reading too close to the path. */
#define WATCH_SHIFT_MIN_MM 30.0f

/* Drive this far past the last point seen before facing ahead again: a
 * flat side at a glancing angle often gives no echo while still there. */
#define WATCH_CLEAR_EXTRA_MM 50u

/* Leg timeout: twice the expected time plus this. */
#define LEG_TIMEOUT_EXTRA_MS 1000u

/* An 8-bit sequence is newer when it is 1..127 steps ahead. */
#define SEQUENCE_HALF 128u

/* Static data. */

/* Private function declarations. */

static bool plan_usable (robot_obstacle_result_t result, bool * p_gap_right);
static uint32_t corridor_clearance (obstacle_scan_corridor_t const * p_corridor,
                                    bool b_right);
static bool end_fits (uint16_t end_mm, int32_t need_mm, uint32_t pass_mm,
                      obstacle_avoid_geometry_t const * p_geometry);
static uint8_t plan_angle (uint16_t sidestep_mm, uint16_t centre_mm,
                           obstacle_avoid_geometry_t const * p_geometry);
static uint32_t along_diagonal (uint32_t lateral_mm, uint8_t angle_deg);
static uint32_t diagonal_forward (uint32_t lateral_mm, uint8_t angle_deg);
static uint32_t pass_left (obstacle_avoid_t const * p_ctx);
static bool elapsed (uint32_t now_ms, uint32_t since_ms, uint32_t limit_ms);
static bool sequence_newer (uint8_t sequence, uint8_t reference);
static bool leg_is_move (obstacle_avoid_leg_t leg);
static char const * leg_reason (obstacle_avoid_leg_t leg);
static void enter (obstacle_avoid_t * p_ctx, obstacle_avoid_state_t state,
                   uint32_t now_ms);
static void latch_fault (obstacle_avoid_t * p_ctx,
                         obstacle_avoid_action_t * p_act,
                         obstacle_avoid_fault_t fault, uint32_t now_ms,
                         char const * p_reason);
static void hold (obstacle_avoid_t * p_ctx, obstacle_avoid_action_t * p_act,
                  obstacle_avoid_state_t state, uint32_t now_ms,
                  char const * p_reason);
static void release (obstacle_avoid_t * p_ctx, obstacle_avoid_action_t * p_act,
                     bool b_to_follow, uint32_t now_ms, char const * p_reason);
static bool try_plan (obstacle_avoid_t * p_ctx, obstacle_avoid_action_t * p_act,
                      obstacle_avoid_input_t const * p_in);
static void start_leg (obstacle_avoid_t * p_ctx,
                       obstacle_avoid_action_t * p_act,
                       obstacle_avoid_input_t const * p_in);
static void send_part (obstacle_avoid_t * p_ctx,
                       obstacle_avoid_action_t * p_act,
                       obstacle_avoid_input_t const * p_in);
static void next_leg (obstacle_avoid_t * p_ctx);
static void start_replan (obstacle_avoid_t * p_ctx,
                          obstacle_avoid_action_t * p_act,
                          obstacle_avoid_input_t const * p_in);
static void step_replan (obstacle_avoid_t * p_ctx,
                         obstacle_avoid_action_t * p_act,
                         obstacle_avoid_input_t const * p_in, bool b_inhibited);
static void resume_after_replan (obstacle_avoid_t * p_ctx,
                                 obstacle_avoid_action_t * p_act,
                                 obstacle_avoid_input_t const * p_in,
                                 bool b_inhibited, char const * p_reason);
static void send_backoff (obstacle_avoid_t * p_ctx,
                          obstacle_avoid_action_t * p_act,
                          obstacle_avoid_input_t const * p_in);
static void move_done (obstacle_avoid_t * p_ctx,
                       obstacle_avoid_action_t * p_act,
                       obstacle_avoid_input_t const * p_in);
static bool record_progress (obstacle_avoid_t * p_ctx,
                             obstacle_avoid_input_t const * p_in);
static void interrupt_leg (obstacle_avoid_t * p_ctx,
                           obstacle_avoid_action_t * p_act,
                           obstacle_avoid_input_t const * p_in,
                           char const * p_reason);
static void retry_or_fault (obstacle_avoid_t * p_ctx,
                            obstacle_avoid_action_t * p_act,
                            obstacle_avoid_input_t const * p_in,
                            bool b_line_missed, char const * p_reason);
static void step_verify (obstacle_avoid_t * p_ctx,
                         obstacle_avoid_action_t * p_act,
                         obstacle_avoid_input_t const * p_in, bool b_critical);
static bool watch_step (obstacle_avoid_t * p_ctx,
                        obstacle_avoid_action_t * p_act,
                        obstacle_avoid_input_t const * p_in);
static void watch_off (obstacle_avoid_t * p_ctx,
                       obstacle_avoid_action_t * p_act);
static void step_run_leg (obstacle_avoid_t * p_ctx,
                          obstacle_avoid_action_t * p_act,
                          obstacle_avoid_input_t const * p_in, bool b_critical);
static void step_prepare (obstacle_avoid_t * p_ctx,
                          obstacle_avoid_action_t * p_act,
                          obstacle_avoid_input_t const * p_in, bool b_critical);

/* Public function definitions. */

obstacle_avoid_plan_t
obstacle_avoid_plan (robot_obstacle_result_t result,
                     obstacle_scan_geometry_t geometry,
                     obstacle_avoid_geometry_t const * p_geometry)
{
    obstacle_avoid_plan_t plan = {.status = OBSTACLE_AVOID_PLAN_NOT_PATH,
                                  .b_gap_right = false,
                                  .b_edge_unknown = false,
                                  .b_limited = false,
                                  .sidestep_mm = 0u,
                                  .pass_mm = 0u,
                                  .angle_deg = ANGLE_MAX_DEG,
                                  .reverse_mm = 0u,
                                  .b_no_fit = false};
    robot_obstacle_status_t const status =
        robot_obstacle_get_status(result.flags);
    if ((p_geometry == NULL) || ((status != ROBOT_OBSTACLE_PATH_FOUND) &&
                                 (status != ROBOT_OBSTACLE_BLOCKED)))
    {
        return plan;
    }
    if (geometry.centre_mm == 0u)
    {
        plan.status = OBSTACLE_AVOID_PLAN_NO_GEOMETRY;
        return plan;
    }

    /* Sideways distance each side needs, where its edge is known. */
    bool const b_width = result.width_mm != ROBOT_OBSTACLE_WIDTH_UNAVAILABLE;
    bool const b_left =
        b_width && ((result.flags & ROBOT_OBSTACLE_LEFT_EDGE_FOUND) != 0u);
    bool const b_right =
        b_width && ((result.flags & ROBOT_OBSTACLE_RIGHT_EDGE_FOUND) != 0u);
    int32_t const clearance_mm =
        (int32_t)p_geometry->half_width_mm + (int32_t)p_geometry->margin_mm;
    int32_t const need_left_mm = (int32_t)geometry.left_x_mm + clearance_mm;
    int32_t const need_right_mm = -(int32_t)geometry.right_x_mm + clearance_mm;
    bool b_scanner_right = false;
    bool const b_scanner_side = (status == ROBOT_OBSTACLE_PATH_FOUND) &&
                                plan_usable(result, &b_scanner_right);

    /* Too close to turn: reverse first, and plan from the new distance. */
    uint16_t room_mm = geometry.centre_mm;
    if (room_mm < p_geometry->min_room_mm)
    {
        plan.reverse_mm = (uint16_t)(p_geometry->min_room_mm - room_mm);
        room_mm = p_geometry->min_room_mm;
    }
    uint32_t pass_mm = (uint32_t)room_mm + p_geometry->axle_to_pivot_mm +
                       p_geometry->depth_mm + p_geometry->rear_mm +
                       p_geometry->margin_mm;
    if (pass_mm > p_geometry->pass_max_mm)
    {
        plan.b_limited = true;
        pass_mm = p_geometry->pass_max_mm;
    }

    /* A clear end counts only if the car, shifted aside, passes what that
     * end measured. */
    bool b_left_clear =
        geometry.b_left_clear &&
        end_fits(geometry.left_end_mm,
                 b_left ? need_left_mm : (int32_t)p_geometry->sidestep_max_mm,
                 pass_mm, p_geometry);
    bool b_right_clear =
        geometry.b_right_clear &&
        end_fits(geometry.right_end_mm,
                 b_right ? need_right_mm : (int32_t)p_geometry->sidestep_max_mm,
                 pass_mm, p_geometry);
    plan.b_no_fit = (geometry.b_left_clear || geometry.b_right_clear) &&
                    !b_left_clear && !b_right_clear;
    if (plan.b_no_fit)
    {
        /* Free space, but tight: best effort on the roomier free end (no
         * echo is roomiest; a tie keeps the scanner's side). The watch
         * widens the path on the way. */
        uint32_t const left_room = !geometry.b_left_clear ? 0u
                                   : (geometry.left_end_mm == 0u)
                                       ? UINT32_MAX
                                       : geometry.left_end_mm;
        uint32_t const right_room = !geometry.b_right_clear ? 0u
                                    : (geometry.right_end_mm == 0u)
                                        ? UINT32_MAX
                                        : geometry.right_end_mm;
        if (left_room == right_room)
        {
            b_left_clear = !b_scanner_right;
            b_right_clear = b_scanner_right;
        }
        else
        {
            b_left_clear = left_room > right_room;
            b_right_clear = !b_left_clear;
        }
    }

    int32_t need_mm;
    if (b_scanner_side && ((!b_left_clear && !b_right_clear) ||
                           (b_scanner_right ? b_right_clear : b_left_clear)))
    {
        /* The scanner ranks measured clearance, not detour length. Keep
         * its choice even when the other edge needs a shorter sidestep. */
        plan.b_gap_right = b_scanner_right;
        bool const b_edge = plan.b_gap_right ? b_right : b_left;
        plan.b_edge_unknown = !b_edge;
        need_mm = b_edge ? (plan.b_gap_right ? need_right_mm : need_left_mm)
                         : (int32_t)p_geometry->sidestep_max_mm;
    }
    else if (b_left_clear || b_right_clear)
    {
        /* Without a usable measured preference, exclude blocked ends
         * and fall back to the edge geometry among the clear ends. */
        int32_t const left_mm =
            b_left ? need_left_mm : (int32_t)p_geometry->sidestep_max_mm;
        int32_t const right_mm =
            b_right ? need_right_mm : (int32_t)p_geometry->sidestep_max_mm;
        if (b_left_clear && b_right_clear)
        {
            plan.b_gap_right =
                (right_mm == left_mm) ? b_scanner_right : (right_mm < left_mm);
        }
        else
        {
            plan.b_gap_right = b_right_clear;
        }
        plan.b_edge_unknown = plan.b_gap_right ? !b_right : !b_left;
        need_mm = plan.b_gap_right ? right_mm : left_mm;
    }
    else if (b_left && b_right)
    {
        plan.b_gap_right = (need_right_mm == need_left_mm)
                               ? b_scanner_right
                               : (need_right_mm < need_left_mm);
        need_mm = plan.b_gap_right ? need_right_mm : need_left_mm;
    }
    else if (b_left || b_right)
    {
        plan.b_gap_right = b_right;
        need_mm = b_right ? need_right_mm : need_left_mm;
    }
    else if (b_scanner_side)
    {
        plan.b_gap_right = b_scanner_right;
        plan.b_edge_unknown = true;
        need_mm = (int32_t)p_geometry->sidestep_max_mm;
    }
    else
    {
        return plan; /* Wider than the sweep on both sides. */
    }
    if (need_mm > (int32_t)p_geometry->sidestep_max_mm)
    {
        plan.b_limited = true;
        need_mm = (int32_t)p_geometry->sidestep_max_mm;
    }
    plan.sidestep_mm = (need_mm < (int32_t)p_geometry->sidestep_min_mm)
                           ? p_geometry->sidestep_min_mm
                           : (uint16_t)need_mm;
    plan.pass_mm = (uint16_t)pass_mm;
    plan.angle_deg = plan_angle(plan.sidestep_mm, room_mm, p_geometry);
    plan.status = OBSTACLE_AVOID_PLAN_OK;
    return plan;
}

uint8_t
obstacle_avoid_split_count (uint32_t distance_mm)
{
    uint32_t count = (distance_mm + (ROBOT_MOTION_CMD_DISTANCE_MAX_MM - 1u)) /
                     ROBOT_MOTION_CMD_DISTANCE_MAX_MM;
    return (count > UINT8_MAX) ? UINT8_MAX : (uint8_t)count;
}

uint16_t
obstacle_avoid_next_part (uint32_t remaining_mm)
{
    uint32_t count = obstacle_avoid_split_count(remaining_mm);
    if (count == 0u)
    {
        return 0u;
    }
    uint32_t part_mm = (remaining_mm + (count - 1u)) / count;
    return (part_mm > ROBOT_MOTION_CMD_DISTANCE_MAX_MM)
               ? (uint16_t)ROBOT_MOTION_CMD_DISTANCE_MAX_MM
               : (uint16_t)part_mm;
}

obstacle_avoid_corridor_t
obstacle_avoid_check_corridor (obstacle_scan_corridor_t const * p_corridor,
                               obstacle_width_config_t const * p_width,
                               obstacle_avoid_geometry_t const * p_geometry,
                               uint16_t pass_mm, bool b_gap_right,
                               uint16_t * p_shift_mm)
{
    if (p_shift_mm == NULL)
    {
        return OBSTACLE_AVOID_CORRIDOR_BLOCKED;
    }
    *p_shift_mm = 0u;
    if ((p_corridor == NULL) || (p_width == NULL) || (p_geometry == NULL))
    {
        return OBSTACLE_AVOID_CORRIDOR_BLOCKED;
    }

    float const half_mm =
        (float)p_geometry->half_width_mm + (float)p_geometry->margin_mm;
    float const length_mm =
        (float)pass_mm + (float)p_geometry->axle_to_pivot_mm;
    float shift_mm = 0.0f;
    float limit_mm = INFINITY;
    for (uint8_t index = 0u; index < OBSTACLE_SWEEP_POINTS; ++index)
    {
        if ((p_corridor->valid_mask & (uint16_t)(1u << index)) == 0u)
        {
            return OBSTACLE_AVOID_CORRIDOR_BLOCKED;
        }
        uint16_t const echo_us = p_corridor->echo_us[index];
        if (echo_us == 0u)
        {
            continue; /* No echo: nothing at this angle. */
        }
        float range_mm;
        if (!obstacle_width_range_mm(echo_us, p_width, &range_mm))
        {
            return OBSTACLE_AVOID_CORRIDOR_BLOCKED;
        }
        float const angle_rad =
            (float)(OBSTACLE_SWEEP_FIRST_DEG +
                    (OBSTACLE_SWEEP_STEP_DEG * (uint32_t)index)) *
            DEG_TO_RAD;
        float const x_mm = range_mm * cosf(angle_rad);
        float const y_mm =
            (float)p_geometry->axle_to_pivot_mm + (range_mm * sinf(angle_rad));
        if (!(y_mm < length_mm))
        {
            continue; /* Beyond the end of the pass. */
        }
        float const gap_mm = b_gap_right ? -x_mm : x_mm;
        if (fabsf(x_mm) < half_mm)
        {
            float const need_mm = gap_mm + half_mm;
            if (need_mm > shift_mm)
            {
                shift_mm = need_mm;
            }
        }
        else if (gap_mm >= half_mm)
        {
            float const room_mm = gap_mm - half_mm;
            if (room_mm < limit_mm)
            {
                limit_mm = room_mm;
            }
        }
        else
        {
            /* Already clear on the far side. */
        }
    }

    if (!(shift_mm > 0.0f))
    {
        return OBSTACLE_AVOID_CORRIDOR_CLEAR;
    }
    if ((shift_mm > limit_mm) || !(shift_mm < (float)UINT16_MAX))
    {
        return OBSTACLE_AVOID_CORRIDOR_BLOCKED;
    }
    *p_shift_mm = (uint16_t)ceilf(shift_mm);
    return OBSTACLE_AVOID_CORRIDOR_SHIFT;
}

void
obstacle_avoid_init (obstacle_avoid_t * p_ctx,
                     obstacle_avoid_config_t const * p_config, uint32_t now_ms)
{
    *p_ctx = (obstacle_avoid_t){0};
    p_ctx->p_config = p_config;
    p_ctx->state = OBSTACLE_AVOID_WAIT_CLEAR;
    p_ctx->fault = OBSTACLE_AVOID_FAULT_NONE;
    p_ctx->entered_ms = now_ms;
    p_ctx->leg = OBSTACLE_AVOID_LEG_FOLLOW;
}

obstacle_avoid_action_t
obstacle_avoid_step (obstacle_avoid_t * p_ctx,
                     obstacle_avoid_input_t const * p_in)
{
    obstacle_avoid_action_t act = {
        .from = p_ctx->state, .to = p_ctx->state, .p_reason = NULL};
    uint32_t const now_ms = p_in->now_ms;
    obstacle_avoid_timing_t const * const p_timing = &p_ctx->p_config->timing;
    if ((p_ctx->state != OBSTACLE_AVOID_FAULT) &&
        (p_in->b_inhibit_failed || (p_in->send == OBSTACLE_AVOID_SEND_ERROR)))
    {
        latch_fault(p_ctx, &act, OBSTACLE_AVOID_FAULT_KERNEL, now_ms,
                    "kernel call failed");
    }
    if (p_ctx->state == OBSTACLE_AVOID_FAULT)
    {
        /* Ask again every loop, in case an earlier set failed. */
        act.b_set_inhibit = true;
        act.to = p_ctx->state;
        return act;
    }

    bool const b_critical = robot_obstacle_get_critical(p_in->scan.flags);
    robot_obstacle_status_t const status =
        robot_obstacle_get_status(p_in->scan.flags);
    bool const b_inhibited = p_in->motion_mode == ROBOT_MOTION_MODE_INHIBITED;
    bool const b_clear = !p_in->b_stop_requested && !b_critical;
    switch (p_ctx->state)
    {
        case OBSTACLE_AVOID_IDLE:
            if (!b_clear)
            {
                hold(p_ctx, &act, OBSTACLE_AVOID_STOPPING, now_ms,
                     b_critical ? "critical" : "stop request");
            }
            break;
        case OBSTACLE_AVOID_STOPPING:
            if (b_inhibited)
            {
                enter(p_ctx, OBSTACLE_AVOID_WAIT_RESULT, now_ms);
                act.p_reason = "motors inhibited";
            }
            else if (elapsed(now_ms, p_ctx->entered_ms, p_timing->stop_ack_ms))
            {
                latch_fault(p_ctx, &act, OBSTACLE_AVOID_FAULT_STOP_ACK, now_ms,
                            "stop not acknowledged");
            }
            else
            {
                /* Wait for tMotion. */
            }
            break;
        case OBSTACLE_AVOID_WAIT_RESULT:
            if (try_plan(p_ctx, &act, p_in))
            {
                /* Planned, or refused into WAIT_CLEAR. */
            }
            else if (b_clear && (status == ROBOT_OBSTACLE_CLEAR))
            {
                release(p_ctx, &act, true, now_ms, "path clear");
            }
            else if ((status == ROBOT_OBSTACLE_BLOCKED) ||
                     (status == ROBOT_OBSTACLE_UNKNOWN) ||
                     (status == ROBOT_OBSTACLE_PATH_FOUND))
            {
                enter(p_ctx, OBSTACLE_AVOID_WAIT_CLEAR, now_ms);
                act.p_reason = "no bypass";
            }
            else if (elapsed(now_ms, p_ctx->entered_ms, p_timing->result_ms))
            {
                enter(p_ctx, OBSTACLE_AVOID_WAIT_CLEAR, now_ms);
                act.p_reason = "no result";
            }
            else
            {
                /* Sweep still running. */
            }
            break;
        case OBSTACLE_AVOID_WAIT_CLEAR:
            if (!try_plan(p_ctx, &act, p_in) && b_clear)
            {
                release(p_ctx, &act, true, now_ms, "path clear");
            }
            break;
        case OBSTACLE_AVOID_RELEASING:
            if (!b_inhibited)
            {
                if (p_ctx->b_to_follow)
                {
                    p_ctx->leg = OBSTACLE_AVOID_LEG_FOLLOW;
                }
                if (p_ctx->b_backoff)
                {
                    send_backoff(p_ctx, &act, p_in);
                }
                else
                {
                    start_leg(p_ctx, &act, p_in);
                }
            }
            else if (elapsed(now_ms, p_ctx->entered_ms, p_timing->stop_ack_ms))
            {
                latch_fault(p_ctx, &act, OBSTACLE_AVOID_FAULT_STOP_ACK, now_ms,
                            "inhibit not released");
            }
            else
            {
                /* Wait for tMotion to flush and leave INHIBITED. */
            }
            break;
        case OBSTACLE_AVOID_PREPARE:
            step_prepare(p_ctx, &act, p_in, b_critical);
            break;
        case OBSTACLE_AVOID_SEND:
            if (p_in->send == OBSTACLE_AVOID_SEND_OK)
            {
                if (p_ctx->leg == OBSTACLE_AVOID_LEG_FOLLOW)
                {
                    enter(p_ctx, OBSTACLE_AVOID_IDLE, now_ms);
                    act.b_side_scans_on = true;
                    act.p_reason = "following";
                }
                else
                {
                    enter(p_ctx, OBSTACLE_AVOID_RUN_LEG, now_ms);
                }
            }
            else if (elapsed(now_ms, p_ctx->entered_ms, p_timing->send_ms))
            {
                latch_fault(p_ctx, &act, OBSTACLE_AVOID_FAULT_KERNEL, now_ms,
                            "command not queued");
            }
            else
            {
                /* Still pending: send it again. */
                act.b_send = true;
                act.cmd = p_ctx->cmd;
            }
            break;
        case OBSTACLE_AVOID_RUN_LEG:
            step_run_leg(p_ctx, &act, p_in, b_critical);
            break;
        case OBSTACLE_AVOID_VERIFY:
            step_verify(p_ctx, &act, p_in, b_critical);
            break;
        case OBSTACLE_AVOID_BLOCKED_WAIT:
            if (p_timing->b_recheck && elapsed(now_ms, p_ctx->blocked_since_ms,
                                               p_timing->block_wait_ms))
            {
                latch_fault(p_ctx, &act, OBSTACLE_AVOID_FAULT_BLOCKED, now_ms,
                            "still blocked");
            }
            else if (b_inhibited &&
                     (p_timing->b_recheck ? b_clear : !b_critical))
            {
                act.b_side_scans_off = !p_timing->b_recheck;
                release(p_ctx, &act, false, now_ms, "path clear again");
            }
            else if (b_inhibited && b_critical && !p_timing->b_recheck &&
                     (p_ctx->backoffs <
                      p_ctx->p_config->geometry.max_backoffs) &&
                     elapsed(now_ms, p_ctx->entered_ms,
                             p_timing->backoff_delay_ms))
            {
                ++p_ctx->backoffs;
                p_ctx->b_backoff = true;
                release(p_ctx, &act, false, now_ms, "backing off");
            }
            else if (b_inhibited && b_critical && !p_timing->b_recheck &&
                     (p_ctx->leg == OBSTACLE_AVOID_LEG_PASS) &&
                     elapsed(now_ms, p_ctx->entered_ms,
                             p_timing->backoff_delay_ms))
            {
                /* Parallel to the line: look again from here. */
                start_replan(p_ctx, &act, p_in);
            }
            else if (b_inhibited && b_critical && !p_timing->b_recheck &&
                     (p_ctx->leg == OBSTACLE_AVOID_LEG_SIDESTEP) &&
                     elapsed(now_ms, p_ctx->entered_ms,
                             p_timing->backoff_delay_ms))
            {
                /* Keep what the diagonal achieved, turn back parallel,
                 * then look again. */
                uint32_t const target =
                    along_diagonal(p_ctx->sidestep_mm, p_ctx->plan.angle_deg);
                uint32_t const done = (target > p_ctx->remaining)
                                          ? (target - p_ctx->remaining)
                                          : 0u;
                float const angle_rad =
                    (float)p_ctx->plan.angle_deg * DEG_TO_RAD;
                p_ctx->lateral_mm += (uint32_t)((float)done * sinf(angle_rad));
                p_ctx->forward_mm += (uint32_t)((float)done * cosf(angle_rad));
                p_ctx->leg = OBSTACLE_AVOID_LEG_TURN_BACK;
                p_ctx->b_leg_started = false;
                p_ctx->b_replan_pending = true;
                release(p_ctx, &act, false, now_ms, "turn back to re-sweep");
            }
            else if (b_inhibited && !p_timing->b_recheck &&
                     elapsed(now_ms, p_ctx->entered_ms, p_timing->resweep_ms))
            {
                /* Best effort: keep looking until the way clears. */
                start_replan(p_ctx, &act, p_in);
            }
            else
            {
                /* Held until the path clears. */
            }
            break;
        case OBSTACLE_AVOID_REPLAN:
            step_replan(p_ctx, &act, p_in, b_inhibited);
            break;
        case OBSTACLE_AVOID_RETRY_WAIT:
        {
            uint32_t const delay_ms = p_ctx->b_shift
                                          ? p_timing->backoff_delay_ms
                                          : p_timing->retry_delay_ms;
            if (b_inhibited && elapsed(now_ms, p_ctx->entered_ms, delay_ms))
            {
                release(p_ctx, &act, false, now_ms,
                        p_ctx->b_shift ? "shifting out" : "retrying");
                p_ctx->b_shift = false;
            }
            else if (elapsed(now_ms, p_ctx->entered_ms,
                             delay_ms + p_timing->stop_ack_ms))
            {
                latch_fault(p_ctx, &act, OBSTACLE_AVOID_FAULT_STOP_ACK, now_ms,
                            "stop not acknowledged");
            }
            else
            {
                /* Held for the retry delay. */
            }
            break;
        }
        case OBSTACLE_AVOID_FAULT:
        default:
            latch_fault(p_ctx, &act, OBSTACLE_AVOID_FAULT_KERNEL, now_ms,
                        "invalid state");
            break;
    }
    act.to = p_ctx->state;
    return act;
}

char const *
obstacle_avoid_state_name (obstacle_avoid_state_t state)
{
    switch (state)
    {
        case OBSTACLE_AVOID_IDLE:
            return "IDLE";
        case OBSTACLE_AVOID_STOPPING:
            return "STOPPING";
        case OBSTACLE_AVOID_WAIT_RESULT:
            return "WAIT_RESULT";
        case OBSTACLE_AVOID_WAIT_CLEAR:
            return "WAIT_CLEAR";
        case OBSTACLE_AVOID_RELEASING:
            return "RELEASING";
        case OBSTACLE_AVOID_PREPARE:
            return "PREPARE";
        case OBSTACLE_AVOID_SEND:
            return "SEND";
        case OBSTACLE_AVOID_RUN_LEG:
            return "RUN_LEG";
        case OBSTACLE_AVOID_VERIFY:
            return "VERIFY";
        case OBSTACLE_AVOID_BLOCKED_WAIT:
            return "BLOCKED_WAIT";
        case OBSTACLE_AVOID_RETRY_WAIT:
            return "RETRY_WAIT";
        case OBSTACLE_AVOID_REPLAN:
            return "REPLAN";
        case OBSTACLE_AVOID_FAULT:
            return "FAULT";
        default:
            return "INVALID";
    }
}

/* Private function definitions. */

/* Only PATH_FOUND toward one side can be bypassed. */
static bool
plan_usable (robot_obstacle_result_t result, bool * p_gap_right)
{
    if (robot_obstacle_get_status(result.flags) != ROBOT_OBSTACLE_PATH_FOUND)
    {
        return false;
    }
    robot_obstacle_direction_t const direction =
        robot_obstacle_get_direction(result.flags);
    if (direction == ROBOT_OBSTACLE_DIR_LEFT_60)
    {
        *p_gap_right = false;
        return true;
    }
    if (direction == ROBOT_OBSTACLE_DIR_RIGHT_120)
    {
        *p_gap_right = true;
        return true;
    }
    return false;
}

/* Strict and wrap-safe, like obstacle_decision_elapsed_exceeds(). */
static bool
elapsed (uint32_t now_ms, uint32_t since_ms, uint32_t limit_ms)
{
    return (uint32_t)(now_ms - since_ms) > limit_ms;
}

static bool
sequence_newer (uint8_t sequence, uint8_t reference)
{
    uint8_t const ahead = (uint8_t)(sequence - reference);
    return (ahead != 0u) && (ahead < SEQUENCE_HALF);
}

static bool
leg_is_move (obstacle_avoid_leg_t leg)
{
    return (leg == OBSTACLE_AVOID_LEG_REVERSE) ||
           (leg == OBSTACLE_AVOID_LEG_SIDESTEP) ||
           (leg == OBSTACLE_AVOID_LEG_PASS) ||
           (leg == OBSTACLE_AVOID_LEG_RETURN) ||
           (leg == OBSTACLE_AVOID_LEG_OVERSHOOT);
}

static char const *
leg_reason (obstacle_avoid_leg_t leg)
{
    switch (leg)
    {
        case OBSTACLE_AVOID_LEG_REVERSE:
            return "reverse for room";
        case OBSTACLE_AVOID_LEG_TURN_OUT:
            return "turn toward the gap";
        case OBSTACLE_AVOID_LEG_SIDESTEP:
            return "sidestep";
        case OBSTACLE_AVOID_LEG_TURN_BACK:
            return "turn back";
        case OBSTACLE_AVOID_LEG_VERIFY:
            return "verify corridor";
        case OBSTACLE_AVOID_LEG_PASS:
            return "pass";
        case OBSTACLE_AVOID_LEG_TURN_IN:
            return "turn toward the line";
        case OBSTACLE_AVOID_LEG_RETURN:
            return "return to the line";
        case OBSTACLE_AVOID_LEG_OVERSHOOT:
            return "axle onto the line";
        case OBSTACLE_AVOID_LEG_TURN_HOME:
            return "original heading";
        case OBSTACLE_AVOID_LEG_FOLLOW:
        default:
            return "resume following";
    }
}

static void
enter (obstacle_avoid_t * p_ctx, obstacle_avoid_state_t state, uint32_t now_ms)
{
    p_ctx->state = state;
    p_ctx->entered_ms = now_ms;
}

static void
latch_fault (obstacle_avoid_t * p_ctx, obstacle_avoid_action_t * p_act,
             obstacle_avoid_fault_t fault, uint32_t now_ms,
             char const * p_reason)
{
    p_ctx->fault = fault;
    p_act->b_set_inhibit = true;
    p_act->b_side_scans_on = true;
    p_act->b_clear_inhibit = false;
    p_act->b_send = false;
    p_act->b_rearm = false;
    p_act->b_corridor = false;
    p_act->p_reason = p_reason;
    watch_off(p_ctx, p_act);
    enter(p_ctx, OBSTACLE_AVOID_FAULT, now_ms);
}

/* Set INHIBIT and wait in a held state; the block timer starts once. */
static void
hold (obstacle_avoid_t * p_ctx, obstacle_avoid_action_t * p_act,
      obstacle_avoid_state_t state, uint32_t now_ms, char const * p_reason)
{
    p_act->b_set_inhibit = true;
    p_act->p_reason = p_reason;
    if ((state == OBSTACLE_AVOID_BLOCKED_WAIT) && !p_ctx->b_blocked)
    {
        p_ctx->b_blocked = true;
        p_ctx->blocked_since_ms = now_ms;
    }
    enter(p_ctx, state, now_ms);
}

static void
release (obstacle_avoid_t * p_ctx, obstacle_avoid_action_t * p_act,
         bool b_to_follow, uint32_t now_ms, char const * p_reason)
{
    p_act->b_clear_inhibit = true;
    p_act->p_reason = p_reason;
    p_ctx->b_to_follow = b_to_follow;
    enter(p_ctx, OBSTACLE_AVOID_RELEASING, now_ms);
}

/* Plan from a PATH_FOUND result not used before; true if one was seen. */
static bool
try_plan (obstacle_avoid_t * p_ctx, obstacle_avoid_action_t * p_act,
          obstacle_avoid_input_t const * p_in)
{
    robot_obstacle_status_t const status =
        robot_obstacle_get_status(p_in->scan.flags);
    if (((status != ROBOT_OBSTACLE_PATH_FOUND) &&
         (status != ROBOT_OBSTACLE_BLOCKED)) ||
        (p_ctx->b_plan_sequence &&
         (p_in->scan.sequence == p_ctx->plan_sequence)))
    {
        return false;
    }
    p_ctx->b_plan_sequence = true;
    p_ctx->plan_sequence = p_in->scan.sequence;
    p_ctx->plan = obstacle_avoid_plan(p_in->scan, p_in->geometry,
                                      &p_ctx->p_config->geometry);
    bool const b_blocked = status == ROBOT_OBSTACLE_BLOCKED;
    if (b_blocked && (p_ctx->plan.status == OBSTACLE_AVOID_PLAN_NOT_PATH))
    {
        /* No edge either side: plan left at the maximum; the corridor
         * sweep after reversing chooses the side. */
        robot_obstacle_result_t fallback = p_in->scan;
        fallback.flags =
            robot_obstacle_pack_flags(ROBOT_OBSTACLE_PATH_FOUND,
                                      ROBOT_OBSTACLE_DIR_LEFT_60, false, false);
        fallback.width_mm = ROBOT_OBSTACLE_WIDTH_UNAVAILABLE;
        p_ctx->plan = obstacle_avoid_plan(fallback, p_in->geometry,
                                          &p_ctx->p_config->geometry);
    }
    if (p_ctx->plan.status != OBSTACLE_AVOID_PLAN_OK)
    {
        static char const * const reasons[] = {"plan ok", "no side to try",
                                               "no geometry"};
        enter(p_ctx, OBSTACLE_AVOID_WAIT_CLEAR, p_in->now_ms);
        p_act->p_reason = ((uint32_t)p_ctx->plan.status <
                           (sizeof(reasons) / sizeof(reasons[0])))
                              ? reasons[p_ctx->plan.status]
                              : "no bypass";
        return true;
    }
    p_ctx->leg = (p_ctx->plan.reverse_mm > 0u) ? OBSTACLE_AVOID_LEG_REVERSE
                                               : OBSTACLE_AVOID_LEG_TURN_OUT;
    p_ctx->b_leg_started = false;
    p_ctx->sidestep_mm = p_ctx->plan.sidestep_mm;
    p_ctx->lateral_mm = 0u;
    p_ctx->forward_mm = 0u;
    p_ctx->b_backoff = false;
    p_ctx->backoffs = 0u;
    p_ctx->escaped_mm = 0u;
    if (b_blocked)
    {
        /* No free space either side: reverse, then sweep again. */
        if (p_ctx->plan.reverse_mm < ESCAPE_STEP_MM)
        {
            uint32_t const pass_mm = (uint32_t)p_ctx->plan.pass_mm +
                                     ESCAPE_STEP_MM - p_ctx->plan.reverse_mm;
            p_ctx->plan.pass_mm =
                (pass_mm > p_ctx->p_config->geometry.pass_max_mm)
                    ? p_ctx->p_config->geometry.pass_max_mm
                    : (uint16_t)pass_mm;
            p_ctx->plan.reverse_mm = ESCAPE_STEP_MM;
        }
        p_ctx->escaped_mm = ESCAPE_STEP_MM;
        p_ctx->leg = OBSTACLE_AVOID_LEG_REVERSE;
    }
    /* A sweep taken very close is unreliable: after reversing for room,
     * sweep again before turning. */
    p_ctx->b_replan_pending = p_ctx->plan.reverse_mm > 0u;
    p_ctx->b_watching = false;
    p_ctx->b_watch_seen = false;
    p_ctx->b_watch_done = false;
    p_ctx->b_watch_widened = false;
    p_ctx->b_shift = false;
    /* Without re-checks, keep the sensor ahead for the whole bypass. */
    p_act->b_side_scans_off = !p_ctx->p_config->timing.b_recheck;
    p_ctx->sidesteps = 0u;
    p_ctx->b_retry_used = false;
    p_ctx->b_blocked = false;
    p_ctx->b_need_prepare = false;
    release(p_ctx, p_act, false, p_in->now_ms,
            b_blocked              ? "no free side, reverse and look again"
            : p_ctx->plan.b_no_fit ? "path found, tight"
            : p_ctx->plan.b_limited
                ? "path found, best effort"
                : (p_ctx->plan.b_edge_unknown ? "path found, gap edge unknown"
                                              : "path found"));
    return true;
}

/* Begin or resume the current leg, with the car stopped and released. */
static void
start_leg (obstacle_avoid_t * p_ctx, obstacle_avoid_action_t * p_act,
           obstacle_avoid_input_t const * p_in)
{
    obstacle_avoid_geometry_t const * const p_geometry =
        &p_ctx->p_config->geometry;
    obstacle_avoid_timing_t const * const p_timing = &p_ctx->p_config->timing;
    for (;;)
    {
        obstacle_avoid_leg_t const leg = p_ctx->leg;
        if (leg > OBSTACLE_AVOID_LEG_PASS)
        {
            watch_off(p_ctx, p_act); /* Past the obstacle: look ahead. */
        }
        if (((leg == OBSTACLE_AVOID_LEG_VERIFY) ||
             (leg == OBSTACLE_AVOID_LEG_TURN_OUT)) &&
            p_ctx->b_replan_pending)
        {
            start_replan(p_ctx, p_act, p_in);
            return;
        }
        if ((leg == OBSTACLE_AVOID_LEG_VERIFY) && !p_timing->b_recheck)
        {
            next_leg(p_ctx); /* Trust the first sweep's edges. */
            continue;
        }
        if (leg == OBSTACLE_AVOID_LEG_VERIFY)
        {
            p_act->b_corridor = true;
            p_act->p_reason = leg_reason(leg);
            p_ctx->b_corridor_started = false;
            p_ctx->corridor_sequence = p_in->corridor.sequence;
            enter(p_ctx, OBSTACLE_AVOID_VERIFY, p_in->now_ms);
            return;
        }
        if (leg == OBSTACLE_AVOID_LEG_FOLLOW)
        {
            p_ctx->part = 0u;
            p_ctx->cmd = robot_motion_cmd_follow();
            p_act->b_send = true;
            p_act->cmd = p_ctx->cmd;
            p_act->p_reason = leg_reason(leg);
            enter(p_ctx, OBSTACLE_AVOID_SEND, p_in->now_ms);
            return;
        }
        if (!p_ctx->b_leg_started)
        {
            p_ctx->b_leg_started = true;
            switch (leg)
            {
                case OBSTACLE_AVOID_LEG_REVERSE:
                    p_ctx->remaining = p_ctx->plan.reverse_mm;
                    break;
                case OBSTACLE_AVOID_LEG_SIDESTEP:
                    p_ctx->remaining = along_diagonal(p_ctx->sidestep_mm,
                                                      p_ctx->plan.angle_deg);
                    break;
                case OBSTACLE_AVOID_LEG_PASS:
                    p_ctx->remaining = pass_left(p_ctx);
                    break;
                case OBSTACLE_AVOID_LEG_RETURN:
                    p_ctx->remaining = along_diagonal(
                        p_ctx->lateral_mm + p_geometry->return_extra_mm,
                        p_ctx->plan.angle_deg);
                    break;
                case OBSTACLE_AVOID_LEG_OVERSHOOT:
                    p_ctx->remaining = p_geometry->overshoot_mm;
                    break;
                default:
                    p_ctx->remaining = p_ctx->plan.angle_deg;
                    break;
            }
            /* A move after a turn or a sweep needs fresh readings. */
            p_ctx->b_need_prepare = (leg != OBSTACLE_AVOID_LEG_OVERSHOOT) &&
                                    (leg != OBSTACLE_AVOID_LEG_REVERSE) &&
                                    leg_is_move(leg);
        }
        uint32_t const resume_min = leg_is_move(leg) ? p_timing->resume_min_mm
                                                     : p_timing->resume_min_deg;
        if ((p_ctx->remaining == 0u) || ((p_ctx->remaining <= resume_min) &&
                                         (leg != OBSTACLE_AVOID_LEG_OVERSHOOT)))
        {
            if (leg == OBSTACLE_AVOID_LEG_RETURN)
            {
                retry_or_fault(p_ctx, p_act, p_in, true, "line not found");
                return;
            }
            next_leg(p_ctx);
            continue;
        }
        if (leg_is_move(leg) && p_ctx->b_need_prepare && p_timing->b_recheck)
        {
            p_act->b_rearm = true;
            p_act->p_reason = leg_reason(leg);
            p_ctx->b_rearm_known = false;
            enter(p_ctx, OBSTACLE_AVOID_PREPARE, p_in->now_ms);
            return;
        }
        send_part(p_ctx, p_act, p_in);
        return;
    }
}

/* Send the next command of the current leg and note where it starts. */
static void
send_part (obstacle_avoid_t * p_ctx, obstacle_avoid_action_t * p_act,
           obstacle_avoid_input_t const * p_in)
{
    obstacle_avoid_timing_t const * const p_timing = &p_ctx->p_config->timing;
    obstacle_avoid_leg_t const leg = p_ctx->leg;
    uint32_t expected_ms;
    if (leg_is_move(leg))
    {
        p_ctx->b_watch_widened = false;
        if (((leg == OBSTACLE_AVOID_LEG_SIDESTEP) ||
             (leg == OBSTACLE_AVOID_LEG_PASS)) &&
            !p_ctx->b_watching && !p_ctx->b_watch_done)
        {
            /* Look ahead along the obstacle side of the path. On the car
             * the servo's 60 degree end faces right of the heading
             * (hardware, 2026-11-04): a left bypass watches at 60. */
            p_ctx->b_watching = true;
            p_ctx->b_watch_seen = false;
            p_ctx->watch_sequence = p_in->watch.sequence;
            p_act->b_watch = true;
            p_act->watch_angle = p_ctx->plan.b_gap_right
                                     ? (int16_t)OBSTACLE_SWEEP_LAST_DEG
                                     : (int16_t)OBSTACLE_SWEEP_FIRST_DEG;
        }
        p_ctx->part = obstacle_avoid_next_part(p_ctx->remaining);
        p_ctx->cmd = robot_motion_cmd_move(p_ctx->part,
                                           leg == OBSTACLE_AVOID_LEG_REVERSE,
                                           leg == OBSTACLE_AVOID_LEG_RETURN);
        expected_ms = (p_ctx->part * 1000u) / p_timing->move_speed_mm_s;
    }
    else
    {
        /* Toward the gap on the way out and home, away from it between. */
        bool const b_away = (leg == OBSTACLE_AVOID_LEG_TURN_BACK) ||
                            (leg == OBSTACLE_AVOID_LEG_TURN_IN);
        p_ctx->part = p_ctx->remaining;
        /* Reacquisition starts after PASS: a line encountered while
         * turning inward is just as usable as one found on the return. */
        p_ctx->cmd = robot_motion_cmd_turn(
            p_ctx->part, b_away != p_ctx->plan.b_gap_right,
            (leg == OBSTACLE_AVOID_LEG_TURN_IN) ||
                (leg == OBSTACLE_AVOID_LEG_TURN_HOME));
        expected_ms = (p_ctx->part * 1000u) / p_timing->turn_speed_deg_s;
    }
    p_ctx->leg_timeout_ms = (2u * expected_ms) + LEG_TIMEOUT_EXTRA_MS;
    p_ctx->start_distance_mm = p_in->distance_mm;
    p_ctx->start_heading_deg = p_in->heading_deg;
    p_act->b_send = true;
    p_act->cmd = p_ctx->cmd;
    p_act->p_reason = leg_reason(leg);
    enter(p_ctx, OBSTACLE_AVOID_SEND, p_in->now_ms);
}

/* Finish the current leg's side effects and move to the next. */
static void
next_leg (obstacle_avoid_t * p_ctx)
{
    if (p_ctx->leg == OBSTACLE_AVOID_LEG_SIDESTEP)
    {
        p_ctx->lateral_mm += p_ctx->sidestep_mm;
        p_ctx->forward_mm +=
            diagonal_forward(p_ctx->sidestep_mm, p_ctx->plan.angle_deg);
        ++p_ctx->sidesteps;
    }
    p_ctx->leg = (p_ctx->leg >= OBSTACLE_AVOID_LEG_FOLLOW)
                     ? OBSTACLE_AVOID_LEG_FOLLOW
                     : (obstacle_avoid_leg_t)((uint32_t)p_ctx->leg + 1u);
    p_ctx->b_leg_started = false;
}

/* MOVE_DONE: the command in flight ended by itself. */
static void
move_done (obstacle_avoid_t * p_ctx, obstacle_avoid_action_t * p_act,
           obstacle_avoid_input_t const * p_in)
{
    p_ctx->b_blocked = false;
    if (p_ctx->b_backoff)
    {
        /* Drive the reversed distance again as part of this leg. */
        p_ctx->b_backoff = false;
        if (leg_is_move(p_ctx->leg))
        {
            p_ctx->remaining += p_ctx->part;
        }
        start_leg(p_ctx, p_act, p_in);
        return;
    }
    if (((p_ctx->leg == OBSTACLE_AVOID_LEG_TURN_IN) ||
         (p_ctx->leg == OBSTACLE_AVOID_LEG_RETURN)) &&
        (p_in->motion_done == ROBOT_MOTION_DONE_LINE))
    {
        /* Motion has braked on fresh, confirmed IR evidence. Let the line
         * follower align with this line, even if it is not the old one.
         * A fixed overshoot/home turn could drive away from the new line. */
        p_ctx->remaining = 0u;
        p_ctx->leg = OBSTACLE_AVOID_LEG_FOLLOW;
        p_ctx->b_leg_started = false;
        start_leg(p_ctx, p_act, p_in);
        if (p_ctx->state == OBSTACLE_AVOID_SEND)
        {
            p_act->p_reason = "line found, aligning";
        }
        return;
    }
    p_ctx->remaining -=
        (p_ctx->part < p_ctx->remaining) ? p_ctx->part : p_ctx->remaining;
    if (p_ctx->remaining > 0u)
    {
        send_part(p_ctx, p_act, p_in);
        return;
    }
    if (p_ctx->leg == OBSTACLE_AVOID_LEG_RETURN)
    {
        retry_or_fault(p_ctx, p_act, p_in, true, "line not found");
        return;
    }
    next_leg(p_ctx);
    start_leg(p_ctx, p_act, p_in);
}

/* Take what the interrupted command achieved off the leg's remainder. */
static bool
record_progress (obstacle_avoid_t * p_ctx, obstacle_avoid_input_t const * p_in)
{
    if (!p_in->b_odometry_valid)
    {
        return false;
    }
    int32_t const moved = leg_is_move(p_ctx->leg)
                              ? (p_in->distance_mm - p_ctx->start_distance_mm)
                              : (p_in->heading_deg - p_ctx->start_heading_deg);
    uint32_t done = (moved < 0) ? (uint32_t)(-moved) : (uint32_t)moved;
    if (done > p_ctx->part)
    {
        done = p_ctx->part;
    }
    p_ctx->remaining -= (done < p_ctx->remaining) ? done : p_ctx->remaining;
    p_ctx->b_need_prepare = leg_is_move(p_ctx->leg);
    return true;
}

static void
interrupt_leg (obstacle_avoid_t * p_ctx, obstacle_avoid_action_t * p_act,
               obstacle_avoid_input_t const * p_in, char const * p_reason)
{
    if (!record_progress(p_ctx, p_in))
    {
        latch_fault(p_ctx, p_act, OBSTACLE_AVOID_FAULT_ODOMETRY, p_in->now_ms,
                    "no odometry");
        return;
    }
    hold(p_ctx, p_act, OBSTACLE_AVOID_BLOCKED_WAIT, p_in->now_ms, p_reason);
}

/* Stall, leg timeout or missed line: one retry per bypass, then FAULT. */
static void
retry_or_fault (obstacle_avoid_t * p_ctx, obstacle_avoid_action_t * p_act,
                obstacle_avoid_input_t const * p_in, bool b_line_missed,
                char const * p_reason)
{
    if (p_ctx->b_retry_used)
    {
        latch_fault(p_ctx, p_act, OBSTACLE_AVOID_FAULT_RETRY_USED, p_in->now_ms,
                    p_reason);
        return;
    }
    p_ctx->b_retry_used = true;
    if (b_line_missed)
    {
        /* Search once more for the line, the same way. */
        p_ctx->leg = OBSTACLE_AVOID_LEG_RETURN;
        p_ctx->b_leg_started = true;
        p_ctx->b_need_prepare = true;
        p_ctx->remaining = p_ctx->p_config->geometry.return_extra_mm;
    }
    else if (!record_progress(p_ctx, p_in))
    {
        latch_fault(p_ctx, p_act, OBSTACLE_AVOID_FAULT_ODOMETRY, p_in->now_ms,
                    "no odometry");
        return;
    }
    else
    {
        /* Resume the remainder after the delay. */
    }
    hold(p_ctx, p_act, OBSTACLE_AVOID_RETRY_WAIT, p_in->now_ms, p_reason);
}

static void
step_prepare (obstacle_avoid_t * p_ctx, obstacle_avoid_action_t * p_act,
              obstacle_avoid_input_t const * p_in, bool b_critical)
{
    if (p_in->b_rearmed && !p_ctx->b_rearm_known)
    {
        p_ctx->b_rearm_known = true;
        p_ctx->rearm_sequence = p_in->rearm_sequence;
    }
    bool const b_fresh =
        p_ctx->b_rearm_known &&
        sequence_newer(p_in->scan.sequence, p_ctx->rearm_sequence);
    robot_obstacle_status_t const status =
        robot_obstacle_get_status(p_in->scan.flags);
    if (b_critical)
    {
        hold(p_ctx, p_act, OBSTACLE_AVOID_BLOCKED_WAIT, p_in->now_ms,
             "critical");
    }
    else if (b_fresh && (status == ROBOT_OBSTACLE_CLEAR) &&
             !p_in->b_stop_requested)
    {
        p_ctx->b_need_prepare = false;
        send_part(p_ctx, p_act, p_in);
    }
    else if (b_fresh && (status != ROBOT_OBSTACLE_SCANNING) &&
             (status != ROBOT_OBSTACLE_CLEAR))
    {
        hold(p_ctx, p_act, OBSTACLE_AVOID_BLOCKED_WAIT, p_in->now_ms,
             "obstacle ahead");
    }
    else if (elapsed(p_in->now_ms, p_ctx->entered_ms,
                     p_ctx->p_config->timing.prepare_ms))
    {
        latch_fault(p_ctx, p_act, OBSTACLE_AVOID_FAULT_SCANNER, p_in->now_ms,
                    "no fresh result");
    }
    else
    {
        /* Waiting for the settle and a fresh centre group. */
    }
}

static void
step_run_leg (obstacle_avoid_t * p_ctx, obstacle_avoid_action_t * p_act,
              obstacle_avoid_input_t const * p_in, bool b_critical)
{
    if (p_in->b_enc_fault)
    {
        latch_fault(p_ctx, p_act, OBSTACLE_AVOID_FAULT_ENCODER, p_in->now_ms,
                    "encoder fault");
    }
    else if (p_in->b_rejected)
    {
        latch_fault(p_ctx, p_act, OBSTACLE_AVOID_FAULT_REJECTED, p_in->now_ms,
                    "command rejected");
    }
    else if (p_in->b_stall)
    {
        retry_or_fault(p_ctx, p_act, p_in, false, "stall");
    }
    else if (p_ctx->b_backoff &&
             (p_in->b_stall ||
              elapsed(p_in->now_ms, p_ctx->entered_ms, p_ctx->leg_timeout_ms)))
    {
        /* Count nothing as driven; the forward remainder grows instead. */
        p_ctx->b_backoff = false;
        if (leg_is_move(p_ctx->leg))
        {
            p_ctx->remaining += p_ctx->part;
        }
        p_ctx->part = 0u;
        retry_or_fault(p_ctx, p_act, p_in, false, "back-off failed");
    }
    else if (b_critical && !p_ctx->b_backoff &&
             (p_ctx->leg != OBSTACLE_AVOID_LEG_REVERSE))
    {
        interrupt_leg(p_ctx, p_act, p_in, "critical");
    }
    else if (leg_is_move(p_ctx->leg) && p_in->b_stop_requested &&
             !p_ctx->b_backoff && (p_ctx->leg != OBSTACLE_AVOID_LEG_REVERSE) &&
             p_ctx->p_config->timing.b_recheck)
    {
        /* Turns sweep the sensor past the obstacle; moves must not. */
        interrupt_leg(p_ctx, p_act, p_in, "obstacle ahead");
    }
    else if (watch_step(p_ctx, p_act, p_in))
    {
        /* Stopped to shift away from the obstacle. */
    }
    else if (p_in->b_move_done)
    {
        move_done(p_ctx, p_act, p_in);
    }
    else if (elapsed(p_in->now_ms, p_ctx->entered_ms, p_ctx->leg_timeout_ms))
    {
        retry_or_fault(p_ctx, p_act, p_in, false, "leg timeout");
    }
    else
    {
        /* Command still running. */
    }
}

/*
 * Use a new watch reading on the diagonal or the pass. The sensor looks
 * (90 - OBSTACLE_SWEEP_FIRST_DEG) degrees toward the obstacle side of the
 * heading, so a point at range r lies r sin of that from the path and
 * r cos of it ahead. A point ahead within this leg and closer to the path
 * than half the car plus the margin widens the offset: on the diagonal by
 * driving it farther, on the pass by stopping to sidestep out. Once the
 * car has driven past the last point seen (plus WATCH_CLEAR_EXTRA_MM)
 * without seeing it again, the sensor faces ahead again.
 * Returns true only when it stopped the car.
 */
static bool
watch_step (obstacle_avoid_t * p_ctx, obstacle_avoid_action_t * p_act,
            obstacle_avoid_input_t const * p_in)
{
    obstacle_avoid_config_t const * const p_config = p_ctx->p_config;
    obstacle_avoid_leg_t const leg = p_ctx->leg;
    if (!p_ctx->b_watching || p_ctx->b_backoff || !p_in->watch.b_active ||
        (p_in->watch.sequence == p_ctx->watch_sequence) ||
        ((leg != OBSTACLE_AVOID_LEG_SIDESTEP) &&
         (leg != OBSTACLE_AVOID_LEG_PASS)))
    {
        return false;
    }
    p_ctx->watch_sequence = p_in->watch.sequence;
    float range_mm = 0.0f;
    bool const b_seen = (p_in->watch.echo_us != 0u) &&
                        obstacle_width_range_mm(p_in->watch.echo_us,
                                                &p_config->width, &range_mm) &&
                        (range_mm < WATCH_RANGE_MM);
    if (!b_seen)
    {
        int32_t const since = p_in->distance_mm - p_ctx->watch_seen_mm;
        uint32_t const past =
            (since < 0) ? (uint32_t)(-since) : (uint32_t)since;
        if (p_ctx->b_watch_seen && p_in->b_odometry_valid &&
            (past >= ((uint32_t)p_ctx->watch_ahead_mm + WATCH_CLEAR_EXTRA_MM)))
        {
            watch_off(p_ctx, p_act);
            p_ctx->b_watch_done = true;
            p_act->p_reason = "obstacle passed, sensor ahead";
        }
        return false;
    }

    float const off_rad =
        (float)(OBSTACLE_CENTRE_DEG - OBSTACLE_SWEEP_FIRST_DEG) * DEG_TO_RAD;
    float const lateral_mm = range_mm * sinf(off_rad);
    float const ahead_mm = range_mm * cosf(off_rad);
    p_ctx->b_watch_seen = true;
    p_ctx->watch_seen_mm = p_in->distance_mm;
    p_ctx->watch_ahead_mm = (uint16_t)ceilf(ahead_mm);
    int32_t const moved = p_in->b_odometry_valid
                              ? (p_in->distance_mm - p_ctx->start_distance_mm)
                              : 0;
    uint32_t const driven = (moved < 0) ? (uint32_t)(-moved) : (uint32_t)moved;
    float const left_mm =
        (p_ctx->remaining > driven) ? (float)(p_ctx->remaining - driven) : 0.0f;
    float const need_mm = (float)p_config->geometry.half_width_mm +
                          (float)p_config->geometry.margin_mm;
    if (p_ctx->b_watch_widened || !(lateral_mm < need_mm) ||
        (ahead_mm > left_mm))
    {
        return false;
    }
    float shift_mm = ceilf(need_mm - lateral_mm);
    if (shift_mm < WATCH_SHIFT_MIN_MM)
    {
        shift_mm = WATCH_SHIFT_MIN_MM;
    }
    uint32_t const shift = (uint32_t)shift_mm;
    uint32_t const offset_mm =
        p_ctx->lateral_mm + ((leg == OBSTACLE_AVOID_LEG_SIDESTEP)
                                 ? (uint32_t)p_ctx->sidestep_mm
                                 : 0u);
    if ((offset_mm + shift) > p_config->geometry.lateral_max_mm)
    {
        return false; /* No more room; CRITICAL still guards. */
    }
    p_ctx->b_watch_widened = true;
    if (leg == OBSTACLE_AVOID_LEG_SIDESTEP)
    {
        /* Drive the diagonal farther once this command ends. */
        p_ctx->sidestep_mm = (uint16_t)(p_ctx->sidestep_mm + shift);
        p_ctx->remaining += along_diagonal(shift, p_ctx->plan.angle_deg);
        p_act->p_reason = "too close, widening";
        return false;
    }

    /* On the pass: keep the progress, then sidestep out and carry on. */
    uint32_t const before = p_ctx->remaining;
    if (!record_progress(p_ctx, p_in))
    {
        latch_fault(p_ctx, p_act, OBSTACLE_AVOID_FAULT_ODOMETRY, p_in->now_ms,
                    "no odometry");
        return true;
    }
    p_ctx->forward_mm += before - p_ctx->remaining;
    p_ctx->sidestep_mm = (uint16_t)shift;
    p_ctx->leg = OBSTACLE_AVOID_LEG_TURN_OUT;
    p_ctx->b_leg_started = false;
    p_ctx->b_shift = true;
    hold(p_ctx, p_act, OBSTACLE_AVOID_RETRY_WAIT, p_in->now_ms,
         "too close, shifting out");
    return true;
}

/* Face the sensor ahead again, if it was watching. */
static void
watch_off (obstacle_avoid_t * p_ctx, obstacle_avoid_action_t * p_act)
{
    if (p_ctx->b_watching)
    {
        p_ctx->b_watching = false;
        p_act->b_watch = true;
        p_act->watch_angle = (int16_t)OBSTACLE_SCAN_WATCH_OFF;
    }
}

static void
step_verify (obstacle_avoid_t * p_ctx, obstacle_avoid_action_t * p_act,
             obstacle_avoid_input_t const * p_in, bool b_critical)
{
    obstacle_avoid_config_t const * const p_config = p_ctx->p_config;
    if (p_in->corridor_request == OBSTACLE_AVOID_CORRIDOR_REQ_STARTED)
    {
        p_ctx->b_corridor_started = true;
    }
    if (b_critical)
    {
        hold(p_ctx, p_act, OBSTACLE_AVOID_BLOCKED_WAIT, p_in->now_ms,
             "critical");
        return;
    }
    if (p_ctx->b_corridor_started &&
        (p_in->corridor.sequence != p_ctx->corridor_sequence))
    {
        uint16_t shift_mm = 0u;
        obstacle_avoid_corridor_t const corridor =
            obstacle_avoid_check_corridor(
                &p_in->corridor, &p_config->width, &p_config->geometry,
                (uint16_t)pass_left(p_ctx), p_ctx->plan.b_gap_right, &shift_mm);
        if (corridor == OBSTACLE_AVOID_CORRIDOR_CLEAR)
        {
            p_ctx->b_blocked = false;
            next_leg(p_ctx);
            start_leg(p_ctx, p_act, p_in);
        }
        else if ((corridor == OBSTACLE_AVOID_CORRIDOR_SHIFT) &&
                 (p_ctx->sidesteps < p_config->timing.max_sidesteps) &&
                 ((p_ctx->lateral_mm + shift_mm) <=
                  p_config->geometry.lateral_max_mm))
        {
            p_ctx->sidestep_mm = shift_mm;
            p_ctx->leg = OBSTACLE_AVOID_LEG_TURN_OUT;
            p_ctx->b_leg_started = false;
            start_leg(p_ctx, p_act, p_in);
            p_act->p_reason = "extra sidestep";
        }
        else
        {
            hold(p_ctx, p_act, OBSTACLE_AVOID_BLOCKED_WAIT, p_in->now_ms,
                 (corridor == OBSTACLE_AVOID_CORRIDOR_SHIFT)
                     ? "no room to shift"
                     : "corridor blocked");
        }
    }
    else if (elapsed(p_in->now_ms, p_ctx->entered_ms,
                     p_config->timing.verify_ms))
    {
        latch_fault(p_ctx, p_act, OBSTACLE_AVOID_FAULT_SCANNER, p_in->now_ms,
                    "no corridor result");
    }
    else if (!p_ctx->b_corridor_started)
    {
        /* Refused, or not yet answered: ask again. */
        p_act->b_corridor = true;
    }
    else
    {
        /* Sweep running. */
    }
}

/*
 * Whether the car fits past an end reading (end_mm from the pivot, at the
 * sweep's first angle off the centre line; 0 = no echo): the point lies
 * beyond the pass, or at least the sidestep plus half the car and the
 * margin out to the side. The sidestep is clamped as the plan clamps it.
 */
static bool
end_fits (uint16_t end_mm, int32_t need_mm, uint32_t pass_mm,
          obstacle_avoid_geometry_t const * p_geometry)
{
    if (end_mm == 0u)
    {
        return true;
    }
    if (need_mm > (int32_t)p_geometry->sidestep_max_mm)
    {
        need_mm = (int32_t)p_geometry->sidestep_max_mm;
    }
    if (need_mm < (int32_t)p_geometry->sidestep_min_mm)
    {
        need_mm = (int32_t)p_geometry->sidestep_min_mm;
    }
    float const angle_rad = (float)OBSTACLE_SWEEP_FIRST_DEG * DEG_TO_RAD;
    float const y_mm = (float)end_mm * sinf(angle_rad);
    if (!(y_mm < (float)pass_mm))
    {
        return true; /* Beyond the end of the pass. */
    }
    float const x_mm = fabsf((float)end_mm * cosf(angle_rad));
    return x_mm >= ((float)need_mm + (float)p_geometry->half_width_mm +
                    (float)p_geometry->margin_mm);
}

/*
 * Shallowest angle whose diagonal reaches the sideways offset before the
 * stopped car's front (about the pivot) covers the room left in front of
 * the obstacle face, less the margin; rounded up to ANGLE_STEP_DEG and kept
 * within angle_min_deg..90. No room left means a quarter turn.
 */
static uint8_t
plan_angle (uint16_t sidestep_mm, uint16_t centre_mm,
            obstacle_avoid_geometry_t const * p_geometry)
{
    uint32_t angle_deg = ANGLE_MAX_DEG;
    if (centre_mm > p_geometry->margin_mm)
    {
        float const room_mm = (float)(centre_mm - p_geometry->margin_mm);
        float const need_deg = atan2f((float)sidestep_mm, room_mm) / DEG_TO_RAD;
        angle_deg =
            (uint32_t)ceilf(need_deg / (float)ANGLE_STEP_DEG) * ANGLE_STEP_DEG;
    }
    if (angle_deg < p_geometry->angle_min_deg)
    {
        angle_deg = p_geometry->angle_min_deg;
    }
    return (angle_deg > ANGLE_MAX_DEG) ? (uint8_t)ANGLE_MAX_DEG
                                       : (uint8_t)angle_deg;
}

/* Distance driven along a diagonal to move lateral_mm sideways. */
static uint32_t
along_diagonal (uint32_t lateral_mm, uint8_t angle_deg)
{
    if (angle_deg >= ANGLE_MAX_DEG)
    {
        return lateral_mm;
    }
    return (uint32_t)ceilf((float)lateral_mm /
                           sinf((float)angle_deg * DEG_TO_RAD));
}

/* Progress along the line made by that diagonal. */
static uint32_t
diagonal_forward (uint32_t lateral_mm, uint8_t angle_deg)
{
    if (angle_deg >= ANGLE_MAX_DEG)
    {
        return 0u;
    }
    return (uint32_t)((float)lateral_mm / tanf((float)angle_deg * DEG_TO_RAD));
}

/* Pass distance still to drive after the diagonals' forward progress. */
static uint32_t
pass_left (obstacle_avoid_t const * p_ctx)
{
    return (p_ctx->forward_mm < p_ctx->plan.pass_mm)
               ? (p_ctx->plan.pass_mm - p_ctx->forward_mm)
               : 0u;
}

/* Reverse backoff_mm along the current heading, away from CRITICAL. */
static void
send_backoff (obstacle_avoid_t * p_ctx, obstacle_avoid_action_t * p_act,
              obstacle_avoid_input_t const * p_in)
{
    obstacle_avoid_timing_t const * const p_timing = &p_ctx->p_config->timing;
    p_ctx->part = p_ctx->p_config->geometry.backoff_mm;
    p_ctx->cmd = robot_motion_cmd_move(p_ctx->part, true, false);
    p_ctx->leg_timeout_ms =
        (2u * ((p_ctx->part * 1000u) / p_timing->move_speed_mm_s)) +
        LEG_TIMEOUT_EXTRA_MS;
    p_ctx->start_distance_mm = p_in->distance_mm;
    p_ctx->start_heading_deg = p_in->heading_deg;
    p_act->b_send = true;
    p_act->cmd = p_ctx->cmd;
    p_act->p_reason = "back off";
    enter(p_ctx, OBSTACLE_AVOID_SEND, p_in->now_ms);
}

/* Measure the whole 60-120 degree arc from here to choose a direction;
 * a corridor sweep runs whatever is straight ahead. */
static void
start_replan (obstacle_avoid_t * p_ctx, obstacle_avoid_action_t * p_act,
              obstacle_avoid_input_t const * p_in)
{
    p_ctx->b_replan_pending = false;
    p_ctx->b_watching = false; /* The corridor sweep takes the servo. */
    p_ctx->b_corridor_started = false;
    p_ctx->corridor_sequence = p_in->corridor.sequence;
    p_act->b_corridor = true;
    p_act->p_reason = "re-sweep";
    enter(p_ctx, OBSTACLE_AVOID_REPLAN, p_in->now_ms);
}

/*
 * Check the corridor sweep for both sides: each is clear, needs a shift,
 * or is blocked. Before any sideways move either side may be taken, after
 * it only the side already used. Nothing in the way: carry on (follow the
 * line if the bypass has not started). Otherwise measured sector clearance
 * wins (ties keep the current side) and sizes the next sidestep; none fits:
 * wait held.
 */
static void
step_replan (obstacle_avoid_t * p_ctx, obstacle_avoid_action_t * p_act,
             obstacle_avoid_input_t const * p_in, bool b_inhibited)
{
    obstacle_avoid_config_t const * const p_config = p_ctx->p_config;
    if (p_in->corridor_request == OBSTACLE_AVOID_CORRIDOR_REQ_STARTED)
    {
        p_ctx->b_corridor_started = true;
    }
    if (!p_ctx->b_corridor_started ||
        (p_in->corridor.sequence == p_ctx->corridor_sequence))
    {
        if (elapsed(p_in->now_ms, p_ctx->entered_ms,
                    p_config->timing.verify_ms))
        {
            latch_fault(p_ctx, p_act, OBSTACLE_AVOID_FAULT_SCANNER,
                        p_in->now_ms, "no re-sweep result");
        }
        else if (!p_ctx->b_corridor_started)
        {
            p_act->b_corridor = true; /* Refused or not answered yet. */
        }
        else
        {
            /* Sweep running. */
        }
        return;
    }

    bool const b_any_side =
        (p_ctx->lateral_mm == 0u) && (p_ctx->sidesteps == 0u);
    uint32_t const left_pass = pass_left(p_ctx);
    uint16_t const pass_mm =
        (left_pass > 0u) ? (uint16_t)left_pass : p_ctx->plan.pass_mm;
    uint16_t shift_mm[2] = {0u, 0u};
    obstacle_avoid_corridor_t side[2];
    for (uint8_t right = 0u; right < 2u; ++right)
    {
        side[right] = obstacle_avoid_check_corridor(
            &p_in->corridor, &p_config->width, &p_config->geometry, pass_mm,
            right != 0u, &shift_mm[right]);
        bool const b_allowed =
            b_any_side || ((right != 0u) == p_ctx->plan.b_gap_right);
        if (!b_allowed)
        {
            side[right] = OBSTACLE_AVOID_CORRIDOR_BLOCKED;
        }
        if (side[right] == OBSTACLE_AVOID_CORRIDOR_SHIFT)
        {
            uint16_t const step =
                (shift_mm[right] < p_config->geometry.sidestep_min_mm)
                    ? p_config->geometry.sidestep_min_mm
                    : shift_mm[right];
            if ((p_ctx->sidesteps >= p_config->timing.max_sidesteps) ||
                ((p_ctx->lateral_mm + step) >
                 p_config->geometry.lateral_max_mm))
            {
                side[right] = OBSTACLE_AVOID_CORRIDOR_BLOCKED;
            }
        }
    }
    uint8_t const current = p_ctx->plan.b_gap_right ? 1u : 0u;
    if (side[current] == OBSTACLE_AVOID_CORRIDOR_CLEAR)
    {
        if (b_any_side && (p_ctx->leg == OBSTACLE_AVOID_LEG_TURN_OUT))
        {
            p_ctx->leg = OBSTACLE_AVOID_LEG_FOLLOW; /* Not in the way. */
        }
        else if (p_ctx->leg == OBSTACLE_AVOID_LEG_VERIFY)
        {
            next_leg(p_ctx);
        }
        else
        {
            /* Carry on with the current leg. */
        }
        resume_after_replan(p_ctx, p_act, p_in, b_inhibited, "path clear");
        return;
    }

    uint8_t const other = (uint8_t)(1u - current);
    uint8_t chosen = current;
    if ((side[current] == OBSTACLE_AVOID_CORRIDOR_BLOCKED) ||
        ((side[other] != OBSTACLE_AVOID_CORRIDOR_BLOCKED) &&
         (corridor_clearance(&p_in->corridor, other != 0u) >
          corridor_clearance(&p_in->corridor, current != 0u))))
    {
        chosen = other;
    }
    uint16_t const step_mm =
        (shift_mm[chosen] < p_config->geometry.sidestep_min_mm)
            ? p_config->geometry.sidestep_min_mm
            : shift_mm[chosen];
    if ((side[chosen] != OBSTACLE_AVOID_CORRIDOR_BLOCKED) &&
        (p_ctx->sidesteps < p_config->timing.max_sidesteps) &&
        ((p_ctx->lateral_mm + step_mm) <= p_config->geometry.lateral_max_mm))
    {
        p_ctx->plan.b_gap_right = chosen != 0u;
        p_ctx->sidestep_mm = step_mm;
        p_ctx->leg = OBSTACLE_AVOID_LEG_TURN_OUT;
        p_ctx->b_leg_started = false;
        resume_after_replan(p_ctx, p_act, p_in, b_inhibited, "re-planned");
    }
    else if (b_any_side &&
             ((p_ctx->escaped_mm + ESCAPE_STEP_MM) <= ESCAPE_MAX_MM))
    {
        /* No side fits from here: reverse, then sweep again. */
        uint32_t const pass_mm = (uint32_t)p_ctx->plan.pass_mm + ESCAPE_STEP_MM;
        p_ctx->plan.pass_mm = (pass_mm > p_config->geometry.pass_max_mm)
                                  ? p_config->geometry.pass_max_mm
                                  : (uint16_t)pass_mm;
        p_ctx->escaped_mm += ESCAPE_STEP_MM;
        p_ctx->leg = OBSTACLE_AVOID_LEG_REVERSE;
        p_ctx->b_leg_started = true;
        p_ctx->remaining = ESCAPE_STEP_MM;
        p_ctx->b_replan_pending = true;
        resume_after_replan(p_ctx, p_act, p_in, b_inhibited,
                            "back away and look again");
    }
    else
    {
        hold(p_ctx, p_act, OBSTACLE_AVOID_BLOCKED_WAIT, p_in->now_ms,
             "no bypass from here");
    }
}

/* Rank a side by its closest measured return across that half of the
 * sweep. The centre obstacle is common to both and is not a side reading.
 * No echo ranks above finite echoes, as in the scanner's side decision. */
static uint32_t
corridor_clearance (obstacle_scan_corridor_t const * p_corridor, bool b_right)
{
    uint32_t nearest = UINT32_MAX;
    uint8_t const first = b_right ? (uint8_t)(OBSTACLE_CENTRE_INDEX + 1u) : 0u;
    uint8_t const end = b_right ? (uint8_t)OBSTACLE_SWEEP_POINTS
                                : (uint8_t)OBSTACLE_CENTRE_INDEX;
    for (uint8_t index = first; index < end; ++index)
    {
        if ((p_corridor->valid_mask & (uint16_t)(1u << index)) == 0u)
        {
            return 0u;
        }
        uint16_t const echo = p_corridor->echo_us[index];
        if ((echo != 0u) && ((uint32_t)echo < nearest))
        {
            nearest = echo;
        }
    }
    return nearest;
}

/* Carry on with side scans off again: release INHIBIT first if held. */
static void
resume_after_replan (obstacle_avoid_t * p_ctx, obstacle_avoid_action_t * p_act,
                     obstacle_avoid_input_t const * p_in, bool b_inhibited,
                     char const * p_reason)
{
    p_ctx->backoffs = 0u;
    if (b_inhibited)
    {
        release(p_ctx, p_act, false, p_in->now_ms, p_reason);
    }
    else
    {
        start_leg(p_ctx, p_act, p_in);
        p_act->p_reason = p_reason;
    }
    p_act->b_side_scans_off = !p_ctx->p_config->timing.b_recheck;
}
