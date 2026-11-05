/**
 * @file obstacle_calibration.c
 * @brief Stationary raw measurements; no normal scanner or motor calls.
 *
 * CAL timestamps are accepted-trigger times, with kernel tick resolution.
 * Logical (scan) angles precede the mirror and trim. CALMETA records follow completed readings so
 * console output cannot delay a measured move-to-trigger interval. Elapsed
 * times include driver spacing retries, tick rounding and scheduling delay.
 */

#include "obstacle_calibration.h"
#include "obstacle_config.h"

#if OBSTACLE_CALIBRATION_MODE != OBSTACLE_CAL_MODE_OFF

#include "../app.h"
#include "drv_servo.h"
#include "drv_ultrasonic.h"

/* Private types, constants, and macros. */
#define CAL_BURST_PERIOD_MS 5000u /* Reposition window after each burst. */
#define CAL_TRIM_FIRST_DEG 80
#define CAL_TRIM_LAST_DEG 100
#define CAL_TRIM_SETTLE_MS 1000u
#define CAL_SETTLE_MARGIN_MAX_MS 300
#define CAL_SETTLE_MARGIN_STEP_MS 50
#define CAL_SETTLE_REPEATS 10u
#define CAL_HOLD_MS 10000u
#define CAL_WIDE_FIRST_DEG 30
#define CAL_WIDE_LAST_DEG 150
#define CAL_WIDE_STEP_DEG 5
#define CAL_WIDE_PINGS 5u
#define CAL_WIDE_PASS_GAP_MS 3000u /* Time to move the test object. */

/* Static data. */
static int commanded_angle;
static uint32_t move_time_ms;
static uint32_t move_wait_ms;

/* Private function declarations. */
static uint32_t cal_now_ms (void);
static void cal_wait (uint32_t wait_ms);
static void cal_move (int angle, uint32_t wait_ms);
static void cal_ping_point (int angle, uint16_t pings);
static void cal_run_mode (void);

/* Public function definitions. */

_Noreturn void
obstacle_calibration_run (int trig_pin, int echo_pin, int servo_pin)
{
    if (servo_init(servo_pin) || ultrasonic_init(trig_pin, echo_pin))
    {
        tm_printf((UB *)"CALMETA,ERROR,initialisation_failed\n");
        tk_ext_tsk();
        for (;;)
        {
            (void)tk_slp_tsk(TMO_FEVR);
        }
    }

    /* Initial physical angle is unknown; servo_init() commands centre. */
    cal_wait(OBSTACLE_SETTLE_MS(180u));
    tm_printf((UB *)"CALMETA,START,%u,%d\n",
              (unsigned int)OBSTACLE_CALIBRATION_MODE,
              (int)OBSTACLE_SERVO_TRIM_DEG);
    cal_run_mode();
    for (;;)
    {
        (void)tk_slp_tsk(TMO_FEVR);
    }
}

/* Private function definitions. */

static uint32_t
cal_now_ms (void)
{
    SYSTIM time;
    (void)tk_get_otm(&time);
    return (uint32_t)time.lo;
}

/* Round positive waits up to whole kernel ticks; never request delay zero. */
static void
cal_wait (uint32_t wait_ms)
{
    if (wait_ms != 0u)
    {
        uint32_t rounded_ms =
            ((wait_ms + OBSTACLE_TICK_MS - 1u) / OBSTACLE_TICK_MS) *
            OBSTACLE_TICK_MS;
        (void)tk_dly_tsk((RELTIM)rounded_ms);
    }
}

static void
cal_move (int angle, uint32_t wait_ms)
{
    move_time_ms = cal_now_ms();
    servo_set_angle(OBSTACLE_SERVO_COMMAND_DEG(angle));
    commanded_angle = angle;
    move_wait_ms = wait_ms;
    cal_wait(wait_ms);
}

static void
cal_ping_point (int angle, uint16_t pings)
{
    for (uint16_t ping = 1u; ping <= pings; ping++)
    {
        /* No console output before trigger: preserve the settle test. */
        while (!ultrasonic_trigger())
        {
            cal_wait(OBSTACLE_TICK_MS);
        }
        uint32_t trigger_ms = cal_now_ms();
        uint16_t echo_us = 0u;
        ultrasonic_result_t result;
        do
        {
            result = ultrasonic_read(&echo_us);
            if (result == ULTRASONIC_PENDING)
            {
                cal_wait(OBSTACLE_TICK_MS);
            }
        } while (result == ULTRASONIC_PENDING);
        if (result != ULTRASONIC_VALID)
        {
            echo_us = 0u;
        }
        tm_printf((UB *)"CAL,%u,%lu,%d,%u,%u\n",
                  (unsigned int)OBSTACLE_CALIBRATION_MODE,
                  (unsigned long)trigger_ms, angle, (unsigned int)result,
                  (unsigned int)echo_us);
        tm_printf((UB *)"CALMETA,MOVE,%lu,%d,%d,%d,%lu\n",
                  (unsigned long)move_time_ms, commanded_angle,
                  OBSTACLE_SERVO_COMMAND_DEG(commanded_angle),
                  (int)OBSTACLE_SERVO_TRIM_DEG, (unsigned long)move_wait_ms);
        tm_printf((UB *)"CALMETA,PING,%lu,%d,%u,%lu\n",
                  (unsigned long)trigger_ms, angle, (unsigned int)ping,
                  (unsigned long)(uint32_t)(trigger_ms - move_time_ms));
    }
}

#if OBSTACLE_CALIBRATION_MODE == OBSTACLE_CAL_MODE_RANGING
static void
cal_run_mode (void)
{
    cal_move(90, OBSTACLE_SETTLE_MS(60u));
    for (uint32_t burst = 1u;; burst++)
    {
        tm_printf((UB *)"CALMETA,CONTEXT,burst,%lu,0,0\n",
                  (unsigned long)burst);
        cal_ping_point(90, OBSTACLE_CAL_PINGS);
        cal_wait(CAL_BURST_PERIOD_MS);
    }
}
#elif OBSTACLE_CALIBRATION_MODE == OBSTACLE_CAL_MODE_TRIM_SWEEP
static void
cal_run_mode (void)
{
    for (;;)
    {
        for (int angle = CAL_TRIM_FIRST_DEG; angle <= CAL_TRIM_LAST_DEG;
             angle++)
        {
            tm_printf((UB *)"CALMETA,CONTEXT,point,%d,0,0\n", angle);
            cal_move(angle, CAL_TRIM_SETTLE_MS);
            cal_ping_point(angle, OBSTACLE_CAL_PINGS);
        }
    }
}
#elif OBSTACLE_CALIBRATION_MODE == OBSTACLE_CAL_MODE_SETTLE
static void
cal_run_mode (void)
{
    const int destination = OBSTACLE_CAL_SETTLE_TARGET_DEG;
    /* Start each trial inside 60..120, one configured move away. */
    const int origin = (destination == 60)
                           ? (destination + OBSTACLE_CAL_SETTLE_MOVE_DEG)
                           : (destination - OBSTACLE_CAL_SETTLE_MOVE_DEG);
    const uint32_t travel_ms = (((uint32_t)OBSTACLE_CAL_SETTLE_MOVE_DEG *
                                 OBSTACLE_SERVO_TRAVEL_MS_PER_60_DEG) +
                                59u) /
                               60u;
    for (;;)
    {
        tm_printf((UB *)"CALMETA,CONTEXT,reference,%d,0,0\n", destination);
        cal_move(destination, OBSTACLE_SETTLE_MS(60u));
        cal_ping_point(destination, OBSTACLE_CAL_PINGS);
        for (int margin = CAL_SETTLE_MARGIN_MAX_MS; margin >= 0;
             margin -= CAL_SETTLE_MARGIN_STEP_MS)
        {
            for (unsigned int repeat = 1u; repeat <= CAL_SETTLE_REPEATS;
                 repeat++)
            {
                tm_printf((UB *)"CALMETA,CONTEXT,margin,%d,%d,%u\n",
                          destination, margin, repeat);
                cal_move(origin, OBSTACLE_SETTLE_MS(60u));
                cal_move(destination, travel_ms + (uint32_t)margin);
                cal_ping_point(destination, 1u);
            }
        }
    }
}
#elif OBSTACLE_CALIBRATION_MODE == OBSTACLE_CAL_MODE_ANGLES
static void
cal_run_mode (void)
{
    static const int angles[] = {60, 120, 60, 90, 120};
    int previous_angle = -1;
    for (;;)
    {
        for (unsigned int point = 0u;
             point < sizeof(angles) / sizeof(angles[0]); point++)
        {
            int angle = angles[point];
            int delta = previous_angle < 0 ? 60 : angle - previous_angle;
            uint32_t travel = (uint32_t)(delta < 0 ? -delta : delta);
            tm_printf((UB *)"CALMETA,CONTEXT,point,%u,0,0\n", point);
            cal_move(angle, OBSTACLE_SETTLE_MS(travel));
            previous_angle = angle;
            /* Let the final ping finish; spacing/polling can extend hold. */
            while ((uint32_t)(cal_now_ms() - move_time_ms) < CAL_HOLD_MS)
            {
                cal_ping_point(angle, 1u);
            }
        }
    }
}
#elif OBSTACLE_CALIBRATION_MODE == OBSTACLE_CAL_MODE_WIDE
/*
 * Repeated passes over the widened sweep, one line per angle:
 * CALWIDE,pass,scan_deg,servo_cmd,valid_pings,nearest_us,farthest_us
 * (echoes 0 when none). Run it with nothing in front to find where the
 * car's body shows, then with an object on the car's left to check the
 * servo direction (OBSTACLE_SERVO_MIRRORED), then with the obstacle
 * whose width is to be checked.
 */
static void
cal_run_mode (void)
{
    for (uint32_t pass = 1u;; pass++)
    {
        int previous = CAL_WIDE_LAST_DEG;
        for (int angle = CAL_WIDE_FIRST_DEG; angle <= CAL_WIDE_LAST_DEG;
             angle += CAL_WIDE_STEP_DEG)
        {
            int delta = angle - previous;
            uint32_t travel = (uint32_t)((delta < 0) ? -delta : delta);
            cal_move(angle, OBSTACLE_SETTLE_MS(travel));
            previous = angle;
            unsigned int valid = 0u;
            uint16_t nearest_us = 0u;
            uint16_t farthest_us = 0u;
            for (uint16_t ping = 0u; ping < CAL_WIDE_PINGS; ping++)
            {
                while (!ultrasonic_trigger())
                {
                    cal_wait(OBSTACLE_TICK_MS);
                }
                uint16_t echo_us = 0u;
                ultrasonic_result_t result;
                do
                {
                    result = ultrasonic_read(&echo_us);
                    if (result == ULTRASONIC_PENDING)
                    {
                        cal_wait(OBSTACLE_TICK_MS);
                    }
                } while (result == ULTRASONIC_PENDING);
                if (result == ULTRASONIC_VALID)
                {
                    valid++;
                    if ((nearest_us == 0u) || (echo_us < nearest_us))
                    {
                        nearest_us = echo_us;
                    }
                    if (echo_us > farthest_us)
                    {
                        farthest_us = echo_us;
                    }
                }
            }
            tm_printf((UB *)"CALWIDE,%lu,%d,%d,%u,%u,%u\n", (unsigned long)pass,
                      angle, OBSTACLE_SERVO_COMMAND_DEG(angle), valid,
                      (unsigned int)nearest_us, (unsigned int)farthest_us);
        }
        cal_wait(CAL_WIDE_PASS_GAP_MS);
    }
}
#endif

#endif /* OBSTACLE_CALIBRATION_MODE != OBSTACLE_CAL_MODE_OFF */
