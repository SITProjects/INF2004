/**
 * @file line_follow.c
 * @brief Line-following subsystem: dual IR line sensing and decisions.
 *
 * The line task reads both IR sensors and runs the line-following logic
 * at a nominal 20 ms interval, and prints periodic raw snapshots. It
 * owns the ADC and submits decisions to the motion task, which owns
 * the motors. It never calls the motor driver directly.
 * APP_TEST prints raw readings and per-sensor line-detection labels.
 * Detection levels require validation at fixed mounting height/spacing.
 */

#include "line_follow.h"
#include "line_command.h"
#include "../motion/motion_command.h"
#include "../motion/motion_command_config.h"

/* Private types, constants, and macros. */
#define SAMPLE_INTERVAL_MS 20
#if APP_TEST
#define SAMPLES_PER_PRINT 5u /* 5 x 20 ms: nominal 100 ms diagnostics. */
#else
#define SAMPLES_PER_PRINT 20u /* Roughly 600 ms with the kernel delay. */
#endif

/* Static data. */
ID line_follow_tid_sense;

/* Private function declarations. */
static void sense_task (INT stacd, void * p_exinf);
static void report_junction (line_command_t command);

/* Public function definitions. */

ER
line_follow_init (void)
{
    /* No kernel objects; the line task opens the ADC itself. */
    return E_OK;
}

ER
line_follow_start (void)
{
    line_follow_tid_sense = app_start_task(sense_task, LINE_FOLLOW_PRI_SENSE, 0,
                                           "line_follow_task");
    return (line_follow_tid_sense < E_OK) ? line_follow_tid_sense : E_OK;
}

/* Private function definitions. */

static void
sense_task (INT stacd, void * p_exinf)
{
    (void)stacd;
    (void)p_exinf;

    if (!ir_sensor_init())
    {
        tm_printf((UB *)"line_follow: failed to open the ADC device\n");
        tk_ext_tsk();
    }
#if !APP_TEST && !IR_CALIBRATION_ONLY && MOTION_COMMAND_ACTUATION_ENABLED
    /* Start sensing after the same preparation delay as the motor owner. */
    (void)tk_dly_tsk(MOTION_COMMAND_START_DELAY_MS);
#endif
#if IR_MOTOR_TEST || IR_CALIBRATION_ONLY
    /* Controlled tests start with the robot manually placed on the line. */
    line_command_begin_tracking();
#else
    line_command_init();
#endif
#if IR_CALIBRATION_ONLY
    tm_printf(
        (UB *)"line_follow: stationary calibration; motors not started\n");
#else
    tm_printf((UB *)"line_follow: IR line sensing started. "
                    "Submitting commands to motion.\n");
#endif

    uint32_t sample = 0u;
    for (;;)
    {
        ir_sensor_reading_t reading;
        bool const b_read = ir_sensor_read(&reading);
        SYSTIM now;
        bool const b_clock = (tk_get_otm(&now) == E_OK);
        bool const b_sample_valid = b_read && b_clock;
        line_command_t command = LINE_COMMAND_STOP;
        if (b_sample_valid)
        {
            command = line_command_update(reading, (uint32_t)now.lo);
        }
        else
        {
            line_command_init();
        }

        report_junction(command);

#if !IR_CALIBRATION_ONLY
        ER const submission =
            motion_command_submit(command, b_sample_valid ? &reading : NULL);
        if ((submission < E_OK) && (submission != E_TMOUT))
        {
            app_check(submission, "line motion submission");
        }
#endif

        if ((sample % SAMPLES_PER_PRINT) == 0u)
        {
            if (b_sample_valid)
            {
#if APP_TEST
                tm_printf((UB *)"LEFT_RAW=%u  RIGHT_RAW=%u  "
                                "LEFT=%s  RIGHT=%s  COMMAND=%s\n",
                          (UINT)reading.left_raw, (UINT)reading.right_raw,
                          (reading.left_raw >= LINE_LEFT_DETECT_MIN) ? "LINE"
                                                                     : "WHITE",
                          (reading.right_raw >= LINE_RIGHT_DETECT_MIN)
                              ? "LINE"
                              : "WHITE",
                          line_command_name(command));
#else
                tm_printf((UB *)"Left AO: %u | Right AO: %u | Command: %s\n",
                          (UINT)reading.left_raw, (UINT)reading.right_raw,
                          line_command_name(command));
#endif
            }
            else
            {
                tm_printf((UB *)(b_read
                                     ? "line_follow: monotonic clock failed\n"
                                     : "line_follow: IR sensor read failed\n"));
            }
        }
        sample++;

        /* A relative delay: late samples are never replayed. */
        (void)tk_dly_tsk(SAMPLE_INTERVAL_MS);
    }
}

/* Detection event is independent of the slower raw-reading snapshots.
 * A dashboard can consume this status later without changing motion policy. */
static void
report_junction (line_command_t command)
{
    static bool b_reported;
    bool const b_junction = (command == LINE_COMMAND_JUNCTION);
    if (b_junction && !b_reported)
    {
        tm_printf((UB *)"line_follow: JUNCTION detected; continuing forward\n");
    }
    b_reported = b_junction;
}
