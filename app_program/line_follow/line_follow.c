/**
 * @file line_follow.c
 * @brief Line-following subsystem: dual IR line sensing and decisions.
 *
 * The line task reads both IR sensors and runs the line-following logic
 * at a nominal 20 ms interval, and prints a snapshot every 200 ms. It
 * owns the ADC and submits decisions to the motion task, which owns
 * the motors. It never calls the motor driver directly.
 * APP_TEST prints raw readings and provisional edge-band labels every
 * ~100 ms. These hand-measured bands require fixed-height validation
 * and are shared with the provisional navigation decisions.
 */

#include "line_follow.h"
#include "line_command.h"
#include "../motion/motion_command.h"

/* Private types, constants, and macros. */
#define SAMPLE_INTERVAL_MS 20
#if APP_TEST
#define SAMPLES_PER_PRINT 5u /* 5 x 20 ms: nominal 100 ms diagnostics. */
#else
#define SAMPLES_PER_PRINT 10u /* 10 x 20 ms: print every 200 ms. */
#endif

/* Static data. */
ID line_follow_tid_sense;

/* Private function declarations. */
static void sense_task (INT stacd, void * p_exinf);
#if APP_TEST
static char const * edge_band_name (uint16_t raw, uint16_t lower,
                                    uint16_t upper);
#endif

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
    line_command_init();
    tm_printf((UB *)"line_follow: IR line sensing started. "
                    "Submitting commands to motion.\n");

    uint32_t sample = 0u;
    for (;;)
    {
        ir_sensor_reading_t reading;
        bool b_read = ir_sensor_read(&reading);
        line_command_t command = LINE_COMMAND_STOP;
        if (b_read)
        {
            command = line_command_update(reading);
        }

        ER const submission =
            motion_command_submit(command, b_read ? &reading : NULL);
        if ((submission < E_OK) && (submission != E_TMOUT))
        {
            app_check(submission, "line motion submission");
        }

        if ((sample % SAMPLES_PER_PRINT) == 0u)
        {
            if (b_read)
            {
#if APP_TEST
                tm_printf((UB *)"LEFT_RAW=%u  RIGHT_RAW=%u  "
                                "LEFT=%s  RIGHT=%s  COMMAND=%s\n",
                          (UINT)reading.left_raw, (UINT)reading.right_raw,
                          edge_band_name(reading.left_raw, LINE_LEFT_EDGE_MIN,
                                         LINE_LEFT_EDGE_MAX),
                          edge_band_name(reading.right_raw, LINE_RIGHT_EDGE_MIN,
                                         LINE_RIGHT_EDGE_MAX),
                          line_command_name(command));
#else
                tm_printf((UB *)"Left AO: %u | Right AO: %u | Command: %s\n",
                          (UINT)reading.left_raw, (UINT)reading.right_raw,
                          line_command_name(command));
#endif
            }
            else
            {
                tm_printf((UB *)"line_follow: IR sensor read failed\n");
            }
        }
        sample++;

        /* A relative delay: late samples are never replayed. */
        (void)tk_dly_tsk(SAMPLE_INTERVAL_MS);
    }
}

#if APP_TEST
/* Label the measured interval only; do not infer surface or line position. */
static char const *
edge_band_name (uint16_t raw, uint16_t lower, uint16_t upper)
{
    char const * p_name;

    if (raw < lower)
    {
        p_name = "BELOW_EDGE";
    }
    else if (raw > upper)
    {
        p_name = "ABOVE_EDGE";
    }
    else
    {
        p_name = "EDGE_BAND";
    }

    return p_name;
}
#endif /* APP_TEST */
