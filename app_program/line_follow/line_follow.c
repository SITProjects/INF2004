/**
 * @file line_follow.c
 * @brief Line-following subsystem: dual IR line sensing and decisions.
 *
 * The line task reads both IR sensors and runs the line-following logic
 * at a nominal 20 ms interval, and prints a snapshot every 200 ms. It
 * owns the ADC (TEAM.md section 3). It does not drive the motors.
 */

#include "line_follow.h"
#include "line_command.h"

/* Private types, constants, and macros. */
#define SAMPLE_INTERVAL_MS 20
#define SAMPLES_PER_PRINT 10u /* 10 x 20 ms: print every 200 ms. */

/* Static data. */
ID line_follow_tid_sense;

/* Private function declarations. */
static void sense_task (INT stacd, void * p_exinf);

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
    tm_printf((UB *)"line_follow: IR line-following dry run started. "
                    "Motors disabled.\n");

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

        if ((sample % SAMPLES_PER_PRINT) == 0u)
        {
            if (b_read)
            {
                tm_printf((UB *)"Left AO: %u | Right AO: %u | Command: %s\n",
                          (UINT)reading.left_raw, (UINT)reading.right_raw,
                          line_command_name(command));
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
