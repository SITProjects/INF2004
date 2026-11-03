/**
 * @file line_command.c
 * @brief Line-following decisions using two IR sensors.
 */

#include "line_command.h"

#include <stdbool.h>
#include <stdint.h>

/* Private types, constants, and macros. */

#define JUNCTION_CONFIRM_SAMPLES 3u

/* Static data. */

static struct
{
    unsigned int junction_samples : 2;
    unsigned int b_searching : 1;
    unsigned int b_clock_started : 1;
    unsigned int b_reacquiring : 1;
    unsigned int b_last_right : 1;
    uint32_t last_detection_ms;
    uint32_t reacquire_since_ms;
} tracking;

/* Public function definitions. */

void
line_command_init (void)
{
    tracking.junction_samples = 0u;
    tracking.b_searching = 1u;
    tracking.b_clock_started = 0u;
    tracking.b_reacquiring = 0u;
    tracking.b_last_right = 0u;
    tracking.last_detection_ms = 0u;
    tracking.reacquire_since_ms = 0u;
}

void
line_command_begin_tracking (void)
{
    line_command_init();
    tracking.b_searching = 0u;
}

line_command_t
line_command_update (ir_sensor_reading_t reading, uint32_t now_ms)
{
    if ((reading.left_raw > 4095u) || (reading.right_raw > 4095u))
    {
        line_command_init();
        return LINE_COMMAND_STOP;
    }
    if (!tracking.b_clock_started)
    {
        tracking.last_detection_ms = now_ms;
        tracking.b_clock_started = 1u;
    }
    bool const b_left_line = (reading.left_raw >= LINE_LEFT_DETECT_MIN);
    bool const b_right_line = (reading.right_raw >= LINE_RIGHT_DETECT_MIN);
    bool const b_both_black = (reading.left_raw >= LINE_LEFT_BLACK_MIN) &&
                              (reading.right_raw >= LINE_RIGHT_BLACK_MIN);
    if ((uint32_t)(now_ms - tracking.last_detection_ms) >= LINE_EDGE_PROBE_MS)
    {
        tracking.b_searching = 1u;
        tracking.b_reacquiring = 0u;
    }
    if (b_both_black)
    {
        if (tracking.junction_samples < JUNCTION_CONFIRM_SAMPLES)
        {
            ++tracking.junction_samples;
        }
    }
    else
    {
        tracking.junction_samples = 0u;
    }

    if (tracking.b_searching && b_both_black)
    {
        tracking.b_reacquiring = 0u;
        if (tracking.junction_samples < JUNCTION_CONFIRM_SAMPLES)
        {
            return LINE_COMMAND_SEARCH;
        }
        return tracking.b_last_right ? LINE_COMMAND_TURN_RIGHT
                                     : LINE_COMMAND_TURN_LEFT;
    }

    if (tracking.b_searching)
    {
        bool const b_single_edge = (b_left_line != b_right_line);
        if (b_single_edge)
        {
            if (!tracking.b_reacquiring)
            {
                tracking.reacquire_since_ms = now_ms;
                tracking.b_reacquiring = 1u;
            }
            tracking.b_last_right = b_right_line;
        }
        if (b_left_line || b_right_line)
        {
            tracking.last_detection_ms = now_ms;
        }
        if (!tracking.b_reacquiring)
        {
            return LINE_COMMAND_SEARCH;
        }
        /* Brief white gaps and changes of side are normal on a curve.
         * Only fresh single-edge evidence may end the slow phase. */
        if (!b_single_edge ||
            ((uint32_t)(now_ms - tracking.reacquire_since_ms) <
             LINE_REACQUIRE_CONFIRM_MS))
        {
            return tracking.b_last_right ? LINE_COMMAND_ALIGN_RIGHT
                                         : LINE_COMMAND_ALIGN_LEFT;
        }
        tracking.b_reacquiring = 0u;
        tracking.b_searching = 0u;
    }

    if (b_left_line || b_right_line)
    {
        tracking.b_searching = 0u;
        tracking.last_detection_ms = now_ms;
        if (b_left_line != b_right_line)
        {
            tracking.b_last_right = b_right_line;
        }
    }
    if (!b_left_line && !b_right_line)
    {
        return tracking.b_last_right ? LINE_COMMAND_PROBE_RIGHT
                                     : LINE_COMMAND_PROBE_LEFT;
    }

    line_command_t command;
    if (b_both_black)
    {
        /* Only full-black pairs override tracking to cross straight. */
        command = (tracking.junction_samples >= JUNCTION_CONFIRM_SAMPLES)
                      ? LINE_COMMAND_JUNCTION
                      : LINE_COMMAND_FORWARD;
    }
    else if (!b_left_line && b_right_line)
    {
        command = LINE_COMMAND_STEER_RIGHT;
    }
    else if (b_left_line && !b_right_line)
    {
        command = LINE_COMMAND_STEER_LEFT;
    }
    else
    {
        /* Both sensors see coverage: keep the crossing motion straight. */
        command = LINE_COMMAND_FORWARD;
    }

    return command;
}

char const *
line_command_name (line_command_t command)
{
    char const * p_name;

    switch (command)
    {
        case LINE_COMMAND_SEARCH:
            p_name = "SEARCH";
            break;

        case LINE_COMMAND_FORWARD:
            p_name = "FORWARD";
            break;

        case LINE_COMMAND_STEER_LEFT:
            p_name = "STEER_LEFT";
            break;

        case LINE_COMMAND_STEER_RIGHT:
            p_name = "STEER_RIGHT";
            break;

        case LINE_COMMAND_JUNCTION:
            p_name = "JUNCTION";
            break;

        case LINE_COMMAND_PROBE_LEFT:
            p_name = "PROBE_LEFT";
            break;

        case LINE_COMMAND_PROBE_RIGHT:
            p_name = "PROBE_RIGHT";
            break;

        case LINE_COMMAND_TURN_LEFT:
            p_name = "TURN_LEFT_90";
            break;

        case LINE_COMMAND_TURN_RIGHT:
            p_name = "TURN_RIGHT_90";
            break;

        case LINE_COMMAND_ALIGN_LEFT:
            p_name = "ALIGN_LEFT";
            break;

        case LINE_COMMAND_ALIGN_RIGHT:
            p_name = "ALIGN_RIGHT";
            break;

        case LINE_COMMAND_STOP:
            p_name = "STOP";
            break;

        default:
            p_name = "UNKNOWN";
            break;
    }

    return p_name;
}
