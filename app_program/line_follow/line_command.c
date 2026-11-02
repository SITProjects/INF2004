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

static uint8_t junction_sample_count;

/* Public function definitions. */

void
line_command_init (void)
{
    junction_sample_count = 0u;
}

line_command_t
line_command_update (ir_sensor_reading_t reading)
{
    bool const b_left_low = (reading.left_raw < LINE_LEFT_EDGE_MIN);
    bool const b_left_high = (reading.left_raw > LINE_LEFT_EDGE_MAX);
    bool const b_right_low = (reading.right_raw < LINE_RIGHT_EDGE_MIN);
    bool const b_right_high = (reading.right_raw > LINE_RIGHT_EDGE_MAX);
    bool const b_both_black = (reading.left_raw >= LINE_LEFT_BLACK_MIN) &&
                              (reading.right_raw >= LINE_RIGHT_BLACK_MIN);
    line_command_t command;

    if (b_both_black)
    {
        if (junction_sample_count < JUNCTION_CONFIRM_SAMPLES)
        {
            ++junction_sample_count;
        }
    }
    else
    {
        junction_sample_count = 0u;
    }

    if (b_left_high && b_right_high)
    {
        command = (junction_sample_count >= JUNCTION_CONFIRM_SAMPLES)
                      ? LINE_COMMAND_JUNCTION
                      : LINE_COMMAND_STOP;
    }
    else if (b_left_low && b_right_low)
    {
        /* Neither sensor observes its edge; acquisition is required. */
        command = LINE_COMMAND_SEARCH;
    }
    else if (b_left_high || b_right_low)
    {
        command = LINE_COMMAND_STEER_LEFT;
    }
    else if (b_left_low || b_right_high)
    {
        command = LINE_COMMAND_STEER_RIGHT;
    }
    else
    {
        /* Both readings lie within their inclusive edge bands. */
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

        case LINE_COMMAND_STOP:
            p_name = "STOP";
            break;

        default:
            p_name = "UNKNOWN";
            break;
    }

    return p_name;
}
