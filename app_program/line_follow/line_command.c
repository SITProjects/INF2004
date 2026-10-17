/**
 * @file line_command.c
 * @brief Line-following decisions using two IR sensors.
 */

#include "line_command.h"

#include <stdbool.h>
#include <stdint.h>

/* Private types, constants, and macros. */

#define LEFT_BLACK_THRESHOLD 700u
#define RIGHT_BLACK_THRESHOLD 700u
#define JUNCTION_CONFIRM_SAMPLES 3u

typedef enum
{
    LINE_FOLLOW_UNKNOWN = 0,
    LINE_FOLLOW_FOLLOWING
} line_follow_state_t;

/* Static data. */

static line_follow_state_t state = LINE_FOLLOW_UNKNOWN;
static uint8_t junction_sample_count;

/* Public function definitions. */

void
line_command_init (void)
{
    state = LINE_FOLLOW_UNKNOWN;
    junction_sample_count = 0u;
}

void
line_command_start_following (void)
{
    state = LINE_FOLLOW_FOLLOWING;
}

line_command_t
line_command_update (ir_sensor_reading_t reading)
{
    bool const b_left_black = (reading.left_raw >= LEFT_BLACK_THRESHOLD);

    bool const b_right_black = (reading.right_raw >= RIGHT_BLACK_THRESHOLD);

    line_command_t command;

    if (b_left_black && b_right_black)
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

    if (b_left_black && b_right_black)
    {
        command = (junction_sample_count >= JUNCTION_CONFIRM_SAMPLES)
                      ? LINE_COMMAND_JUNCTION
                      : LINE_COMMAND_STOP;
    }
    else if (b_left_black)
    {
        command = LINE_COMMAND_STEER_LEFT;
    }
    else if (b_right_black)
    {
        command = LINE_COMMAND_STEER_RIGHT;
    }
    else if (LINE_FOLLOW_UNKNOWN == state)
    {
        command = LINE_COMMAND_SEARCH;
    }
    else
    {
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
