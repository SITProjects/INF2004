/**
 * @file line_follow.c
 * @brief Line-following decisions using two IR sensors.
 */

#include "line_follow.h"

#include <stdbool.h>

/* Private types, constants, and macros. */

#define LEFT_BLACK_THRESHOLD 700u
#define RIGHT_BLACK_THRESHOLD 700u

typedef enum
{
    LINE_FOLLOW_SEARCHING = 0,
    LINE_FOLLOW_FOLLOWING
} line_follow_state_t;

/* Static data. */

static line_follow_state_t state = LINE_FOLLOW_SEARCHING;

/* Public function definitions. */

void
line_follow_init (void)
{
    state = LINE_FOLLOW_SEARCHING;
}

line_command_t
line_follow_update (line_sensor_reading_t reading)
{
    bool const b_left_black = (reading.left_raw >= LEFT_BLACK_THRESHOLD);

    bool const b_right_black = (reading.right_raw >= RIGHT_BLACK_THRESHOLD);

    line_command_t command;

    if (b_left_black || b_right_black)
    {
        state = LINE_FOLLOW_FOLLOWING;
    }

    if (b_left_black && b_right_black)
    {
        command = LINE_COMMAND_JUNCTION;
    }
    else if (b_left_black)
    {
        command = LINE_COMMAND_STEER_LEFT;
    }
    else if (b_right_black)
    {
        command = LINE_COMMAND_STEER_RIGHT;
    }
    else if (LINE_FOLLOW_SEARCHING == state)
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
line_follow_command_name (line_command_t command)
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

        default:
            p_name = "UNKNOWN";
            break;
    }

    return p_name;
}
