/**
 * @file terrain_process.c
 * @brief IMU processing: axis mapping, tilt, motion state, impacts, bumps.
 *
 * Copyright (c) 2026 10x Slackers
 * SPDX-License-Identifier: MIT
 */

#include "terrain_process.h"
#include <stddef.h>

/* Private types, constants, and macros. */

/* Static data. */

/* Private function declarations. */
static float map_one (terrain_axis_map_t const * p_map, uint32_t car_axis,
                      int16_t const board[TERRAIN_AXES], float scale);

/* Public function definitions. */

terrain_vec3_t
terrain_process_map_axes (terrain_axis_map_t const * p_map,
                          int16_t const board[TERRAIN_AXES], float scale)
{
    terrain_vec3_t car = {0.0f, 0.0f, 0.0f};

    if ((p_map == NULL) || (board == NULL))
    {
        return car;
    }

    for (uint32_t i = 0u; i < TERRAIN_AXES; i++)
    {
        if (p_map->from[i] >= TERRAIN_AXES)
        {
            return car;
        }
    }

    car.x = map_one(p_map, 0u, board, scale);
    car.y = map_one(p_map, 1u, board, scale);
    car.z = map_one(p_map, 2u, board, scale);
    return car;
}

/* Private function definitions. */

static float
map_one (terrain_axis_map_t const * p_map, uint32_t car_axis,
         int16_t const board[TERRAIN_AXES], float scale)
{
    float value = (float)board[p_map->from[car_axis]] * scale;
    return (p_map->sign[car_axis] < 0) ? -value : value;
}
