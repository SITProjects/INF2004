/**
 * @file terrain_hint.c
 * @brief Movement hints for the terrain module: stub until the motion
 *        subsystem publishes its speed.
 *
 * To plug in a real source, fill the fields from it in terrain_hint_read()
 * and set the matching _known flags; nothing else needs to change.
 *
 * Copyright (c) 2026 10x Slackers
 * SPDX-License-Identifier: MIT
 */

#include "terrain_hint.h"
#include <stddef.h>

/* Private types, constants, and macros. */

/* Static data. */

/* Private function declarations. */

/* Public function definitions. */

void
terrain_hint_read (terrain_hint_t * p_out)
{
    if (p_out == NULL)
    {
        return;
    }

    /* No source yet: everything unknown, so tilt is never held. */
    terrain_hint_t const unknown = {false, false, false, false,
                                    false, false, 0u};
    *p_out = unknown;
}

/* Private function definitions. */
