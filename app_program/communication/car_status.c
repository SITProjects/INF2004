/**
 * @file car_status.c
 * @brief The car's state, packed for the dashboard.
 *
 * Kernel side: motion and obstacle headers cannot share a file with lwIP's.
 */

#include "car_status.h"
#include "../app.h"
#include "../robot_types.h"
#include "../motion/motion.h"
#include "../motion/motion_command.h"
#include "../obstacle/obstacle.h"
#include "../terrain/terrain_monitor.h"

/* Private types, constants, and macros. */

/* Private function declarations. */

/* Public function definitions. */

void
car_status_encode (uint32_t * p_words)
{
    motion_shared_odometry_t odometry = {0};
    (void)motion_get_odometry(&odometry);
    motion_task_status_t const motion = motion_get_status();

    obstacle_shared_state_t obstacle = {0};
    (void)obstacle_get_state(&obstacle);

    terrain_data_t terrain = {0};
    bool const b_level = terrain_monitor_get_latest(&terrain) == TERRAIN_OK;
    terrain_summary_t summary = {{0u}};
    (void)terrain_monitor_get_summary(&summary);

    uint32_t requests = 0u;
    uint32_t taken = 0u;
    motion_command_get_counts(&requests, &taken);

    robot_status_t const status = {
        .distance_mm = odometry.distance_mm,
        .speed_mm_s = odometry.speed_mm_s,
        .left_mm_s = odometry.left_speed_mm_s,
        .right_mm_s = odometry.right_speed_mm_s,
        .heading_deg = odometry.heading_deg,
        .pitch_ddeg = b_level ? terrain_monitor_data_pitch_ddeg(&terrain) : 0,
        .roll_ddeg = b_level ? terrain_monitor_data_roll_ddeg(&terrain) : 0,
        .turn_dps = terrain_monitor_data_turn_rate_dps(&terrain),
        .mode = (uint32_t)motion.mode,
        .obstacle = (uint32_t)robot_obstacle_get_status(obstacle.result.flags),
        .avoid = obstacle.avoid_state,
        .width_mm = obstacle.result.width_mm,
        .range_mm = obstacle.geometry.centre_mm,
        .calibration = (uint32_t)terrain_monitor_data_calib_state(&terrain),
        .humps = terrain_monitor_summary_hump_count(&summary),
        .collisions = terrain_monitor_summary_collisions(&summary),
        .requests = requests,
        .taken = taken};

    robot_status_pack(&status, p_words);
}
