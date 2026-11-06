/**
 * @file car_status.c
 * @brief One console line of the car's state, for the dashboard.
 *
 * Kernel side: motion and obstacle headers cannot share a file with lwIP's.
 */

#include "car_status.h"
#include "../app.h"
#include "../motion/motion.h"
#include "../obstacle/obstacle.h"
#include "../obstacle/obstacle_avoid.h"
#include "../terrain/terrain_monitor.h"

/* Private types, constants, and macros. */

/* Private function declarations. */

/* Public function definitions. */

void
car_status_print (void)
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

    tm_printf(
        (UB *)"CAR v=%d l=%d r=%d d=%d h=%d m=%u os=%u ow=%u od=%u "
              "av=%s p=%d ro=%d tr=%d hu=%u co=%u\n",
        (INT)odometry.speed_mm_s, (INT)odometry.left_speed_mm_s,
        (INT)odometry.right_speed_mm_s, (INT)odometry.distance_mm,
        (INT)odometry.heading_deg, (UINT)motion.mode,
        (UINT)robot_obstacle_get_status(obstacle.result.flags),
        (UINT)obstacle.result.width_mm, (UINT)obstacle.geometry.centre_mm,
        obstacle_avoid_state_name((obstacle_avoid_state_t)obstacle.avoid_state),
        b_level ? (INT)terrain_monitor_data_pitch_ddeg(&terrain) : 0,
        b_level ? (INT)terrain_monitor_data_roll_ddeg(&terrain) : 0,
        (INT)terrain_monitor_data_turn_rate_dps(&terrain),
        (UINT)terrain_monitor_summary_hump_count(&summary),
        (UINT)terrain_monitor_summary_collisions(&summary));
}
