/* Drive the real terrain_compass module with a stubbed magnetometer. */
#include "../../../app_program/terrain/terrain_compass.h"
#include "../../../app_program/terrain/drv_imu.h"
#include "../../../app_program/terrain/terrain_config.h"
#include "../../../app_program/terrain/terrain_platform.h"
#include <math.h>
#include <stdio.h>

static imu_mag_raw_t next;
static imu_status_t init_status = IMU_OK;
static imu_status_t read_status = IMU_OK;
static unsigned int init_calls;
static unsigned int failures;

imu_status_t
imu_mag_init (uint8_t addr, imu_mag_odr_t odr, imu_mag_gain_t gain)
{
    (void)addr;
    (void)odr;
    (void)gain;
    init_calls++;
    return init_status;
}

imu_status_t
imu_mag_read (imu_mag_raw_t * p_raw)
{
    *p_raw = next;
    return read_status;
}

uint32_t
terrain_platform_lock (void)
{
    return 0u;
}

void
terrain_platform_unlock (uint32_t saved)
{
    (void)saved;
}

static void
check (char const * p_name, bool ok)
{
    printf("%s %s\n", ok ? "PASS" : "FAIL", p_name);
    failures += ok ? 0u : 1u;
}

/* One update with the car turned @p deg counter-clockwise from north in a
 * 400-count horizontal field plus the configured hard-iron offset
 * (default axes: board = car). */
#define OFF_X ((int)TERRAIN_MAG_OFFSET_X)
#define OFF_Y ((int)TERRAIN_MAG_OFFSET_Y)

static terrain_compass_t
at (float deg)
{
    float const rad = deg * 3.14159265f / 180.0f;
    next.x = (int16_t)(OFF_X + lroundf(400.0f * sinf(rad)));
    next.y = (int16_t)(OFF_Y + lroundf(400.0f * cosf(rad)));
    next.z = -300;
    terrain_compass_update();
    terrain_compass_t reading;
    (void)terrain_compass_get(&reading);
    return reading;
}

static bool
near (int16_t ddeg, int expected)
{
    int diff = ((int)ddeg - expected + 5400) % 3600 - 1800;
    return (diff >= -3) && (diff <= 3);
}

int
main (void)
{
    terrain_compass_t reading;
    check("CMP01 nothing valid before set-up",
          !terrain_compass_get(&reading) && (0 == reading.samples));

    (void)terrain_compass_init();
    bool const b_north = near(at(0.0f).heading_ddeg, 0);
    bool const b_left = near(at(30.0f).heading_ddeg, 300);
    bool const b_right = near(at(-30.0f).heading_ddeg, 3300);
    bool const b_back = near(at(180.0f).heading_ddeg, 1800);
    reading = at(90.0f);
    check("CMP02 heading turns counter-clockwise (+ is left), 0 to 3599, "
          "offset removed",
          b_north && b_left && b_right && b_back &&
              near(reading.heading_ddeg, 900) && reading.b_valid &&
              (OFF_X + 400 == reading.x) && (-300 == reading.z));

    (void)at(270.0f);
    terrain_compass_range_t range;
    terrain_compass_get_range(&range);
    check("CMP03 the range spans the field seen: centre the offset, span 800",
          (OFF_X - 400 == range.min_x) && (OFF_X + 400 == range.max_x) &&
              (OFF_Y - 400 == range.min_y) && (OFF_Y + 400 == range.max_y));

    next.x = IMU_MAG_OVERFLOW;
    terrain_compass_update();
    bool const b_overflow = !terrain_compass_get(&reading);
    bool const b_back_ok = at(45.0f).b_valid;
    check("CMP04 an overflowed axis is no reading; the next good one is",
          b_overflow && b_back_ok);

    read_status = IMU_ERR_BUS;
    terrain_compass_update();
    bool const b_failed = !terrain_compass_get(&reading);
    read_status = IMU_OK;
    unsigned int const calls = init_calls;
    for (unsigned int i = 0u; i < 100u; i++)
    {
        terrain_compass_update();
    }
    bool const b_waited = (calls == init_calls);
    terrain_compass_update();
    bool const b_retried = (calls + 1u == init_calls);
    check("CMP05 a failed read is no reading; set-up is retried after 1 s",
          b_failed && b_waited && b_retried && at(10.0f).b_valid);

    printf("%u failure(s)\n", failures);
    return (0u == failures) ? 0 : 1;
}
