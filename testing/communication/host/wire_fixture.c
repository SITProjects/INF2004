/* Print the bytes the firmware sends, for dashboard_test.py to decode. */
#include "telemetry_queue.h"
#include <stdio.h>

static void
show (char const * p_name, uint32_t const * p_words, unsigned int count)
{
    printf("%s ", p_name);
    for (unsigned int w = 0u; w < count; w++)
    {
        for (unsigned int b = 0u; b < 4u; b++)
        {
            printf("%02x", (unsigned int)((p_words[w] >> (b * 8u)) & 0xFFu));
        }
    }
    printf("\n");
}

static void
queued (char const * p_name, uint32_t ms, robot_telemetry_payload_t payload)
{
    uint32_t sequence = 0u;
    telemetry_record_t record;
    (void)telemetry_queue_push(ms, payload, &sequence);
    (void)telemetry_queue_peek(&record);
    (void)telemetry_queue_ack(sequence);
    show(p_name, record.words, ROBOT_TELEMETRY_WORDS);
}

static void
status (char const * p_name, robot_status_t const * p_status)
{
    uint32_t words[ROBOT_STATUS_WORDS];
    robot_status_pack(p_status, words);
    show(p_name, words, ROBOT_STATUS_WORDS);
}

int
main (void)
{
    (void)telemetry_queue_init();
    queued("obstacle", 1230u, robot_telemetry_obstacle(149u, 33u, 508u));
    queued("avoid", 1240u, robot_telemetry_avoid(7u, 12u));
    queued("line", 1250u, robot_telemetry_line(1u, false, 0u, -1234, -90, 32));
    queued("junction", 1260u, robot_telemetry_line(0u, true, 3u, 524287, 1, -1));
    queued("barcode_a", 689470u, robot_telemetry_barcode("A", true));
    queued("barcode_q", 689480u, robot_telemetry_barcode("Q", false));
    queued("barcode_long", 10u, robot_telemetry_barcode("ABCDE12345XY", false));
    queued("barcode_symbols", 20u, robot_telemetry_barcode("$/+%-.", true));

    robot_status_t basic = {.speed_mm_s = 80,
                            .left_mm_s = 70,
                            .right_mm_s = 90,
                            .distance_mm = 1234,
                            .heading_deg = -5,
                            .mode = 5u,
                            .obstacle = 2u,
                            .width_mm = 208u,
                            .range_mm = 159u,
                            .avoid = 0u,
                            .pitch_ddeg = 12,
                            .roll_ddeg = -3,
                            .turn_dps = 15,
                            .humps = 1u,
                            .collisions = 2u,
                            .requests = 300u,
                            .taken = 299u,
                            .calibration = 2u};
    status("status_basic", &basic);
    basic.width_mm = 0u;
    basic.range_mm = 0u;
    status("status_clear", &basic);
    robot_status_t const extremes = {.speed_mm_s = -1024,
                                     .left_mm_s = 1023,
                                     .right_mm_s = -1,
                                     .distance_mm = -8388608,
                                     .heading_deg = 32767,
                                     .mode = 7u,
                                     .obstacle = 7u,
                                     .width_mm = 511u,
                                     .range_mm = 4095u,
                                     .avoid = 12u,
                                     .pitch_ddeg = -2048,
                                     .roll_ddeg = 2047,
                                     .turn_dps = -1024,
                                     .humps = 255u,
                                     .collisions = 255u,
                                     .requests = 511u,
                                     .taken = 256u,
                                     .calibration = 7u};
    status("status_limits", &extremes);
    return 0;
}
