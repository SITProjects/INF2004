/* Check every minimum-bit layout in robot_types.h: min, max, one past. */
#include "../../../app_program/robot_types.h"
#include <stdio.h>

static unsigned int failures;

static void
check (char const * p_name, bool b_pass)
{
    printf("%s %s\n", b_pass ? "PASS" : "FAIL", p_name);
    failures += b_pass ? 0u : 1u;
}

static void
test_fields (void)
{
    uint32_t const others = 0xFFFFF00Fu;
    check("put min", robot_field_put(others, 4u, 8u, 0u) == others);
    check("put max", robot_field_get(robot_field_put(0u, 4u, 8u, 255u), 4u,
                                     8u) == 255u);
    check("put one past saturates",
          robot_field_get(robot_field_put(0u, 4u, 8u, 256u), 4u, 8u) == 255u);
    check("put leaves neighbours",
          (robot_field_put(others, 4u, 8u, 256u) & others) == others);
    check("put stays in its field",
          (robot_field_put(0u, 4u, 8u, 0xFFFFFFFFu) & others) == 0u);
    check("get ignores neighbours", robot_field_get(others, 4u, 8u) == 0u);

    check("signed min", robot_field_to_signed(robot_field_from_signed(-128, 8u),
                                              8u) == -128);
    check("signed max", robot_field_to_signed(robot_field_from_signed(127, 8u),
                                              8u) == 127);
    check("signed one below saturates",
          robot_field_to_signed(robot_field_from_signed(-129, 8u), 8u) == -128);
    check("signed one above saturates",
          robot_field_to_signed(robot_field_from_signed(128, 8u), 8u) == 127);
    check("signed minus one",
          robot_field_to_signed(robot_field_from_signed(-1, 8u), 8u) == -1);
    check("signed stays in its bits",
          (robot_field_from_signed(-1, 8u) & 0xFFFFFF00u) == 0u);

    uint64_t const wide = 0xFFFFFFFFFFFFFFFFu;
    check("64-bit field above bit 32",
          robot_field64_get(robot_field64_put(0u, 50u, 12u, 4095u), 50u, 12u) ==
              4095u);
    check("64-bit one past saturates",
          robot_field64_get(robot_field64_put(0u, 50u, 12u, 4096u), 50u, 12u) ==
              4095u);
    check("64-bit leaves neighbours",
          robot_field64_put(wide, 50u, 12u, 0u) ==
              (wide & ~((uint64_t)4095u << 50u)));
}

static void
test_line_request (void)
{
    uint32_t const flags = ROBOT_LINE_REQ_SEEN | ROBOT_LINE_REQ_JUNCTION |
                           ROBOT_LINE_REQ_SLOW | ROBOT_LINE_REQ_VALID;
    uint32_t const max = robot_line_request_pack(3u, flags, 255u, 32767);
    uint32_t const min = robot_line_request_pack(0u, 0u, 0u, -32768);
    uint32_t const past = robot_line_request_pack(4u, flags, 256u, 32768);
    uint32_t const below = robot_line_request_pack(0u, 0u, 0u, -32769);

    check("request max", (robot_line_request_get_mode(max) == 3u) &&
                             (robot_line_request_get_reverse(max) == 255u) &&
                             (robot_line_request_get_heading(max) == 32767) &&
                             ((max & flags) == flags));
    check("request min", (robot_line_request_get_mode(min) == 0u) &&
                             (robot_line_request_get_reverse(min) == 0u) &&
                             (robot_line_request_get_heading(min) == -32768) &&
                             ((min & flags) == 0u));
    check("request one past saturates",
          (robot_line_request_get_mode(past) == 3u) &&
              (robot_line_request_get_reverse(past) == 255u) &&
              (robot_line_request_get_heading(past) == 32767));
    check("request one below saturates",
          robot_line_request_get_heading(below) == -32768);
    check("request uses 30 bits", (max >> 30u) == 0u);

    uint32_t const just_reverse = robot_line_request_pack(0u, 0u, 170u, 0);
    check("reverse leaves neighbours",
          (robot_line_request_get_reverse(just_reverse) == 170u) &&
              (robot_line_request_get_mode(just_reverse) == 0u) &&
              ((just_reverse & flags) == 0u) &&
              (robot_line_request_get_heading(just_reverse) == 0));
    uint32_t const just_heading = robot_line_request_pack(0u, 0u, 0u, -90);
    check("heading leaves neighbours",
          (robot_line_request_get_mode(just_heading) == 0u) &&
              (robot_line_request_get_reverse(just_heading) == 0u) &&
              ((just_heading & flags) == 0u) &&
              (robot_line_request_get_heading(just_heading) == -90));
    check("flags stay out of other fields",
          (robot_line_request_get_mode(flags) == 0u) &&
              (robot_line_request_get_reverse(flags) == 0u));
}

static bool
odometry_same (robot_odometry_t a, robot_odometry_t b)
{
    return (a.updates == b.updates) && (a.distance_mm == b.distance_mm) &&
           (a.heading_deg == b.heading_deg) && (a.speed_mm_s == b.speed_mm_s) &&
           (a.left_speed_mm_s == b.left_speed_mm_s) &&
           (a.right_speed_mm_s == b.right_speed_mm_s);
}

static robot_odometry_t
odometry_round_trip (robot_odometry_t in)
{
    uint32_t words[ROBOT_ODOMETRY_WORDS];
    robot_odometry_pack(&in, words);
    return robot_odometry_unpack(words);
}

static void
test_odometry (void)
{
    robot_odometry_t const max = {255u, 8388607, 32767, 1023, 1023, 1023};
    robot_odometry_t const min = {1u, -8388608, -32768, -1024, -1024, -1024};
    robot_odometry_t const past = {255u, 8388608, 32768, 1024, 1024, 1024};
    robot_odometry_t const below = {1u, -8388609, -32769, -1025, -1025, -1025};
    check("odometry max", odometry_same(odometry_round_trip(max), max));
    check("odometry min", odometry_same(odometry_round_trip(min), min));
    check("odometry one past saturates",
          odometry_same(odometry_round_trip(past), max));
    check("odometry one below saturates",
          odometry_same(odometry_round_trip(below), min));

    robot_odometry_t in = {0u, 0, 0, 0, 0, 0};
    in.left_speed_mm_s = -620;
    robot_odometry_t out = odometry_round_trip(in);
    check("left speed leaves right speed",
          (out.left_speed_mm_s == -620) && (out.right_speed_mm_s == 0));
    in = (robot_odometry_t){0u, 0, 0, 0, 0, 0};
    in.right_speed_mm_s = 620;
    out = odometry_round_trip(in);
    check("right speed leaves left speed",
          (out.right_speed_mm_s == 620) && (out.left_speed_mm_s == 0));
    in = (robot_odometry_t){200u, 0, 0, 0, 0, 0};
    out = odometry_round_trip(in);
    check("count leaves distance", (out.updates == 200u) && (out.distance_mm == 0));
    in = (robot_odometry_t){0u, -1, 0, 0, 0, 0};
    out = odometry_round_trip(in);
    check("negative distance leaves count",
          (out.distance_mm == -1) && (out.updates == 0u));
    in = (robot_odometry_t){0u, 0, -1, 0, 0, 0};
    out = odometry_round_trip(in);
    check("negative heading leaves speed",
          (out.heading_deg == -1) && (out.speed_mm_s == 0));
}

static bool
geometry_same (robot_obstacle_geometry_t a, robot_obstacle_geometry_t b)
{
    return (a.centre_mm == b.centre_mm) && (a.left_x_mm == b.left_x_mm) &&
           (a.right_x_mm == b.right_x_mm) &&
           (a.b_left_clear == b.b_left_clear) &&
           (a.b_right_clear == b.b_right_clear) &&
           (a.left_end_mm == b.left_end_mm) &&
           (a.right_end_mm == b.right_end_mm);
}

static robot_obstacle_geometry_t
geometry_round_trip (robot_obstacle_geometry_t in)
{
    return robot_obstacle_geometry_unpack(robot_obstacle_geometry_pack(&in));
}

static void
test_geometry (void)
{
    robot_obstacle_geometry_t const max = {4095u, 4095,  4095, true,
                                           true,  4095u, 4095u};
    robot_obstacle_geometry_t const min = {0u,    -4096, -4096, false,
                                           false, 0u,    0u};
    robot_obstacle_geometry_t const past = {4096u, 4096,  4096, true,
                                            true,  4096u, 4096u};
    robot_obstacle_geometry_t const below = {0u,    -4097, -4097, false,
                                             false, 0u,    0u};
    check("geometry max", geometry_same(geometry_round_trip(max), max));
    check("geometry min", geometry_same(geometry_round_trip(min), min));
    check("geometry one past saturates",
          geometry_same(geometry_round_trip(past), max));
    check("geometry one below saturates",
          geometry_same(geometry_round_trip(below), min));
    check("geometry zero packs to zero",
          robot_obstacle_geometry_pack(&(robot_obstacle_geometry_t){0}) == 0u);

    robot_obstacle_geometry_t only = {0};
    only.left_x_mm = -1;
    check("left offset alone", geometry_same(geometry_round_trip(only), only));
    only = (robot_obstacle_geometry_t){0};
    only.right_x_mm = -1;
    check("right offset alone", geometry_same(geometry_round_trip(only), only));
    only = (robot_obstacle_geometry_t){0};
    only.left_end_mm = 4095u;
    check("left end alone", geometry_same(geometry_round_trip(only), only));
    only = (robot_obstacle_geometry_t){0};
    only.right_end_mm = 4095u;
    check("right end alone", geometry_same(geometry_round_trip(only), only));
    only = (robot_obstacle_geometry_t){0};
    only.b_left_clear = true;
    check("left clear alone", geometry_same(geometry_round_trip(only), only));
    only = (robot_obstacle_geometry_t){0};
    only.b_right_clear = true;
    check("right clear alone", geometry_same(geometry_round_trip(only), only));
}

static void
test_control_and_watch (void)
{
    uint16_t const control = robot_obstacle_control_pack(true, 15u, 15u);
    uint16_t const past = robot_obstacle_control_pack(false, 16u, 16u);
    check("control max", control == 0x01FFu);
    check("control one past saturates", past == 0x01FEu);
    check("control state alone", robot_obstacle_control_pack(false, 12u, 0u) ==
                                     (12u << ROBOT_OBSTACLE_CONTROL_STATE_SHIFT));
    check("control fault alone", robot_obstacle_control_pack(false, 0u, 8u) ==
                                     (8u << ROBOT_OBSTACLE_CONTROL_FAULT_SHIFT));

    robot_obstacle_watch_t const in = {255u, 32767u, true};
    robot_obstacle_watch_t out =
        robot_obstacle_watch_unpack(robot_obstacle_watch_pack(&in));
    check("watch max", (out.sequence == 255u) && (out.echo_us == 32767u) &&
                           out.b_active);
    robot_obstacle_watch_t const past_echo = {0u, 32768u, false};
    out = robot_obstacle_watch_unpack(robot_obstacle_watch_pack(&past_echo));
    check("watch echo one past saturates", (out.echo_us == 32767u) &&
                                               (out.sequence == 0u) &&
                                               !out.b_active);
    robot_obstacle_watch_t const zero = {0u, 0u, false};
    check("watch zero packs to zero", robot_obstacle_watch_pack(&zero) == 0u);
    robot_obstacle_watch_t const only_active = {0u, 0u, true};
    out = robot_obstacle_watch_unpack(robot_obstacle_watch_pack(&only_active));
    check("active leaves the sequence",
          out.b_active && (out.sequence == 0u) && (out.echo_us == 0u));
    robot_obstacle_watch_t const only_sequence = {255u, 0u, false};
    out = robot_obstacle_watch_unpack(robot_obstacle_watch_pack(&only_sequence));
    check("sequence leaves active",
          !out.b_active && (out.sequence == 255u) && (out.echo_us == 0u));
    check("watch uses 24 bits", (robot_obstacle_watch_pack(&in) >> 24u) == 0u);
}

static void
test_edge_word (void)
{
    uint32_t const before = 0xFFFFFFFEu;
    uint32_t const after = 0x00000004u;
    uint32_t const a = (before & ROBOT_BARCODE_EDGE_TIME_MASK) |
                       ROBOT_BARCODE_EDGE_BLACK;
    uint32_t const b = after & ROBOT_BARCODE_EDGE_TIME_MASK;
    check("edge level reads back", (a & ROBOT_BARCODE_EDGE_BLACK) != 0u);
    check("edge width across the clock wrap",
          (uint32_t)((b & ROBOT_BARCODE_EDGE_TIME_MASK) -
                     (a & ROBOT_BARCODE_EDGE_TIME_MASK)) == 6u);
}

int
main (void)
{
    test_fields();
    test_line_request();
    test_odometry();
    test_geometry();
    test_control_and_watch();
    test_edge_word();
    printf("%u failure(s)\n", failures);
    return (0u == failures) ? 0 : 1;
}
