/* Check the tracker's packed state directly: the private helpers. */
#include "../../../app_program/line_follow/line_track.c"
#include <stdio.h>

static unsigned int failures;

static void
check (char const * p_name, bool b_pass)
{
    printf("%s %s\n", b_pass ? "PASS" : "FAIL", p_name);
    failures += b_pass ? 0u : 1u;
}

static int32_t
read_mark (int32_t now_mm, int16_t mark)
{
    line_track_input_t const in = {.distance_mm = now_mm};
    return mark_mm(&in, mark);
}

static void
test_marks (void)
{
    check("LM01 a mark reads back at zero", 0 == read_mark(0, robot_mark16(0)));
    check("LM02 a mark reads back below the 16-bit limit",
          32760 == read_mark(32761, robot_mark16(32760)));
    check("LM03 a mark reads back across the 16-bit limit",
          32760 == read_mark(32800, robot_mark16(32760)));
    check("LM04 and across the full wrap",
          65530 == read_mark(65600, robot_mark16(65530)));
    check("LM05 a negative travel reads back",
          -40 == read_mark(10, robot_mark16(-40)));
    check("LM06 a future end mark reads ahead across the limit",
          32900 == read_mark(32700, robot_mark16(32900)));

    int32_t now_mm = 0;
    int16_t mark = robot_mark16(now_mm);
    int32_t oldest = 0;
    for (int metres = 0; metres < 140; metres++)
    {
        now_mm += 1000;
        age_mark(&mark, now_mm);
        int32_t const age = now_mm - read_mark(now_mm, mark);
        oldest = (age > oldest) ? age : oldest;
    }
    check("LM07 an old mark is held at the oldest age and never reads recent",
          (MARK_OLDEST_MM == (now_mm - read_mark(now_mm, mark))) &&
              (MARK_OLDEST_MM == oldest));

    int16_t recent = robot_mark16(50000 - 20000);
    age_mark(&recent, 50000);
    check("LM08 a younger mark is not moved",
          30000 == read_mark(50000, recent));
    int16_t ahead = robot_mark16(50200);
    age_mark(&ahead, 50000);
    check("LM09 a future mark is not moved", 50200 == read_mark(50000, ahead));
}

static void
test_limits (void)
{
    check("LM10 heading max", INT16_MAX == robot_clamp_i16(INT16_MAX));
    check("LM11 heading min", INT16_MIN == robot_clamp_i16(INT16_MIN));
    check("LM12 heading one past saturates",
          INT16_MAX == robot_clamp_i16((int32_t)INT16_MAX + 1));
    check("LM13 heading one below saturates",
          INT16_MIN == robot_clamp_i16((int32_t)INT16_MIN - 1));

    uint8_t small = UINT8_MAX - 1u;
    count_up(&small);
    bool const b_top = (UINT8_MAX == small);
    count_up(&small);
    check("LM14 a byte count stops at its limit",
          b_top && (UINT8_MAX == small));
    check("LM15 a 16-bit count stops at its limit",
          (UINT16_MAX == count_up16(UINT16_MAX - 1u)) &&
              (UINT16_MAX == count_up16(UINT16_MAX)));
}

/* Every field reaches its limit, saturates and leaves the rest alone. */
static void
test_fields (void)
{
    static struct
    {
        uint32_t shift;
        uint32_t bits;
    } const fields[] = {
        {LINE_TRACK_FIELD_MODE},           {LINE_TRACK_FIELD_TOUCH_SIDE},
        {LINE_TRACK_FIELD_COUNTER_SIDE},   {LINE_TRACK_FIELD_ENTRY_SIDE},
        {LINE_TRACK_FIELD_PIVOT_SIDE},     {LINE_TRACK_FIELD_BARS_SIDE},
        {LINE_TRACK_FIELD_LAST_SIDE},      {LINE_TRACK_FIELD_SEARCH_SIDE},
        {LINE_TRACK_FIELD_JUNCTION_PHASE}, {LINE_TRACK_FIELD_JUNCTION_TURN},
        {LINE_TRACK_FIELD_SEARCH_PHASE},   {LINE_TRACK_FIELD_BARCODE_PHASE},
        {LINE_TRACK_FIELD_TREND_NEXT},     {LINE_TRACK_FIELD_JUNCTION_COUNT},
        {LINE_TRACK_CHANGE_SHIFT, LINE_TRACK_CHANGE_BITS},
        {LINE_TRACK_CHANGE_SHIFT + LINE_TRACK_CHANGE_BITS,
         LINE_TRACK_CHANGE_BITS}};
    uint32_t const count = sizeof(fields) / sizeof(fields[0]);
    bool b_disjoint = true;
    bool b_saturates = true;
    bool b_neighbours = true;
    line_track_t track = {0};

    for (uint32_t i = 0u; i < count; i++)
    {
        uint32_t const max = (1u << fields[i].bits) - 1u;
        track.fields = 0xFFFFFFFFu;
        field_set(&track, fields[i].shift, fields[i].bits, 0u);
        b_disjoint =
            b_disjoint && (0u == field_get(&track, fields[i].shift, fields[i].bits));
        for (uint32_t other = 0u; other < count; other++)
        {
            if (other != i)
            {
                b_neighbours =
                    b_neighbours &&
                    (((1u << fields[other].bits) - 1u) ==
                     field_get(&track, fields[other].shift, fields[other].bits));
            }
        }
        track.fields = 0u;
        field_set(&track, fields[i].shift, fields[i].bits, max + 1u);
        b_saturates = b_saturates &&
                      (max == field_get(&track, fields[i].shift, fields[i].bits));
    }
    check("LM16 a field clears on its own", b_disjoint);
    check("LM17 no field touches another", b_neighbours);
    check("LM18 one past a field's limit saturates", b_saturates);

    static uint32_t const flags[] = {
        LINE_TRACK_F_BLACK_LEFT,    LINE_TRACK_F_BLACK_RIGHT,
        LINE_TRACK_F_CURVE,         LINE_TRACK_F_CROSS_HOLD,
        LINE_TRACK_F_BOTH_SEEN,     LINE_TRACK_F_SKEW_SMALL,
        LINE_TRACK_F_OLD_LINE_SEEN, LINE_TRACK_F_REJOIN,
        LINE_TRACK_F_SEARCH_HIT,    LINE_TRACK_F_HUMP_SLOW,
        LINE_TRACK_F_HUMP_SPENT,    LINE_TRACK_F_BACK_STARTED,
        LINE_TRACK_F_RETRACE,       LINE_TRACK_F_SETTLE};
    uint32_t all = 0u;
    bool b_single = true;
    for (uint32_t i = 0u; i < (sizeof(flags) / sizeof(flags[0])); i++)
    {
        b_single = b_single && (0u == (all & flags[i])) &&
                   (0u == (flags[i] & (flags[i] - 1u)));
        all |= flags[i];
    }
    check("LM19 every flag is its own bit", b_single);

    track.flags = 0u;
    black_set(&track, SIDE_RIGHT, true);
    bool const b_right_only =
        is_black(&track, SIDE_RIGHT) && !is_black(&track, SIDE_LEFT);
    black_set(&track, SIDE_LEFT, true);
    black_set(&track, SIDE_RIGHT, false);
    check("LM20 the two sensors' levels are separate",
          b_right_only && is_black(&track, SIDE_LEFT) &&
              !is_black(&track, SIDE_RIGHT));

    track.fields = 0u;
    change_set(&track, SIDE_RIGHT, 3u);
    bool const b_change = (3u == change_get(&track, SIDE_RIGHT)) &&
                          (0u == change_get(&track, SIDE_LEFT));
    change_set(&track, SIDE_LEFT, 3u);
    change_set(&track, SIDE_RIGHT, 0u);
    check("LM21 the two sensors' change counts are separate",
          b_change && (3u == change_get(&track, SIDE_LEFT)) &&
              (0u == change_get(&track, SIDE_RIGHT)));
}

int
main (void)
{
    test_marks();
    test_limits();
    test_fields();
    printf("%u failure(s)\n", failures);
    return (0u == failures) ? 0 : 1;
}
