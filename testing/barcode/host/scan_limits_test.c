/* Check the barcode scan's packed state directly: the private helpers. */
#include "../../../app_program/barcode/barcode_scan.c"
#include <stdio.h>

static unsigned int failures;

static void
check (char const * p_name, bool b_pass)
{
    printf("%s %s\n", b_pass ? "PASS" : "FAIL", p_name);
    failures += b_pass ? 0u : 1u;
}

int
main (void)
{
    uint32_t const unit = BARCODE_WIDTH_UNIT_US;

    check("BW01 a zero width still counts as one step", 1u == width_units(0u));
    check("BW02 a width rounds to the nearest step",
          (1u == width_units(unit + (unit / 2u) - 1u)) &&
              (2u == width_units(unit + (unit / 2u))));
    check("BW03 the longest stored width",
          UINT16_MAX == width_units(UINT16_MAX * unit));
    check("BW04 one past saturates",
          UINT16_MAX == width_units((UINT16_MAX + 1u) * unit));

    barcode_scan_t scan = {0};
    scan_flag_set(&scan, BARCODE_SCAN_F_ACTIVE, true);
    scan_flag_set(&scan, BARCODE_SCAN_F_LONG_BLACK, true);
    bool const b_two = barcode_scan_flag(&scan, BARCODE_SCAN_F_ACTIVE) &&
                       barcode_scan_flag(&scan, BARCODE_SCAN_F_LONG_BLACK) &&
                       !barcode_scan_flag(&scan, BARCODE_SCAN_F_BLACK) &&
                       !barcode_scan_flag(&scan, BARCODE_SCAN_F_OVERFLOW);
    scan_flag_set(&scan, BARCODE_SCAN_F_ACTIVE, false);
    check("BW05 the scan's flags are separate",
          b_two && !barcode_scan_flag(&scan, BARCODE_SCAN_F_ACTIVE) &&
              barcode_scan_flag(&scan, BARCODE_SCAN_F_LONG_BLACK));

    barcode_trigger_t trigger = {0};
    trigger_field_set(&trigger, BARCODE_TRIGGER_NEXT, 3u);
    bool const b_next = (3u == trigger_field(&trigger, BARCODE_TRIGGER_NEXT)) &&
                        (0u == trigger_field(&trigger, BARCODE_TRIGGER_COUNT));
    trigger_field_set(&trigger, BARCODE_TRIGGER_COUNT, 3u);
    trigger_field_set(&trigger, BARCODE_TRIGGER_NEXT, 0u);
    bool const b_count = (0u == trigger_field(&trigger, BARCODE_TRIGGER_NEXT)) &&
                         (3u == trigger_field(&trigger, BARCODE_TRIGGER_COUNT));
    trigger_flag_set(&trigger, BARCODE_TRIGGER_F_HELD, true);
    bool const b_held = trigger_flag(&trigger, BARCODE_TRIGGER_F_HELD) &&
                        !trigger_flag(&trigger, BARCODE_TRIGGER_F_HUMP) &&
                        (3u == trigger_field(&trigger, BARCODE_TRIGGER_COUNT));
    trigger_field_set(&trigger, BARCODE_TRIGGER_COUNT, 4u);
    check("BW06 the trigger's fields and flags are separate; one past saturates",
          b_next && b_count && b_held &&
              (3u == trigger_field(&trigger, BARCODE_TRIGGER_COUNT)) &&
              trigger_flag(&trigger, BARCODE_TRIGGER_F_HELD));
    check("BW07 the edge history's index fits its field",
          (BARCODE_TRIGGER_EDGES - 1u) <= 3u);

    printf("%u failure(s)\n", failures);
    return (0u == failures) ? 0 : 1;
}
