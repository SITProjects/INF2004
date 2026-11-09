/**
 * @file barcode_test.c
 * @brief On-target tests for the barcode subsystem. Built only with:
 *        make TEST=1
 *
 * Each TEST_CHECK prints a PASS/FAIL line on the console. Save the console
 * output to testing/barcode/logs/ and record the result in
 * testing/barcode/TEST_LOG.md. Test IDs: BAR-U01, BAR-U02, ...
 */

#include "barcode.h"
#include "barcode_code39.h"
#include "barcode_config.h"
#include "../test_report.h"

#if APP_TEST

/* Private types, constants, and macros. */
#define SAMPLES 10
#define SAMPLE_GAP_MS 97 /* Not a multiple of the poll interval. */
#define SELF_TEST_TEXT "*INF2004*"
#define SELF_TEST_CHARACTERS 9u
#define SELF_TEST_ELEMENTS (SELF_TEST_CHARACTERS * 10u - 1u)
#define NARROW_US 33000u
#define WIDE_US 82500u

/* Private function declarations. */
static BOOL decodes_both_ways (void);
static BOOL text_is (char const * p_text, char const * p_expected);
static BOOL pin_is_digital_input (UW pin);

/* Public function definitions. */

void
barcode_test (void)
{
    TEST_INIT();

    /* BAR-U01: the decoder reads a built-in symbol both ways round. */
    TEST_CHECK("BAR-U01", decodes_both_ways(),
               "Code 39 decodes INF2004 forwards and backwards");

    /* BAR-U02: tBarcode attached its interrupt and sleeps between polls. */
    INT waiting = 0;
    for (INT i = 0; i < SAMPLES; i++)
    {
        T_RTSK rtsk;
        (void)tk_dly_tsk(SAMPLE_GAP_MS);
        if ((tk_ref_tsk(barcode_tid_read, &rtsk) == E_OK) &&
            ((rtsk.tskstat & TTS_WAI) != 0u))
        {
            waiting++;
        }
    }
    TEST_CHECK("BAR-U02", waiting >= (SAMPLES - 1),
               "barcode task runs and sleeps between polls");

    /* BAR-U03: GP28 is a digital input, not an ADC pin or output. */
    TEST_CHECK("BAR-U03", pin_is_digital_input(BARCODE_PIN),
               "barcode DO pin is a digital input");

    TEST_SUMMARY("barcode");
}

/* Private function definitions. */

static BOOL
decodes_both_ways (void)
{
    static uint32_t widths[SELF_TEST_ELEMENTS];
    char const symbols[] = SELF_TEST_TEXT;
    uint32_t count = 0u;
    for (uint32_t c = 0u; c < SELF_TEST_CHARACTERS; c++)
    {
        uint32_t const pattern = barcode_code39_pattern(symbols[c]);
        if (c > 0u)
        {
            widths[count++] = NARROW_US;
        }
        for (uint32_t i = 0u; i < BARCODE_CODE39_ELEMENTS; i++)
        {
            widths[count++] =
                (((pattern >> (8u - i)) & 1u) != 0u) ? WIDE_US : NARROW_US;
        }
    }

    char text[BARCODE_MAX_TEXT];
    bool b_reversed = true;
    BOOL b_forward =
        (barcode_code39_decode(widths, count, text, sizeof(text),
                               &b_reversed) == BARCODE_CODE39_OK) &&
        !b_reversed && text_is(text, "INF2004");
    for (uint32_t i = 0u; i < count / 2u; i++)
    {
        uint32_t const swap = widths[i];
        widths[i] = widths[count - 1u - i];
        widths[count - 1u - i] = swap;
    }
    BOOL b_backward =
        (barcode_code39_decode(widths, count, text, sizeof(text),
                               &b_reversed) == BARCODE_CODE39_OK) &&
        b_reversed && text_is(text, "INF2004");
    return b_forward && b_backward;
}

static BOOL
text_is (char const * p_text, char const * p_expected)
{
    while ((*p_text != '\0') && (*p_text == *p_expected))
    {
        p_text++;
        p_expected++;
    }
    return *p_text == *p_expected;
}

/* Input enabled, output disabled, routed to SIO, not driven. */
static BOOL
pin_is_digital_input (UW pin)
{
    UW const pad = in_w(GPIO(pin));
    UW const funcsel = in_w(GPIO_CTRL(pin)) & 0x1Fu;
    return ((pad & GPIO_IE) != 0u) && ((pad & GPIO_OD) != 0u) &&
           (GPIO_CTRL_FUNCSEL_SIO == funcsel) &&
           ((in_w(GPIO_OE) & (1u << pin)) == 0u);
}

#endif /* APP_TEST */
