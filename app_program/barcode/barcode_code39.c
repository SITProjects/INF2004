/**
 * @file barcode_code39.c
 * @brief Decode Code 39 from element widths, in either scan direction.
 *
 * Each character's 3 widest elements are wide. Widths are only compared
 * within one character, so a steady speed across a character is enough.
 * A reversed scan is the same element list backwards; '*' read backwards
 * is not '*', so the frame tells the direction.
 */

#include "barcode_code39.h"
#include <stddef.h>
#include "barcode_config.h"

/* Private types, constants, and macros. */
#define WIDE_BARS 2u
#define WIDE_SPACES 1u
#define WIDE_SPACES_ONLY 3u /* $ / + % have no wide bar. */
#define FIRST_BAR 0u
#define FIRST_SPACE 1u
#define COLOUR_STEP 2u
/* A character and the gap after it. */
#define CHARACTER_STRIDE (BARCODE_CODE39_ELEMENTS + 1u)
#define MIN_CHARACTERS 3u
#define FRAME '*'
/* A partial read: two characters and their gap, up to a whole "*X*" plus
 * a stray bar and space at each end. */
#define PARTIAL_MIN_ELEMENTS ((2u * CHARACTER_STRIDE) - 1u)
#define PARTIAL_MAX_ELEMENTS ((3u * CHARACTER_STRIDE) + 1u)

typedef struct
{
    char symbol;
    uint16_t pattern; /* Bit 8 is the first element; 1 is wide. */
} code39_entry_t;

/* Static data. */
static code39_entry_t const table[] = {
    {'0', 0x034u}, {'1', 0x121u}, {'2', 0x061u}, {'3', 0x160u}, {'4', 0x031u},
    {'5', 0x130u}, {'6', 0x070u}, {'7', 0x025u}, {'8', 0x124u}, {'9', 0x064u},
    {'A', 0x109u}, {'B', 0x049u}, {'C', 0x148u}, {'D', 0x019u}, {'E', 0x118u},
    {'F', 0x058u}, {'G', 0x00Du}, {'H', 0x10Cu}, {'I', 0x04Cu}, {'J', 0x01Cu},
    {'K', 0x103u}, {'L', 0x043u}, {'M', 0x142u}, {'N', 0x013u}, {'O', 0x112u},
    {'P', 0x052u}, {'Q', 0x007u}, {'R', 0x106u}, {'S', 0x046u}, {'T', 0x016u},
    {'U', 0x181u}, {'V', 0x0C1u}, {'W', 0x1C0u}, {'X', 0x091u}, {'Y', 0x190u},
    {'Z', 0x0D0u}, {'-', 0x085u}, {'.', 0x184u}, {' ', 0x0C4u}, {'*', 0x094u},
    {'$', 0x0A8u}, {'/', 0x0A2u}, {'+', 0x08Au}, {'%', 0x02Au}};

#define TABLE_SIZE (sizeof(table) / sizeof(table[0]))

/* One character of a scan, in reading order. */
typedef struct
{
    uint32_t const * p_widths;
    uint32_t count;
    uint32_t first;
    bool b_reverse;
} scan_view_t;

/* Private function declarations. */
static barcode_code39_status_t decode_direction (uint32_t const * p_widths,
                                                 uint32_t count, bool b_reverse,
                                                 char * p_text,
                                                 uint32_t text_size);
static barcode_code39_status_t read_character (uint32_t const * p_widths,
                                               uint32_t count, bool b_reverse,
                                               uint32_t first, char * p_symbol);
static char partial_symbol (uint32_t const * p_widths, uint32_t count,
                            bool b_reverse);
static bool mark_wide (scan_view_t const * p_view, uint32_t start,
                       uint32_t wide, uint32_t * p_pattern);
static bool bars_all_narrow (scan_view_t const * p_view);
static uint32_t element_bit (uint32_t index);
static uint32_t view_width (scan_view_t const * p_view, uint32_t index);
static uint32_t element (uint32_t const * p_widths, uint32_t count,
                         bool b_reverse, uint32_t index);

/* Public function definitions. */

barcode_code39_status_t
barcode_code39_decode (uint32_t const * p_widths, uint32_t count, char * p_text,
                       uint32_t text_size, bool * p_reversed)
{
    if ((NULL == p_widths) || (NULL == p_text) || (0u == text_size) ||
        (NULL == p_reversed))
    {
        return BARCODE_CODE39_ERR_SPACE;
    }
    p_text[0] = '\0';
    *p_reversed = false;
    barcode_code39_status_t const forward =
        decode_direction(p_widths, count, false, p_text, text_size);
    if ((BARCODE_CODE39_OK == forward) ||
        (BARCODE_CODE39_ERR_LENGTH == forward))
    {
        return forward;
    }
    barcode_code39_status_t const reverse =
        decode_direction(p_widths, count, true, p_text, text_size);
    if (BARCODE_CODE39_OK == reverse)
    {
        *p_reversed = true;
        return reverse;
    }
    p_text[0] = '\0';
    return (BARCODE_CODE39_ERR_FRAME == forward) ? reverse : forward;
}

uint32_t
barcode_code39_pattern (char symbol)
{
    for (uint32_t i = 0u; i < TABLE_SIZE; i++)
    {
        if (table[i].symbol == symbol)
        {
            return table[i].pattern;
        }
    }
    return 0u;
}

barcode_code39_status_t
barcode_code39_decode_partial (uint32_t const * p_widths, uint32_t count,
                               char * p_text, uint32_t text_size,
                               bool * p_reversed)
{
    if ((count < PARTIAL_MIN_ELEMENTS) || (count > PARTIAL_MAX_ELEMENTS))
    {
        return BARCODE_CODE39_ERR_LENGTH;
    }
    if (text_size < 2u)
    {
        return BARCODE_CODE39_ERR_SPACE;
    }
    char const forward = partial_symbol(p_widths, count, false);
    char const backward = partial_symbol(p_widths, count, true);
    if (('\0' == forward) == ('\0' == backward))
    {
        /* Neither, or a reading each way: no way to tell. */
        return BARCODE_CODE39_ERR_FRAME;
    }
    p_text[0] = ('\0' != forward) ? forward : backward;
    p_text[1] = '\0';
    *p_reversed = ('\0' == forward);
    return BARCODE_CODE39_OK;
}

char const *
barcode_code39_status_name (barcode_code39_status_t status)
{
    switch (status)
    {
        case BARCODE_CODE39_OK:
            return "ok";
        case BARCODE_CODE39_ERR_LENGTH:
            return "length";
        case BARCODE_CODE39_ERR_RATIO:
            return "ratio";
        case BARCODE_CODE39_ERR_PATTERN:
            return "pattern";
        case BARCODE_CODE39_ERR_FRAME:
            return "frame";
        case BARCODE_CODE39_ERR_SPACE:
            return "space";
        default:
            return "?";
    }
}

/* Private function definitions. */

static barcode_code39_status_t
decode_direction (uint32_t const * p_widths, uint32_t count, bool b_reverse,
                  char * p_text, uint32_t text_size)
{
    if (((count + 1u) % CHARACTER_STRIDE) != 0u)
    {
        return BARCODE_CODE39_ERR_LENGTH;
    }
    uint32_t const characters = (count + 1u) / CHARACTER_STRIDE;
    if (characters < MIN_CHARACTERS)
    {
        return BARCODE_CODE39_ERR_LENGTH;
    }
    if ((characters - 2u) >= text_size)
    {
        return BARCODE_CODE39_ERR_SPACE;
    }

    uint32_t length = 0u;
    for (uint32_t c = 0u; c < characters; c++)
    {
        char symbol = '\0';
        barcode_code39_status_t const status = read_character(
            p_widths, count, b_reverse, c * CHARACTER_STRIDE, &symbol);
        if (status != BARCODE_CODE39_OK)
        {
            p_text[0] = '\0';
            return status;
        }
        bool const b_end = (0u == c) || ((characters - 1u) == c);
        if (b_end != (FRAME == symbol))
        {
            p_text[0] = '\0';
            return BARCODE_CODE39_ERR_FRAME;
        }
        if (!b_end)
        {
            p_text[length] = symbol;
            length++;
        }
    }
    p_text[length] = '\0';
    return BARCODE_CODE39_OK;
}

/*
 * The character beside a clean '*' in this reading direction, or '\0' if
 * there is none or two disagree. Characters start on bars, so at even
 * offsets from the first bar.
 */
static char
partial_symbol (uint32_t const * p_widths, uint32_t count, bool b_reverse)
{
    char found = '\0';
    for (uint32_t first = 0u; (first + PARTIAL_MIN_ELEMENTS) <= count;
         first += COLOUR_STEP)
    {
        char left = '\0';
        char right = '\0';
        if ((BARCODE_CODE39_OK !=
             read_character(p_widths, count, b_reverse, first, &left)) ||
            (BARCODE_CODE39_OK != read_character(p_widths, count, b_reverse,
                                                 first + CHARACTER_STRIDE,
                                                 &right)) ||
            ((FRAME == left) == (FRAME == right)))
        {
            continue;
        }
        char const symbol = (FRAME == left) ? right : left;
        if (('\0' != found) && (found != symbol))
        {
            return '\0';
        }
        found = symbol;
    }
    return found;
}

/*
 * Classify the 9 elements from @p first. Bars are compared with bars and
 * spaces with spaces, because the sensor reads black thin and white wide
 * (2026-11-06: narrow 60 and 85, wide 200 and 240 tenths of a ms). A symbol
 * has 2 wide bars and 1 wide space, or, for $ / + %, 3 wide spaces.
 */
static barcode_code39_status_t
read_character (uint32_t const * p_widths, uint32_t count, bool b_reverse,
                uint32_t first, char * p_symbol)
{
    scan_view_t const view = {p_widths, count, first, b_reverse};
    uint32_t pattern = 0u;
    bool b_clear = mark_wide(&view, FIRST_BAR, WIDE_BARS, &pattern) &&
                   mark_wide(&view, FIRST_SPACE, WIDE_SPACES, &pattern);
    if (!b_clear)
    {
        pattern = 0u;
        b_clear = bars_all_narrow(&view) &&
                  mark_wide(&view, FIRST_SPACE, WIDE_SPACES_ONLY, &pattern);
    }
    if (!b_clear)
    {
        return BARCODE_CODE39_ERR_RATIO;
    }

    for (uint32_t i = 0u; i < TABLE_SIZE; i++)
    {
        if (table[i].pattern == pattern)
        {
            *p_symbol = table[i].symbol;
            return BARCODE_CODE39_OK;
        }
    }
    return BARCODE_CODE39_ERR_PATTERN;
}

/*
 * Set @p wide bits in @p p_pattern for the widest elements of one colour
 * (every second element from @p start). False if they do not stand clear
 * of the rest of that colour.
 */
static bool
mark_wide (scan_view_t const * p_view, uint32_t start, uint32_t wide,
           uint32_t * p_pattern)
{
    uint32_t narrowest_wide = UINT32_MAX;
    for (uint32_t pick = 0u; pick < wide; pick++)
    {
        uint32_t widest = 0u;
        uint32_t widest_index = start;
        for (uint32_t i = start; i < BARCODE_CODE39_ELEMENTS; i += COLOUR_STEP)
        {
            uint32_t const width = view_width(p_view, i);
            if (((*p_pattern & element_bit(i)) == 0u) && (width > widest))
            {
                widest = width;
                widest_index = i;
            }
        }
        *p_pattern |= element_bit(widest_index);
        narrowest_wide = widest;
    }

    uint32_t widest_narrow = 0u;
    for (uint32_t i = start; i < BARCODE_CODE39_ELEMENTS; i += COLOUR_STEP)
    {
        uint32_t const width = view_width(p_view, i);
        if (((*p_pattern & element_bit(i)) == 0u) && (width > widest_narrow))
        {
            widest_narrow = width;
        }
    }
    /* 64-bit products: widths in us cannot overflow the comparison. */
    return ((uint64_t)narrowest_wide * 10u) >=
           ((uint64_t)widest_narrow * BARCODE_WIDE_RATIO_X10);
}

/* True if no bar stands clear of the others. */
static bool
bars_all_narrow (scan_view_t const * p_view)
{
    uint32_t widest = 0u;
    uint32_t narrowest = UINT32_MAX;
    for (uint32_t i = FIRST_BAR; i < BARCODE_CODE39_ELEMENTS; i += COLOUR_STEP)
    {
        uint32_t const width = view_width(p_view, i);
        widest = (width > widest) ? width : widest;
        narrowest = (width < narrowest) ? width : narrowest;
    }
    return ((uint64_t)widest * 10u) <
           ((uint64_t)narrowest * BARCODE_WIDE_RATIO_X10);
}

static uint32_t
element_bit (uint32_t index)
{
    return 1u << (BARCODE_CODE39_ELEMENTS - 1u - index);
}

static uint32_t
view_width (scan_view_t const * p_view, uint32_t index)
{
    return element(p_view->p_widths, p_view->count, p_view->b_reverse,
                   p_view->first + index);
}

/* Width @p index of the scan, read backwards when @p b_reverse. */
static uint32_t
element (uint32_t const * p_widths, uint32_t count, bool b_reverse,
         uint32_t index)
{
    return b_reverse ? p_widths[count - 1u - index] : p_widths[index];
}
