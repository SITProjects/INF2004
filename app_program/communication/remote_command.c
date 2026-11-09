/**
 * @file remote_command.c
 * @brief Commands from the dashboard: run state and barcode actions.
 *
 * An entry is one word, so a reader always sees a whole one: the text in
 * the low bytes, zero padded, and the action in the top byte. A zero word
 * is a free entry.
 */

#include "remote_command.h"
#include "../robot_types.h"
#include "../line_follow/line_follow_config.h"

/* Private types, constants, and macros. */
#define MAX_WORDS 3u

_Static_assert(REMOTE_TEXT_MAX == ROBOT_BARCODE_MAP_CHARS,
               "An entry's text must fit below its action");

/** @brief One word of a message. */
typedef struct
{
    char const * p_text;
    uint32_t length;
} word_t;

/* Static data. */

/* MQTT builds wait for the dashboard; others have nothing to start them. */
#if defined(TM_MQTT) && TM_MQTT
static volatile bool b_stopped = true;
#else
static volatile bool b_stopped = false;
#endif
static volatile bool b_reversing = false;
static volatile uint32_t table[REMOTE_MAP_SIZE];

/* Private function declarations. */
static bool find_entry (char const * p_text, uint32_t * p_entry);
static uint32_t char_code (char symbol);
static uint32_t split_words (char const * p_text, uint32_t length,
                             word_t * p_words);
static bool word_is (word_t const * p_word, char const * p_expected);
static bool text_key (char const * p_text, uint32_t length, uint32_t * p_key);
static bool turn_of (word_t const * p_word, line_turn_t * p_turn);
static remote_command_result_t map_barcode (word_t const * p_text,
                                            word_t const * p_action);

/* Public function definitions. */

remote_command_result_t
remote_command_handle (char const * p_text, uint32_t length)
{
    word_t words[MAX_WORDS] = {{0}};
    uint32_t const count = split_words(p_text, length, words);
    if ((1u == count) && word_is(&words[0], "start"))
    {
        b_stopped = false;
        b_reversing = false;
        return REMOTE_COMMAND_OK;
    }
    if ((1u == count) && word_is(&words[0], "stop"))
    {
        b_stopped = true;
        b_reversing = false;
        return REMOTE_COMMAND_OK;
    }
    if ((1u == count) && word_is(&words[0], "reverse"))
    {
        if (!b_stopped)
        {
            return REMOTE_COMMAND_BAD;
        }
        b_reversing = true;
        return REMOTE_COMMAND_OK;
    }
    if ((1u == count) && word_is(&words[0], "clear"))
    {
        for (uint32_t i = 0u; i < REMOTE_MAP_SIZE; i++)
        {
            table[i] = 0u;
        }
        return REMOTE_COMMAND_OK;
    }
    if ((MAX_WORDS == count) && word_is(&words[0], "map"))
    {
        return map_barcode(&words[1], &words[2]);
    }
    return REMOTE_COMMAND_BAD;
}

bool
remote_command_stopped (void)
{
    return b_stopped;
}

bool
remote_command_reversing (void)
{
    return b_reversing;
}

void
remote_command_reverse_done (void)
{
    b_reversing = false;
}

line_turn_t
remote_command_turn (char const * p_text)
{
    uint32_t entry = 0u;
    return find_entry(p_text, &entry)
               ? (line_turn_t)robot_field_get(entry,
                                              ROBOT_BARCODE_MAP_TURN_SHIFT,
                                              ROBOT_BARCODE_MAP_TURN_BITS)
               : LINE_JUNCTION_DEFAULT_TURN;
}

bool
remote_command_mapped (char const * p_text)
{
    uint32_t entry = 0u;
    return find_entry(p_text, &entry);
}

/* Private function definitions. */

/* A character's 6-bit code, 1 to 42; 0 if it is not a Code 39 character. */
static uint32_t
char_code (char symbol)
{
    static char const symbols[] = "0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ-.$/+%";
    for (uint32_t i = 0u; i < (sizeof(symbols) - 1u); i++)
    {
        if (symbols[i] == symbol)
        {
            return i + 1u;
        }
    }
    return 0u;
}

/* The table entry for a barcode; false if it has none. */
static bool
find_entry (char const * p_text, uint32_t * p_entry)
{
    uint32_t length = 0u;
    while ((length <= REMOTE_TEXT_MAX) && (p_text[length] != '\0'))
    {
        length++;
    }
    uint32_t key = 0u;
    if (text_key(p_text, length, &key))
    {
        for (uint32_t i = 0u; i < REMOTE_MAP_SIZE; i++)
        {
            uint32_t const entry = table[i];
            if ((entry & ROBOT_BARCODE_MAP_TEXT_MASK) == key)
            {
                *p_entry = entry;
                return true;
            }
        }
    }
    return false;
}

/* Split on spaces; returns the word count, one over if there are more. */
static uint32_t
split_words (char const * p_text, uint32_t length, word_t * p_words)
{
    uint32_t count = 0u;
    uint32_t i = 0u;
    while (i < length)
    {
        if (' ' == p_text[i])
        {
            i++;
            continue;
        }
        if (count >= MAX_WORDS)
        {
            return MAX_WORDS + 1u;
        }
        uint32_t const start = i;
        while ((i < length) && (p_text[i] != ' '))
        {
            i++;
        }
        p_words[count] = (word_t){&p_text[start], i - start};
        count++;
    }
    return count;
}

static bool
word_is (word_t const * p_word, char const * p_expected)
{
    uint32_t i = 0u;
    while ((i < p_word->length) && (p_expected[i] != '\0') &&
           (p_word->p_text[i] == p_expected[i]))
    {
        i++;
    }
    return (i == p_word->length) && ('\0' == p_expected[i]);
}

/* Pack barcode text into a table key; false if it cannot be one. */
static bool
text_key (char const * p_text, uint32_t length, uint32_t * p_key)
{
    if ((0u == length) || (length > REMOTE_TEXT_MAX))
    {
        return false;
    }
    uint32_t key = 0u;
    for (uint32_t i = 0u; i < length; i++)
    {
        uint32_t const code = char_code(p_text[i]);
        if (0u == code)
        {
            return false;
        }
        key |= code << (i * ROBOT_BARCODE_MAP_CHAR_BITS);
    }
    *p_key = key;
    return true;
}

static bool
turn_of (word_t const * p_word, line_turn_t * p_turn)
{
    static line_turn_t const turns[] = {LINE_TURN_STRAIGHT, LINE_TURN_LEFT,
                                        LINE_TURN_RIGHT, LINE_TURN_UTURN};
    for (uint32_t i = 0u; i < (sizeof(turns) / sizeof(turns[0])); i++)
    {
        if (word_is(p_word, line_track_turn_name(turns[i])))
        {
            *p_turn = turns[i];
            return true;
        }
    }
    return false;
}

/* Replace the barcode's entry, or take a free one. */
static remote_command_result_t
map_barcode (word_t const * p_text, word_t const * p_action)
{
    uint32_t key = 0u;
    line_turn_t turn = LINE_TURN_STRAIGHT;
    if (!text_key(p_text->p_text, p_text->length, &key) ||
        !turn_of(p_action, &turn))
    {
        return REMOTE_COMMAND_BAD;
    }
    uint32_t slot = REMOTE_MAP_SIZE;
    for (uint32_t i = 0u; i < REMOTE_MAP_SIZE; i++)
    {
        uint32_t const entry = table[i];
        if ((entry & ROBOT_BARCODE_MAP_TEXT_MASK) == key)
        {
            slot = i;
            break;
        }
        if ((0u == entry) && (REMOTE_MAP_SIZE == slot))
        {
            slot = i;
        }
    }
    if (REMOTE_MAP_SIZE == slot)
    {
        return REMOTE_COMMAND_FULL;
    }
    table[slot] = robot_field_put(key, ROBOT_BARCODE_MAP_TURN_SHIFT,
                                  ROBOT_BARCODE_MAP_TURN_BITS, (uint32_t)turn);
    return REMOTE_COMMAND_OK;
}
