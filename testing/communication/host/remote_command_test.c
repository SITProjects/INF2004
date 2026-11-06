/* Drive the real remote command module with dashboard messages. */
#include "../../../app_program/communication/remote_command.h"
#include <stdio.h>
#include <string.h>

static unsigned int failures;

static void
check (char const * p_name, bool ok)
{
    printf("%s %s\n", ok ? "PASS" : "FAIL", p_name);
    failures += ok ? 0u : 1u;
}

static remote_command_result_t
send (char const * p_text)
{
    return remote_command_handle(p_text, (uint32_t)strlen(p_text));
}

int
main (void)
{
    bool const b_boot = remote_command_stopped();
    bool const b_start = (REMOTE_COMMAND_OK == send("start")) &&
                         !remote_command_stopped();
    bool const b_stop = (REMOTE_COMMAND_OK == send("stop")) &&
                        remote_command_stopped();
    check("RC01 an MQTT build boots stopped; start and stop change it",
          b_boot && b_start && b_stop);

    bool const b_unmapped = (LINE_TURN_STRAIGHT == remote_command_turn("F"));
    bool const b_mapped = (REMOTE_COMMAND_OK == send("map F LEFT")) &&
                          (REMOTE_COMMAND_OK == send("map A1 UTURN")) &&
                          (LINE_TURN_LEFT == remote_command_turn("F")) &&
                          (LINE_TURN_UTURN == remote_command_turn("A1")) &&
                          (LINE_TURN_STRAIGHT == remote_command_turn("A")) &&
                          (LINE_TURN_STRAIGHT == remote_command_turn("FF"));
    check("RC02 a barcode takes its action; others, and near matches, go "
          "straight on",
          b_unmapped && b_mapped);

    bool const b_replaced = (REMOTE_COMMAND_OK == send("map F RIGHT")) &&
                            (LINE_TURN_RIGHT == remote_command_turn("F"));
    bool const b_straight = (REMOTE_COMMAND_OK == send("map F STRAIGHT")) &&
                            (LINE_TURN_STRAIGHT == remote_command_turn("F"));
    check("RC03 mapping a barcode again replaces it, STRAIGHT included",
          b_replaced && b_straight);

    bool const b_cleared = (REMOTE_COMMAND_OK == send("map F LEFT")) &&
                           (REMOTE_COMMAND_OK == send("clear")) &&
                           (LINE_TURN_STRAIGHT == remote_command_turn("F")) &&
                           (LINE_TURN_STRAIGHT == remote_command_turn("A1"));
    check("RC04 clear forgets every barcode", b_cleared);

    bool b_filled = true;
    char text[] = "map A LEFT";
    for (uint32_t i = 0u; i < REMOTE_MAP_SIZE; i++)
    {
        text[4] = (char)('A' + i);
        b_filled = b_filled && (REMOTE_COMMAND_OK == send(text));
    }
    bool const b_full = (REMOTE_COMMAND_FULL == send("map Z LEFT"));
    bool const b_update = (REMOTE_COMMAND_OK == send("map A RIGHT")) &&
                          (LINE_TURN_RIGHT == remote_command_turn("A"));
    check("RC05 a full table refuses a new barcode but updates a known one",
          b_filled && b_full && b_update &&
              (LINE_TURN_STRAIGHT == remote_command_turn("Z")));

    (void)send("clear");
    bool const b_refused =
        (REMOTE_COMMAND_BAD == send("")) &&
        (REMOTE_COMMAND_BAD == send("go")) &&
        (REMOTE_COMMAND_BAD == send("starts")) &&
        (REMOTE_COMMAND_BAD == send("star")) &&
        (REMOTE_COMMAND_BAD == send("map F")) &&
        (REMOTE_COMMAND_BAD == send("map F SIDEWAYS")) &&
        (REMOTE_COMMAND_BAD == send("map LONG LEFT")) &&
        (REMOTE_COMMAND_BAD == send("map F LEFT now"));
    check("RC06 anything else is refused and changes nothing",
          b_refused && remote_command_stopped() &&
              (LINE_TURN_STRAIGHT == remote_command_turn("F")));

    printf("%u failure(s)\n", failures);
    return (0u == failures) ? 0 : 1;
}
