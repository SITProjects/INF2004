/**
 * @file communication_test.c
 * @brief On-target tests for the communication subsystem. Built only with:
 *        make TEST=1
 *
 * Each TEST_CHECK prints a PASS/FAIL line on the console. Save the console
 * output to testing/communication/logs/ and record the result in
 * testing/communication/TEST_LOG.md. Test IDs: COM-U01, COM-U02, ...
 *
 * COM-U02 needs the network in config/wifi_secrets.h to be in range.
 */

#include "communication.h"
#include "../test_report.h"
#include "connectivity.h"
#if TM_WIFI_CYW43
#include "cyw43_utk.h"
#endif

#if APP_TEST

/* Private types, constants, and macros. */
#define RADIO_TIMEOUT_MS 10000
#define JOIN_TIMEOUT_MS 30000
#define POLL_MS 250

/* Public function definitions. */

void
communication_test (void)
{
    TEST_INIT();

#if TM_WIFI_CYW43
    /* COM-U01: the kernel's radio service started the CYW43 radio. */
    T_CYW43_UTK_STATUS status = {0};
    for (INT waited = 0; waited < RADIO_TIMEOUT_MS; waited += POLL_MS)
    {
        cyw43_utk_get_status(&status);
        if (status.ready != 0u)
        {
            break;
        }
        (void)tk_dly_tsk(POLL_MS);
    }
    TEST_CHECK("COM-U01", status.ready != 0u, "radio ready within 10 s");
#endif

#if TM_WIFI_CYW43 && TM_NET_LWIP
    /* COM-U02: the station joins the network and gets an IP address. */
    bool b_connected = false;
    for (INT waited = 0; waited < JOIN_TIMEOUT_MS; waited += POLL_MS)
    {
        b_connected = connectivity_is_connected();
        if (b_connected)
        {
            break;
        }
        (void)tk_dly_tsk(POLL_MS);
    }
    TEST_CHECK("COM-U02", b_connected, "joined Wi-Fi with an IP within 30 s");
#endif

    TEST_SUMMARY("communication");
}

#endif /* APP_TEST */
