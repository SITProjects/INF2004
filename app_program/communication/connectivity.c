/**
 * @file connectivity.c
 * @brief Wi-Fi connection monitoring from the kernel radio service status.
 *
 * Reading the status with cyw43_utk_get_status() is safe from any task.
 * The CYW43 and lwIP headers are not included: their register and size_t
 * definitions clash with the kernel's, and no driver call is needed here.
 */
#include "connectivity.h"

#include <stdint.h>
#include <tk/tkernel.h>
#include <tm/tmonitor.h>
#if TM_WIFI_CYW43
#include "cyw43_utk.h"
#endif

/* Private types, constants, and macros. */

/* CYW43_LINK_UP in the cyw43-driver's cyw43.h: joined, with an address. */
#define LINK_UP 3

/* lwIP stores IPv4 addresses in network byte order, so on the
   little-endian RP2040 the first octet is the lowest byte. */
#define OCTET_BITS 8u
#define OCTET_MASK 0xFFu

typedef enum
{
    CONNECTIVITY_STOPPED,
    CONNECTIVITY_WAITING,
    CONNECTIVITY_CONNECTED
} connectivity_state_t;

/* Static data. */
static connectivity_state_t state = CONNECTIVITY_STOPPED;

/* Private function declarations. */
#if TM_WIFI_CYW43 && TM_NET_LWIP
static void print_network_info (uint32_t ip_addr);
#endif

/* Public function definitions. */

void
connectivity_init (void)
{
#if TM_WIFI_CYW43 && TM_NET_LWIP
    tm_printf((UB *)"Wi-Fi: the kernel radio service joins the network "
                    "from config/wifi_secrets.h\n");
    state = CONNECTIVITY_WAITING;
#elif TM_WIFI_CYW43
    tm_printf((UB *)"Wi-Fi: built with NET=none; the radio is on but does "
                    "not join a network\n");
#else
    tm_printf((UB *)"Wi-Fi: built with WIFI=none; the radio is off\n");
#endif
}

void
connectivity_update (void)
{
    if (state == CONNECTIVITY_STOPPED)
    {
        return;
    }

#if TM_WIFI_CYW43 && TM_NET_LWIP
    T_CYW43_UTK_STATUS status;
    cyw43_utk_get_status(&status);

    bool b_up = (status.link_status == LINK_UP);
    if (b_up && (state != CONNECTIVITY_CONNECTED))
    {
        tm_printf((UB *)"Wi-Fi connected\n");
        print_network_info(status.ip_addr);
        state = CONNECTIVITY_CONNECTED;
    }
    else if (!b_up && (state == CONNECTIVITY_CONNECTED))
    {
        tm_printf((UB *)"Wi-Fi unavailable; the radio service retries "
                        "automatically\n");
        state = CONNECTIVITY_WAITING;
    }
#endif
}

bool
connectivity_is_connected (void)
{
#if TM_WIFI_CYW43 && TM_NET_LWIP
    T_CYW43_UTK_STATUS status;
    cyw43_utk_get_status(&status);
    return (status.link_status == LINK_UP);
#else
    return false;
#endif
}

/* Private function definitions. */

#if TM_WIFI_CYW43 && TM_NET_LWIP
/* The service reports only the IP address, not the mask or gateway. */
static void
print_network_info (uint32_t ip_addr)
{
    tm_printf((UB *)"IP Address: %u.%u.%u.%u\n", (UINT)(ip_addr & OCTET_MASK),
              (UINT)((ip_addr >> OCTET_BITS) & OCTET_MASK),
              (UINT)((ip_addr >> (2u * OCTET_BITS)) & OCTET_MASK),
              (UINT)((ip_addr >> (3u * OCTET_BITS)) & OCTET_MASK));
}
#endif
