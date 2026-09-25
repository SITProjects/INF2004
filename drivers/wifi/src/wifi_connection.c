/**
 * @file wifi_connection.c
 * @brief Low-level Wi-Fi operations for Raspberry Pi Pico W.
 */
#include "wifi_connection.h"

#include <stddef.h>
#include <stdio.h>
#include "pico/cyw43_arch.h"
#include "lwip/ip4_addr.h"
#include "lwip/netif.h"

/* Public function definitions. */
bool
wifi_init (void)
{
    if (cyw43_arch_init() != 0)
    {
        return false;
    }
    cyw43_arch_enable_sta_mode();
    return true;
}

bool
wifi_connect_start (const char * ssid, const char * password)
{
    if ((ssid == NULL) || (password == NULL))
    {
        return false;
    }
    return cyw43_arch_wifi_connect_async(ssid, password,
                                         CYW43_AUTH_WPA2_AES_PSK) == 0;
}

bool
wifi_is_connected (void)
{
    cyw43_arch_lwip_begin();
    int status = cyw43_tcpip_link_status(&cyw43_state, CYW43_ITF_STA);
    cyw43_arch_lwip_end();
    return status == CYW43_LINK_UP;
}

void
wifi_print_network_info (void)
{
    cyw43_arch_lwip_begin();

    struct netif * netif = netif_default;

    if (netif == NULL)
    {
        printf("Network interface unavailable\n");
        cyw43_arch_lwip_end();
        return;
    }

    printf("\nNetwork Config:\n");
    printf("IP Address: %s\n", ip4addr_ntoa(netif_ip4_addr(netif)));
    printf("Subnet Mask: %s\n", ip4addr_ntoa(netif_ip4_netmask(netif)));
    printf("Gateway: %s\n", ip4addr_ntoa(netif_ip4_gw(netif)));

    cyw43_arch_lwip_end();
}
