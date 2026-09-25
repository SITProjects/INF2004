/**
 * @file wifi_connection.c
 * @brief Handles Wi-Fi connection and automatic reconnection for Raspberry Pi Pico W.
 */

#include "wifi_connection.h"
#include "main.h"

#include <stdbool.h>
#include <stdio.h>

#include "pico/cyw43_arch.h"
#include "pico/stdlib.h"

#include "lwip/ip4_addr.h"
#include "lwip/netif.h"

#define WIFI_CONNECT_TIMEOUT_MS 10000
#define WIFI_RETRY_DELAY_MS 5000

static bool
wifi_connect ()
{
    printf("Connecting to Wi-Fi: %s\n", WIFI_SSID);

    int result = cyw43_arch_wifi_connect_timeout_ms(WIFI_SSID, WIFI_PASSWORD,
                                                    CYW43_AUTH_WPA2_AES_PSK,
                                                    WIFI_CONNECT_TIMEOUT_MS);

    if (result != 0)
    {
        printf("Wi-Fi connection failed. Error code: %d\n", result);
        return false;
    }

    printf("Connected successfully!\n");
    return true;
}

static void
wifi_connect_until_success (void)
{
    while (!wifi_connect(WIFI_SSID, WIFI_PASSWORD))
    {
        printf("Retrying Wi-Fi connection in %d seconds...\n",
               WIFI_RETRY_DELAY_MS / 1000);
        sleep_ms(WIFI_RETRY_DELAY_MS);
    }
}

static bool
wifi_is_connected (void)
{
    int status = cyw43_tcpip_link_status(&cyw43_state, CYW43_ITF_STA);
    return status == CYW43_LINK_UP;
}

bool
wifi_init (const char * ssid, const char * password)
{
    if (cyw43_arch_init())
    {
        printf("Failed to initialise Wi-Fi\n");
        return false;
    }

    cyw43_arch_enable_sta_mode();
    wifi_connect_until_success();
    wifi_print_network_info();

    return true;
}

void
wifi_monitor (void)
{
    if (!wifi_is_connected())
    {
        printf("Wi-Fi connection lost!\n");
        wifi_connect_until_success();
        printf("Wi-Fi connection restored!\n");
        wifi_print_network_info();
    }
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
