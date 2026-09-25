/**
 * @file wifi_connection.c
 * @brief Handles Wi-Fi connection and automatic reconnection for Raspberry Pi Pico W.
 */

#include <stdio.h>
#include <stdbool.h>

#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"

#include "lwip/netif.h"
#include "lwip/ip4_addr.h"

#include "wifi_connection.h"

#define WIFI_SSID "Jerome"
#define WIFI_PASSWORD "ym123456"

#define WIFI_CONNECT_TIMEOUT_MS 10000
#define WIFI_RETRY_DELAY_MS 5000
#define WIFI_CHECK_INTERVAL_MS 2000

/**
 * @brief Attempts to connect to Wi-Fi once.
 *
 * @param ssid Wi-Fi SSID.
 * @param password Wi-Fi password.
 *
 * @return true if connection succeeds, false otherwise.
 */
static bool
wifi_connect_once (const char * ssid, const char * password)
{
    printf("Connecting to Wi-Fi: %s\n", ssid);

    int result = cyw43_arch_wifi_connect_timeout_ms(
        ssid, password, CYW43_AUTH_WPA2_AES_PSK, WIFI_CONNECT_TIMEOUT_MS);

    if (result != 0)
    {
        printf("Wi-Fi connection failed. Error code: %d\n", result);
        return false;
    }

    printf("Connected successfully!\n");

    return true;
}

/**
 * @brief Keeps attempting to connect until Wi-Fi connection succeeds.
 */
static void
wifi_connect_until_success (const char * ssid, const char * password)
{
    while (!wifi_connect_once(ssid, password))
    {
        printf("Retrying Wi-Fi connection in %d seconds...\n",
               WIFI_RETRY_DELAY_MS / 1000);

        sleep_ms(WIFI_RETRY_DELAY_MS);
    }
}

/**
 * @brief Checks whether the Pico W currently has a Wi-Fi connection.
 *
 * @return true if connected and IP address is available.
 */
static bool
wifi_is_connected (void)
{
    int status = cyw43_tcpip_link_status(&cyw43_state, CYW43_ITF_STA);

    return status == CYW43_LINK_UP;
}

/**
 * @brief Prints the current network configuration.
 */
void
wifi_print_network_info (void)
{
    cyw43_arch_lwip_begin(); // Prevent background network interrupt from colliding with the read

    struct netif * netif = netif_default;

    if (netif == NULL)
    {
        printf("Network interface unavailable\n");
        return;
    }

    printf("\nNetwork Config:\n");
    printf("IP Address: %s\n", ip4addr_ntoa(netif_ip4_addr(netif)));
    printf("Subnet Mask: %s\n", ip4addr_ntoa(netif_ip4_netmask(netif)));
    printf("Gateway: %s\n", ip4addr_ntoa(netif_ip4_gw(netif)));
    printf("\n");
    cyw43_arch_lwip_end(); // CRITICAL: Release lock
}

/**
 * @brief Deinitialises the Wi-Fi subsystem.
 */
void
wifi_deinit (void)
{
    cyw43_arch_deinit();
}

int
main (void)
{
    stdio_init_all();

    sleep_ms(2000);

    /*
	 * Initialise Wi-Fi hardware once.
	 */
    if (cyw43_arch_init())
    {
        printf("Failed to initialise Wi-Fi\n");
        return 1;
    }

    cyw43_arch_enable_sta_mode();

    /*
	 * Initial connection.
	 *
	 * If Wi-Fi is unavailable, this keeps retrying
	 * until the connection succeeds.
	 */
    wifi_connect_until_success(WIFI_SSID, WIFI_PASSWORD);

    wifi_print_network_info();
    printf("%d\n", wifi_is_connected());

    /*
	 * Continuously monitor the Wi-Fi connection.
	 */
    while (true)
    {
        if (!wifi_is_connected())
        {
            printf("\nWi-Fi connection lost!\n");

            /*
			 * Keep reconnecting until connection
			 * has been restored.
			 */
            wifi_connect_until_success(WIFI_SSID, WIFI_PASSWORD);

            printf("Wi-Fi connection restored!\n");

            wifi_print_network_info();
        }

        sleep_ms(WIFI_CHECK_INTERVAL_MS);
    }

    return 0;
}
