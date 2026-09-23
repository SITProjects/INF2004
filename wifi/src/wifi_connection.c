/**
 * @file wifi_connection.c
 * @brief TODO: Describe the module responsibility.
 *
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

bool wifi_connect(const char *ssid, const char *password)
{
	if (cyw43_arch_init())
	{
		printf("Failed to initialise Wi-Fi\n");
		return false;
	}

	cyw43_arch_enable_sta_mode();

	printf("Connecting to Wi-Fi: %s\n", ssid);

	int result = cyw43_arch_wifi_connect_timeout_ms(
		ssid, password, CYW43_AUTH_WPA2_AES_PSK, 10000);

	if (result != 0)
	{
		printf("Connection failed. Error code: %d\n", result);
		cyw43_arch_deinit();
		return false;
	}

	// printf("Connected successfully!\n");
	while (true)
	{
		cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 1);
		sleep_ms(250);
		cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 0);
		sleep_ms(250);
	}
	return true;
}

void wifi_print_network_info(void)
{
	struct netif *netif = netif_default;

	if (netif == NULL)
	{
		printf("Network interface unavailable\n");
		return;
	}

	printf("Network config:\n");
	printf("IP Address: %s\n", ip4addr_ntoa(netif_ip4_addr(netif)));

	printf("Subnet Mask: %s\n", ip4addr_ntoa(netif_ip4_netmask(netif)));

	printf("Gateway: %s\n", ip4addr_ntoa(netif_ip4_gw(netif)));
}

void wifi_deinit(void)
{
	cyw43_arch_deinit();
}

int main()
{
	stdio_init_all();

	sleep_ms(2000);

	if (wifi_connect(WIFI_SSID, WIFI_PASSWORD))
	{
		wifi_print_network_info();
	}
	else
	{
		printf("Connection failed. Check credentials or signal.\n");
	}

	// Keeps the Pico Running
	while (true)
	{
		sleep_ms(1000);
	}

	return 0;
}
