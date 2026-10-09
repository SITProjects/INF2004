/**
 * @file connectivity.c
 * @brief Wi-Fi connection monitoring and retry scheduling.
 */
#include "connectivity.h"

#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include "pico/time.h"
#include "wifi.h"

/* Private types, constants, and macros. */
#define CONNECTIVITY_TIMEOUT_US UINT64_C(10000000)
#define CONNECTIVITY_RETRY_US UINT64_C(5000000)

typedef enum
{
    CONNECTIVITY_STOPPED,
    CONNECTIVITY_CONNECTING,
    CONNECTIVITY_WAITING,
    CONNECTIVITY_CONNECTED
} connectivity_state_t;

/* Static data. */
static connectivity_state_t state = CONNECTIVITY_STOPPED;
static const char * network_ssid;
static const char * network_password;
static uint64_t deadline_us;

/* Private function declarations. */
static void connectivity_start_attempt (void);

/* Public function definitions. */
int
connectivity_init (const char * ssid, const char * password)
{
    if ((state != CONNECTIVITY_STOPPED) || (ssid == NULL) || (password == NULL))
    {
        return 1;
    }
    if (!wifi_init())
    {
        return 1;
    }
    network_ssid = ssid;
    network_password = password;
    connectivity_start_attempt();
    return 0;
}

void
connectivity_update (void)
{
    if (state == CONNECTIVITY_STOPPED)
    {
        return;
    }
    if (wifi_is_connected())
    {
        if (state != CONNECTIVITY_CONNECTED)
        {
            printf("Wi-Fi connected\n");
            wifi_print_network_info();
        }
        state = CONNECTIVITY_CONNECTED;
        return;
    }

    uint64_t now_us = time_us_64();
    if ((state == CONNECTIVITY_CONNECTED) ||
        ((state == CONNECTIVITY_CONNECTING) && (now_us >= deadline_us)))
    {
        printf("Wi-Fi unavailable; retrying in 5 seconds\n");
        state = CONNECTIVITY_WAITING;
        deadline_us = now_us + CONNECTIVITY_RETRY_US;
    }
    else if ((state == CONNECTIVITY_WAITING) && (now_us >= deadline_us))
    {
        connectivity_start_attempt();
    }
}

bool
connectivity_is_connected (void)
{
    return (state != CONNECTIVITY_STOPPED) && wifi_is_connected();
}

/* Private function definitions. */
static void
connectivity_start_attempt (void)
{
    if (wifi_connect_start(network_ssid, network_password))
    {
        state = CONNECTIVITY_CONNECTING;
        deadline_us = time_us_64() + CONNECTIVITY_TIMEOUT_US;
    }
    else
    {
        state = CONNECTIVITY_WAITING;
        deadline_us = time_us_64() + CONNECTIVITY_RETRY_US;
    }
}
