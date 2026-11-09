/**
 * @file console_mirror_mqtt.c
 * @brief Send the console mirror's lines to MQTT (lwIP half).
 *
 * Everything here runs in the radio task, including lwIP's sent callback,
 * so the batch state needs no lock.
 */

#include "console_mirror_mqtt.h"

#if TM_MQTT
#include <stdbool.h>
#include <stdint.h>
#include "console_mirror.h"
#include "lwip/apps/mqtt.h"
#include "wifi_secrets.h"

/* Private types, constants, and macros. */
#define CONSOLE_TOPIC MQTT_TOPIC "/console"
#define QOS_AT_MOST_ONCE 0u
#define NO_RETAIN 0u

/* Static data. */
static char batch[CONSOLE_MIRROR_BATCH_MAX];
static uint32_t batch_len;
static bool b_in_flight;
static bool b_sent;
static uintptr_t token;

/* Private function declarations. */
static void sent_cb (void * p_arg, err_t result);

/* Public function definitions. */

void
console_mirror_mqtt_service (struct mqtt_client_s * p_client)
{
    if ((NULL == p_client) || !mqtt_client_is_connected(p_client))
    {
        /* lwIP drops requests on close without callbacks: keep the batch
         * and send it again after the reconnect. */
        if (b_in_flight)
        {
            b_in_flight = false;
            ++token;
        }
        return;
    }
    if (b_in_flight)
    {
        if (!b_sent)
        {
            return;
        }
        b_in_flight = false;
        batch_len = 0u;
        console_mirror_mark_online();
    }
    if (0u == batch_len)
    {
        batch_len = console_mirror_take(batch);
        if (0u == batch_len)
        {
            return;
        }
    }

    ++token;
    b_sent = false;
    /* ERR_MEM while lwIP's output buffer is full: try on the next pass. */
    err_t const result =
        mqtt_publish(p_client, CONSOLE_TOPIC, batch, (u16_t)batch_len,
                     QOS_AT_MOST_ONCE, NO_RETAIN, sent_cb, (void *)token);
    b_in_flight = (ERR_OK == result);
}

/* Private function definitions. */

/*
 * lwIP core context (radio task): the batch has gone out over TCP, or lwIP
 * gave up waiting after MQTT_REQ_TIMEOUT of a stalled link. Its bytes are
 * still in lwIP's output buffer and go out when the link returns, so drop it
 * and move on: sending it again would repeat it, and waiting for a callback
 * that will not come stopped the console for good (2026-11-07).
 */
static void
sent_cb (void * p_arg, err_t result)
{
    if ((uintptr_t)p_arg == token)
    {
        if (ERR_OK == result)
        {
            b_sent = true;
        }
        else
        {
            b_in_flight = false;
            batch_len = 0u;
        }
    }
}
#endif /* TM_MQTT */
