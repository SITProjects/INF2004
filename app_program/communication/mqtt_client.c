/**
 * @file mqtt_client.c
 * @brief MQTT telemetry outbox owned by the radio service.
 *
 * All functions and callbacks execute in the radio task. Reconnects and
 * telemetry retries use elapsed milliseconds rather than extra lwIP timers.
 */
#include "mqtt_client.h"

#if TM_MQTT
#include <stdint.h>
#include "telemetry_queue.h"
#include "lwip/apps/mqtt.h"
#include "lwip/ip_addr.h"
#include "lwip/sys.h"
#include "wifi_secrets.h"
#include <tm/tmonitor.h>

/* Private types, constants, and macros. */
#define RETRY_PERIOD_MS 3000u
#define KEEPALIVE_S 60u
#define TELEMETRY_TOPIC MQTT_TOPIC "/telemetry"
#define TELEMETRY_RETRY_MS 1000u
#define TELEMETRY_DRAIN_MS 100u

/* Static data. */
static mqtt_client_t * client;
static bool b_connecting;
static bool b_retry_wait;
static uint32_t retry_started_ms;
static bool b_telemetry_pending;
static bool b_telemetry_acked;
static uintptr_t telemetry_token;
static uint32_t telemetry_sequence;
static uint32_t telemetry_attempt_ms;
static uint32_t telemetry_wait_ms;

/* Private function declarations. */
static void connection_cb (mqtt_client_t * p_client, void * p_arg,
                           mqtt_connection_status_t status);
static void telemetry_service (uint32_t now_ms);
static void telemetry_request_cb (void * p_arg, err_t result);
static void telemetry_disconnected (void);
static void telemetry_complete (void);

/* Public function definitions. */
void
mqtt_client_service (bool b_network_up)
{
    uint32_t now_ms = sys_now();
    telemetry_complete();

    if (!b_network_up)
    {
        telemetry_disconnected();
        if ((client != NULL) &&
            (b_connecting || mqtt_client_is_connected(client)))
        {
            mqtt_disconnect(client);
            b_connecting = false;
            b_retry_wait = true;
            retry_started_ms = now_ms;
        }
        return;
    }

    if ((client != NULL) && mqtt_client_is_connected(client))
    {
        telemetry_service(now_ms);
        return;
    }

    if (b_connecting ||
        (b_retry_wait &&
         ((uint32_t)(now_ms - retry_started_ms) < RETRY_PERIOD_MS)))
    {
        return;
    }

    /* Also throttle allocation, configuration, and immediate connect errors. */
    b_retry_wait = true;
    retry_started_ms = now_ms;
    if (client == NULL)
    {
        client = mqtt_client_new();
        if (client == NULL)
        {
            tm_printf((UB *)"mqtt: client allocation failed\n");
            return;
        }
    }

    ip_addr_t broker_addr;
    if (!ip4addr_aton(MQTT_BROKER_IP, ip_2_ip4(&broker_addr)))
    {
        tm_printf((UB *)"mqtt: bad MQTT_BROKER_IP\n");
        return;
    }
    IP_SET_TYPE(&broker_addr, IPADDR_TYPE_V4);

    struct mqtt_connect_client_info_t info = {0};
    info.client_id = MQTT_CLIENT_ID;
    info.keep_alive = KEEPALIVE_S;
    b_connecting = true;
    err_t result = mqtt_client_connect(client, &broker_addr, MQTT_BROKER_PORT,
                                       connection_cb, NULL, &info);
    if (result != ERR_OK)
    {
        b_connecting = false;
        tm_printf((UB *)"mqtt: connect request failed, err=%d\n", (int)result);
    }
}

/* Private function definitions. */
static void
telemetry_complete (void)
{
    if (b_telemetry_acked && telemetry_queue_ack(telemetry_sequence))
    {
        tm_printf((UB *)"telemetry: ACK seq=%lu\n",
                  (unsigned long)telemetry_sequence);
        b_telemetry_acked = false;
        b_telemetry_pending = false;
    }
}

static void
telemetry_disconnected (void)
{
    /* lwIP discards outstanding requests on close without request callbacks.
     * An already received PUBACK still needs its deferred queue removal. */
    if (!b_telemetry_acked && b_telemetry_pending)
    {
        b_telemetry_pending = false;
        ++telemetry_token;
    }
    telemetry_wait_ms = 0u;
}

static void
telemetry_request_cb (void * p_arg, err_t result)
{
    if (!b_telemetry_pending || ((uintptr_t)p_arg != telemetry_token))
    {
        return;
    }
    if (result == ERR_OK)
    {
        /* Do not take the kernel mutex inside lwIP's callback. */
        b_telemetry_acked = true;
    }
    else
    {
        b_telemetry_pending = false;
        telemetry_attempt_ms = sys_now();
        telemetry_wait_ms = TELEMETRY_RETRY_MS;
        tm_printf((UB *)"telemetry: retry seq=%lu err=%d\n",
                  (unsigned long)telemetry_sequence, (int)result);
    }
}

static void
telemetry_service (uint32_t now_ms)
{
    if (b_telemetry_pending ||
        ((uint32_t)(now_ms - telemetry_attempt_ms) < telemetry_wait_ms))
    {
        return;
    }
    telemetry_record_t record;
    if (!telemetry_queue_peek(&record))
    {
        return;
    }
    UB payload[128];
    int len = tm_sprintf(payload, (const UB *)"seq=%lu captured_ms=%lu %s",
                         (unsigned long)record.sequence,
                         (unsigned long)record.captured_ms, record.text);
    telemetry_sequence = record.sequence;
    ++telemetry_token;
    /* Set before publish: even an immediate callback sees the current token. */
    b_telemetry_pending = true;
    telemetry_attempt_ms = now_ms;
    telemetry_wait_ms = TELEMETRY_DRAIN_MS;
    err_t result =
        mqtt_publish(client, TELEMETRY_TOPIC, payload, (uint16_t)len, 1, 0,
                     telemetry_request_cb, (void *)telemetry_token);
    if (result != ERR_OK)
    {
        b_telemetry_pending = false;
        telemetry_wait_ms = TELEMETRY_RETRY_MS;
        tm_printf((UB *)"telemetry: publish failed seq=%lu err=%d\n",
                  (unsigned long)record.sequence, (int)result);
    }
    else
    {
        tm_printf((UB *)"telemetry: TX [%s] \"%s\"\n", TELEMETRY_TOPIC,
                  payload);
    }
}

static void
connection_cb (mqtt_client_t * p_client, void * p_arg,
               mqtt_connection_status_t status)
{
    (void)p_client;
    (void)p_arg;
    b_connecting = false;
    if (status == MQTT_CONNECT_ACCEPTED)
    {
        b_retry_wait = false;
        tm_printf((UB *)"mqtt: connected to " MQTT_BROKER_IP "\n");
    }
    else
    {
        telemetry_disconnected();
        b_retry_wait = true;
        retry_started_ms = sys_now();
        tm_printf((UB *)"mqtt: connection status %d, retrying in 3s\n",
                  (int)status);
    }
}

#endif /* TM_MQTT */
