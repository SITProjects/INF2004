/**
 * @file mqtt_client.c
 * @brief MQTT heartbeat and round-trip logging owned by the radio service.
 *
 * This translation unit uses lwIP headers, not tk/tkernel.h. All functions,
 * including callbacks, execute in the radio task. The service uses elapsed
 * milliseconds rather than extra lwIP timers, so reconnects cannot create
 * duplicate heartbeat timer chains.
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
#define HEARTBEAT_PERIOD_MS 5000u
#define RETRY_PERIOD_MS 3000u
#define KEEPALIVE_S 60u
#define RX_TOPIC_SIZE 64u
#define RX_DATA_SIZE 128u
#define TELEMETRY_TOPIC MQTT_TOPIC "/telemetry"
#define TELEMETRY_RETRY_MS 1000u
#define TELEMETRY_DRAIN_MS 100u

/* Static data. */
static mqtt_client_t * client;
static bool b_connecting;
static bool b_retry_wait;
static bool b_subscription_failed;
static uint32_t retry_started_ms;
static uint32_t heartbeat_started_ms;
static uint32_t heartbeat_count;
static char rx_topic[RX_TOPIC_SIZE];
static UB rx_data[RX_DATA_SIZE];
static uint16_t rx_len;
static bool b_rx_truncated;
static bool b_telemetry_pending;
static bool b_telemetry_acked;
static uintptr_t telemetry_token;
static uint32_t telemetry_sequence;
static uint32_t telemetry_attempt_ms;
static uint32_t telemetry_wait_ms;

/* Private function declarations. */
static void connection_cb (mqtt_client_t * p_client, void * p_arg,
                           mqtt_connection_status_t status);
static void request_cb (void * p_arg, err_t result);
static void subscribe_cb (void * p_arg, err_t result);
static void incoming_publish_cb (void * p_arg, const char * p_topic,
                                 uint32_t total_len);
static void incoming_data_cb (void * p_arg, const uint8_t * p_data,
                              uint16_t len, uint8_t flags);

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

    /* Defer cleanup until outside lwIP's receive/request callbacks. */
    if (b_subscription_failed)
    {
        telemetry_disconnected();
        mqtt_disconnect(client);
        b_subscription_failed = false;
        b_connecting = false;
        b_retry_wait = true;
        retry_started_ms = now_ms;
    }

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
        if ((uint32_t)(now_ms - heartbeat_started_ms) >= HEARTBEAT_PERIOD_MS)
        {
            UB payload[48];
            int len = tm_sprintf(payload, (const UB *)"hb %u",
                                 (unsigned int)heartbeat_count);
            err_t result = mqtt_publish(client, MQTT_TOPIC, payload,
                                        (uint16_t)len, 0, 0, request_cb, NULL);
            heartbeat_started_ms = now_ms;
            if (result == ERR_OK)
            {
                tm_printf((UB *)"mqtt: TX [" MQTT_TOPIC "] \"%s\"\n", payload);
                heartbeat_count++;
            }
            else
            {
                tm_printf((UB *)"mqtt: publish failed, err=%d\n", (int)result);
            }
        }
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
        mqtt_set_inpub_callback(client, incoming_publish_cb, incoming_data_cb,
                                NULL);
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
    (void)p_arg;
    b_connecting = false;
    if (status == MQTT_CONNECT_ACCEPTED)
    {
        b_retry_wait = false;
        heartbeat_started_ms = sys_now();
        tm_printf((UB *)"mqtt: connected to " MQTT_BROKER_IP "\n");
        err_t result =
            mqtt_subscribe(p_client, MQTT_TOPIC, 0, subscribe_cb, NULL);
        if (result != ERR_OK)
        {
            b_subscription_failed = true;
            tm_printf((UB *)"mqtt: subscribe request failed, err=%d\n",
                      (int)result);
        }
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

static void
subscribe_cb (void * p_arg, err_t result)
{
    (void)p_arg;
    if (result == ERR_OK)
    {
        tm_printf((UB *)"mqtt: subscribed to " MQTT_TOPIC "\n");
    }
    else
    {
        b_subscription_failed = true;
        tm_printf((UB *)"mqtt: subscribe failed, err=%d\n", (int)result);
    }
}

static void
request_cb (void * p_arg, err_t result)
{
    (void)p_arg;
    if (result != ERR_OK)
    {
        tm_printf((UB *)"mqtt: request failed, err=%d\n", (int)result);
    }
}

static void
incoming_publish_cb (void * p_arg, const char * p_topic, uint32_t total_len)
{
    (void)p_arg;
    uint32_t index = 0u;
    while ((index < (RX_TOPIC_SIZE - 1u)) && (p_topic[index] != '\0'))
    {
        rx_topic[index] = p_topic[index];
        index++;
    }
    rx_topic[index] = '\0';
    rx_len = 0u;
    b_rx_truncated = (p_topic[index] != '\0') || (total_len >= RX_DATA_SIZE);
}

static void
incoming_data_cb (void * p_arg, const uint8_t * p_data, uint16_t len,
                  uint8_t flags)
{
    (void)p_arg;
    for (uint16_t index = 0u; index < len; index++)
    {
        if (rx_len < (RX_DATA_SIZE - 1u))
        {
            UB value = p_data[index];
            rx_data[rx_len++] =
                ((value >= 0x20u) && (value < 0x7fu)) ? value : '.';
        }
        else
        {
            b_rx_truncated = true;
        }
    }
    if ((flags & MQTT_DATA_FLAG_LAST) != 0u)
    {
        rx_data[rx_len] = '\0';
        tm_printf((UB *)"mqtt: RX [%s] \"%s\"%s\n", rx_topic, rx_data,
                  b_rx_truncated ? " (truncated)" : "");
    }
}
#endif /* TM_MQTT */
