/* Host-only checks of the real client with a simulated MQTT transport. */
#include <assert.h>
#include <stdbool.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include "lwip/apps/mqtt.h"
#include "lwip/sys.h"
#include "wifi_secrets.h"

#include "mqtt_client.h"
#include "telemetry_queue.h"
static u32_t now;
static unsigned connects, publishes, subscriptions, disconnects;
static bool connected;
static err_t connect_error, publish_error, subscribe_error, subscribe_ack;
static mqtt_connection_cb_t connection_cb;
static mqtt_incoming_publish_cb_t incoming_cb;
static mqtt_incoming_data_cb_t data_cb;
static mqtt_client_t * handle = (mqtt_client_t *)&connects;
static char payload[192], log_text[32768];
static mqtt_request_cb_t pending_cb;
static void * pending_arg;
static unsigned telemetry_publishes;
static bool locked;
static bool inline_ack;
int
telemetry_queue_port_init (void)
{
    return 0;
}
bool
telemetry_queue_port_lock (void)
{
    return !locked;
}
void
telemetry_queue_port_unlock (void)
{
}
static size_t log_used;
u32_t
sys_now (void)
{
    return now;
}
void
lwip_utk_assert (const char * s)
{
    fprintf(stderr, "%s\n", s);
    assert(0);
}
int
tm_printf (const unsigned char * fmt, ...)
{
    va_list args;
    va_start(args, fmt);
    int n = vsnprintf(log_text + log_used, sizeof(log_text) - log_used,
                      (const char *)fmt, args);
    va_end(args);
    assert(n >= 0);
    log_used += (size_t)n;
    assert(log_used < sizeof(log_text));
    return n;
}
int
tm_sprintf (unsigned char * buf, const unsigned char * fmt, ...)
{
    va_list args;
    va_start(args, fmt);
    int n = vsprintf((char *)buf, (const char *)fmt, args);
    va_end(args);
    return n;
}
int
ip4addr_aton (const char * s, ip4_addr_t * addr)
{
    assert(strcmp(s, MQTT_BROKER_IP) == 0);
    addr->addr = 1;
    return 1;
}
mqtt_client_t *
mqtt_client_new (void)
{
    return handle;
}
u8_t
mqtt_client_is_connected (mqtt_client_t * c)
{
    assert(c == handle);
    return connected;
}
void
mqtt_disconnect (mqtt_client_t * c)
{
    assert(c == handle);
    disconnects++;
    connected = false;
}
err_t
mqtt_client_connect (mqtt_client_t * c, const ip_addr_t * a, u16_t port,
                     mqtt_connection_cb_t cb, void * arg,
                     const struct mqtt_connect_client_info_t * info)
{
    assert(c == handle && a != NULL && arg == NULL);
    assert(port == MQTT_BROKER_PORT);
    assert(strcmp(info->client_id, MQTT_CLIENT_ID) == 0);
    assert(info->keep_alive == 60);
    connects++;
    connection_cb = cb;
    return connect_error;
}
void
mqtt_set_inpub_callback (mqtt_client_t * c, mqtt_incoming_publish_cb_t pub,
                         mqtt_incoming_data_cb_t data, void * arg)
{
    assert(c == handle && arg == NULL);
    incoming_cb = pub;
    data_cb = data;
}
err_t
mqtt_sub_unsub (mqtt_client_t * c, const char * topic, u8_t qos,
                mqtt_request_cb_t cb, void * arg, u8_t sub)
{
    assert(c == handle && strcmp(topic, MQTT_TOPIC) == 0);
    assert(qos == 0 && sub == 1);
    subscriptions++;
    if (subscribe_error == ERR_OK)
        cb(arg, subscribe_ack);
    return subscribe_error;
}
err_t
mqtt_publish (mqtt_client_t * c, const char * topic, const void * data,
              u16_t len, u8_t qos, u8_t retain, mqtt_request_cb_t cb,
              void * arg)
{
    assert(c == handle && retain == 0 && len < sizeof(payload));
    assert(strcmp(topic, MQTT_TOPIC "/telemetry") == 0);
    memcpy(payload, data, len);
    payload[len] = 0;
    publishes++;
    if (strcmp(topic, MQTT_TOPIC "/telemetry") == 0)
    {
        assert(qos == 1);
        telemetry_publishes++;
        pending_cb = cb;
        pending_arg = arg;
        if (inline_ack && publish_error == ERR_OK)
            cb(arg, ERR_OK);
    }
    else
    {
        assert(strcmp(topic, MQTT_TOPIC) == 0 && qos == 0);
        if (publish_error == ERR_OK)
            cb(arg, ERR_OK);
    }
    return publish_error;
}

int
main (void)
{
    telemetry_record_t head;
    uint32_t seq;
    assert(telemetry_queue_init() == 0);
    assert(!telemetry_queue_peek(&head));
    assert(!telemetry_queue_peek(NULL));
    assert(!telemetry_queue_ack(0));
    assert(telemetry_queue_push(1, "", &seq) == TELEMETRY_INVALID);
    assert(telemetry_queue_push(1, NULL, &seq) == TELEMETRY_INVALID);
    char oversized[TELEMETRY_TEXT_SIZE + 1];
    memset(oversized, 'x', sizeof(oversized));
    oversized[sizeof(oversized) - 1] = 0;
    assert(telemetry_queue_push(1, oversized, &seq) == TELEMETRY_INVALID);
    locked = true;
    assert(telemetry_queue_push(1, "busy", &seq) == TELEMETRY_BUSY);
    locked = false;
    for (unsigned i = 0; i < TELEMETRY_QUEUE_CAPACITY; ++i)
    {
        assert(telemetry_queue_push(i, "sample", &seq) == TELEMETRY_QUEUED);
        assert(seq == i);
    }
    assert(telemetry_queue_push(999, "full", &seq) == TELEMETRY_FULL);
    assert(telemetry_queue_peek(&head) && head.sequence == 0);
    assert(!telemetry_queue_ack(99));
    for (unsigned i = 0; i < TELEMETRY_QUEUE_CAPACITY; ++i)
    {
        assert(telemetry_queue_peek(&head) && head.sequence == i);
        assert(head.captured_ms == i && strcmp(head.text, "sample") == 0);
        assert(telemetry_queue_ack(i));
    }
    assert(!telemetry_queue_peek(&head));
    assert(telemetry_queue_push(42, "offline-A", &seq) == TELEMETRY_QUEUED);
    assert(seq == TELEMETRY_QUEUE_CAPACITY + 1);
    uint32_t first_seq = seq;
    assert(telemetry_queue_push(43, "offline-B", &seq) == TELEMETRY_QUEUED);
    mqtt_client_service(false);
    assert(publishes == 0);
    mqtt_client_service(true);
    connected = true;
    connection_cb(handle, NULL, MQTT_CONNECT_ACCEPTED);
    mqtt_client_service(true);
    assert(telemetry_publishes == 1);
    assert(strstr(payload, "offline-A") != NULL);
    assert(strstr(payload, "captured_ms=42") != NULL);
    assert(telemetry_queue_peek(&head) && head.sequence == first_seq);
    mqtt_client_service(true);
    assert(telemetry_publishes == 1); /* no publish before PUBACK */
    mqtt_request_cb_t stale_cb = pending_cb;
    void * stale_arg = pending_arg;
    mqtt_client_service(false); /* disconnect with ACK lost */
    assert(telemetry_queue_peek(&head) && head.sequence == first_seq);
    now = 3000;
    mqtt_client_service(true);
    connected = true;
    connection_cb(handle, NULL, MQTT_CONNECT_ACCEPTED);
    mqtt_client_service(true);
    assert(telemetry_publishes == 2 && strstr(payload, "offline-A"));
    stale_cb(stale_arg, ERR_OK); /* old attempt cannot remove current head */
    mqtt_client_service(true);
    assert(telemetry_queue_peek(&head) && head.sequence == first_seq);
    locked = true;
    pending_cb(pending_arg, ERR_OK);
    mqtt_client_service(true); /* ACK waits for queue lock */
    assert(telemetry_publishes == 2);
    locked = false;
    now += 100;
    mqtt_client_service(true);
    assert(telemetry_queue_peek(&head) && head.sequence == first_seq + 1);
    assert(telemetry_publishes == 3 && strstr(payload, "offline-B"));
    pending_cb(pending_arg, ERR_TIMEOUT);
    now += 99;
    mqtt_client_service(true);
    assert(telemetry_publishes == 3);
    now += 901;
    publish_error = ERR_MEM;
    mqtt_client_service(true);
    assert(telemetry_publishes == 4);
    assert(telemetry_queue_peek(&head) && head.sequence == first_seq + 1);
    publish_error = ERR_OK;
    now += 1000;
    mqtt_client_service(true);
    assert(telemetry_publishes == 5 && strstr(payload, "offline-B"));
    pending_cb(pending_arg, ERR_OK);
    mqtt_client_service(true);
    assert(!telemetry_queue_peek(&head));
    /* New data after drain is not stuck behind an old pending request. */
    assert(telemetry_queue_push(44, "online-C", &seq) == TELEMETRY_QUEUED);
    now += 100;
    mqtt_client_service(true);
    assert(telemetry_publishes == 6 && strstr(payload, "online-C"));
    pending_cb(pending_arg, ERR_OK);
    mqtt_client_service(true);
    assert(!telemetry_queue_peek(&head));
    /* A broker-triggered disconnect also drops lwIP's request callbacks. */
    assert(telemetry_queue_push(45, "broker-outage", &seq) == TELEMETRY_QUEUED);
    now += 100;
    mqtt_client_service(true);
    unsigned before = telemetry_publishes;
    connected = false;
    connection_cb(handle, NULL, MQTT_CONNECT_DISCONNECTED);
    now += 3000;
    mqtt_client_service(true);
    connected = true;
    connection_cb(handle, NULL, MQTT_CONNECT_ACCEPTED);
    mqtt_client_service(true);
    assert(telemetry_publishes == before + 1);
    assert(telemetry_queue_peek(&head) && head.sequence == seq);
    /* Received PUBACK survives Wi-Fi loss even if removal was contended. */
    pending_cb(pending_arg, ERR_OK);
    locked = true;
    mqtt_client_service(false);
    locked = false;
    mqtt_client_service(false);
    assert(!telemetry_queue_peek(&head));
    now += 3000;
    mqtt_client_service(true);
    connected = true;
    connection_cb(handle, NULL, MQTT_CONNECT_ACCEPTED);
    /* The publish callback may complete before mqtt_publish returns. */
    assert(telemetry_queue_push(46, "inline-ack", &seq) == TELEMETRY_QUEUED);
    inline_ack = true;
    mqtt_client_service(true);
    mqtt_client_service(true);
    assert(!telemetry_queue_peek(&head));
    inline_ack = false;
    /* Retry elapsed-time arithmetic must tolerate 32-bit clock rollover. */
    assert(telemetry_queue_push(47, "clock-wrap", &seq) == TELEMETRY_QUEUED);
    now = 0xfffffff0U;
    mqtt_client_service(true);
    before = telemetry_publishes;
    pending_cb(pending_arg, ERR_TIMEOUT);
    now = 983;
    mqtt_client_service(true);
    assert(telemetry_publishes == before);
    now = 984;
    mqtt_client_service(true);
    assert(telemetry_publishes == before + 1);
    pending_cb(pending_arg, ERR_OK);
    mqtt_client_service(true);
    assert(!telemetry_queue_peek(&head));
    puts("PASS: FIFO capacity/wrap, validation, contention, offline replay, "
         "lost ACK, stale callback, asynchronous/inline ACK, timeout, "
         "publish error, broker loss, deferred ACK/disconnect, clock wrap");
}
