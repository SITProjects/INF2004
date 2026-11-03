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
static u32_t now;
static unsigned connects, publishes, subscriptions, disconnects;
static bool connected;
static err_t connect_error, publish_error, subscribe_error, subscribe_ack;
static mqtt_connection_cb_t connection_cb;
static mqtt_incoming_publish_cb_t incoming_cb;
static mqtt_incoming_data_cb_t data_cb;
static mqtt_client_t * handle = (mqtt_client_t *)&connects;
static char payload[48], log_text[8192];
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
    assert(c == handle && strcmp(topic, MQTT_TOPIC) == 0);
    assert(qos == 0 && retain == 0 && len < sizeof(payload));
    memcpy(payload, data, len);
    payload[len] = 0;
    publishes++;
    if (publish_error == ERR_OK)
        cb(arg, ERR_OK);
    return publish_error;
}
int
main (void)
{
    mqtt_client_service(false);
    assert(connects == 0);
    mqtt_client_service(true);
    assert(connects == 1);
    connected = true;
    connection_cb(handle, NULL, MQTT_CONNECT_ACCEPTED);
    assert(subscriptions == 0);
    for (now = 0; now <= 15000; now += 1000)
    {
        mqtt_client_service(true);
    }
    assert(publishes == 0);
    connected = false;
    connection_cb(handle, NULL, MQTT_CONNECT_DISCONNECTED);
    now = 18999;
    mqtt_client_service(true);
    assert(connects == 1);
    now = 19000;
    connect_error = ERR_CONN;
    mqtt_client_service(true);
    assert(connects == 2);
    now = 21999;
    mqtt_client_service(true);
    assert(connects == 2);
    now = 22000;
    connect_error = ERR_OK;
    mqtt_client_service(true);
    assert(connects == 3);
    connected = true;
    connection_cb(handle, NULL, MQTT_CONNECT_ACCEPTED);
    mqtt_client_service(false);
    assert(disconnects == 1);
    now = 25000;
    mqtt_client_service(true);
    assert(connects == 4);
    connected = true;
    connection_cb(handle, NULL, MQTT_CONNECT_ACCEPTED);
    now = 35000;
    mqtt_client_service(true);
    assert(publishes == 0 && subscriptions == 0);
    puts("PASS: no heartbeat publish/subscription, reconnect retry, Wi-Fi loss");
}
