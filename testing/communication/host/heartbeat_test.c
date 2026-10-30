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
    for (now = 1; now < 100; now++)
        mqtt_client_service(true);
    assert(connects == 1);
    connected = true;
    connection_cb(handle, NULL, MQTT_CONNECT_ACCEPTED);
    assert(subscriptions == 1);
    now = 5099;
    mqtt_client_service(true);
    assert(publishes == 0);
    now = 5100;
    mqtt_client_service(true);
    assert(publishes == 1 && strcmp(payload, "hb 0") == 0);
    mqtt_client_service(true);
    assert(publishes == 1);
    incoming_cb(NULL, MQTT_TOPIC, 4);
    data_cb(NULL, (const u8_t *)"hb", 2, 0);
    assert(strstr(log_text, "mqtt: RX") == NULL);
    data_cb(NULL, (const u8_t *)" 0", 2, MQTT_DATA_FLAG_LAST);
    assert(strstr(log_text, "\"hb 0\"") != NULL);
    u8_t big[200];
    memset(big, 'x', sizeof(big));
    incoming_cb(NULL, MQTT_TOPIC, sizeof(big));
    data_cb(NULL, big, sizeof(big), MQTT_DATA_FLAG_LAST);
    assert(strstr(log_text, "(truncated)") != NULL);
    publish_error = ERR_MEM;
    now = 10100;
    mqtt_client_service(true);
    assert(publishes == 2 && strstr(log_text, "publish failed") != NULL);
    mqtt_client_service(true);
    assert(publishes == 2);
    publish_error = ERR_OK;
    now = 15100;
    mqtt_client_service(true);
    assert(strcmp(payload, "hb 1") == 0);
    connected = false;
    connection_cb(handle, NULL, MQTT_CONNECT_DISCONNECTED);
    now = 18099;
    mqtt_client_service(true);
    assert(connects == 1);
    now = 18100;
    connect_error = ERR_CONN;
    mqtt_client_service(true);
    assert(connects == 2);
    now = 21099;
    mqtt_client_service(true);
    assert(connects == 2);
    now = 21100;
    connect_error = ERR_OK;
    mqtt_client_service(true);
    assert(connects == 3);
    mqtt_client_service(false);
    assert(disconnects == 1);
    now = 24100;
    mqtt_client_service(false);
    assert(connects == 3);
    mqtt_client_service(true);
    assert(connects == 4);
    now = 0xfffffff0U;
    connected = true;
    connection_cb(handle, NULL, MQTT_CONNECT_ACCEPTED);
    now = 4983;
    mqtt_client_service(true);
    assert(publishes == 3);
    now = 4984;
    mqtt_client_service(true);
    assert(publishes == 4);
    assert(strcmp(payload, "hb 2") == 0 && subscriptions == 2);
    connected = false;
    connection_cb(handle, NULL, MQTT_CONNECT_DISCONNECTED);
    now = 7984;
    mqtt_client_service(true);
    assert(connects == 5);
    subscribe_error = ERR_MEM;
    connected = true;
    connection_cb(handle, NULL, MQTT_CONNECT_ACCEPTED);
    mqtt_client_service(true);
    assert(disconnects == 2);
    now = 10983;
    mqtt_client_service(true);
    assert(connects == 5);
    now = 10984;
    mqtt_client_service(true);
    assert(connects == 6);
    subscribe_error = ERR_OK;
    subscribe_ack = ERR_TIMEOUT;
    connected = true;
    connection_cb(handle, NULL, MQTT_CONNECT_ACCEPTED);
    mqtt_client_service(true);
    assert(disconnects == 3);
    now = 13984;
    mqtt_client_service(true);
    assert(connects == 7);
    subscribe_ack = ERR_OK;
    connected = true;
    connection_cb(handle, NULL, MQTT_CONNECT_ACCEPTED);
    now = 18984;
    mqtt_client_service(true);
    assert(publishes == 5);
    size_t before_empty = log_used;
    incoming_cb(NULL, MQTT_TOPIC, 0);
    data_cb(NULL, NULL, 0, MQTT_DATA_FLAG_LAST);
    char * first = strstr(log_text + before_empty, "mqtt: RX");
    assert(first != NULL && strstr(first + 1, "mqtt: RX") == NULL);
    puts("PASS: heartbeat, retry/subscription recovery, Wi-Fi loss, RX bounds, "
         "wrap");
}
