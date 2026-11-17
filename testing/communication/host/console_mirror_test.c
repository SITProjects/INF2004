/* Drive the real console_mirror_mqtt.c against a stand-in for lwIP's client. */
#include "../../../app_program/console_mirror/console_mirror_mqtt.h"
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "../../../app_program/console_mirror/console_mirror.h"
#include "lwip/apps/mqtt.h"

static char client_storage;
static mqtt_client_t * const p_client = (mqtt_client_t *)(void *)&client_storage;
static bool b_connected = true;
static unsigned int next_batch;  /* Number of the next batch to take. */
static unsigned int batches_left;
static unsigned int publishes;
static char published[8];        /* The latest batch handed to lwIP. */
static mqtt_request_cb_t p_cb;
static void * p_cb_arg;
static err_t publish_result = ERR_OK;
static bool b_online;
static unsigned int failures;

uint32_t
console_mirror_take (char * p_out)
{
    if (0u == batches_left)
    {
        return 0u;
    }
    batches_left--;
    next_batch++;
    return (uint32_t)sprintf(p_out, "b%u", next_batch);
}

void
console_mirror_mark_online (void)
{
    b_online = true;
}

u8_t
mqtt_client_is_connected (mqtt_client_t * p_in)
{
    return ((p_in == p_client) && b_connected) ? 1u : 0u;
}

err_t
mqtt_publish (mqtt_client_t * p_in, char const * p_topic, void const * p_data,
              u16_t length, u8_t qos, u8_t retain, mqtt_request_cb_t p_done,
              void * p_arg)
{
    (void)p_in;
    (void)p_topic;
    (void)qos;
    (void)retain;
    if (publish_result != ERR_OK)
    {
        return publish_result;
    }
    publishes++;
    memcpy(published, p_data, length);
    published[length] = '\0';
    p_cb = p_done;
    p_cb_arg = p_arg;
    return ERR_OK;
}

static void
check (char const * p_name, bool ok)
{
    printf("%s %s\n", ok ? "PASS" : "FAIL", p_name);
    failures += ok ? 0u : 1u;
}

static void
service (void)
{
    console_mirror_mqtt_service(p_client);
}

int
main (void)
{
    batches_left = 8u;

    service();
    check("CM01 a batch goes out and the next waits for it",
          (1u == publishes) && (0 == strcmp(published, "b1")));
    service();
    check("CM02 nothing more is sent while it is in flight", 1u == publishes);

    p_cb(p_cb_arg, ERR_OK);
    service();
    check("CM03 once sent, the next batch goes and the broker counts as online",
          (2u == publishes) && (0 == strcmp(published, "b2")) && b_online);

    /* The link stalls past MQTT_REQ_TIMEOUT: lwIP gives up on the request. */
    b_online = false;
    p_cb(p_cb_arg, ERR_TIMEOUT);
    service();
    check("CM04 a timed-out batch does not stop the console",
          3u == publishes);
    check("CM05 it is not sent twice: its bytes are still queued in lwIP",
          0 == strcmp(published, "b3"));
    check("CM06 a timeout is no proof the broker has it", !b_online);

    /* A callback for a batch that is over must not finish the current one. */
    void * const p_stale = (void *)((uintptr_t)p_cb_arg - 1u);
    p_cb(p_stale, ERR_OK);
    service();
    check("CM07 a stale callback changes nothing", 3u == publishes);
    p_cb(p_stale, ERR_TIMEOUT);
    service();
    check("CM08 nor does a stale timeout", 3u == publishes);

    /* lwIP drops requests on close without a callback. */
    b_connected = false;
    service();
    b_connected = true;
    service();
    check("CM09 after a reconnect the batch in flight is sent again",
          (4u == publishes) && (0 == strcmp(published, "b3")));

    p_cb(p_cb_arg, ERR_OK);
    publish_result = ERR_MEM;
    service();
    publish_result = ERR_OK;
    service();
    check("CM10 a full output buffer only delays the batch",
          (5u == publishes) && (0 == strcmp(published, "b4")));

    printf("%u failure(s)\n", failures);
    return (0u == failures) ? 0 : 1;
}
