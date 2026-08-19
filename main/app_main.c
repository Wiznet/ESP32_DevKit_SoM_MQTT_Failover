/*
 * SPDX-FileCopyrightText: 2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */
/* MQTT example with Ethernet + Wi-Fi failover, on a WIZnet W5500 / W6300.

   This is ESP-IDF's examples/protocols/mqtt example with two changes: the
   Ethernet interface comes up through the wiznet/wsm_driver component instead
   of through example_connect(), and a second interface (Wi-Fi STA) is brought
   up alongside it so the MQTT session survives losing either one.

   The MQTT half below — the event handler and mqtt_app_start() — is the
   upstream one. On this backend the chip is a plain SPI Ethernet MAC and the
   ESP32's own LwIP owns TCP/IP, so both interfaces are ordinary esp_netifs and
   ESP-MQTT runs over them unchanged.

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

#include <inttypes.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_check.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "sdkconfig.h"

#if CONFIG_EXAMPLE_CONNECT_ETHERNET
#include "esp_eth.h"            /* ETH_EVENT */
#include "wizchip_conf.h"       /* wiz_NetInfo, NETINFO_STATIC */
#include "net_backend.h"        /* wiznet_net_init / wiznet_net_is_up */
#endif
#if CONFIG_EXAMPLE_CONNECT_WIFI
#include "esp_wifi.h"           /* WIFI_EVENT */
#include "wifi_backend.h"       /* wifi_net_init / wifi_net_is_up */
#endif

#include "mqtt_client.h"
#if CONFIG_EXAMPLE_MQTT_USE_TLS
#include "esp_crt_bundle.h"
#endif

/*
 * ESP-MQTT needs a real BSD socket API — it reaches LwIP through esp-tls and
 * tcp_transport, which call getaddrinfo(), select() and fcntl(). The TOE backend
 * redirects only 13 lwip_* entry points with -Wl,--wrap and those are not among
 * them, so on TOE the call chain falls through to the software stack midway and
 * loses the connection the chip actually owns. Failover would be worse still:
 * with the wrap active the chip owns every socket, leaving no way for a Wi-Fi
 * socket to exist at all.
 */
#if !defined(CONFIG_WSM_DRIVER_BACKEND_ETH)
#error "Select Component config -> WIZnet WSM Driver -> Network backend -> esp_eth MACRAW + software LwIP. ESP-MQTT cannot run on the TOE backend."
#endif

#if !CONFIG_EXAMPLE_CONNECT_ETHERNET && !CONFIG_EXAMPLE_CONNECT_WIFI
#error "Enable at least one interface under Example Configuration."
#endif

static const char *TAG = "mqtt_example";

static esp_mqtt_client_handle_t s_client;

/* ---- interface priority ---------------------------------------------------
 * esp_netif picks the default netif — the one that carries traffic with no more
 * specific route — as the highest route_prio among those that are up, and moves
 * it automatically when one goes down. That is the whole failover mechanism.
 *
 * The IDF defaults are Wi-Fi 100 (ESP_NETIF_INHERENT_DEFAULT_WIFI_STA) and
 * Ethernet 50 (ESP_NETIF_INHERENT_DEFAULT_ETH), i.e. Wi-Fi wins by default.
 * Preferring Ethernet therefore means raising it ABOVE 100, not merely leaving
 * it enabled — a detail that is easy to miss and silently sends traffic out of
 * the wrong interface.
 */
#define PRIO_PREFERRED   150
#define PRIO_BACKUP       10

/* ---- Ethernet identity ----------------------------------------------------
 * Upstream gets its addressing from protocol_examples_common over DHCP; that
 * helper is replaced here, and wiznet_net_init() takes a wiz_NetInfo whose
 * address fields are byte arrays. The values come from menuconfig as strings
 * and are parsed below.
 */
#if CONFIG_EXAMPLE_CONNECT_ETHERNET
static wiz_NetInfo g_net_info = {
#if _WIZCHIP_ > W5500
    .ipmode = NETINFO_STATIC_ALL,
#endif
    .dhcp = NETINFO_STATIC,
};

static esp_err_t parse_ipv4(const char *name, const char *s, uint8_t out[4])
{
    unsigned o[4];
    char tail;

    /* The trailing %c is what rejects "1.2.3.4.5" and "1.2.3.4x": sscanf on its
     * own stops at the first mismatch and still reports four conversions. */
    if (sscanf(s, "%u.%u.%u.%u%c", &o[0], &o[1], &o[2], &o[3], &tail) != 4) {
        ESP_LOGE(TAG, "%s: \"%s\" is not a dotted-quad IPv4 address", name, s);
        return ESP_ERR_INVALID_ARG;
    }
    for (int i = 0; i < 4; i++) {
        if (o[i] > 255) {
            ESP_LOGE(TAG, "%s: \"%s\" has an octet above 255", name, s);
            return ESP_ERR_INVALID_ARG;
        }
        out[i] = (uint8_t)o[i];
    }
    return ESP_OK;
}

static esp_err_t parse_mac(const char *name, const char *s, uint8_t out[6])
{
    unsigned o[6];
    char tail;

    if (sscanf(s, "%x:%x:%x:%x:%x:%x%c",
               &o[0], &o[1], &o[2], &o[3], &o[4], &o[5], &tail) != 6) {
        ESP_LOGE(TAG, "%s: \"%s\" is not six colon-separated hex octets", name, s);
        return ESP_ERR_INVALID_ARG;
    }
    for (int i = 0; i < 6; i++) {
        if (o[i] > 0xFF) {
            ESP_LOGE(TAG, "%s: \"%s\" has an octet above 0xFF", name, s);
            return ESP_ERR_INVALID_ARG;
        }
        out[i] = (uint8_t)o[i];
    }
    /* A set low bit in the first octet marks a multicast address, which a
     * station may not use as its own; the link comes up and nothing routes. */
    if (out[0] & 0x01) {
        ESP_LOGE(TAG, "%s: \"%s\" is a multicast address and cannot be a station address",
                 name, s);
        return ESP_ERR_INVALID_ARG;
    }
    return ESP_OK;
}

static esp_err_t load_net_info(void)
{
    ESP_RETURN_ON_ERROR(parse_mac("Ethernet MAC address",
                                  CONFIG_EXAMPLE_ETH_MAC_ADDR, g_net_info.mac),
                        TAG, "check Example Configuration");
    ESP_RETURN_ON_ERROR(parse_ipv4("Ethernet static IP address",
                                   CONFIG_EXAMPLE_ETH_IP_ADDR, g_net_info.ip),
                        TAG, "check Example Configuration");
    ESP_RETURN_ON_ERROR(parse_ipv4("Ethernet subnet mask",
                                   CONFIG_EXAMPLE_ETH_NETMASK, g_net_info.sn),
                        TAG, "check Example Configuration");
    ESP_RETURN_ON_ERROR(parse_ipv4("Ethernet default gateway",
                                   CONFIG_EXAMPLE_ETH_GATEWAY, g_net_info.gw),
                        TAG, "check Example Configuration");

    /* Optional: an empty string leaves dns zeroed, which the backend reads as
     * "do not set a DNS server". */
    if (CONFIG_EXAMPLE_ETH_DNS_SERVER[0] != '\0') {
        ESP_RETURN_ON_ERROR(parse_ipv4("Ethernet DNS server",
                                       CONFIG_EXAMPLE_ETH_DNS_SERVER, g_net_info.dns),
                            TAG, "check Example Configuration");
    }
    return ESP_OK;
}
#endif /* CONFIG_EXAMPLE_CONNECT_ETHERNET */

/* ---- failover -------------------------------------------------------------
 * Two things have to happen when an interface changes state, and esp_netif only
 * does the first:
 *
 *   1. the default route moves — automatic, from route_prio;
 *   2. the MQTT session moves — NOT automatic. A TCP connection is bound to the
 *      source address it was opened with, so it does not migrate. On a link
 *      going down LwIP aborts the pcb (netif_set_addr -> tcp_netif_ip_addr_changed)
 *      and ESP-MQTT reconnects on its own, which is enough. But on a link coming
 *      BACK it does nothing: LwIP's source-address routing hook pins the live
 *      connection to the netif whose address the socket already holds, so a
 *      session that failed over to the backup stays there even once the
 *      preferred interface returns.
 *
 * Hence: nudge the client on both edges. Reconnecting costs one short outage
 * and is the only way back to the preferred interface.
 */
static void nudge_reconnect(const char *why)
{
    if (s_client == NULL) {
        return;     /* interface event arrived before mqtt_app_start() */
    }
    ESP_LOGI(TAG, "%s — reconnecting MQTT so it picks the current route", why);
    /* Asks the client's own task to tear the session down and rebuild it; the
     * new socket redoes the route lookup. Safe to call when already
     * disconnected — it just shortens the wait. */
    esp_mqtt_client_reconnect(s_client);
}

static void iface_event_handler(void *arg, esp_event_base_t base, int32_t id, void *data)
{
#if CONFIG_EXAMPLE_CONNECT_ETHERNET
    if (base == ETH_EVENT && id == ETHERNET_EVENT_CONNECTED) {
        nudge_reconnect("Ethernet link up");
        return;
    }
#endif
#if CONFIG_EXAMPLE_CONNECT_WIFI
    if (base == WIFI_EVENT && id == WIFI_EVENT_STA_CONNECTED) {
        nudge_reconnect("Wi-Fi associated");
        return;
    }
#endif
}

/*
 * @brief Event handler registered to receive MQTT events
 *
 *  This function is called by the MQTT client event loop.
 *
 * @param handler_args user data registered to the event.
 * @param base Event base for the handler(always MQTT Base in this example).
 * @param event_id The id for the received event.
 * @param event_data The data for the event, esp_mqtt_event_handle_t.
 */
static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data)
{
    ESP_LOGD(TAG, "Event dispatched from event loop base=%s, event_id=%" PRIi32, base, event_id);
    esp_mqtt_event_handle_t event = event_data;
    esp_mqtt_client_handle_t client = event->client;
    int msg_id;
    switch ((esp_mqtt_event_id_t)event_id) {
    case MQTT_EVENT_CONNECTED:
        ESP_LOGI(TAG, "MQTT_EVENT_CONNECTED");
        msg_id = esp_mqtt_client_subscribe(client, "topic/qos0", 0);
        ESP_LOGI(TAG, "sent subscribe successful, msg_id=%d", msg_id);

        msg_id = esp_mqtt_client_subscribe(client, "topic/qos1", 1);
        ESP_LOGI(TAG, "sent subscribe successful, msg_id=%d", msg_id);

        msg_id = esp_mqtt_client_unsubscribe(client, "topic/qos1");
        ESP_LOGI(TAG, "sent unsubscribe successful, msg_id=%d", msg_id);
        break;

    case MQTT_EVENT_DISCONNECTED:
        ESP_LOGI(TAG, "MQTT_EVENT_DISCONNECTED");
        break;

    case MQTT_EVENT_SUBSCRIBED:
        ESP_LOGI(TAG, "MQTT_EVENT_SUBSCRIBED, msg_id=%d, return code=0x%02x ", event->msg_id, (uint8_t)*event->data);
        msg_id = esp_mqtt_client_publish(client, "topic/qos0", "data", 0, 0, 0);
        ESP_LOGI(TAG, "sent publish successful, msg_id=%d", msg_id);
        break;

    case MQTT_EVENT_UNSUBSCRIBED:
        ESP_LOGI(TAG, "MQTT_EVENT_UNSUBSCRIBED, msg_id=%d", event->msg_id);
        break;

    case MQTT_EVENT_PUBLISHED:
        ESP_LOGI(TAG, "MQTT_EVENT_PUBLISHED, msg_id=%d", event->msg_id);
        break;

    case MQTT_EVENT_DATA:
        ESP_LOGI(TAG, "MQTT_EVENT_DATA");
        printf("TOPIC=%.*s\r\n", event->topic_len, event->topic);
        printf("DATA=%.*s\r\n", event->data_len, event->data);
        break;

    case MQTT_EVENT_ERROR:
        ESP_LOGI(TAG, "MQTT_EVENT_ERROR");
        if (event->error_handle->error_type == MQTT_ERROR_TYPE_TCP_TRANSPORT) {
            ESP_LOGI(TAG, "Last error code reported from esp-tls: 0x%x", event->error_handle->esp_tls_last_esp_err);
            ESP_LOGI(TAG, "Last tls stack error number: 0x%x", event->error_handle->esp_tls_stack_err);
            ESP_LOGI(TAG, "Last captured errno : %d (%s)", event->error_handle->esp_transport_sock_errno,
                     strerror(event->error_handle->esp_transport_sock_errno));
        } else if (event->error_handle->error_type == MQTT_ERROR_TYPE_CONNECTION_REFUSED) {
            ESP_LOGI(TAG, "Connection refused error: 0x%x", event->error_handle->connect_return_code);
        } else {
            ESP_LOGW(TAG, "Unknown error type: 0x%x", event->error_handle->error_type);
        }
        break;

    default:
        ESP_LOGI(TAG, "Other event id:%d", event->event_id);
        break;
    }
}

static void mqtt_app_start(void)
{
    const esp_mqtt_client_config_t mqtt_cfg = {
        .broker = {
            .address.uri = CONFIG_EXAMPLE_MQTT_BROKER_URI,
#if CONFIG_EXAMPLE_MQTT_USE_TLS
            .verification.crt_bundle_attach = esp_crt_bundle_attach, /* Use built-in certificate bundle */
#endif
        },
        /* Shortened from the library defaults so a dead path is noticed in
         * seconds rather than minutes — see the Kconfig help. */
        .session = {
            .keepalive = CONFIG_EXAMPLE_MQTT_KEEPALIVE_S,
        },
        .network = {
            .reconnect_timeout_ms = CONFIG_EXAMPLE_MQTT_RECONNECT_MS,
#if CONFIG_EXAMPLE_TCP_KEEPALIVE_ENABLE
            /* Transport-level liveness, independent of MQTT traffic: catches a
             * path that died while the link stayed up, which produces no netif
             * event and so would otherwise only surface via MQTT keep-alive. */
            .tcp_keep_alive_cfg = {
                .keep_alive_enable   = true,
                .keep_alive_idle     = CONFIG_EXAMPLE_TCP_KEEPALIVE_IDLE_S,
                .keep_alive_interval = CONFIG_EXAMPLE_TCP_KEEPALIVE_INTERVAL_S,
                .keep_alive_count    = CONFIG_EXAMPLE_TCP_KEEPALIVE_COUNT,
            },
#endif
            /* network.if_name is deliberately NOT set. Binding the socket to an
             * interface (SO_BINDTODEVICE) would pin the session to it and defeat
             * failover entirely; leaving it unset lets each new socket follow
             * whatever the default route is at that moment. */
        },
    };

    ESP_LOGI(TAG, "[APP] Free memory: %" PRIu32 " bytes", esp_get_free_heap_size());
    s_client = esp_mqtt_client_init(&mqtt_cfg);
    /* The last argument may be used to pass data to the event handler, in this example mqtt_event_handler */
    esp_mqtt_client_register_event(s_client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);
    esp_mqtt_client_start(s_client);
}

/* ---- failover test traffic ------------------------------------------------
 * Publishing on a fixed interval, with the outgoing interface named in the
 * payload, is what makes a failover observable. Watching the broker with
 *
 *     mosquitto_sub -h <broker> -t 'topic/qos0' -v
 *
 * a switch looks like this — the gap measures the outage, the interface name
 * proves the traffic really moved:
 *
 *     topic/qos0 seq=40 via=ETH_DEF 192.168.11.2
 *     topic/qos0 seq=41 via=ETH_DEF 192.168.11.2      <- cable pulled here
 *     topic/qos0 seq=44 via=WIFI_STA_DEF 192.168.0.42
 *
 * The upstream example publishes only in response to a SUBACK, which shows
 * neither the timing nor the path.
 */
#if CONFIG_EXAMPLE_MQTT_PUBLISH_PERIOD_MS > 0

/* Name and address of the netif that currently owns the default route — the one
 * a new socket would go out of. Read fresh on every publish, since that is
 * exactly the thing under test. */
static void describe_route(char *out, size_t len)
{
    esp_netif_t *netif = esp_netif_get_default_netif();
    esp_netif_ip_info_t ip;

    if (netif == NULL) {
        snprintf(out, len, "via=none");
        return;
    }
    if (esp_netif_get_ip_info(netif, &ip) != ESP_OK) {
        snprintf(out, len, "via=%s", esp_netif_get_ifkey(netif));
        return;
    }
    snprintf(out, len, "via=%s " IPSTR, esp_netif_get_ifkey(netif), IP2STR(&ip.ip));
}

static void publisher_task(void *arg)
{
    unsigned seq = 0;

    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(CONFIG_EXAMPLE_MQTT_PUBLISH_PERIOD_MS));

        if (s_client == NULL) {
            continue;
        }

        char route[64];
        char payload[96];
        describe_route(route, sizeof(route));
        snprintf(payload, sizeof(payload), "seq=%u %s", seq, route);

        /* QoS 0 and no outbox queueing, so a publish attempted while
         * disconnected is dropped rather than delivered late — the gap in the
         * sequence is then a true measure of the outage. */
        int msg_id = esp_mqtt_client_publish(s_client, "topic/qos0", payload, 0, 0, 0);
        if (msg_id < 0) {
            ESP_LOGW(TAG, "publish dropped (disconnected): %s", payload);
        } else {
            ESP_LOGI(TAG, "published: %s", payload);
        }
        seq++;
    }
}
#endif /* CONFIG_EXAMPLE_MQTT_PUBLISH_PERIOD_MS > 0 */

/* True once at least one interface can carry traffic. */
static bool any_link_up(void)
{
#if CONFIG_EXAMPLE_CONNECT_ETHERNET
    if (wiznet_net_is_up()) {
        return true;
    }
#endif
#if CONFIG_EXAMPLE_CONNECT_WIFI
    if (wifi_net_is_up()) {
        return true;
    }
#endif
    return false;
}

static void set_route_priorities(void)
{
#if CONFIG_EXAMPLE_CONNECT_ETHERNET && CONFIG_EXAMPLE_CONNECT_WIFI
    esp_netif_t *eth  = esp_netif_get_handle_from_ifkey("ETH_DEF");
    esp_netif_t *wifi = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");

#if CONFIG_EXAMPLE_PRIMARY_ETHERNET
    const int eth_prio = PRIO_PREFERRED, wifi_prio = PRIO_BACKUP;
    ESP_LOGI(TAG, "Preferred interface: Ethernet");
#else
    const int eth_prio = PRIO_BACKUP, wifi_prio = PRIO_PREFERRED;
    ESP_LOGI(TAG, "Preferred interface: Wi-Fi");
#endif

    if (eth != NULL) {
        esp_netif_set_route_prio(eth, eth_prio);
    }
    if (wifi != NULL) {
        esp_netif_set_route_prio(wifi, wifi_prio);
    }
#endif
}

void app_main(void)
{
    ESP_LOGI(TAG, "[APP] Startup..");
    ESP_LOGI(TAG, "[APP] Free memory: %" PRIu32 " bytes", esp_get_free_heap_size());
    ESP_LOGI(TAG, "[APP] IDF version: %s", esp_get_idf_version());

    ESP_ERROR_CHECK(nvs_flash_init());

    /* The upstream example calls esp_netif_init() and esp_event_loop_create_default()
     * here, before example_connect(). Do NOT do that: wiznet_net_init() performs
     * both itself and wraps them in ESP_ERROR_CHECK, and a second
     * esp_event_loop_create_default() returns ESP_ERR_INVALID_STATE — so leaving
     * the upstream lines in place aborts the firmware during boot.
     *
     * Ethernet must also come up FIRST for the same reason: it is the one that
     * hard-fails on an already-initialised event loop, while wifi_net_init()
     * tolerates it. */
#if CONFIG_EXAMPLE_CONNECT_ETHERNET
    ESP_ERROR_CHECK(load_net_info());
    wiznet_net_init(&g_net_info);
#endif
#if CONFIG_EXAMPLE_CONNECT_WIFI
    wifi_net_init(CONFIG_EXAMPLE_WIFI_SSID, CONFIG_EXAMPLE_WIFI_PASSWORD);
#endif

    set_route_priorities();

    /* Both interfaces exist now; either one is enough to start. Waiting for a
     * specific one would stall the demo when only the other is plugged in. */
#if CONFIG_EXAMPLE_CONNECT_ETHERNET
    ESP_ERROR_CHECK(esp_event_handler_register(ETH_EVENT, ESP_EVENT_ANY_ID,
                                               iface_event_handler, NULL));
#endif
#if CONFIG_EXAMPLE_CONNECT_WIFI
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID,
                                               iface_event_handler, NULL));
#endif

    ESP_LOGI(TAG, "Waiting for a network link...");
    while (!any_link_up()) {
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    ESP_LOGI(TAG, "Link is up");

    mqtt_app_start();

#if CONFIG_EXAMPLE_MQTT_PUBLISH_PERIOD_MS > 0
    /* Started after the client exists so the task never sees s_client == NULL
     * on its first tick. Priority 5 matches the MQTT task's own. */
    xTaskCreate(publisher_task, "mqtt_pub", 4096, NULL, 5, NULL);
#endif
}
