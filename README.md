| Supported Targets | ESP32-S3 |
| ----------------- | -------- |

# MQTT over WIZnet Ethernet

ESP-IDF's [`examples/protocols/mqtt`](https://github.com/espressif/esp-idf/tree/master/examples/protocols/mqtt)
example, running over a WIZnet W5500 or W6300 instead of over `example_connect()`.

The MQTT half is the standard example, unmodified in shape: the same
`mqtt_event_handler` switch, the same `mqtt_app_start()`, the same
`esp_mqtt_client` API, the same broker-URI setting in menuconfig. Only the
network comes up differently.

## The one thing that changed

The stock example brings the network up like this:

```c
ESP_ERROR_CHECK(nvs_flash_init());
ESP_ERROR_CHECK(esp_netif_init());
ESP_ERROR_CHECK(esp_event_loop_create_default());
ESP_ERROR_CHECK(example_connect());     /* Wi-Fi or Ethernet, per menuconfig */
mqtt_app_start();
```

`example_connect()` drives ESP-IDF's own `esp_eth` W5500 driver, which is
exactly what the WIZnet component replaces. Here it becomes:

```c
ESP_ERROR_CHECK(nvs_flash_init());
wiznet_net_init(&g_net_info);           /* WIZnet chip -> esp_eth netif */
while (!wiznet_net_is_up()) { vTaskDelay(pdMS_TO_TICKS(100)); }
mqtt_app_start();
```

> **`esp_netif_init()` and `esp_event_loop_create_default()` are deliberately
> gone.** `wiznet_net_init()` calls both itself, wrapped in `ESP_ERROR_CHECK`,
> and a second `esp_event_loop_create_default()` returns
> `ESP_ERR_INVALID_STATE` — so leaving the stock lines in place aborts the
> firmware during boot. (`esp_netif_init()` alone would be harmless; it is
> idempotent.) This is the one place where copying the upstream example
> verbatim does not work.

Everything after that point is ordinary ESP-IDF: the chip is a plain SPI
Ethernet MAC, the ESP32-S3's own LwIP owns TCP/IP, and ESP-MQTT runs on top with
TLS, QoS 1/2, automatic reconnection and the rest of its feature set available.

### Why the backend is not a choice here

`Component config → WIZnet WSM Driver → Network backend` offers two options, and
this example requires **esp_eth MACRAW + software LwIP**:

- **esp_eth MACRAW** — the chip is a MAC, the ESP32 runs TCP/IP. Standard BSD
  sockets, so ESP-MQTT works unchanged. ← what this example uses
- **TOE** — the chip runs TCP/IP, reached by wrapping 13 `lwip_*` symbols with
  `-Wl,--wrap`. ESP-MQTT also calls `getaddrinfo()`, `select()` and `fcntl()`,
  none of which are wrapped, so its call chain falls through to the software
  stack midway and loses the connection the chip owns.

`sdkconfig.defaults` sets the right one, and [app_main.c](main/app_main.c) has an
`#error` that stops the build with that explanation if the TOE backend is
selected — rather than producing a firmware that silently cannot connect.

If you need the chip's hardware TCP/IP, use the component's own
`examples/mqtt` instead: it ships a small MQTT engine written against the socket
vtable precisely so that it works on both backends.

## Hardware Required

An ESP32-S3 board plus a WIZnet W5500 or W6300 module. Pick the board in
menuconfig and the chip and the whole SPI wiring follow from it:

| Board         | MOSI | MISO | IO2 | IO3 | SCLK | CS | RST | INT |
| ------------- | ---- | ---- | --- | --- | ---- | -- | --- | --- |
| W5500 Dev-kit | 11   | 13   | —   | —   | 12   | 10 | 9   | 14  |
| W5500 SoM     | 11   | 13   | —   | —   | 12   | 10 | 9   | 14  |
| W6300 Dev-kit | 11   | 13   | 14  | 9   | 12   | 10 | 21  | 8   |
| W6300 SoM     | 34   | 35   | 36  | 37  | 42   | 41 | 21  | 33  |

SPI host is 2 (SPI2) and the clock 33 MHz on every board. IO2/IO3 exist only on
the W6300 in Quad QSPI mode. For a board that is not listed, choose **Custom** —
that is what turns the pin values into editable prompts; a fixed board shows
them read-only.

## Configure the project

### `idf.py menuconfig`

- **Example Configuration**
  - *MQTT broker URI* — default `mqtt://192.168.11.100:1883`. The scheme picks
    the transport (`mqtt://`, `mqtts://`, `ws://`, `wss://`).
  - *Use TLS* — attaches ESP-IDF's certificate bundle, as the upstream example
    does. Off by default because the bundle costs tens of kilobytes and the
    usual setup here is a local mosquitto over plain TCP.
  - *Publish topic* / *Publish payload* / *Subscribe topic*.
- **Component config → WIZnet WSM Driver**
  - *Board* — W5500 Dev-kit (default), W5500 SoM, W6300 Dev-kit, W6300 SoM or
    Custom. This is the only hardware choice most users make: it sets the chip
    and every pin. Custom unlocks the pins for editing.
  - *Network backend* — must stay **esp_eth MACRAW**, see above.

### `main/net_config.h`

The interface's static identity — MAC, IP, netmask, gateway, DNS. It is the one
setting outside menuconfig, because `wiznet_net_init()` takes a `wiz_NetInfo`
whose address fields are byte arrays and a Kconfig string would have to be
parsed back into one. Put the address on the broker's subnet.

## Build and Flash

```
idf.py set-target esp32s3
idf.py -p PORT flash monitor
```

(To exit the serial monitor, type `Ctrl-]`.)

Start a broker on the PC named by the URI first, e.g. `mosquitto -v`, then:

```
mosquitto_sub -h <broker> -t 'publish_topic/eth' -v      # watch what the board sends
mosquitto_pub -h <broker> -t 'subscribe_topic/eth' -m hi # send something back
```

## Example Output

```
I (312) mqtt_example: [APP] Startup..
I (322) mqtt_example: [APP] IDF version: v6.0.2
I (512) w5500_eth: Ethernet started
I (532) mqtt_example: Waiting for Ethernet link...
I (1892) w5500_eth: Ethernet got IP 192.168.11.2
I (1902) mqtt_example: Ethernet is up
I (2012) mqtt_example: MQTT_EVENT_CONNECTED
I (2022) mqtt_example: sent subscribe successful, msg_id=41123
I (2112) mqtt_example: MQTT_EVENT_SUBSCRIBED, msg_id=41123
I (2122) mqtt_example: sent publish successful, msg_id=0
TOPIC=subscribe_topic/eth
DATA=hi
```

## Notes

- **`wsm_driver` is tracked from `main`, not from a release.** The registry's
  1.1.0 predates the board selection this example relies on.
  `dependencies.lock` pins the exact commit, so builds stay reproducible;
  deleting the lock picks up whatever `main` is at that moment. Pin `version:`
  to a commit hash if you need that frozen.
- **ESP-MQTT is a managed component now.** It was removed from the ESP-IDF tree
  in v6.0, so `main/idf_component.yml` depends on `espressif/mqtt` — the same
  way the upstream example does.
- **Wi-Fi is not used.** The WSM component can bring up a Wi-Fi STA alongside
  Ethernet (`wifi_backend.h`), and its own `examples/mqtt` runs an MQTT client
  on each interface at once. This example follows the upstream ESP-IDF one,
  which is single-interface.
