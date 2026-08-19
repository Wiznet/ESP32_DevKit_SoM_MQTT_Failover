idf_component_register(
    SRCS "app_main.c"
    INCLUDE_DIRS "."
    REQUIRES
        wsm_driver          # WIZnet chip + esp_eth netif (replaces example_connect)
        mqtt                # espressif/mqtt, i.e. ESP-MQTT
        nvs_flash
        esp_wifi             # wifi_backend.h (WIFI_EVENT)
        esp_eth              # ETH_EVENT
        esp_netif
        esp_event
)

# app_main.c includes wizchip_conf.h, which pulls in ioLibrary's socket.h. That
# header declares connect_IO_6 / sendto_IO_6 / recvfrom_IO_6 `static` without
# defining them; nothing here can fix that, and the warnings would bury real ones.
target_compile_options(${COMPONENT_LIB} PRIVATE -Wall -Wno-unused-function)
