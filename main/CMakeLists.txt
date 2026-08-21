idf_component_register(
    SRCS
        "app_main.c"
        "toe_socket_shim.c"     # TOE backend only; compiles to nothing otherwise
    INCLUDE_DIRS "."
    REQUIRES
        wsm_driver          # WIZnet chip + esp_eth netif (replaces example_connect)
        mqtt                # espressif/mqtt, i.e. ESP-MQTT
        nvs_flash
        esp_wifi            # wifi_backend.h (WIFI_EVENT)
        esp_eth             # ETH_EVENT
        esp_netif
        esp_event
        esp_timer
)

# --wrap flags for toe_socket_shim.c: lwip_fcntl/lwip_select (missing from the
# component's list) and the wiztoe_* layer below its __wrap_lwip_* (the failover
# seam; private symbols - see the NOTE in the shim). TOE only; harmless on
# esp_eth, where the shim compiles away.
if(CONFIG_WSM_DRIVER_SOCKET_WRAP)
    foreach(fn lwip_fcntl lwip_select
               wiztoe_socket wiztoe_connect wiztoe_send wiztoe_recv
               wiztoe_close wiztoe_setsockopt wiztoe_getsockopt)
        target_link_libraries(${COMPONENT_LIB} INTERFACE "-Wl,--wrap=${fn}")
        target_link_libraries(${COMPONENT_LIB} INTERFACE "-u __wrap_${fn}")
    endforeach()
endif()

# app_main.c includes wizchip_conf.h, which pulls in ioLibrary's socket.h. That
# header declares connect_IO_6 / sendto_IO_6 / recvfrom_IO_6 `static` without
# defining them; nothing here can fix that, and the warnings would bury real ones.
target_compile_options(${COMPONENT_LIB} PRIVATE -Wall -Wno-unused-function)
