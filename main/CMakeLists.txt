idf_component_register(
    SRCS "main.c" "discipline.c" "gps.c" "gnss_tcp.c" "ntp.c" "eth_if.c" "monitor.c"
    INCLUDE_DIRS "."
    REQUIRES esp_common
             esp_eth
             esp_netif
             esp_event
             esp_http_server
             esp_hw_support
             esp_timer
             esp_system
             esp_driver_gpio
             esp_driver_uart
             lwip
)
