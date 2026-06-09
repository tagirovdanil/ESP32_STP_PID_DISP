set(srcs "main.c" "pressure_sensor.c" "st7789.c" "fontx.c")

idf_component_register(SRCS "wifi_tcp.cpp" "config.c" "pressure_regulator.c" "${srcs}" 
                       INCLUDE_DIRS "."
                       PRIV_REQUIRES esp_driver_gpio esp_driver_uart esp_driver_spi esp_driver_ledc esp_timer spiffs
                                     esp_wifi esp_event esp_netif lwip nvs_flash esp_hw_support)
