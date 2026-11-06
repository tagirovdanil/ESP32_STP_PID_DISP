set(srcs "main.c" "pressure_sensor.c" "settings.c"
         "config.c" "compressor_control.c")

idf_component_register(SRCS ${srcs}
                       INCLUDE_DIRS "."
                       PRIV_REQUIRES esp_driver_gpio esp_driver_uart esp_timer
                                     nvs_flash esp_hw_support)
