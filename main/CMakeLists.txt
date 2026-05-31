set(srcs "main.c" "pressure_sensor.c" "control.c" "st7789.c" "fontx.c")

idf_component_register(SRCS "${srcs}" 
                       INCLUDE_DIRS "."
                       PRIV_REQUIRES esp_driver_gpio esp_driver_uart esp_driver_spi esp_driver_ledc esp_timer spiffs)
