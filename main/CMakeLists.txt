# settings.c (NVS) в сборку не входит: уставку хранит дисплейный контроллер.
set(srcs "main.c" "pressure_sensor.c"
         "config.c" "compressor_control.c")

idf_component_register(SRCS ${srcs}
                       INCLUDE_DIRS "."
                       PRIV_REQUIRES esp_driver_gpio esp_driver_uart esp_timer
                                     esp_hw_support)
