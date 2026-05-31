cmake_minimum_required(VERSION 3.16)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(ESP32_STP_PID_DISP)

# ИСПОЛЬЗУЕМ НОВУЮ СИНТАКСИЧЕСКУЮ КОМАНДУ ДЛЯ V6.0+:
spiffs_create_partition_image(storage1 fonts FLASH_IN_PROJECT)
