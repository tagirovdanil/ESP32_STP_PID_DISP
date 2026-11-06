cmake_minimum_required(VERSION 3.16)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(ESP32_STP_PID_DISP)

# Локального экрана и шрифтов больше нет — образ storage1 (spiffs с fonts)
# не собирается и не прошивается. Раздел storage1 в partitions.csv остаётся
# свободным (можно переразметить позже).
