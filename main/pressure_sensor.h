#ifndef PRESSURE_SENSOR_H
#define PRESSURE_SENSOR_H

#include <stdint.h>
#include <stdbool.h>
#include "driver/uart.h"
#include "esp_timer.h"
#include "st7789.h"
#include "fontx.h"

// настройки UART датчика 1
#define SENSOR_UART_NUM         UART_NUM_1
#define PIN_RX1                 26
#define PIN_TX1                 27
#define BUF_SIZE                1024
#define SERIAL_UART1_INTERVAL   50

// настройки UART датчика 2 — отдельная НЕЗАВИСИМАЯ шина (UART2), т.к. у обоих
// датчиков одинаковый адрес (DEVICE_ADDRESS1=0x02) и на одной шине их ответы
// накладывались бы друг на друга. Пины UART2 по умолчанию 16/17 переназначены:
// GPIO16 занят под LCD (DC). Шаговик убран из проекта, поэтому занятые им ранее
// GPIO12/13 свободны: RX2=12, TX2=13 (RX=GPIO12, TX=GPIO13). Если припаяете
// иначе — меняйте значения здесь.
#define SENSOR2_UART_NUM        UART_NUM_2
#define PIN_RX2                 12
#define PIN_TX2                 13
#define SERIAL_UART2_INTERVAL   50

// пины дисплея
#define PIN_MOSI        19
#define PIN_SCLK        18
#define PIN_CS          5
#define PIN_DC          16
#define PIN_RST         23
#define PIN_BL          4

#define SCREEN_WIDTH    135
#define SCREEN_HEIGHT   240
#define OFFSET_X        52
#define OFFSET_Y        40

// протокол датчика
#define DEVICE_ADDRESS1 0x02
#define ACK             0x06

// extern-переменные (уставки каналов объявлены в config.h)
extern volatile float pressure1_kPa;
extern volatile float pressure2_kPa;   // показания второго датчика (UART2)
extern uint32_t sum_err;
extern uint32_t sum_err2;              // счётчик CRC-ошибок второго датчика

// функции датчика и дисплея
void pressure_sensor_init(void);
void pressure_sensor2_init(void); // инициализация второго датчика (UART2)
void LCD_init(void);
bool performAdvancedZeroCalibration(float offset_kPa, uart_port_t uart_num);
void display_update_task(void *pvParameters);
void pressure_ui_and_usb_init(TFT_t *p_dev);

#endif // PRESSURE_SENSOR_H