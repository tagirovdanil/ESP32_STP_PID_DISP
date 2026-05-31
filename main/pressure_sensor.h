#ifndef PRESSURE_SENSOR_H
#define PRESSURE_SENSOR_H

#include <stdint.h>
#include <stdbool.h>
#include "driver/uart.h"
#include "esp_timer.h"
#include "st7789.h"
#include "fontx.h"

/* ========================================================================== */
/*   НАСТРОЙКИ UART ДЛЯ ДАТЧИКА ДАВЛЕНИЯ                                      */
/* ========================================================================== */
#define SENSOR_UART_NUM         UART_NUM_1
#define PIN_RX1                 26
#define PIN_TX1                 27
#define BUF_SIZE                1024
#define SERIAL_UART1_INTERVAL   50

/* ========================================================================== */
/*   ПИНЫ И ГЕОМЕТРИЯ ДИСПЛЕЯ TTGO                                           */
/* ========================================================================== */
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

/* ========================================================================== */
/*   ПРОТОКОЛ ДАТЧИКА ДАВЛЕНИЯ                                                */
/* ========================================================================== */
#define DEVICE_ADDRESS1 0x02
#define ACK             0x06

/* ========================================================================== */
/*   ГЛОБАЛЬНЫЕ ПЕРЕМЕННЫЕ (ТОЛЬКО ССЫЛКИ EXTERN)                            */
/* ========================================================================== */
extern volatile float setpoint_kPa;
extern volatile float pressure1_kPa;
extern bool is_calibrating;

/* ========================================================================== */
/*   ПУБЛИЧНЫЕ ФУНКЦИИ МОДУЛЯ                                                 */
/* ========================================================================== */
void pressure_sensor_init(void);
void LCD_init(void);
bool performAdvancedZeroCalibration(float offset_kPa, uart_port_t uart_num);
void display_update_task(void *pvParameters);
extern uint32_t sum_err;

// ДОБАВЛЯЕМ СЮДА: функция инициализации USB UART и отрисовки начального UI
void pressure_ui_and_usb_init(TFT_t *p_dev);

#endif // PRESSURE_SENSOR_H
