#ifndef PRESSURE_SENSOR_H
#define PRESSURE_SENSOR_H

#include <stdint.h>
#include <stdbool.h>
#include "driver/uart.h"
#include "esp_timer.h"
#include "st7789.h"
#include "fontx.h"

/* ============================================================================
 *  РАСКЛАДКА UART
 * ----------------------------------------------------------------------------
 *    UART1 -> ОБЩАЯ шина датчиков: 0x01 (P1) и 0x02 (P2) на одних проводах,
 *             RX=GPIO26, TX=GPIO27, 19200 8O1. Опрос по кругу с периодом
 *             SENSOR_POLL_INTERVAL_MS (задаётся в config.h).
 *    UART2 -> внешний дисплей (HMI), TX=GPIO33, RX=GPIO25 (см. config.h)
 *    UART0 -> консоль/логи ESP-IDF (CONFIG_ESP_CONSOLE_UART_DEFAULT=y)
 *
 *  ВНИМАНИЕ: GPIO23 — это RST локального экрана ST7789 (PIN_RST), его нельзя
 *  назначать ни датчикам, ни чему-либо ещё.
 * ========================================================================== */

// настройки общей шины датчиков (UART1)
#define SENSOR_UART_NUM         UART_NUM_1
#define PIN_RX1                 26
#define PIN_TX1                 27
#define BUF_SIZE                1024

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
#define DEVICE_ADDRESS1 0x01
#define DEVICE_ADDRESS2 0x02
#define ACK             0x06

// extern-переменные (уставки каналов объявлены в config.h)
extern volatile float pressure1_kPa;
extern volatile float pressure2_kPa;   // показания датчика №2 (адрес 0x02)
extern uint32_t sum_err;               // счётчик ошибок кадра датчика 0x01
extern uint32_t sum_err2;              // счётчик ошибок кадра датчика 0x02

// функции датчика и дисплея
void pressure_sensor_init(void);       // поднимает общую шину UART1 и опрос обоих датчиков
void LCD_init(void);
// Калибровка нуля конкретного датчика на общей шине (адрес 0x01 или 0x02)
bool performAdvancedZeroCalibration(float offset_kPa, uart_port_t uart_num, uint8_t device_addr);
void display_update_task(void *pvParameters);
void pressure_ui_and_usb_init(TFT_t *p_dev);
// Немедленно отправить один кадр $SLSD (после обнуления, чтобы панель сразу
// увидела нулевое давление, не дожидаясь очередных 200 мс)
void display_send_status_now(void);

#endif // PRESSURE_SENSOR_H