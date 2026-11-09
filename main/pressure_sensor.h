#ifndef PRESSURE_SENSOR_H
#define PRESSURE_SENSOR_H

#include <stdint.h>
#include <stdbool.h>
#include "driver/uart.h"

/* ============================================================================
 *  РАСКЛАДКА UART
 * ----------------------------------------------------------------------------
 *    UART1 -> ОБЩАЯ шина датчиков: 0x01 (P1) и 0x02 (P2) на одних проводах,
 *             RX=GPIO26, TX=GPIO27, 19200 8O1. Опрос по кругу с периодом
 *             SENSOR_POLL_INTERVAL_MS (задаётся в config.h).
 *    UART2 -> панель (HMI), TX=GPIO33, RX=GPIO25 (см. config.h)
 *    UART0 -> консоль/логи ESP-IDF (CONFIG_ESP_CONSOLE_UART_DEFAULT=y)
 *
 *  Локального экрана на плате нет: SPI/ST7789/шрифты не используются.
 * ========================================================================== */

// настройки общей шины датчиков (UART1)
#define SENSOR_UART_NUM         UART_NUM_1
#define PIN_RX1                 26
#define PIN_TX1                 27
#define BUF_SIZE                1024

// протокол датчика
#define DEVICE_ADDRESS1 0x01
#define DEVICE_ADDRESS2 0x02
#define ACK             0x06

// extern-переменные (уставки каналов объявлены в config.h)
extern volatile float pressure1_kPa;
extern volatile float pressure2_kPa;   // показания датчика №2 (адрес 0x02)
extern uint32_t sum_err;               // счётчик ошибок кадра датчика 0x01
extern uint32_t sum_err2;              // счётчик ошибок кадра датчика 0x02

// функции датчика и связи с панелью по UART2
void pressure_sensor_init(void);       // поднимает общую шину UART1 и опрос обоих датчиков
// Калибровка нуля конкретного датчика на общей шине (адрес 0x01 или 0x02)
bool performAdvancedZeroCalibration(float offset_kPa, uart_port_t uart_num, uint8_t device_addr);
// Поднимает UART2 (TX=GPIO33, RX=GPIO25) и задачу отправки $SLSD.
// Вызывается В САМОМ НАЧАЛЕ app_main(), до стартовой паузы: приём от панели
// работает сразу, а передача $SLSD включается позже — pressure_display_link_start().
// Локального экрана на плате нет — SPI/ST7789/шрифты не используются.
void pressure_display_uart_init(void);

// Разрешить передачу $SLSD на панель (после стартовой паузы и инициализации).
// До этого вызова задача связи молчит, приём $SLSS/$SLSC при этом уже работает.
void pressure_display_link_start(void);
// Немедленно отправить один кадр $SLSD (после обнуления, чтобы панель сразу
// увидела нулевое давление, не дожидаясь очередных 200 мс)
void display_send_status_now(void);

// Попросить у панели уставку прямо сейчас: $SLSC,GET -> панель отвечает $SLSS.
// Вызывается после любой команды панели (START/STOP/...), если уставка ещё
// не приходила; внутри ограничение не чаще 1 раза в 500 мс.
void display_request_setpoint_now(void);

#endif // PRESSURE_SENSOR_H