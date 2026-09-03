#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/uart.h"
#include "esp_mac.h"          // Для чтения заводского MAC (серийного номера) из efuse

#include "st7789.h"           // Для работы с дисплеем TTGO
#include "pressure_sensor.h"  // Для работы с датчиками давления
#include "config.h"           // Глобалы: давления, уставки двух каналов
#include "compressor_control.h" // Управление двумя компрессорами (реле)
#include "wifi_tcp.h"         // WiFi (AP/STA) + TCP-сервер для приёма команд по сети

#define USB_UART_PORT       UART_NUM_0
#define USB_BUF_SIZE        256

TaskHandle_t display_task_handle = NULL;

static const char *TAG = "MAIN_APP";

// ==========================================================================
// СЕРИЙНЫЙ НОМЕР ESP32 (заводской MAC из efuse — уникален для каждого чипа)
// ==========================================================================
static bool get_chip_serial(char *buf, size_t buf_size) {
    uint8_t mac[6] = {0};
    if (esp_efuse_mac_get_default(mac) != ESP_OK) {
        snprintf(buf, buf_size, "UNKNOWN");
        return false;
    }
    snprintf(buf, buf_size, "%02X%02X%02X%02X%02X%02X",
             mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    return true;
}

static void log_chip_serial(void) {
    char sn[16];
    if (get_chip_serial(sn, sizeof(sn))) {
        ESP_LOGI(TAG, "Серийный номер ESP32 (SN): %s", sn);
    } else {
        ESP_LOGE(TAG, "Не удалось прочитать серийный номер ESP32");
    }
}

// ==========================================================================
// ВЫВОД ОТВЕТА: дублируем и в USB-консоль, и всем WiFi/TCP-клиентам
// ==========================================================================
static void respond(const char *msg) {
    uart_write_bytes(USB_UART_PORT, msg, strlen(msg));
    wifi_tcp_send(msg);
}

// ==========================================================================
// РАЗБОР И ИСПОЛНЕНИЕ ОДНОЙ КОМАНДЫ
// Источник команды — USB UART0 или WiFi/TCP (обработчик общий).
// Формат: SET1 X / SET2 X / SET X (алиас SET1) / STAT / SN / ZERO.
// ==========================================================================
static void handle_command(char *str) {

    // Команда "sn" – вывод серийного номера чипа
    if (strstr(str, "sn") || strstr(str, "SN")) {
        char sn[16];
        get_chip_serial(sn, sizeof(sn));
        char msg[40];
        snprintf(msg, sizeof(msg), "\r\n>> SN: %s\r\n", sn);
        respond(msg);
        return;
    }

    // Команда "stat" – статус обоих каналов
    if (strstr(str, "stat") || strstr(str, "STAT")) {
        char msg[160];
        snprintf(msg, sizeof(msg),
                 "\r\n>> CH1: P=%.2f SET=%.1f %s | CH2: P=%.2f SET=%.1f %s\r\n",
                 pressure1_kPa, setpoint1_kPa, comp1_on ? "ON" : "OFF",
                 pressure2_kPa, setpoint2_kPa, comp2_on ? "ON" : "OFF");
        respond(msg);
        return;
    }

    // Команда "zero" – калибровка нуля датчика №1 (когда реально атмосфера).
    if (strstr(str, "zero") || strstr(str, "ZERO")) {
        respond("\r\n>> Zeroing pressure sensor 1...\r\n");
        bool ok = performAdvancedZeroCalibration(0.0f, SENSOR_UART_NUM);
        respond(ok ? "\r\n>> [OK] Sensor 1 zeroed\r\n"
                   : "\r\n>> [ERR] Sensor 1 zero failed (no response)\r\n");
        return;
    }

    // Уставка канала 2: SET2 X
    char *ptr2 = strstr(str, "set2 ");
    if (ptr2 == NULL) ptr2 = strstr(str, "SET2 ");
    if (ptr2 != NULL) {
        float v = strtof(ptr2 + 5, NULL);
        update_setpoint(2, v);
        char msg[64];
        snprintf(msg, sizeof(msg), "\r\n>> CH2 target updated to: %.1f kPa\r\n", setpoint2_kPa);
        respond(msg);
        return;
    }

    // Уставка канала 1: SET1 X (и legacy "set X" как алиас)
    char *ptr1 = strstr(str, "set1 ");
    if (ptr1 == NULL) ptr1 = strstr(str, "SET1 ");
    char *ptrL = strstr(str, "set ");
    if (ptrL == NULL) ptrL = strstr(str, "SET ");
    if (ptr1 != NULL || ptrL != NULL) {
        float v = (ptr1 != NULL) ? strtof(ptr1 + 5, NULL) : strtof(ptrL + 4, NULL);
        update_setpoint(1, v);
        char msg[64];
        snprintf(msg, sizeof(msg), "\r\n>> CH1 target updated to: %.1f kPa\r\n", setpoint1_kPa);
        respond(msg);
        return;
    }

    respond("\r\n>> Unknown command. Usage: SET1 X | SET2 X | STAT | SN | ZERO\r\n");
}

// ==========================================================================
// Задача приёма команд с USB-консоли
// ==========================================================================
void usb_uart_rx_task(void *pvParameters) {
    uint8_t *data = (uint8_t *) malloc(USB_BUF_SIZE);
    if (data == NULL) {
        ESP_LOGE(TAG, "Не удалось выделить память для буфера USB UART");
        vTaskDelete(NULL);
        return;
    }

    ESP_LOGI(TAG, "Задача чтения команд через USB UART запущена. Формат: SET1 X / SET2 X / STAT / SN / ZERO");

    while (1) {
        int len = uart_read_bytes(USB_UART_PORT, data, USB_BUF_SIZE - 1, pdMS_TO_TICKS(20));
        if (len > 0) {
            data[len] = '\0';
            handle_command((char *)data);
        }
    }
    free(data);
    vTaskDelete(NULL);
}

// ==========================================================================
// ТОЧКА ВХОДА
// ==========================================================================
void app_main(void) {

    log_chip_serial();

    LCD_init();                       // дисплей + шрифты
    pressure_ui_and_usb_init(&dev);   // USB UART, рамки/подписи, задача экрана
    pressure_sensor_init();           // датчик №1 (UART1) + датчик №2 (UART2)
    compressor_control_init();        // пины реле компрессоров, всё выключено
    compressor_control_start();       // задача управления двумя компрессорами

    // Поднимаем WiFi (AP/STA) и TCP-сервер для приёма команд по сети.
    // Внутри инициализируется NVS. Команды кладутся в очередь и исполняются
    // ниже в этом же цикле — теми же handle_command(), что и USB-команды.
    wifi_tcp_init();

    char wcmd[256];
    while (1) {
        while (wifi_tcp_poll_command(wcmd, sizeof(wcmd))) {
            ESP_LOGI(TAG, "WIFI команда: %s", wcmd);
            handle_command(wcmd);
        }
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}
