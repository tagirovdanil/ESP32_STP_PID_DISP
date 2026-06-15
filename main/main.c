// 1. Стандартные системные библиотеки
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "esp_spiffs.h"
#include <string.h>
#include <stdlib.h>
#include "driver/uart.h"
#include "esp_mac.h"          // Для чтения заводского MAC (серийного номера) из efuse

// 2. Ваши собственные модули (интерфейсы управления)
#include "st7789.h"           // Для работы с дисплеем TTGO
#include "pressure_sensor.h"  // Для работы с датчиком давления и калибровкой
#include "config.h"          // Для работы с ПИД-регулятором, сервой и шаговиком
#include "pressure_regulator.h"
#include "wifi_tcp.h"         // WiFi (AP/STA) + TCP-сервер для приёма команд по сети

#define USB_UART_PORT       UART_NUM_0
#define USB_BUF_SIZE        256

TaskHandle_t display_task_handle = NULL;
extern void calibrate_valve_home(void);

static const char *TAG = "MAIN_APP";

// ==========================================================================
// СЕРИЙНЫЙ НОМЕР ESP32 (заводской MAC из efuse — уникален для каждого чипа)
// ==========================================================================
// Формирует строку вида "240AC4123456" в buf. Возвращает true при успехе.
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

// Выводит серийный номер чипа в лог при запуске.
static void log_chip_serial(void) {
    char sn[16];
    if (get_chip_serial(sn, sizeof(sn))) {
        ESP_LOGI(TAG, "Серийный номер ESP32 (SN): %s", sn);
    } else {
        ESP_LOGE(TAG, "Не удалось прочитать серийный номер ESP32");
    }
}

// ==========================================================================
// ВЫВОД ОТВЕТА: дублируем и в USB-консоль, и всем WiFi/TCP-клиентам,
// чтобы пользователь видел подтверждение независимо от источника команды.
// ==========================================================================
static void respond(const char *msg) {
    uart_write_bytes(USB_UART_PORT, msg, strlen(msg));
    wifi_tcp_send(msg);
}

// ==========================================================================
// РАЗБОР И ИСПОЛНЕНИЕ ОДНОЙ КОМАНДЫ
// Источник команды — USB UART0 или WiFi/TCP (обработчик общий, чтобы не
// плодить гонки с глобальным состоянием регулятора). Формат: set X / reset /
// idle / abort / home / sn / zero / range [N] / setrange N.
// ==========================================================================
static void handle_command(char *str) {

    // Обработка команды "idle"
    if (strstr(str, "idle") || strstr(str, "IDLE")) {
        regulator_request_state(REG_STATE_IDLE);
        respond("\r\n>> Switched to IDLE\r\n");
        return;
    }

    // Обработка команды "abort" (то же, что idle)
    if (strstr(str, "abort") || strstr(str, "ABORT")) {
        regulator_request_state(REG_STATE_IDLE);
        respond("\r\n>> Aborted, switched to IDLE\r\n");
        return;
    }

    // Команда "home" – принудительный сброс давления
    if (strstr(str, "home") || strstr(str, "HOME")) {
        calibrate_valve_home();
        respond("\r\n>> Homing started\r\n");
        return;
    }

    // Команда "sn" – вывод серийного номера чипа
    if (strstr(str, "sn") || strstr(str, "SN")) {
        char sn[16];
        get_chip_serial(sn, sizeof(sn));
        char msg[40];
        snprintf(msg, sizeof(msg), "\r\n>> SN: %s\r\n", sn);
        respond(msg);
        return;
    }

    // Команда "zero" – ОБНУЛЕНИЕ (калибровка нуля) самого ДАТЧИКА давления.
    // Сообщает датчику, что текущее физическое давление = 0 (offset = 0).
    // ВАЖНО: запускать только когда в системе реально атмосфера/ноль, иначе
    // занулишь датчик по неверному давлению. Это то же, что кнопка Boot, но по
    // USB/WiFi. На время калибровки регулятор и отрисовка экрана ставятся на паузу
    // (через флаг is_calibrating внутри самой функции). Не путать с "reset",
    // который физически стравливает давление в системе.
    if (strstr(str, "zero") || strstr(str, "ZERO")) {
        respond("\r\n>> Zeroing pressure sensor...\r\n");
        bool ok = performAdvancedZeroCalibration(0.0f, SENSOR_UART_NUM);
        respond(ok ? "\r\n>> [OK] Sensor zeroed\r\n"
                   : "\r\n>> [ERR] Sensor zero failed (no response)\r\n");
        return;
    }

    // ==========================================================================
    // ОБРАБОТКА КОМАНДЫ "SER <n> <angle>" — РУЧНОЕ управление серво-краном.
    // Формат: SER 1 0 / SER 1 90 / SER 1 180 (угол 0..180; серво № пока только 1).
    // Прямо выставляет ШИМ серво — для теста. ВНИМАНИЕ: пока регулятор в RUNNING,
    // он перебьёт это значение на следующем тике, поэтому пользоваться в IDLE.
    // ==========================================================================
    char *ser_ptr = strstr(str, "ser ");
    if (ser_ptr == NULL) ser_ptr = strstr(str, "SER ");
    if (ser_ptr != NULL) {
        int   idx   = 0;
        float angle = 0.0f;
        if (sscanf(ser_ptr + 4, "%d %f", &idx, &angle) == 2) {
            if (idx != 1) {
                respond("\r\n>> [ERR] SER: only servo 1 supported\r\n");
            } else if (angle < 0.0f || angle > 180.0f) {
                respond("\r\n>> [ERR] SER: angle out of range 0..180\r\n");
            } else {
                set_servo_angle(angle);
                char msg[48];
                snprintf(msg, sizeof(msg), "\r\n>> Servo %d -> %.0f deg\r\n", idx, angle);
                respond(msg);
            }
        } else {
            respond("\r\n>> [ERR] SER: format 'SER 1 90'\r\n");
        }
        return;
    }

    // ==========================================================================
    // ОБРАБОТКА КОМАНДЫ "RESET"
    // ==========================================================================
    char *reset_ptr = strstr(str, "reset");
    if (reset_ptr == NULL) {
        reset_ptr = strstr(str, "RESET");
    }

    if (reset_ptr != NULL) {
        // Запускаем процедуру физического сброса давления
        start_pressure_homing();
        respond("\r\n>> [OK] Physical pressure reset started...\r\n");
        return;
    }

    // ==========================================================================
    // КОМАНДА "SETRANGE N" — выбор рабочего диапазона датчика (0..4), протокол 0x23.
    // ВАЖНО: проверяем РАНЬШЕ "range" и "set " — обе они подстроки этой команды.
    // ==========================================================================
    char *setrange_ptr = strstr(str, "setrange");
    if (setrange_ptr == NULL) setrange_ptr = strstr(str, "SETRANGE");
    if (setrange_ptr != NULL) {
        int n = -1;
        if (sscanf(setrange_ptr + 8, "%d", &n) == 1 && n >= 0 && n <= 4) {
            ESP_LOGI(TAG, "Команда SETRANGE %d", n);
            bool ok = SetRange((uint8_t)n, SENSOR_UART_NUM);
            ESP_LOGI(TAG, "SetRange %d -> %s", n, ok ? "OK" : "FAILED (нет/неверный ответ)");
        } else {
            ESP_LOGE(TAG, "SETRANGE: формат 'setrange N' (N=0..4)");
        }
        return;
    }

    // ==========================================================================
    // КОМАНДА "RANGE [N]" — чтение диапазона датчика. Без аргумента — текущий
    // (посылаем 0xFF), N=0..4 — конкретный диапазон. Печатает pmin/pmax.
    // ==========================================================================
    char *range_ptr = strstr(str, "range");
    if (range_ptr == NULL) range_ptr = strstr(str, "RANGE");
    if (range_ptr != NULL) {
        int n = -1;
        uint8_t rb = 0xFF;                      // нет аргумента → текущий диапазон
        if (sscanf(range_ptr + 5, "%d", &n) == 1) {
            if (n < 0 || n > 4) {
                ESP_LOGE(TAG, "RANGE: N должен быть 0..4 (или без аргумента — текущий)");
                return;
            }
            rb = (uint8_t)n;
        }
        ESP_LOGI(TAG, "Команда RANGE 0x%02X", rb);
        float pmax = 0.0f, pmin = 0.0f;
        bool ok = ReadRange(rb, SENSOR_UART_NUM, &pmax, &pmin);
        if (!ok) {
            ESP_LOGE(TAG, "Не удалось прочитать диапазон");
        } else if (rb == 0xFF) {
            ESP_LOGI(TAG, "Текущий диапазон: pmin=%.3f pmax=%.3f", pmin, pmax);
        } else {
            ESP_LOGI(TAG, "Диапазон %d: pmin=%.3f pmax=%.3f", rb, pmin, pmax);
        }
        return;
    }

    // ==========================================================================
    // ОБРАБОТКА КОМАНДЫ "SET X"
    // ==========================================================================
    char *cmd_ptr = strstr(str, "set ");
    if (cmd_ptr == NULL) {
        cmd_ptr = strstr(str, "SET ");
    }

    if (cmd_ptr != NULL) {
        // Извлекаем числовое значение уставки, идущее после ключевого слова "set "
        float target = strtof(cmd_ptr + 4, NULL);

        // Передаем новое значение напрямую в наш ПИД-регулятор через безопасную обертку
        update_setpoint(target);

        // Отправляем эхо-ответ обратно (в терминал ПК и WiFi-клиенту)
        char response_msg[64];
        snprintf(response_msg, sizeof(response_msg), "\r\n>> Target updated to: %.1f kPa\r\n", setpoint_kPa);
        respond(response_msg);
    }
}

void usb_uart_rx_task(void *pvParameters) {
    uint8_t *data = (uint8_t *) malloc(USB_BUF_SIZE);
    if (data == NULL) {
        ESP_LOGE(TAG, "Не удалось выделить память для буфера USB UART");
        vTaskDelete(NULL);
        return;
    }

    ESP_LOGI(TAG, "Задача чтения команд через USB UART запущена. Формат: set X / ser N A / reset / idle / home / sn / range [N] / setrange N");

    while (1) {
        // Читаем данные из UART_NUM_0. Таймаут 20 мс позволяет задаче «засыпать», если порт пуст
        int len = uart_read_bytes(USB_UART_PORT, data, USB_BUF_SIZE - 1, pdMS_TO_TICKS(20));

        if (len > 0) {
            data[len] = '\0'; // Гарантируем корректный нуль-терминатор для работы со строками
            handle_command((char *)data);
        }
    }
    free(data);
    vTaskDelete(NULL);
}


void app_main(void) {

    // Самым первым делом печатаем серийный номер чипа в лог
    log_chip_serial();

    LCD_init();
    pressure_ui_and_usb_init(&dev);
    pressure_sensor_init();
    hardware_setup_and_calibrate();   // пины, серво, хоминг, продувка
    regulator_start_task();           // запуск задачи ПИД

    // Поднимаем WiFi (AP/STA) и TCP-сервер для приёма команд по сети.
    // Внутри инициализируется NVS. Команды кладутся в очередь и исполняются
    // ниже в этом же цикле — теми же handle_command(), что и USB-команды.
    wifi_tcp_init();

    char wcmd[256];
    while (1) {
        // Забираем все накопившиеся WiFi/TCP-команды и исполняем их в основном
        // таске (чтобы не было гонок с командами из USB UART).
        while (wifi_tcp_poll_command(wcmd, sizeof(wcmd))) {
            ESP_LOGI(TAG, "WIFI команда: %s", wcmd);
            handle_command(wcmd);
        }
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}
