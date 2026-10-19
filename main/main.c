#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/uart.h"
#include "esp_mac.h"          // Для чтения заводского MAC (серийного номера) из efuse

#include "st7789.h"           // Локальный маленький экран TTGO
#include "pressure_sensor.h"  // Датчики давления
#include "config.h"           // Глобалы: давления, уставка P2, порт дисплея
#include "compressor_control.h"
#include "wifi_tcp.h"         // WiFi (AP/STA) + TCP-сервер для приёма команд по сети

#define KPA_PER_MPA 1000.0f

// ==========================================================================
// ВЫВОД ОТВЕТА: WiFi/TCP-клиентам и внешнему дисплею (UART0). Дисплей в ответ
// может просто игнорировать не-NMEA-строки (его состояние обновляется $SLSD).
// ==========================================================================
static void respond(const char *msg) {
    wifi_tcp_send(msg);
#if !SAFE_NO_PINS
    uart_write_bytes(DISPLAY_UART_NUM, msg, strlen(msg));
#else
    // Заглушка: UART0 (GPIO21/22) не установлен — в порт не пишем.
    (void)msg;
#endif
}

// ==========================================================================
// XOR-контрольная сумма NMEA: XOR байтов между '$' и '*' (не включая их).
// ==========================================================================
static uint8_t nmea_checksum(const char *line) {
    uint8_t cs = 0;
    const char *p = line;
    if (*p == '$') p++;   // по протоколу '$' не входит в XOR
    while (*p && *p != '*' && *p != '\r' && *p != '\n') {
        cs ^= (uint8_t)*p;
        p++;
    }
    return cs;
}

// Разбор кадра от дисплея: $SLSS,<P2_Set МПа>  и  $SLSC,<CMD>
static bool parse_display_nmea(char *line) {
    if (line[0] != '$') return false;
    char *star = strchr(line, '*');
    if (star == NULL) return false;

    // контрольная сумма
    uint8_t cs = nmea_checksum(line);
    unsigned int cs_rx = 0;
    if (sscanf(star + 1, "%02x", &cs_rx) != 1) return false;
    if (cs_rx != cs) return false;

    char type[5] = {0};
    char *p1 = line + 1;
    if (sscanf(p1, "%4[^,]", type) != 1) return false;
    char *rest = strchr(p1, ',');
    if (rest == NULL) return false;
    rest++; // после первой запятой

    if (strcmp(type, "SLSS") == 0) {
        // $SLSS,<P2_Set МПа>
        float mpa = 0.0f;
        if (sscanf(rest, "%f", &mpa) != 1) return false;
        float applied = compressor_set_setpoint_kpa(mpa * KPA_PER_MPA);
        char msg[64];
        snprintf(msg, sizeof(msg), "SET OK %.2f MPa\r\n", applied / KPA_PER_MPA);
        respond(msg);
        return true;
    }

    if (strcmp(type, "SLSC") == 0) {
        char cmd[16] = {0};
        // читаем команду до '*' (иначе %s зацепит "*XX")
        if (sscanf(rest, "%15[^*]", cmd) != 1) return false;
        if (strcmp(cmd, "START") == 0) {
            compressor_cmd_start();
            respond("\r\n>> START\r\n");
        } else if (strcmp(cmd, "STOP") == 0) {
            compressor_cmd_stop();
            respond("\r\n>> STOP\r\n");
        } else if (strcmp(cmd, "RESET") == 0) {
            compressor_cmd_reset();
            respond("\r\n>> RESET\r\n");
        } else {
            return false;
        }
        return true;
    }

    return false;
}

// ==========================================================================
// ТЕКСТОВЫЕ КОМАНДЫ (WiFi и не-NMEA строки с дисплея)
// Формат: START / STOP / RESET / SET <МПа> / STAT / SN / ZERO
// ==========================================================================
static void handle_command(char *str) {

    if (strstr(str, "sn") || strstr(str, "SN")) {
        uint8_t mac[6];
        if (esp_efuse_mac_get_default(mac) != ESP_OK) {
            respond("\r\n>> SN: UNKNOWN\r\n");
            return;
        }
        char msg[48];
        snprintf(msg, sizeof(msg), "\r\n>> SN: %02X%02X%02X%02X%02X%02X\r\n",
                 mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
        respond(msg);
        return;
    }

    if (strstr(str, "stat") || strstr(str, "STAT")) {
        char msg[160];
        snprintf(msg, sizeof(msg),
                 "\r\n>> P1=%.2f P2=%.2f SET=%.2f MPa | K1=%s K2=%s ALM=%d RUN=%s\r\n",
                 pressure1_kPa / KPA_PER_MPA, pressure2_kPa / KPA_PER_MPA,
                 p2_set_kPa / KPA_PER_MPA,
                 comp1_on ? "ON" : "OFF", comp2_on ? "ON" : "OFF",
                 compressor_get_alarm(),
                 compressor_is_running() ? "ON" : "OFF");
        respond(msg);
        return;
    }

    if (strstr(str, "start") || strstr(str, "START")) {
        compressor_cmd_start();
        respond(compressor_is_running() ? "\r\n>> RUNNING\r\n" : "\r\n>> [ERR] alarm active, do RESET first\r\n");
        return;
    }

    if (strstr(str, "stop") || strstr(str, "STOP")) {
        compressor_cmd_stop();
        respond("\r\n>> STOPPED\r\n");
        return;
    }

    if (strstr(str, "reset") || strstr(str, "RESET")) {
        compressor_cmd_reset();
        respond("\r\n>> ALARM RESET\r\n");
        return;
    }

    // SET <МПа> — уставка P2 (1.0 .. 4.0)
    char *ps = strstr(str, "set ");
    if (ps == NULL) ps = strstr(str, "SET ");
    if (ps != NULL) {
        float mpa = strtof(ps + 4, NULL);
        float applied = compressor_set_setpoint_kpa(mpa * KPA_PER_MPA);
        char msg[64];
        snprintf(msg, sizeof(msg), "\r\n>> P2 target set to %.2f MPa\r\n", applied / KPA_PER_MPA);
        respond(msg);
        return;
    }

    if (strstr(str, "zero") || strstr(str, "ZERO")) {
        respond("\r\n>> Zeroing pressure sensor 1...\r\n");
        bool ok = performAdvancedZeroCalibration(0.0f, SENSOR_UART_NUM);
        respond(ok ? "\r\n>> [OK] Sensor 1 zeroed\r\n"
                   : "\r\n>> [ERR] Sensor 1 zero failed\r\n");
        return;
    }

    respond("\r\n>> Usage: START | STOP | RESET | SET <МПа> | STAT | SN | ZERO\r\n");
}

// ==========================================================================
// Задача приёма с внешнего дисплея (UART0, RX=GPIO22): строки NMEA $SLSS/$SLSC
// или текстовые команды. Строка между \r\n.
// ==========================================================================
static void display_rx_task(void *pvParameters) {
    char line[128];
    int  idx = 0;

    while (1) {
        uint8_t b;
        int n = uart_read_bytes(DISPLAY_UART_NUM, &b, 1, pdMS_TO_TICKS(20));
        if (n > 0) {
            if (b == '\n') {
                if (idx > 0) {
                    // убрать возможный \r в конце
                    if (line[idx - 1] == '\r') line[idx - 1] = '\0';
                    else                       line[idx] = '\0';
                    if (line[0] == '$') {
                        parse_display_nmea(line);
                    } else if (line[0] != '\0') {
                        handle_command(line);
                    }
                    idx = 0;
                }
            } else if (b == '\r') {
                // игнорируем, кадр заканчивается по \n
            } else if (idx < (int)sizeof(line) - 1) {
                line[idx++] = (char)b;
            }
        }
    }
    vTaskDelete(NULL);
}

// ==========================================================================
// ТОЧКА ВХОДА
// ==========================================================================
void app_main(void) {

    LCD_init();                     // маленький экран TTGO + шрифты
    pressure_ui_and_usb_init(&dev); // подписи/экран + UART0 (TX21/RX22) на дисплей
    pressure_sensor_init();         // датчик №1 (UART1) + датчик №2 (UART2)

    compressor_control_init();      // реле К1/К2 (GPIO2/GPIO17) — выключены
    compressor_control_start();     // автомат двухступенчатой установки

#if !SAFE_NO_PINS
    xTaskCreate(display_rx_task, "display_rx", 3072, NULL, 6, NULL);
#else
    // Заглушка: UART0/RX (GPIO22) не настроен, приём от панели не запускаем.
    (void)display_rx_task;
    ESP_LOGW("INIT", "SAFE: приём с внешнего дисплея (UART0 RX=GPIO%d) отключён", PIN_DISP_RX);
#endif

    // WiFi (AP/STA) + TCP для приёма команд по сети.
    wifi_tcp_init();

    char wcmd[256];
    while (1) {
        while (wifi_tcp_poll_command(wcmd, sizeof(wcmd))) {
            handle_command(wcmd);
        }
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}
