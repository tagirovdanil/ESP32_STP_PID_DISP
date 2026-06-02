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

// 2. Ваши собственные модули (интерфейсы управления)
#include "st7789.h"           // Для работы с дисплеем TTGO
#include "pressure_sensor.h"  // Для работы с датчиком давления и калибровкой
#include "control.h"          // Для работы с ПИД-регулятором, сервой и шаговиком

#define USB_UART_PORT       UART_NUM_0
#define USB_BUF_SIZE        256


static const char *TAG = "MAIN_APP";

    void usb_uart_rx_task(void *pvParameters) {
        uint8_t *data = (uint8_t *) malloc(USB_BUF_SIZE);
        if (data == NULL) {
            ESP_LOGE(TAG, "Не удалось выделить память для буфера USB UART");
            vTaskDelete(NULL);
            return;
        }

        ESP_LOGI(TAG, "Задача чтения команд через USB UART запущена. Формат: set X или reset");

        while (1) {
            // Читаем данные из UART_NUM_0. Таймаут 20 мс позволяет задаче «засыпать», если порт пуст
            int len = uart_read_bytes(USB_UART_PORT, data, USB_BUF_SIZE - 1, pdMS_TO_TICKS(20));
            
            if (len > 0) {
                data[len] = '\0'; // Гарантируем корректный нуль-терминатор для работы со строками
                char *str = (char *)data;

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

                    // Отправляем ответ на ПК
                    const char *response_msg = "\r\n>> [OK] Physical pressure reset started...\r\n";
                    uart_write_bytes(USB_UART_PORT, response_msg, strlen(response_msg));
                    
                    // Пропускаем дальнейшие проверки в этой итерации, так как команда обработана
                    continue; 
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
                    
                    // Отправляем эхо-ответ обратно в терминал ПК для подтверждения
                    char response_msg[64];
                    snprintf(response_msg, sizeof(response_msg), "\r\n>> Target updated to: %.1f kPa\r\n", setpoint_kPa);
                    uart_write_bytes(USB_UART_PORT, response_msg, strlen(response_msg));
                }
            }
        }
        free(data);
        vTaskDelete(NULL);
    }


void app_main(void) {
    LCD_init(); 
    pressure_sensor_init(); 
    init_pid_regulator();
    //char screen_buffer[32];
    pressure_ui_and_usb_init(&dev);

    while(1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }

}
