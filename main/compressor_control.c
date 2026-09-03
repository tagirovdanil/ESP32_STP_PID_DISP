#include <stdio.h>
#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "config.h"
#include "compressor_control.h"

/*
 * ============================================================================
 *  compressor_control.c — управление двумя компрессорами
 * ============================================================================
 *  Два независимых канала:
 *    канал 1: датчик №1 (pressure1_kPa) -> реле компрессора 1 (PIN_COMP1)
 *    канал 2: датчик №2 (pressure2_kPa) -> реле компрессора 2 (PIN_COMP2)
 *
 *  Закон — гистерезис вокруг уставки:
 *    компрессор ВЫКЛючается, когда P >= SET;
 *    включается, когда P < SET - COMP_HYST_KPA.
 *  При уставке <= 0 канал выключен (безопасный режим).
 */

static const char *TAG = "COMPR";

volatile bool comp1_on = false;
volatile bool comp2_on = false;

// Локальная установка реле; логируем только реальные смены состояния.
static void set_relay(uint8_t pin, volatile bool *state, bool on) {
    if (*state == on) return;
    *state = on;
    gpio_set_level(pin, on ? 1 : 0);
    ESP_LOGI(TAG, "Реле на GPIO%d -> %s", pin, on ? "ВКЛ" : "ВЫКЛ");
}

// Гистерезис одного канала.
static void channel_update(float p, float set, uint8_t pin, volatile bool *state) {
    if (set <= 0.0f) {                    // уставка не задана -> выключен
        set_relay(pin, state, false);
        return;
    }
    if (*state) {                         // работает: ждём достижения уставки
        if (p >= set) set_relay(pin, state, false);
    } else {                              // выключен: включаем при провале ниже зоны
        if (p < set - COMP_HYST_KPA) set_relay(pin, state, true);
    }
}

void compressor_control_init(void) {
    gpio_config_t out = {
        .pin_bit_mask = (1ULL << PIN_COMP1) | (1ULL << PIN_COMP2),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&out);

    // На старте оба компрессора гарантированно выключены (безопасно).
    gpio_set_level(PIN_COMP1, 0);
    gpio_set_level(PIN_COMP2, 0);
    comp1_on = false;
    comp2_on = false;

    ESP_LOGI(TAG, "Реле компрессоров инициализированы (К1=GPIO%d, К2=GPIO%d), выключены.",
             PIN_COMP1, PIN_COMP2);
}

static void compressor_task(void *arg) {
    uint32_t log_cnt = 0;
    while (1) {
        channel_update(pressure1_kPa, setpoint1_kPa, PIN_COMP1, &comp1_on);
        channel_update(pressure2_kPa, setpoint2_kPa, PIN_COMP2, &comp2_on);

        // Лог раз в ~1 с (тик 50 мс)
        if ((++log_cnt % 20) == 0) {
            ESP_LOGI(TAG, "P1=%.2f (set=%.1f) %s | P2=%.2f (set=%.1f) %s",
                     pressure1_kPa, setpoint1_kPa, comp1_on ? "ON" : "OFF",
                     pressure2_kPa, setpoint2_kPa, comp2_on ? "ON" : "OFF");
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

void compressor_control_start(void) {
    xTaskCreate(compressor_task, "compressor", 3072, NULL, 5, NULL);
}
