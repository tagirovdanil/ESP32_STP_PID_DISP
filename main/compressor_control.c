#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "config.h"
#include "compressor_control.h"

/*
 * ============================================================================
 *  Двухступенчатая компрессорная установка — автомат управления.
 *
 *  Давления в кПа (1 МПа = 1000 кПа). Порядок приоритетов (алгоритм.txt, р.3):
 *    1) аварийный останов/аппаратная защита (сейчас не задействовано);
 *    2) неисправность датчика;
 *    3) достижение P2_Set -> общий останов;
 *    4) диагностика 2-й ступени (P1 растёт, P2 нет -> повторные запуски);
 *    5) управление K2 (по P2 и P2_Start);
 *    6) управление K1 (гистерезис по P1: 0,8..1,0 МПа).
 * ============================================================================
 */

// Время последнего валидного отсчёта датчиков (мс от старта) — определены в pressure_sensor.c
extern uint32_t lastPressureTime;   // датчик №1 (P1)
extern uint32_t lastPressureTime2;  // датчик №2 (P2)

volatile bool comp1_on = false;
volatile bool comp2_on = false;
volatile int  current_alarm = ALARM_NONE;

// ---- состояние автомата ----
static bool run = false;            // установка запущена (Пуск)
static bool pending_start = false;  // только что нажали Пуск -> включить обе ступени
static int  retry2 = 0;             // счётчик повторных запусков K2
static bool restart2_pause = false; // идёт пауза повторного запуска K2
static uint32_t restart2_t0 = 0;    // начало паузы, мс
// Аварии датчика/ступеней требуют ручного RESET (алгоритм.txt: автозапуск запрещён
// до восстановления и сброса). Пока флаг стоит — Пуск игнорируется.
static bool alarm_latched = false;

// ---- диагностика 2-й ступени ----
static bool diag2_active = false;
static uint32_t diag2_t0 = 0;
static uint32_t warm2_t0 = 0;
static float diag2_p1 = 0.0f;
static float diag2_p2 = 0.0f;

// ---- диагностика 1-й ступени ----
static bool diag1_active = false;
static uint32_t diag1_t0 = 0;
static uint32_t warm1_t0 = 0;
static float diag1_p1 = 0.0f;
static float diag1_p2 = 0.0f;

static uint32_t now_ms(void) { return (uint32_t)(esp_timer_get_time() / 1000ULL); }

static void set_relay(volatile bool *state, uint8_t pin, bool on) {
    if (*state == on) return;
    *state = on;
#if SAFE_NO_PINS
    // Заглушка: состояние в переменной меняем, но физический вывод не трогаем.
    (void)pin;
    (void)on;
#else
    gpio_set_level(pin, on ? 1 : 0);
#endif
}
static void set_k1(bool on) { set_relay(&comp1_on, PIN_COMP1, on); }
static void set_k2(bool on) { set_relay(&comp2_on, PIN_COMP2, on); }
static void both_off(void)  { set_k1(false); set_k2(false); }

static void diag1_reset(void) { diag1_active = false; diag1_t0 = 0; warm1_t0 = 0; }
static void diag2_reset(void) { diag2_active = false; diag2_t0 = 0; warm2_t0 = 0; }

// ============================================================================
//  Основной шаг автомата
// ============================================================================
static void control_step(void) {
    uint32_t now = now_ms();
    float p1 = pressure1_kPa;
    float p2 = pressure2_kPa;

    // --- контроль датчиков: связь + физический диапазон (приоритет 2) ---
    bool link1 = (now - lastPressureTime) < SENSOR_STALE_MS;
    bool link2 = (now - lastPressureTime2) < SENSOR_STALE_MS;
    bool bad1  = !link1 || (p1 < PRESSURE_MIN_KPA || p1 > PRESSURE_MAX_KPA);
    bool bad2  = !link2 || (p2 < PRESSURE_MIN_KPA || p2 > PRESSURE_MAX_KPA);

    // --- штатный останов / не запущена ---
    if (!run) {
        both_off();
        pending_start = false;
        diag1_reset(); diag2_reset();
        return;
    }

    // --- запущена: неисправность датчика -> останов + авария (приоритет 2) ---
    // Латчим аварию: автозапуск запрещён до восстановления датчика и RESET.
    if (bad1 || bad2) {
        current_alarm = ALARM_SENSOR;
        alarm_latched = true;
        run = false;
        both_off();
        pending_start = false;
        diag1_reset(); diag2_reset();
        return;
    }

    // --- достижение уставки: общий останов (приоритет 3) ---
    if (p2 >= p2_set_kPa) {
        both_off();
        retry2 = 0;
        restart2_pause = false;
        pending_start = false;
        diag1_reset(); diag2_reset();
        if (current_alarm == ALARM_NONE) { /* штатный останов */ }
        return;
    }

    // Первый пуск / пуск после общего останова: включаем обе ступени сразу.
    if (pending_start) {
        set_k1(true);
        set_k2(true);
        pending_start = false;
        warm1_t0 = now; warm2_t0 = now;
        diag1_t0 = 0; diag2_t0 = 0; diag1_active = false; diag2_active = false;
    }

    // ================= УПРАВЛЕНИЕ K1 (гистерезис по P1) =================
    // Включение: P1 <= 0,8 МПа. Отключение: P1 >= 1,0 МПа.
    // В полосе 0,8..1,0 состояние не меняем.
    if (p1 <= P1_ON_KPA) {
        set_k1(true);
        if (warm1_t0 == 0) warm1_t0 = now;   // запустили K1 -> начнём отсчёт диагностики
    } else if (p1 >= P1_OFF_KPA) {
        set_k1(false);
        diag1_reset();
    }

    // ================= УПРАВЛЕНИЕ K2 =================
    bool sync = (p2_set_kPa <= P2_SET_MIN_KPA + 0.5f); // уставка ровно 1,0 МПа

    if (sync) {
        // ---- синхронный режим: обе ступени как одна группа по P2 ----
        if (p2 <= P_SYNC_ON_KPA) {
            if (p1 <= P1_OFF_KPA) set_k1(true);
            set_k2(true);
            if (warm2_t0 == 0) warm2_t0 = now;
        } else if (p2 >= P_SYNC_OFF_KPA) {
            set_k1(false);
            set_k2(false);
            retry2 = 0;
        }
        // P1 контролируется отдельно: защита от превышения 1,0 МПа
        if (p1 >= P1_OFF_KPA) set_k1(false);
    } else {
        // ---- нормальный режим (уставка выше 1 МПа) ----
        if (!restart2_pause) {
            if (!comp2_on) {
                float p2_start = p2_set_kPa - P2_START_DROP_KPA;
                if (p2_start < P2_SET_MIN_KPA) p2_start = P2_SET_MIN_KPA;
                if (p2 <= p2_start && retry2 < RETRY2_MAX) {
                    set_k2(true);
                    warm2_t0 = now;
                    diag2_t0 = 0; diag2_active = false;
                }
            } else {
                // K2 включён: держим до P2_Set (выше обрабатывается общий останов).
                // Диагностика роста P2 — ниже.
            }
        } else {
            // пауза повторного запуска ещё идёт — K2 не включаем
        }
    }

    // ================= ДИАГНОСТИКА 2-й ступени =================
    // Условия: K2 под командой, P2 < P2_Set, датчики исправны.
    if (comp2_on && p2 < p2_set_kPa && current_alarm == ALARM_NONE) {
        if (!diag2_active) {
            // после выхода на режим запускаем диагностическое окно
            if (warm2_t0 != 0 && (now - warm2_t0) >= WARMUP_MS) {
                diag2_active = true;
                diag2_t0 = now;
                diag2_p1 = p1;
                diag2_p2 = p2;
            }
        } else if ((now - diag2_t0) >= DIAG_WINDOW_MS) {
            float d1 = p1 - diag2_p1;
            float d2 = p2 - diag2_p2;
            if (d1 >= D_P1_MIN_KPA && d2 < D_P2_MIN_KPA && p2 < p2_set_kPa) {
                // P1 растёт, P2 нет -> 2-я ступень не даёт давления.
                set_k2(false);
                retry2++;
                restart2_pause = true;
                restart2_t0 = now;
                diag2_active = false; diag2_t0 = 0;
                if (retry2 >= RETRY2_MAX) {
                    both_off();
                    current_alarm = ALARM_STAGE2;
                    alarm_latched = true;
                    run = false;          // ждём ручного сброса + Пуск
                    pending_start = false;
                    return;
                }
            } else {
                // нормальный рост P2 -> сбрасываем счётчик повторных запусков
                // (алгоритм.txt, р.9: сброс Retry2 после успешного роста давления)
                if (d2 >= D_P2_MIN_KPA) retry2 = 0;
                diag2_t0 = now;
                diag2_p1 = p1;
                diag2_p2 = p2;
            }
        }
    } else {
        if (!comp2_on) { diag2_active = false; diag2_t0 = 0; }
    }

    // ================= ПАУЗА ПОВТОРНОГО ЗАПУСКА K2 =================
    if (restart2_pause) {
        if ((now - restart2_t0) >= RESTART2_MS) {
            restart2_pause = false;
            // после паузы запускаем K2, если P2 < P2_Set и есть попытки
            if (p2 < p2_set_kPa && retry2 < RETRY2_MAX) {
                set_k2(true);
                warm2_t0 = now;
                diag2_active = false; diag2_t0 = 0;
            }
        }
    }

    // ================= ДИАГНОСТИКА 1-й ступени =================
    // K1 под командой, но ничего не растёт (ни P1, ни P2) -> 1-я ступень не
    // даёт давления. Диагностика по давлению — косвенная (см. алгоритм.txt р.14).
    // Не ведём диагностику вплотную к уставке (рост там и так замедляется) —
    // иначе ложная авария на финише. Порог 0,2 МПа подстраивается при наладке.
    if (comp1_on && p1 < P1_OFF_KPA && p2 < p2_set_kPa - 200.0f && current_alarm == ALARM_NONE) {
        if (!diag1_active) {
            if (warm1_t0 != 0 && (now - warm1_t0) >= WARMUP_MS) {
                diag1_active = true;
                diag1_t0 = now;
                diag1_p1 = p1;
                diag1_p2 = p2;
            }
        } else if ((now - diag1_t0) >= DIAG_WINDOW_MS) {
            float d1 = p1 - diag1_p1;
            float d2 = p2 - diag1_p2;
            if (d1 < D_P1_MIN_KPA && d2 < D_P2_MIN_KPA && p1 < P1_OFF_KPA) {
                both_off();
                current_alarm = ALARM_STAGE1;
                alarm_latched = true;
                run = false;
                pending_start = false;
                return;
            }
            diag1_t0 = now;
            diag1_p1 = p1;
            diag1_p2 = p2;
        }
    } else {
        if (!comp1_on) diag1_reset();
    }
}

// ============================================================================
//  Публичное API
// ============================================================================
void compressor_control_init(void) {
#if SAFE_NO_PINS
    // Заглушка: пины реле K1(GPIO2)/K2(GPIO17) НЕ настраиваются и не переключаются.
    ESP_LOGW("COMP", "SAFE: реле K1(GPIO%d)/K2(GPIO%d) не подключены к выводам",
             PIN_COMP1, PIN_COMP2);
#else
    gpio_config_t out = {
        .pin_bit_mask = (1ULL << PIN_COMP1) | (1ULL << PIN_COMP2),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&out);
    gpio_set_level(PIN_COMP1, 0);
    gpio_set_level(PIN_COMP2, 0);
#endif
    comp1_on = false;
    comp2_on = false;
    current_alarm = ALARM_NONE;
    alarm_latched = false;
    run = false;
    pending_start = false;
    retry2 = 0;
    restart2_pause = false;
}

static void compressor_task(void *arg) {
    while (1) {
        control_step();
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

void compressor_control_start(void) {
    xTaskCreate(compressor_task, "compressor", 4096, NULL, 5, NULL);
}

void compressor_cmd_start(void) {
    // Пуск только после сброса аварии (алгоритм.txt, р.10/12).
    if (alarm_latched || current_alarm != ALARM_NONE) return;
    if (!run) {
        run = true;
        pending_start = true;
        retry2 = 0;
        restart2_pause = false;
        diag1_reset(); diag2_reset();
    }
}

void compressor_cmd_stop(void) {
    run = false;
    pending_start = false;
    both_off();
    diag1_reset(); diag2_reset();
}

void compressor_cmd_reset(void) {
    current_alarm = ALARM_NONE;
    alarm_latched = false;
    retry2 = 0;
    restart2_pause = false;
    diag1_reset(); diag2_reset();
    // run не меняем: после аварии оператор нажимает Пуск.
}

float compressor_set_setpoint_kpa(float v) {
    if (v < P2_SET_MIN_KPA) v = P2_SET_MIN_KPA;
    if (v > P2_SET_MAX_KPA) v = P2_SET_MAX_KPA;
    p2_set_kPa = v;
    return v;
}

bool compressor_is_running(void) { return run; }
int  compressor_get_alarm(void)  { return current_alarm; }
