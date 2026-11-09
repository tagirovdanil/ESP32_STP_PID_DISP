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
 *    6) управление K1 (гистерезис по P1).
 *
 *  Дополнительно (PANEL_PROTOCOL.md):
 *    - режим SERVICE: тест выходов $SLSC,OUT только на «холодной» установке;
 *    - R3 (K3) — автоматическое реле охлаждения: (K1 || K2) + выдержка 60 с;
 *    - R4 (K4) — резерв, штатно всегда выключен (только тест из SERVICE);
 *    - compressor_get_state() — поле state для $SLSD.
 * ============================================================================
 */

// Время последнего валидного отсчёта датчиков (мс от старта) — определены в pressure_sensor.c
extern uint32_t lastPressureTime;   // датчик №1 (P1)
extern uint32_t lastPressureTime2;  // датчик №2 (P2)
// Счётчики ошибок кадров — для диагностики аварии 2 (определены в pressure_sensor.c)
extern uint32_t sum_err;            // ошибки кадров датчика 0x01
extern uint32_t sum_err2;           // ошибки кадров датчика 0x02

volatile bool comp1_on = false;
volatile bool comp2_on = false;
volatile bool comp3_on = false;   // R3 — реле охлаждения (GPIO17)
volatile bool comp4_on = false;   // R4 — резерв (GPIO22)
volatile int  current_alarm = ALARM_NONE;

// ---- состояние автомата ----
static bool run = false;            // установка запущена (Пуск)
static bool pending_start = false;  // только что нажали Пуск -> включить обе ступени
static int  retry2 = 0;             // счётчик повторных запусков K2
static bool restart2_pause = false; // идёт пауза повторного запуска K2
static uint32_t restart2_t0 = 0;    // начало паузы, мс
// После останова по уставке установка ждёт P2 <= P2_Start: K2 не включается.
// K1 при этом продолжает работать по своей петле гистерезиса (700/1000 кПа)
// и выключается только по достижении ВП1 — даже если K2 уже остановлен.
static bool wait_restart = false;
// Аварии датчика/ступеней требуют ручного RESET (алгоритм.txt: автозапуск запрещён
// до восстановления и сброса). Пока флаг стоит — Пуск игнорируется.
static bool alarm_latched = false;
// Была ли принята хотя бы одна уставка ($SLSS/$SLSC,TH) — для WARN «нет уставки».
static bool setpoint_rx = false;

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

// ---- сервисный тест выходов ($SLSC,OUT) ----
static bool service_active = false;
static bool service_k1 = false;
static bool service_k2 = false;
static bool service_r3 = false;
static bool service_r4 = false;
static uint32_t service_t0 = 0;        // вход в сервисный режим
static uint32_t service_comp_t0 = 0;   // начало теста K1/K2
static uint32_t service_r4_t0 = 0;     // начало теста R4

// ---- автоматика реле охлаждения R3 ----
static bool r3_cooling = false;        // идёт выдержка после выключения компрессоров
static bool compressors_were_on = false;
static uint32_t r3_cool_t0 = 0;

// Демо-проверка реле: пока идёт, автомат не трогает выходы
static volatile bool demo_active = false;
static volatile bool demo_stop_req = false;   // запрос корректного завершения демо

static uint32_t now_ms(void) { return (uint32_t)(esp_timer_get_time() / 1000ULL); }

// ---- Ограничение непрерывной работы K1/K2 ----
// Непрерывно дольше COMP_MAX_CONT_RUN_MS -> принудительный отдых COMP_MIN_REST_MS.
// Пока идёт отдых, включить компрессор нельзя (запрос от алгоритма игнорируется).
static uint32_t k1_on_since = 0;      // 0 = K1 выключен
static uint32_t k2_on_since = 0;      // 0 = K2 выключен
static bool     k1_resting = false;
static bool     k2_resting = false;
static uint32_t k1_rest_until = 0;
static uint32_t k2_rest_until = 0;

// Прототипы: используются ограничением непрерывной работы ниже
static void diag1_reset(void);
static void diag2_reset(void);

static void set_relay(volatile bool *state, uint8_t pin, bool on) {
    if (*state == on) return;
    *state = on;
#if SAFE_NO_PINS
    // Заглушка: состояние в переменной меняем, но физический вывод не трогаем.
    (void)pin;
    (void)on;
#else
    // Реле с оптопарой управляются реверсивно: LOW = ВКЛ (RELAY_ACTIVE_LOW = 1)
#if RELAY_ACTIVE_LOW
    gpio_set_level(pin, on ? 0 : 1);
#else
    gpio_set_level(pin, on ? 1 : 0);
#endif
#endif
}
// Обёртки K1/K2 дополнительно:
//  - не дают включить компрессор во время обязательного отдыха;
//  - ведут отсчёт непрерывной работы (нужен для ограничения 10 мин).
static void set_k1(bool on) {
    if (on) {
        if (k1_resting) return;               // отдых — включение запрещено
        if (!comp1_on) k1_on_since = now_ms();
    } else {
        k1_on_since = 0;
    }
    set_relay(&comp1_on, PIN_COMP1, on);
}

static void set_k2(bool on) {
    if (on) {
        if (k2_resting) return;
        if (!comp2_on) k2_on_since = now_ms();
    } else {
        k2_on_since = 0;
    }
    set_relay(&comp2_on, PIN_COMP2, on);
}

static void set_k3(bool on) { set_relay(&comp3_on, PIN_COMP3, on); }
static void set_k4(bool on) { set_relay(&comp4_on, PIN_COMP4, on); }

// ============================================================================
//  Ограничение непрерывной работы: 10 минут работы -> минимум 5 минут отдыха
//  (по каждому компрессору отдельно). Проверяется каждый такт.
// ============================================================================
static void update_runtime_limits(void) {
    uint32_t now = now_ms();

    if (comp1_on && k1_on_since != 0 &&
        (uint32_t)(now - k1_on_since) >= COMP_MAX_CONT_RUN_MS) {
        ESP_LOGW("COMP", "K1 работал непрерывно %u мин — принудительный отдых %u мин",
                 (unsigned)(COMP_MAX_CONT_RUN_MS / 60000u),
                 (unsigned)(COMP_MIN_REST_MS / 60000u));
        set_k1(false);
        k1_resting = true;
        k1_rest_until = now + COMP_MIN_REST_MS;
        diag1_reset();
    }
    if (comp2_on && k2_on_since != 0 &&
        (uint32_t)(now - k2_on_since) >= COMP_MAX_CONT_RUN_MS) {
        ESP_LOGW("COMP", "K2 работал непрерывно %u мин — принудительный отдых %u мин",
                 (unsigned)(COMP_MAX_CONT_RUN_MS / 60000u),
                 (unsigned)(COMP_MIN_REST_MS / 60000u));
        set_k2(false);
        k2_resting = true;
        k2_rest_until = now + COMP_MIN_REST_MS;
        restart2_pause = false;
        diag2_reset();
    }

    if (k1_resting && (int32_t)(now - k1_rest_until) >= 0) {
        k1_resting = false;
        ESP_LOGI("COMP", "K1: отдых окончен (%u мин), компрессор снова доступен",
                 (unsigned)(COMP_MIN_REST_MS / 60000u));
    }
    if (k2_resting && (int32_t)(now - k2_rest_until) >= 0) {
        k2_resting = false;
        ESP_LOGI("COMP", "K2: отдых окончен (%u мин), компрессор снова доступен",
                 (unsigned)(COMP_MIN_REST_MS / 60000u));
    }
}

// Полное выключение всех четырёх выходов (демо, аварийный сброс теста).
static void all_off(void)  { set_k1(false); set_k2(false); set_k3(false); set_k4(false); }
// Останов компрессоров по алгоритму: трогаем ТОЛЬКО K1/K2. R3 (охлаждение)
// выключает автоматика update_r3_r4() после выдержки — иначе реле «мигает»,
// погаснув на одном такте и включившись снова для охлаждения.
static void both_off(void) { set_k1(false); set_k2(false); }

static void diag1_reset(void) { diag1_active = false; diag1_t0 = 0; warm1_t0 = 0; }
static void diag2_reset(void) { diag2_active = false; diag2_t0 = 0; warm2_t0 = 0; }

// ============================================================================
//  Синхронный режим: уставка ровно 1,0 МПа (алгоритм.txt, р.7)
// ============================================================================
static bool is_sync_setpoint(float set_kpa) {
    return (set_kpa >= P_SYNC_SET_KPA - 0.5f) && (set_kpa <= P_SYNC_SET_KPA + 0.5f);
}

// ============================================================================
//  Автоматика реле охлаждения R3 и резерва R4 (PANEL_PROTOCOL.md §7.1)
//  R3 = (K1 || K2), после выключения ОБОИХ держим ещё T_COOL_R3_MS.
//  Повторный пуск отменяет таймер без разрыва питания реле.
// ============================================================================
static void update_r3_r4(void) {
    uint32_t now = now_ms();

    if (comp1_on || comp2_on) {
        compressors_were_on = true;
        r3_cooling = false;
        set_k3(true);
    } else if (compressors_were_on) {
        if (!r3_cooling) {                 // переход (K1|K2): 1 -> 0
            r3_cooling = true;
            r3_cool_t0 = now;
        }
        if ((uint32_t)(now - r3_cool_t0) < T_COOL_R3_MS) {
            set_k3(true);                  // охлаждаем ещё T_COOL_R3_MS
        } else {
            set_k3(false);
            r3_cooling = false;
            compressors_were_on = false;
        }
    } else {
        set_k3(false);
    }

    set_k4(false);                          // R4 — резерв, штатно всегда выключен
}

// ============================================================================
//  Выход из сервисного режима: снимаем тестовые включения
// ============================================================================
static void service_exit(const char *why) {
    if (!service_active) return;
    service_active = false;
    service_k1 = service_k2 = service_r3 = service_r4 = false;
    set_k1(false);
    set_k2(false);
    set_k3(false);
    set_k4(false);
    ESP_LOGW("SERVICE", "Тест выходов снят (%s): K1..K4 = OFF", why ? why : "причина не указана");
}

// ============================================================================
//  Шаг сервисного режима: контроль условий, таймауты, выдача выходов
// ============================================================================
static void service_step(void) {
    uint32_t now = now_ms();

    // Условия сервисного режима: STOP, без аварии, «холодная» установка, датчики живы
    bool link1 = (now - lastPressureTime) < SENSOR_STALE_MS;
    bool link2 = (now - lastPressureTime2) < SENSOR_STALE_MS;
    if (run || alarm_latched || current_alarm != ALARM_NONE) {
        service_exit("поступил START или авария");
        return;
    }
    if (!link1 || !link2) {
        service_exit("нет связи с датчиками");
        return;
    }
    if (pressure1_kPa >= SERVICE_MAX_P_KPA || pressure2_kPa >= SERVICE_MAX_P_KPA) {
        service_exit("давление выше 0,1 МПа");
        return;
    }

    // Таймаут теста компрессоров: «забытый» включённый K1/K2 недопустим
    if ((service_k1 || service_k2) &&
        (uint32_t)(now - service_comp_t0) >= T_TEST_COMP_MS) {
        service_k1 = false;
        service_k2 = false;
        set_k1(false);
        set_k2(false);
        ESP_LOGE("SERVICE", "Таймаут теста компрессоров %d мс — K1/K2 выключены",
                 T_TEST_COMP_MS);
    }

    // Автоотключение теста резервного R4
    if (service_r4 && (uint32_t)(now - service_r4_t0) >= T_TEST_R4_MS) {
        service_r4 = false;
        set_k4(false);
        ESP_LOGW("SERVICE", "Таймаут теста R4 %d мс — реле 4 выключено", T_TEST_R4_MS);
    }

    set_k1(service_k1);
    set_k2(service_k2);
    set_k3(service_r3);
    set_k4(service_r4);
}

// ============================================================================
//  Петля гистерезиса K1 по P1 (одна на все режимы).
//  Вкл при P1 <= НП1 (700 кПа), выкл при P1 >= ВП1 (1000 кПа) — 300 кПа.
//  Используется и в работе, и при ожидании P2 <= P2_Start, чтобы первая
//  ступень всегда добирала P1 до ВП1 перед выключением.
// ============================================================================
static void k1_hysteresis_step(uint32_t now, float p1) {
    if (p1 <= P1_ON_KPA) {
        set_k1(true);
        if (warm1_t0 == 0) warm1_t0 = now;   // запустили K1 -> отсчёт диагностики
    } else if (p1 >= P1_OFF_KPA) {
        set_k1(false);
        diag1_reset();
    }
}

// ============================================================================
//  Разрешение пуска K2 по давлению 1-й ступени.
//  Требование заказчика (2026-11-09): при уставке выше 1 МПа вторую ступень
//  НЕ включать, пока P1 не набран хотя бы до 700 кПа (НП1 гистерезиса K1) —
//  иначе K2 работает «в пустоту» на низком P1.
//  В синхронном режиме (уставка 1,0 МПа) P1 и P2 практически равны, поэтому
//  ограничение не применяется.
//  Гейт действует только на ВКЛЮЧЕНИЕ: уже работающий K2 им не выключается.
// ============================================================================
static bool k2_wait_p1_logged = false;

static bool k2_p1_ready(float p1) {
    if (!run) {
        k2_wait_p1_logged = false;
        return true;
    }
    if (is_sync_setpoint(p2_set_kPa)) {
        k2_wait_p1_logged = false;   // в синхронном режиме гейт не нужен
        return true;
    }

    bool ready = (p1 >= K2_START_P1_KPA);
    if (!ready && !k2_wait_p1_logged) {
        k2_wait_p1_logged = true;
        ESP_LOGI("COMP", "K2 удерживается: P1=%.3f < %.2f МПа — ждём набор 1-й ступенью",
                 p1 / 1000.0f, K2_START_P1_KPA / 1000.0f);
    } else if (ready && k2_wait_p1_logged) {
        k2_wait_p1_logged = false;
        ESP_LOGI("COMP", "P1=%.3f >= %.2f МПа — пуск K2 разрешён",
                 p1 / 1000.0f, K2_START_P1_KPA / 1000.0f);
    }
    return ready;
}

// ============================================================================
//  Основной шаг автомата (без сервиса/демо)
// ============================================================================
static void control_body(void) {
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
        wait_restart = false;
        diag1_reset(); diag2_reset();
        return;
    }

    // --- запущена: неисправность датчика -> останов + авария (приоритет 2) ---
    // Латчим аварию: автозапуск запрещён до восстановления датчика и RESET.
    if (bad1 || bad2) {
        // Причина срабатывания — один раз при входе в аварию (не заливаем лог)
        if (current_alarm != ALARM_SENSOR) {
            bool range1 = (p1 < PRESSURE_MIN_KPA || p1 > PRESSURE_MAX_KPA);
            bool range2 = (p2 < PRESSURE_MIN_KPA || p2 > PRESSURE_MAX_KPA);
            ESP_LOGE("COMP",
                     "ALM=2 (датчик): link1=%d P1=%.3f МПа%s age1=%u мс err1=%u | "
                     "link2=%d P2=%.3f МПа%s age2=%u мс err2=%u | допуск %.1f..%.1f кПа",
                     link1 ? 1 : 0, p1 / 1000.0f, range1 ? " [ВНЕ ДИАПАЗОНА]" : "",
                     (unsigned)(now - lastPressureTime), (unsigned)sum_err,
                     link2 ? 1 : 0, p2 / 1000.0f, range2 ? " [ВНЕ ДИАПАЗОНА]" : "",
                     (unsigned)(now - lastPressureTime2), (unsigned)sum_err2,
                     PRESSURE_MIN_KPA, PRESSURE_MAX_KPA);
        }
        current_alarm = ALARM_SENSOR;
        alarm_latched = true;
        run = false;
        both_off();
        pending_start = false;
        wait_restart = false;
        diag1_reset(); diag2_reset();
        return;
    }

    // --- достижение уставки: останов ТОЛЬКО K2 (приоритет 3) ---
    // K2 выключается сразу. K1 продолжает работать по своей петле гистерезиса
    // (700/1000 кПа) и выключается только на ВП1 — даже если вторая ступень
    // уже остановилась.
    if (p2 >= p2_set_kPa) {
        set_k2(false);
        retry2 = 0;
        restart2_pause = false;
        pending_start = false;
        diag2_reset();
        if (!wait_restart) {
            wait_restart = true;
            ESP_LOGI("COMP",
                     "P2=%.3f >= P2_Set=%.3f МПа — K2 OFF, K1 добирает P1 до %.2f МПа; "
                     "жду P2 <= %.3f МПа для повторного пуска",
                     p2 / 1000.0f, p2_set_kPa / 1000.0f, P1_OFF_KPA / 1000.0f,
                     p2_start_kPa / 1000.0f);
        }
        k1_hysteresis_step(now, p1);
        return;
    }

    // --- ждём P2 <= P2_Start: K2 заблокирован, K1 работает по гистерезису ---
    if (wait_restart) {
        if (p2 <= p2_start_kPa) {
            wait_restart = false;
            // Повторный пуск после общего останова: обе ступени стартуют вместе
            // (алгоритм.txt п.11) и в тандеме набирают P1 до ВП1.
            pending_start = true;
            ESP_LOGI("COMP", "P2=%.3f <= P2_Start=%.3f МПа — разрешён повторный пуск",
                     p2 / 1000.0f, p2_start_kPa / 1000.0f);
        } else {
            set_k2(false);
            diag2_reset();
            k1_hysteresis_step(now, p1);
            return;
        }
    }

    // Первый пуск / пуск после общего останова.
    // K1 не включаем, если P1 уже на ВП1 (иначе короткий импульс реле).
    // K2 не включаем, если P2 уже на уставке, и — при уставке выше 1 МПа —
    // только после набора P1 >= 700 кПа (k2_p1_ready): иначе вторая ступень
    // стартует «в пустоту» на низком P1. Позже её включит обычный блок
    // управления K2, как только P1 дорастёт до порога.
    if (pending_start) {
        pending_start = false;
        if (p1 < P1_OFF_KPA) set_k1(true);
        if (p2 < p2_set_kPa && k2_p1_ready(p1)) set_k2(true);
        warm1_t0 = now; warm2_t0 = now;
        diag1_t0 = 0; diag2_t0 = 0; diag1_active = false; diag2_active = false;
    }

    // ================= УПРАВЛЕНИЕ K1 (гистерезис по P1) =================
    // Включение: P1 <= НП1 (700 кПа). Отключение: P1 >= ВП1 (1000 кПа).
    // Гистерезис 300 кПа — в полосе 700..1000 состояние не меняем.
    k1_hysteresis_step(now, p1);

    // ================= УПРАВЛЕНИЕ K2 =================
    bool sync = is_sync_setpoint(p2_set_kPa);

    if (sync) {
        // ---- синхронный режим: обе ступени как одна группа по P2 ----
        if (p2 <= P_SYNC_ON_KPA) {
            // Гистерезис K1 тот же: включаем только от НП1 (700 кПа),
            // иначе около 1,0 МПа реле начинает «дребезжать».
            if (p1 <= P1_ON_KPA) set_k1(true);
            set_k2(true);
            if (warm2_t0 == 0) warm2_t0 = now;
        } else if (p2 >= P_SYNC_OFF_KPA) {
            set_k1(false);
            set_k2(false);
            retry2 = 0;
        }
        // P1 контролируется отдельно: защита от превышения ВП1
        if (p1 >= P1_OFF_KPA) set_k1(false);
    } else {
        // ---- нормальный режим (уставка выше 1 МПа) ----
        // Порог повторного включения берём из p2_start_kPa: его задаёт панель
        // вторым полем $SLSS,<P2_Set>,<P2_Start> (см. compressor_set_pressure_task_kpa).
        if (!restart2_pause) {
            if (!comp2_on) {
                // Включение только ниже порога, ниже уставки и после набора P1
                if (p2 <= p2_start_kPa && p2 < p2_set_kPa && retry2 < RETRY2_MAX &&
                    k2_p1_ready(p1)) {
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
            // после паузы запускаем K2, если P2 < P2_Set, есть попытки и набран P1
            if (p2 < p2_set_kPa && retry2 < RETRY2_MAX && k2_p1_ready(p1)) {
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
//  Жёсткий предел перепуска (PANEL_PROTOCOL.md §7.2)
//  P2 >= P2_Set + P2_OVERSHOOT_KPA -> K1=K2=OFF + авария 5 (latched до RESET).
//  Проверяется КАЖДЫЙ такт до всей остальной логики: в RUN, STOP, SERVICE, демо.
// ============================================================================
static bool overshoot_check(void) {
    // Работаем по последнему значению датчика: даже при потере связи давление
    // не должно «убегать» выше уставки.
    if (pressure2_kPa < p2_set_kPa + P2_OVERSHOOT_KPA) return false;

    if (current_alarm != ALARM_OVERSHOOT) {
        ESP_LOGE("COMP",
                 "ALM=5 ПЕРЕПУСК: P2=%.3f МПа >= уставка %.3f + %.0f кПа — аварийный останов K1/K2",
                 pressure2_kPa / 1000.0f, p2_set_kPa / 1000.0f, P2_OVERSHOOT_KPA);
    }
    current_alarm = ALARM_OVERSHOOT;
    alarm_latched = true;
    run = false;
    pending_start = false;
    wait_restart = false;
    retry2 = 0;
    restart2_pause = false;
    diag1_reset(); diag2_reset();
    service_exit("перепуск по P2");
    if (compressor_relay_demo_active()) compressor_relay_demo_stop();
    both_off();          // K1/K2 выключаем немедленно; R3 — по автоматике
    update_r3_r4();
    return true;
}

// ============================================================================
//  Тик автомата: перепуск -> демо -> сервис -> штатный алгоритм + R3/R4
// ============================================================================
static void control_step(void) {
    if (overshoot_check()) return;

    // Ограничение непрерывной работы: 10 мин -> принудительный отдых 5 мин.
    // Проверяем до алгоритма, чтобы включать компрессор в этом такте уже нельзя было.
    update_runtime_limits();

    // Демо реле: автомат не должен перезаписывать состояния выходов
    if (demo_active) return;

    if (service_active) {
        service_step();
        return;
    }

    control_body();
    update_r3_r4();
}

static void compressor_task(void *arg) {
    uint32_t tick = 0;
    while (1) {
        control_step();

        // Раз в секунду — сводка состояния (PANEL_PROTOCOL.md §11)
        if ((++tick % 50u) == 0) {
            // Связь с панелью: был ли хоть один байт за DISPLAY_LINK_MS
            uint32_t now_ms = (uint32_t)(esp_timer_get_time() / 1000);
            bool panel_ok = (panel_last_rx_ms != 0) &&
                            ((uint32_t)(now_ms - panel_last_rx_ms) <= DISPLAY_LINK_MS);
            // Возраст последней ПРИМЕНЁННОЙ уставки от панели, с (0 = не было)
            uint32_t set_age_s = 0;
            if (panel_slss_last_ms != 0) {
                set_age_s = (uint32_t)(now_ms - panel_slss_last_ms) / 1000u;
            }
            ESP_LOGI("STATE",
                     "P1=%.2f P2=%.2f Set=%.2f Start=%.2f K1=%d K2=%d R3=%d alarm=%d state=%d "
                     "retry2=%d panel=%s src=%s rx=%u slss=%u th=%u set_age=%us",
                     pressure1_kPa / 1000.0f, pressure2_kPa / 1000.0f,
                     p2_set_kPa / 1000.0f, p2_start_kPa / 1000.0f,
                     comp1_on ? 1 : 0, comp2_on ? 1 : 0, comp3_on ? 1 : 0,
                     current_alarm, compressor_get_state(), retry2,
                     panel_ok ? "ON" : "OFF",
                     setpoint_rx ? "PANEL" : "DEFAULT",
                     (unsigned)panel_rx_frames_total,
                     (unsigned)panel_slss_count,
                     (unsigned)panel_th_count,
                     (unsigned)set_age_s);
        }

        // Раз в 5 секунд напоминаем, что уставку от панели мы так и не видели.
        // NVS на этой плате нет: уставку хранит дисплейный контроллер.
        if (!setpoint_rx && (tick % 250u) == 0) {
            ESP_LOGW("COMP",
                     "Уставка от панели не приходила ни разу — работаю на умолчаниях в RAM "
                     "(Set=%.2f Start=%.2f МПа). Проверьте линию RX (GPIO%d)",
                     p2_set_kPa / 1000.0f, p2_start_kPa / 1000.0f, PIN_DISP_RX);
        }

        vTaskDelay(pdMS_TO_TICKS(CONTROL_TICK_MS));
    }
}

// ============================================================================
//  Публичное API
// ============================================================================
void compressor_control_init(void) {
#if SAFE_NO_PINS
    // Заглушка: пины реле K1..K4 НЕ настраиваются и не переключаются.
    ESP_LOGW("COMP", "SAFE: реле K1..K4 (GPIO%d/%d/%d/%d) не подключены к выводам",
             PIN_COMP1, PIN_COMP2, PIN_COMP3, PIN_COMP4);
#else
    gpio_config_t out = {
        .pin_bit_mask = (1ULL << PIN_COMP1) | (1ULL << PIN_COMP2) |
                        (1ULL << PIN_COMP3) | (1ULL << PIN_COMP4),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    // Сначала пишем «выключено» в регистр выхода, и только потом переводим
    // выводы в OUTPUT: при активном LOW «выключено» = HIGH, поэтому реле
    // не дёргаются в момент инициализации.
    const int off_level = RELAY_ACTIVE_LOW ? 1 : 0;
    gpio_set_level(PIN_COMP1, off_level);
    gpio_set_level(PIN_COMP2, off_level);
    gpio_set_level(PIN_COMP3, off_level);
    gpio_set_level(PIN_COMP4, off_level);

    gpio_config(&out);

    ESP_LOGI("COMP", "Реле: K1=GPIO%d K2=GPIO%d K3(R3)=GPIO%d K4(R4)=GPIO%d (активный %s)",
             PIN_COMP1, PIN_COMP2, PIN_COMP3, PIN_COMP4,
             RELAY_ACTIVE_LOW ? "LOW" : "HIGH");
#endif
    comp1_on = false;
    comp2_on = false;
    comp3_on = false;
    comp4_on = false;
    current_alarm = ALARM_NONE;
    alarm_latched = false;
    run = false;
    pending_start = false;
    retry2 = 0;
    restart2_pause = false;
    service_active = false;
    service_k1 = service_k2 = service_r3 = service_r4 = false;
    r3_cooling = false;
    compressors_were_on = false;
}

void compressor_control_start(void) {
    xTaskCreate(compressor_task, "compressor", 4096, NULL, 5, NULL);
}

bool compressor_cmd_start(void) {
    // Пуск всегда отменяет демо-цикл и сервисный тест: иначе автомат заблокирован
    // и кажется, что START снова «включает демо».
    if (compressor_relay_demo_active()) {
        ESP_LOGW("COMP", "START: демо реле активно — останавливаю демо");
        compressor_relay_demo_stop();
    }
    service_exit("поступил START");

    // Пуск только после сброса аварии (алгоритм.txt, р.10/12).
    if (alarm_latched || current_alarm != ALARM_NONE) {
        ESP_LOGW("COMP", "START ОТКЛОНЁН: активна авария %d (нужен RESET, затем START)",
                 current_alarm);
        return false;
    }
    if (!run) {
        run = true;
        retry2 = 0;
        restart2_pause = false;
        wait_restart = false;
        diag1_reset(); diag2_reset();

        if (pressure2_kPa >= p2_set_kPa) {
            // PANEL_PROTOCOL.md §7.3: не включать компрессоры,
            // режим «ждём P2 <= P2_Start», ответ панели всё равно OK
            pending_start = false;
            ESP_LOGW("COMP", "START ignored: P2 (%.2f) >= set (%.2f) — жду P2 <= %.2f МПа",
                     pressure2_kPa / 1000.0f, p2_set_kPa / 1000.0f, p2_start_kPa / 1000.0f);
        } else {
            pending_start = true;
            ESP_LOGI("COMP", "START принят: run=true, аварий нет, P1=%.3f P2=%.3f МПа (set=%.3f start=%.3f)",
                     pressure1_kPa / 1000.0f, pressure2_kPa / 1000.0f,
                     p2_set_kPa / 1000.0f, p2_start_kPa / 1000.0f);
        }
    } else {
        // Повторный START идемпотентен: таймеры T_Restart2/T_Diag не сбрасываем.
        ESP_LOGI("COMP", "START: установка уже запущена (run=true)");
    }
    return true;
}

void compressor_cmd_stop(void) {
    compressor_relay_demo_stop();   // STOP также останавливает демо реле
    service_exit("поступил STOP");
    run = false;
    pending_start = false;
    wait_restart = false;
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

bool compressor_set_pressure_task_kpa(float set_kpa, float start_kpa,
                                      float *applied_set, float *applied_start) {
    // 1) Диапазоны панели (PANEL_PROTOCOL.md §4.1)
    if (!(set_kpa >= P2_SET_MIN_KPA && set_kpa <= P2_SET_MAX_KPA)) {
        ESP_LOGW("COMP", "SLSS отклонён: P2_Set=%.3f МПа вне %.2f..%.2f МПа",
                 set_kpa / 1000.0f, P2_SET_MIN_KPA / 1000.0f, P2_SET_MAX_KPA / 1000.0f);
        return false;
    }
    if (!(start_kpa >= P2_START_MIN_KPA)) {
        ESP_LOGW("COMP", "SLSS отклонён: P2_Start=%.3f МПа ниже %.2f МПа",
                 start_kpa / 1000.0f, P2_START_MIN_KPA / 1000.0f);
        return false;
    }
    if (start_kpa > set_kpa - P2_MIN_GAP_KPA + 0.01f) {
        ESP_LOGW("COMP", "SLSS отклонён: P2_Start=%.3f >= P2_Set=%.3f (зазор < %.0f кПа)",
                 start_kpa / 1000.0f, set_kpa / 1000.0f, P2_MIN_GAP_KPA);
        return false;
    }

    // 2) Физические ограничения: уставка выше 4,0 МПа для датчика P2 недостижима
    float set = set_kpa;
    if (set > P2_SET_APPLY_MAX_KPA) {
        ESP_LOGW("COMP", "P2_Set %.2f МПа ограничена до %.2f МПа (предел датчика P2)",
                 set_kpa / 1000.0f, P2_SET_APPLY_MAX_KPA / 1000.0f);
        set = P2_SET_APPLY_MAX_KPA;
    }
    float start = start_kpa;
    if (start > set - P2_MIN_GAP_KPA) start = set - P2_MIN_GAP_KPA;
    if (start < P2_START_MIN_KPA) start = P2_START_MIN_KPA;

    // 3) Смена режима (синхронный <-> обычный) сбрасывает попытки повторного пуска
    bool was_sync = is_sync_setpoint(p2_set_kPa);
    bool now_sync = is_sync_setpoint(set);
    if (was_sync != now_sync) {
        retry2 = 0;
        restart2_pause = false;
        diag2_reset();
        ESP_LOGW("COMP", "Уставка %.2f МПа -> %.2f МПа: смена синхронного режима, Retry2=0",
                 p2_set_kPa / 1000.0f, set / 1000.0f);
    }

    // 4) Атомарное применение: оба значения рядом, без «полуприменённого» состояния
    p2_set_kPa = set;
    p2_start_kPa = start;
    setpoint_rx = true;

    // 5) Новая уставка ниже текущего P2 — это не STOP: общий останов выполнит
    //    такт автомата (p2 >= set), установка перейдёт в ожидание P2 <= P2_Start.
    if (pressure2_kPa >= set) {
        ESP_LOGW("COMP", "Новая уставка %.3f МПа ниже текущего P2=%.3f МПа — останов по уставке",
                 set / 1000.0f, pressure2_kPa / 1000.0f);
    }

    if (applied_set)   *applied_set = set;
    if (applied_start) *applied_start = start;
    return true;
}

float compressor_set_setpoint_kpa(float v) {
    if (v < P2_SET_MIN_KPA) v = P2_SET_MIN_KPA;
    if (v > P2_SET_MAX_KPA) v = P2_SET_MAX_KPA;

    // Старый/упрощённый формат: P2_Start = MAX(1.0; Set - 0.4), но с зазором.
    float st = v - P2_START_DROP_KPA;
    if (st < P_SYNC_SET_KPA) st = P_SYNC_SET_KPA;
    if (st > v - P2_MIN_GAP_KPA) st = v - P2_MIN_GAP_KPA;
    if (st < P2_START_MIN_KPA) st = P2_START_MIN_KPA;

    float applied = 0.0f;
    if (!compressor_set_pressure_task_kpa(v, st, &applied, NULL)) {
        // Сюда попасть не должны: v и st уже приведены к диапазонам.
        ESP_LOGE("COMP", "SET %.3f МПа не применён (внутренняя ошибка диапазонов)", v / 1000.0f);
        return p2_set_kPa;
    }
    return applied;
}

// $SLSC,TH — СПРАВОЧНЫЙ кадр (PANEL_PROTOCOL.md §4.2):
// пользовательских порогов больше нет, уставку задаёт только $SLSS.
// Функция ничего не применяет и не сохраняет: только проверяет корректность
// значений и сверяет их с тем, что уже пришло в $SLSS.
bool compressor_check_thresholds_kpa(float s1_np, float s1_vp, float s2_np, float s2_vp) {
    if (!(s1_np >= P_TH_MIN_KPA && s1_vp <= P_TH_MAX_KPA &&
          s1_vp >= s1_np + P_TH_MIN_GAP_KPA)) {
        ESP_LOGW("COMP", "TH отклонён: ступень 1 НП=%.0f ВП=%.0f кПа", s1_np, s1_vp);
        return false;
    }
    if (!(s2_np >= P_TH_MIN_KPA && s2_vp <= P_TH_MAX_KPA &&
          s2_vp >= s2_np + P_TH_MIN_GAP_KPA)) {
        ESP_LOGW("COMP", "TH отклонён: ступень 2 НП=%.0f ВП=%.0f кПа", s2_np, s2_vp);
        return false;
    }

    // Пороги 1-й ступени фиксированы алгоритмом — кадр лишь должен с ними совпадать
    if (s1_np < P1_ON_KPA - 0.5f || s1_np > P1_ON_KPA + 0.5f ||
        s1_vp < P1_OFF_KPA - 0.5f || s1_vp > P1_OFF_KPA + 0.5f) {
        // Панель может ещё присылать старые 600/1000; мастер работает по своим
        ESP_LOGI("COMP", "TH: S1 %.0f/%.0f кПа (в кадре) — у мастера фиксированы "
                 "%.0f/%.0f кПа, гистерезис 300 кПа; работаю по своим",
                 s1_np, s1_vp, P1_ON_KPA, P1_OFF_KPA);
    }

    // Пороги 2-й ступени в кадре — копия ручек главного экрана ($SLSS)
    float d_set   = s2_vp - p2_set_kPa;
    float d_start = s2_np - p2_start_kPa;
    if (d_set < -0.5f || d_set > 0.5f || d_start < -0.5f || d_start > 0.5f) {
        ESP_LOGW("COMP", "TH: S2 %.0f/%.0f кПа не совпадает с $SLSS %.0f/%.0f кПа — "
                 "TH справочный, уставка не меняется", s2_np, s2_vp, p2_start_kPa, p2_set_kPa);
    }

    if (!setpoint_rx) {
        ESP_LOGW("COMP", "TH пришёл раньше $SLSS: уставка остаётся по умолчанию в RAM — "
                 "TH не применяется, нужен кадр $SLSS");
    }

    ESP_LOGI("COMP", "TH (справочно): S1=%.0f/%.0f S2=%.0f/%.0f кПа; применены только $SLSS "
             "(set=%.0f start=%.0f кПа)",
             s1_np, s1_vp, s2_np, s2_vp, p2_set_kPa, p2_start_kPa);
    return true;
}

bool compressor_test_outputs(bool k1, bool k2, bool r3, bool r4) {
    uint32_t now = now_ms();

    if (compressor_relay_demo_active()) {
        ESP_LOGW("SERVICE", "OUT отклонён: идёт демо реле");
        return false;
    }

    // Снимок «всё выключено»: выход из сервиса (если он был) без его включения
    if (!k1 && !k2 && !r3 && !r4) {
        service_exit("OUT,0,0,0,0");
        set_k1(false);
        set_k2(false);
        set_k3(false);
        set_k4(false);
        return true;
    }

    bool link1 = (now - lastPressureTime) < SENSOR_STALE_MS;
    bool link2 = (now - lastPressureTime2) < SENSOR_STALE_MS;

    // Условия сервисного режима: STOP, без аварии, P1 и P2 < 0.1 МПа, датчики живы
    if (run || alarm_latched || current_alarm != ALARM_NONE) {
        ESP_LOGW("SERVICE", "OUT отклонён: установка не в STOP или есть авария (run=%d alm=%d)",
                 run ? 1 : 0, current_alarm);
        return false;
    }
    if (!link1 || !link2) {
        ESP_LOGW("SERVICE", "OUT отклонён: нет связи с датчиками P1/P2");
        return false;
    }
    if (pressure1_kPa >= SERVICE_MAX_P_KPA || pressure2_kPa >= SERVICE_MAX_P_KPA) {
        ESP_LOGW("SERVICE", "OUT отклонён: P1=%.3f P2=%.3f МПа (нужно < %.2f)",
                 pressure1_kPa / 1000.0f, pressure2_kPa / 1000.0f,
                 SERVICE_MAX_P_KPA / 1000.0f);
        return false;
    }

    if (!service_active) {
        service_active = true;
        service_t0 = now;
        ESP_LOGW("SERVICE", "Вход в сервисный режим: тест выходов");
    }

    // Новый тест компрессоров/R4 — начинаем отсчёт таймаута заново
    if ((k1 || k2) && !(service_k1 || service_k2)) service_comp_t0 = now;
    if (r4 && !service_r4) service_r4_t0 = now;

    service_k1 = k1;
    service_k2 = k2;
    service_r3 = r3;
    service_r4 = r4;

    set_k1(k1);
    set_k2(k2);
    set_k3(r3);
    set_k4(r4);

    ESP_LOGW("SERVICE", "OUT: K1=%d K2=%d R3=%d R4=%d (таймауты: компрессоры %d мс, R4 %d мс)",
             k1 ? 1 : 0, k2 ? 1 : 0, r3 ? 1 : 0, r4 ? 1 : 0,
             T_TEST_COMP_MS, T_TEST_R4_MS);
    return true;
}

void compressor_test_cancel(void) {
    service_exit("остановка теста");
}

bool compressor_test_active(void) {
    return service_active;
}

bool compressor_is_running(void) { return run; }
int  compressor_get_alarm(void)  { return current_alarm; }

bool compressor_has_panel_setpoint(void) { return setpoint_rx; }

int compressor_get_state(void) {
    if (service_active)            return COMP_STATE_SERVICE;
    if (current_alarm != ALARM_NONE) return COMP_STATE_ALARM;
    if (!run)                      return COMP_STATE_STOP;
    if (restart2_pause)            return COMP_STATE_PAUSE2;
    return COMP_STATE_RUN;
}

// ============================================================================
//  ДЕМО-ПРОВЕРКА РЕЛЕ K1..K4 (команда DEMO)
//  Непрерывный цикл:
//    K1 ON -> K2 ON -> K3 ON -> K4 ON -> K4 OFF -> K3 OFF -> K2 OFF -> K1 OFF
//  Шаг между переключениями — RELAY_DEMO_HOLD_MS. Состояние всех четырёх
//  выводов пишется в лог UART0 на каждом шаге.
// ============================================================================
static void relay_demo_log(const char *step) {
    ESP_LOGW("RELAY_DEMO", "%-10s K1=%d K2=%d K3=%d K4=%d  (GPIO %d/%d/%d/%d)",
             step,
             comp1_on ? 1 : 0, comp2_on ? 1 : 0,
             comp3_on ? 1 : 0, comp4_on ? 1 : 0,
             PIN_COMP1, PIN_COMP2, PIN_COMP3, PIN_COMP4);
}

bool compressor_relay_demo_active(void) { return demo_active; }

void compressor_relay_demo_stop(void) {
    if (!demo_active) return;
    demo_stop_req = true;
    ESP_LOGW("RELAY_DEMO", "Запрошен останов демо — завершаю после текущего шага");
}

static void relay_demo_task(void *arg) {
    (void)arg;

    ESP_LOGW("RELAY_DEMO", "Старт демо (цикл): K1->K2->K3->K4 ON, затем K4->K3->K2->K1 OFF; шаг %d мс",
             RELAY_DEMO_HOLD_MS);

    while (!demo_stop_req) {
        // ---- последовательное включение: K1 -> K2 -> K3 -> K4 ----
        set_k1(true);  relay_demo_log("K1 ON");
        vTaskDelay(pdMS_TO_TICKS(RELAY_DEMO_HOLD_MS));
        if (demo_stop_req) break;

        set_k2(true);  relay_demo_log("K2 ON");
        vTaskDelay(pdMS_TO_TICKS(RELAY_DEMO_HOLD_MS));
        if (demo_stop_req) break;

        set_k3(true);  relay_demo_log("K3 ON");
        vTaskDelay(pdMS_TO_TICKS(RELAY_DEMO_HOLD_MS));
        if (demo_stop_req) break;

        set_k4(true);  relay_demo_log("K4 ON ALL");
        vTaskDelay(pdMS_TO_TICKS(RELAY_DEMO_HOLD_MS));

        // ---- отключение в обратном порядке: K4 -> K3 -> K2 -> K1 ----
        set_k4(false); relay_demo_log("K4 OFF");
        vTaskDelay(pdMS_TO_TICKS(RELAY_DEMO_HOLD_MS));
        if (demo_stop_req) break;

        set_k3(false); relay_demo_log("K3 OFF");
        vTaskDelay(pdMS_TO_TICKS(RELAY_DEMO_HOLD_MS));
        if (demo_stop_req) break;

        set_k2(false); relay_demo_log("K2 OFF");
        vTaskDelay(pdMS_TO_TICKS(RELAY_DEMO_HOLD_MS));
        if (demo_stop_req) break;

        set_k1(false); relay_demo_log("K1 OFF ALL");
        vTaskDelay(pdMS_TO_TICKS(RELAY_DEMO_HOLD_MS));
    }

    all_off();
    relay_demo_log("ALL OFF");
    ESP_LOGW("RELAY_DEMO", "Демо остановлено: все реле выключены, автомат снова активен");
    demo_active = false;
    vTaskDelete(NULL);
}

void compressor_relay_demo_start(void) {
    if (demo_active) {
        ESP_LOGW("RELAY_DEMO", "Демо уже идёт — повторный запуск проигнорирован");
        return;
    }
    // Демо и сервисный тест несовместимы: тестовые включения снимаем
    service_exit("запущено демо реле");

    demo_stop_req = false;
    demo_active = true;

    // Демо стартует только из безопасного состояния: установка остановлена
    run = false;
    pending_start = false;
    wait_restart = false;
    retry2 = 0;
    restart2_pause = false;
    diag1_reset(); diag2_reset();

    if (xTaskCreate(relay_demo_task, "relay_demo", 3072, NULL, 6, NULL) != pdPASS) {
        demo_active = false;
        ESP_LOGE("RELAY_DEMO", "Не удалось создать задачу демо");
    }
}
