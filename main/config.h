#pragma once

#include <stdbool.h> 
#include "esp_rom_sys.h"
#include "driver/uart.h"
#include "stdbool.h"


// 1 Настройки периферии (Серво и Шаговик) эта схема Андрея
#define PIN_SERVO       2  // Пин управления RC серво
#define PIN_STEP        21  // Пин шага драйвера Nema17
#define PIN_DIR         17  // Пин направления драйвера Nema17
#define PIN_ENABLE      32   // Выход сброса ошибки (на EN+ мотора) -> БЕЗОПАСНО
#define PIN_ALARM       33  // Вход аварии (на AL+ мотора) -> ОЧЕНЬ ВАЖНО!

#define MAX_VALVE_STEPS 10000 // Лимит безопасности: 10 оборотов (3600 градусов)

// ============================================================================
//  МОДУЛЬ ДАВЛЕНИЯ (датчик) и зависимые от него пороги
// ----------------------------------------------------------------------------
//  Простыми словами: один переключатель под установленный датчик. Все пороги,
//  выраженные В кПа (шум датчика, полосы «дошли», дозы и т.п.), в коде домножаются
//  на P_SCALE, поэтому при смене датчика их НЕ надо переписывать руками.
//  P_SCALE = текущая шкала / опорная (6000). Отсюда:
//    - CURRENT_SENSOR_KPA = 6000 -> P_SCALE = 1.0   -> всё ровно как было (старый стенд);
//    - CURRENT_SENSOR_KPA = 63   -> P_SCALE ≈ 0.0105 -> пороги мельче пропорционально шкале.
//  Меняешь датчик — правишь ТОЛЬКО CURRENT_SENSOR_KPA, остальное пересчитается само.
//  (Скорости подхода, шаги иглы, тайминги и PI-коэффициенты НЕ масштабируются —
//   они зависят от привода/подачи, а не от диапазона датчика.)
// ============================================================================
#define P_SENSOR_REF_KPA   6000.0f                                  // опорная шкала: при ней P_SCALE = 1.0
#define CURRENT_SENSOR_KPA 6000.0f                                    // полная шкала установленного датчика (старый стенд: 6000.0f)  // ПОМЕНЯТЬ НА 63 КОГДА БУДЕТ 63
#define P_SCALE            (CURRENT_SENSOR_KPA / P_SENSOR_REF_KPA)  // общий множитель порогов в кПа

// --- Мелкие ступени «медленного удержания» у цели (slow holding) ---------------
//  Отдельный масштаб ТОЛЬКО для дозовой доводки в HOLD у самой цели. В отличие от
//  P_SCALE (опора — 6000), здесь опора — 63 кПа: на ней пороги берутся как есть, на
//  более крупном датчике домножаются. SCALE_for_slow_holding = шкала / 63
//  (например датчик 300 -> ~4.76). Эти ступени включаются ТОЛЬКО на мелком датчике
//  (<= 300 кПа); на крупном (>300) дозовые пороги остаются прежними — из
//  regulator_init, ×P_SCALE (поведение не меняется).
#define SLOW_HOLDING_REF_KPA   63.0f
#define SCALE_for_slow_holding (CURRENT_SENSOR_KPA / SLOW_HOLDING_REF_KPA)
#define SLOW_HOLDING_ENABLED   (CURRENT_SENSOR_KPA <= 300.0f)

// Аппаратный потолок давления (кПа) = полная шкала датчика. Как только P перевалит за
// него во время регулирования — регулятор бросает уставку и АВАРИЙНО стравливает всё
// (см. REG_STATE_OVERPRESSURE в pressure_regulator.c), после чего уходит в IDLE.
#define P_MAX_KPA CURRENT_SENSOR_KPA

#define SERVO_MIN_US    600 // 0 градусов
#define SERVO_MAX_US    2400 // 180 градусов

// 3. Объявляем глобальные переменные через extern (чтобы не было дубликатов)
extern volatile float setpoint_kPa;
extern volatile bool fast_mode;
extern volatile float pressure1_kPa;

extern volatile bool is_homing;
extern volatile bool is_calibrating;
extern volatile int32_t current_valve_position;

// 4. Прототипы функций
void set_servo_angle(float angle);
void move_valve_absolute(int32_t target_position, uint32_t speed_us);
void move_valve_absolute_inv(int32_t target_position, uint32_t speed_us);
void start_pressure_homing(void);
void hardware_setup_and_calibrate(void);
void update_setpoint(float new_setpoint, bool new_fast_mode);
void init_servo(void);
void calibrate_valve_home(void);
