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

// Аппаратный потолок давления (кПа). Как только P перевалит за него во время
// регулирования — регулятор бросает уставку и АВАРИЙНО стравливает всё
// (см. REG_STATE_OVERPRESSURE в pressure_regulator.c), после чего уходит в IDLE.
// ВАЖНО: подобрать под железо! Ставить ВЫШЕ самой большой рабочей уставки +
// ожидаемого перелёта, но НИЖЕ опасного для бака/датчика уровня. Уставка сейчас
// клампится в update_setpoint до 4000 кПа -> потолок берём с запасом над ней.
#define P_MAX_KPA 6300.0f

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
void start_pressure_homing(void);
void hardware_setup_and_calibrate(void);
void update_setpoint(float new_setpoint, bool new_fast_mode);
void init_servo(void);
void calibrate_valve_home(void);
