#pragma once

#include <stdbool.h> 
#include "esp_rom_sys.h"
#include "driver/uart.h"
#include "stdbool.h"


// 1 Настройки периферии (Серво и Шаговик)
#define PIN_SERVO       21  // Пин управления RC серво
#define PIN_STEP        12  // Пин шага драйвера Nema17
#define PIN_DIR         13  // Пин направления драйвера Nema17
#define PIN_ENABLE      2   // Выход сброса ошибки (на EN+ мотора) -> БЕЗОПАСНО
#define PIN_ALARM       15  // Вход аварии (на AL+ мотора) -> ОЧЕНЬ ВАЖНО!



// 2 Настройки ШИМ для Серво (14-битное разрешение)
#define LEDC_TIMER      LEDC_TIMER_0
#define LEDC_MODE       LEDC_LOW_SPEED_MODE
#define LEDC_CHANNEL    LEDC_CHANNEL_0
#define STEP_LEDC_TIMER     LEDC_TIMER_1
#define STEP_LEDC_CHANNEL   LEDC_CHANNEL_1
#define MAX_VALVE_STEPS 80000 // Лимит безопасности: 10 оборотов (3600 градусов)
#define VALVE_MAX_SPEED_HZ  10000 

#define SERVO_MIN_US    1000 // 0 градусов
#define SERVO_MAX_US    2000 // 180 градусов


// Глобальный флаг: идет ли сейчас процедура сброса давления (хоуминга)


// Прототип функции запуска хоуминга

// 3. Объявляем глобальные переменные через extern (чтобы не было дубликатов)
extern volatile float setpoint_kPa;
extern volatile float pressure1_kPa;

extern volatile bool is_homing;
extern volatile bool is_calibrating;             
extern volatile int32_t current_valve_position;
extern volatile bool is_homing;

// 4. Прототипы функций
void set_servo_angle(float angle);
void move_stepper(int steps, int speed_us, bool direction);
void move_valve_absolute(int32_t target_position, uint32_t speed_us);
void start_pressure_homing(void);
void hardware_setup_and_calibrate(void);
void update_setpoint(float new_setpoint);
void init_servo(void);
