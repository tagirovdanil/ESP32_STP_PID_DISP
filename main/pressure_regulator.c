#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "esp_spiffs.h"
#include "esp_timer.h"
#include "driver/ledc.h" // Драйвер ШИМ для Сервопривода
#include "esp_log.h"
#include <math.h>
#include "stdbool.h"
#include "driver/uart.h"

#include "pressure_regulator.h"
#include "st7789.h" 
#include "fontx.h"
#include "config.h"  
#include "pressure_sensor.h"

//static const char *TAG = "control_APP";
// Инициализация аппаратного ШИМ для Серво


extern TFT_t dev;
extern FontxFile fx16[2];

extern TaskHandle_t display_task_handle;

TaskHandle_t pid_task_handle = NULL; // Хэндл для контроля запущенной ПИД-задачи

bool performAdvancedZeroCalibration(float offset_kPa, uart_port_t uart_num);

volatile RegulatorState requested_reg_state = REG_STATE_IDLE;

// Глобальная переменная текущего абсолютного положения вентиля (в шагах)

void start_pressure_homing(void) {
    if (!is_homing) {
        ESP_LOGW("CONTROL", "Запущен принудительный сброс давления (Homing)!");
        is_homing = true;
    }
}

void regulator_request_state(RegulatorState new_state) {
    requested_reg_state = new_state;
}

// Функция установки угла сервопривода (0 - 180 градусов)
void set_servo_angle(float angle) {
    if (angle < 0.0f) angle = 0.0f;
    if (angle > 180.0f) angle = 180.0f;

    uint32_t us = SERVO_MIN_US + (uint32_t)((angle / 180.0f) * (SERVO_MAX_US - SERVO_MIN_US));
    uint32_t duty = (us * 16383) / 20000;
    
    // ==========================================================================
    // ИСПРАВЛЕНИЕ: Физически отправляем ШИМ-сигнал на ногу сервопривода
    // ==========================================================================
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, duty);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
    // ==========================================================================
}

// Функция перемещения иглы в абсолютную координату руками (без ШИМ)
void move_valve_absolute(int32_t target_position, uint32_t speed_us) {
    // Защита от выхода за механические рамки резьбы
    if (target_position < 0) target_position = 0;
    if (target_position > MAX_VALVE_STEPS) target_position = MAX_VALVE_STEPS;

    if (current_valve_position == target_position) return; // Мы уже на месте

    // Вычисляем, сколько шагов нужно сделать физически
    int32_t steps_to_move = labs(target_position - current_valve_position);

    // Определяем направление
    bool dir = (target_position > current_valve_position) ? false : true; // false - открыть, true - закрыть
    gpio_set_level(PIN_DIR, dir);
    esp_rom_delay_us(5); // Короткая пауза для фиксации пина направления

    // Выдаем строго посчитанное количество импульсов
    for (int32_t i = 0; i < steps_to_move; i++) {
        gpio_set_level(PIN_STEP, 1);
        esp_rom_delay_us(10); 
        gpio_set_level(PIN_STEP, 0);
        esp_rom_delay_us(speed_us); // Пауза скорости (например, 190 мкс для 5000 Гц)

        // Изменяем абсолютную координату в памяти строго на 1 шаг
        if (dir == false) {
            current_valve_position++;
        } else {
            current_valve_position--;
        }
    }
}

// Функция генерации шагов для Nema17
void move_stepper(int steps, int speed_us, bool direction) {
    gpio_set_level(PIN_DIR, direction);
    
    for (int i = 0; i < steps; i++) {
        // Программные концевики: не даем ПИДу сломать вентиль!
        if (direction == true && current_valve_position >= MAX_VALVE_STEPS) {
            break; // Вентиль открыт на все 10 оборотов, дальше крутить нельзя
        }
        if (direction == false && current_valve_position <= 0) {
            break; // Вентиль полностью закрыт, уперлись в ноль, дальше давить нельзя
        }

        // Выдаем ОДИН честный импульс
        gpio_set_level(PIN_STEP, 1);
        esp_rom_delay_us(10); 
        gpio_set_level(PIN_STEP, 0);
        
        // Пауза скорости (например, 125 мкс для быстрого движения)
        esp_rom_delay_us(speed_us); 

        // ИЗМЕНЯЕМ АБСОЛЮТНУЮ КООРДИНАТУ В ПАМЯТИ НА +1 ИЛИ -1
        if (direction == true) {
            current_valve_position++; // Считаем шаги вперед (открытие)
        } else {
            current_valve_position--; // Считаем шаги назад (закрытие)
        }
    }
}


void update_ramp(PressureRegulator* reg, float current_pressure) {
    RampController* r = &reg->ramp;
    if (!r->ramp_active || r->final_setpoint < 0.01f) return;

    float error_ramp = r->final_setpoint - r->ramp_setpoint;
    if (fabsf(error_ramp) < 0.01f) {
        r->ramp_setpoint = r->final_setpoint;
        r->ramp_active = false;
        return;
    }

    float distance = fabsf(r->ramp_setpoint - r->final_setpoint);
    float speed_kPa_s;

    // Умеренные скорости, соответствующие возможностям системы
    if (r->ramp_setpoint > r->final_setpoint) {
        // Сброс (снижение)
        if (distance > 50.0f)       speed_kPa_s = 50.0f;
        else if (distance > 20.0f)  speed_kPa_s = 10.0f;
        else if (distance > 10.0f)  speed_kPa_s = 3.0f;
        else if (distance > 5.0f)   speed_kPa_s = 1.0f;
        else if (distance > 2.0f)   speed_kPa_s = 0.3f;
        else                        speed_kPa_s = 0.1f;
    } else {
        // Набор (повышение)
        if (distance > 0.2f * r->final_setpoint)        speed_kPa_s = 0.01f * r->final_setpoint;
        else if (distance > 0.1f * r->final_setpoint)   speed_kPa_s = 0.005f * r->final_setpoint;
        else if (distance > 5.0f)                       speed_kPa_s = 2.0f;
        else                                            speed_kPa_s = 0.5f;
    }

    // Адаптивное ограничение: ramp не должен отрываться от реального давления
    // более чем на заданную величину (10 кПа для набора, 20 кПа для сброса).
    float max_ahead = (r->ramp_setpoint > r->final_setpoint) ? 50.0f : 50.0f;
    if ((r->ramp_setpoint > r->final_setpoint && r->ramp_setpoint < current_pressure - max_ahead) ||
        (r->ramp_setpoint < r->final_setpoint && r->ramp_setpoint > current_pressure + max_ahead)) {
        speed_kPa_s = 0.0f;  // останавливаем ramp, пока давление не приблизится
    }

    float step = speed_kPa_s * reg->pid.dt;
    if (step > distance) step = distance;
    r->ramp_setpoint += (error_ramp > 0 ? step : -step);
}


float calculate_pid(PIDController* pid, float error, float derivative, bool is_venting) {
    float abs_err = fabsf(error);
    
    // Нелинейные коэффициенты
    float kp_eff, ki_eff;
    if (abs_err > 50.0f) {
        kp_eff = pid->Kp * pid->kp_scale_high;
        ki_eff = pid->Ki * pid->ki_scale_high;
    } else if (abs_err < 2.0f) {
        kp_eff = pid->Kp * pid->kp_scale_low;
        ki_eff = pid->Ki * pid->ki_scale_low;
    } else {
        float ratio = (abs_err - 2.0f) / 48.0f;
        kp_eff = pid->Kp * (pid->kp_scale_low + (pid->kp_scale_high - pid->kp_scale_low) * ratio);
        ki_eff = pid->Ki * (pid->ki_scale_low + (pid->ki_scale_high - pid->ki_scale_low) * ratio);
    }
    
    if (is_venting) {
        // Сброс: работаем с модулем ошибки и отдельным интегратором
        if (error < -0.01f) {
            float proportional = kp_eff * abs_err;
            pid->vent_integral += error * pid->dt;
            // Anti-windup: если выход на максимуме, останавливаем накопление
            float raw_output = proportional + ki_eff * (-pid->vent_integral) + pid->Kd * derivative;
            if (raw_output > 8000.0f && error < 0) {
                pid->vent_integral -= error * pid->dt; // откат
            }
            float output = proportional + ki_eff * (-pid->vent_integral) + pid->Kd * derivative;
            if (output > 8000.0f) output = 8000.0f;
            if (output < 0.0f) output = 0.0f;
            return output;
        } else {
            pid->vent_integral = 0.0f;
            return 0.0f;
        }
    } else {
        // Обычный режим (набор / нейтраль)
        if ((error > 0 && pid->prev_error_cross < 0) || (error < 0 && pid->prev_error_cross > 0)) {
            pid->integral = 0.0f;
        }
        pid->prev_error_cross = error;
        
        float output = kp_eff * error + ki_eff * pid->integral + pid->Kd * derivative;
        
        // Anti-windup с back-calculation
        if (output > 8000.0f) {
            if (error > 0) pid->integral -= (output - 8000.0f) / ki_eff * 0.1f;
            output = 8000.0f;
        } else if (output < -8000.0f) {
            if (error < 0) pid->integral -= (output + 8000.0f) / ki_eff * 0.1f;
            output = -8000.0f;
        }
        
        if (output < 0.0f) output = 0.0f;
        
        // Накапливаем интеграл (только если не в насыщении)
        if ((output < 8000.0f || error <= 0) && (output > 0.0f || error >= 0)) {
            pid->integral += error * pid->dt;
            if (pid->integral > pid->integral_max) pid->integral = pid->integral_max;
            if (pid->integral < pid->integral_min) pid->integral = pid->integral_min;
        }
        
        return output;
    }
}

void update_servo_state(PressureRegulator* reg, float error) {
    if (reg->ramp.ramp_active) return;  // в ramp не трогаем
    
    switch (reg->servo_state) {
        case SERVO_NEUTRAL:
            if (error > reg->final_charge_on_th) {
                reg->servo_state = SERVO_CHARGING;
                set_servo_angle(180.0f);
            } else if (error < reg->vent_on_th) {
                reg->servo_state = SERVO_VENTING;
                set_servo_angle(0.0f);
                reg->pid.integral = 0.0f;
            }
            break;
        case SERVO_CHARGING:
            if (error <= reg->final_charge_off_th) {
                reg->servo_state = SERVO_NEUTRAL;
                set_servo_angle(90.0f);
            } else if (error < reg->vent_on_th) {
                reg->servo_state = SERVO_VENTING;
                set_servo_angle(0.0f);
                reg->pid.integral = 0.0f;
            }
            break;
        case SERVO_VENTING:
            if (error > reg->vent_off_th) {
                reg->servo_state = SERVO_NEUTRAL;
                set_servo_angle(90.0f);
                reg->pid.vent_integral = 0.0f;
            }
            break;
    }
}

void apply_neutral_dwell(PressureRegulator* reg, float error, int32_t* target) {
    // Дожим только при очень малых ошибках (режим микронастройки)
    if (fabsf(error) > 0.2f) return;   // не вмешиваемся, работает основной ПИД

    // Плавное пропорциональное воздействие (без интегратора, чтобы не раскачивать)
    float gain = 1500.0f;              // 1500 шагов на 1 кПа ошибки
    float boost = gain * error;        // при error = 0.01 кПа → 15 шагов, очень мало

    // Жёстко ограничиваем максимальное изменение за один вызов
    if (boost > 50.0f) boost = 50.0f;
    if (boost < -50.0f) boost = -50.0f;

    int32_t new_target = *target + (int32_t)boost;

    // Не позволяем выходить за пределы нейтрали (0…max_neutral_pos)
    if (new_target > reg->max_neutral_pos) new_target = reg->max_neutral_pos;
    if (new_target < 0) new_target = 0;

    *target = new_target;
}

void regulator_init(PressureRegulator* reg, float dt) {
    memset(reg, 0, sizeof(*reg));
    
    // PID
    reg->pid.Kp = 10.0f;
    reg->pid.Ki = 10.0f;
    reg->pid.Kd = 5.0f;
    reg->pid.dt = dt;
    reg->pid.derivative_alpha = 0.7f;   // было 0.7/0.3
    reg->pid.kp_scale_high = 2.0f;
    reg->pid.kp_scale_low  = 0.5f;
    reg->pid.ki_scale_high = 2.0f;
    reg->pid.ki_scale_low  = 0.7f;
    reg->pid.integral_max = 200.0f;
    reg->pid.integral_min = -200.0f;
    
    // пороги (твои)
    reg->final_charge_off_th = 0.5f;   // серво перекроется, когда ошибка ≤ 0.5 кПа
    reg->final_charge_on_th  = 1.0f;   // снова включится только при ошибке > 1.0 кПа (почти никогда)
    reg->vent_on_th          = -0.5f;  // сброс включится только при перелёте < -0.5 кПа
    reg->vent_off_th         = -0.2f;  // выключится при > -0.2 кПа
    
    // ограничения
    reg->max_neutral_pos = 1500;
    reg->max_step = 100;
    
    // начальное положение сервы
    reg->state = REG_STATE_IDLE; // стартуем в плоожении ни чего не далаю. 
    reg->servo_state = SERVO_NEUTRAL;
    reg->prev_state = REG_STATE_IDLE;
}


//############################################################################################################
//############################################################################################################
//############################################################################################################

void pid_regulator_task(void *pvParameters) {
    PressureRegulator reg;
    regulator_init(&reg, 0.1f);  // тут задаем частоту дискритизации ПИД регулятора 
    
    set_pressure_filter_k(0.5f);  // на старте быстрая реакция
    
    float previous_pressure = pressure1_kPa;
    float filtered_rate = 0.0f;
    int32_t target_position = 0;

    // Таймер для периодического логирования
    uint32_t last_log_time_ms = 0;
    const uint32_t LOG_PERIOD_MS = 200;

    while (1) {
        // Обработка запрошенного состояния
        if (requested_reg_state != REG_STATE_IDLE && requested_reg_state != reg.state) {
            if (requested_reg_state == REG_STATE_HOMING) {
                is_homing = true;
            } else {
                reg.state = requested_reg_state;
                if (requested_reg_state == REG_STATE_IDLE) {
                    reg.ramp.final_setpoint = 0.0f;
                    reg.ramp.ramp_setpoint = 0.0f;
                    reg.ramp.ramp_active = false;
                    setpoint_kPa = 0.0f;
                    reg.pid.integral = 0.0f;
                    reg.pid.vent_integral = 0.0f;
                    reg.pid.prev_error = 0.0f;
                    reg.pid.derivative_filtered = 0.0f;
                    reg.stuck_counter_pos = 0;
                    reg.stuck_counter_neg = 0;
                    if (reg.servo_state != SERVO_NEUTRAL) {
                        set_servo_angle(90.0f);
                        reg.servo_state = SERVO_NEUTRAL;
                    }
                }
            }
            requested_reg_state = REG_STATE_IDLE; // сбросим запрос
        }

        if (is_calibrating) {
            vTaskDelay(pdMS_TO_TICKS(50));
            continue;
        }

        // Обработка команды HOMING
        if (is_homing) {
            reg.state = REG_STATE_HOMING;
        }

        // Новая уставка
        if (setpoint_kPa != reg.ramp.final_setpoint && setpoint_kPa > 0.01f) {
            set_pressure_filter_k(0.5f);
            reg.ramp.final_setpoint = setpoint_kPa;
            reg.ramp.ramp_setpoint = pressure1_kPa;
            reg.ramp.ramp_active = true;
            reg.state = REG_STATE_RAMPING;

            reg.pid.integral = 0.0f;
            reg.pid.vent_integral = 0.0f;
            reg.pid.prev_error = 0.0f;
            reg.pid.prev_error_cross = 0.0f;
            reg.stuck_counter_pos = 0;
            reg.stuck_counter_neg = 0;
            target_position = 0;

            if (setpoint_kPa > pressure1_kPa) {
                reg.servo_state = SERVO_CHARGING;
                set_servo_angle(180.0f);
            } else {
                reg.servo_state = SERVO_VENTING;
                set_servo_angle(0.0f);
            }
        }

        // Главный автомат состояний
        switch (reg.state) {
            case REG_STATE_IDLE: {
                if (reg.servo_state != SERVO_NEUTRAL) {
                    set_servo_angle(90.0f);
                    reg.servo_state = SERVO_NEUTRAL;
                }
                target_position = 0;
                if (current_valve_position != 0) {
                    move_valve_absolute(0, 100);
                }
                reg.pid.integral = 0.0f;
                reg.pid.vent_integral = 0.0f;
                reg.pid.derivative_filtered = 0.0f;
                reg.pid.prev_error = 0.0f;
                reg.stuck_counter_pos = 0;
                reg.stuck_counter_neg = 0;
                previous_pressure = pressure1_kPa;
                filtered_rate = 0.0f;

                if (esp_timer_get_time() / 1000 - last_log_time_ms > LOG_PERIOD_MS) {
                    ESP_LOGI("PID", "Idle | Pres=%.2f kPa | Pos=%ld", pressure1_kPa, (long int)current_valve_position);
                    last_log_time_ms = esp_timer_get_time() / 1000;
                }
                vTaskDelay(pdMS_TO_TICKS(20));
                continue;
            }

            case REG_STATE_HOLD: {
                if (reg.servo_state != SERVO_NEUTRAL) {
                    set_servo_angle(90.0f);
                    reg.servo_state = SERVO_NEUTRAL;
                }
                target_position = 0;
                if (current_valve_position != 0) {
                    move_valve_absolute(0, 100);
                }
                if (esp_timer_get_time() / 1000 - last_log_time_ms > LOG_PERIOD_MS) {
                    ESP_LOGI("PID", "Hold | Pres=%.2f kPa | Pos=%ld", pressure1_kPa, (long int)current_valve_position);
                    last_log_time_ms = esp_timer_get_time() / 1000;
                }
                vTaskDelay(pdMS_TO_TICKS(20));
                continue;
            }

            case REG_STATE_HOMING: {
                ESP_LOGW("PID", "Homing: сброс давления...");
                set_servo_angle(0.0f);
                reg.servo_state = SERVO_VENTING;

                int32_t chunk_target = current_valve_position;
                const int32_t STEP_CHUNK = 200;
                while (current_valve_position < 10000) {
                    chunk_target += STEP_CHUNK;
                    if (chunk_target > 10000) chunk_target = 10000;
                    move_valve_absolute(chunk_target, 100);
                    vTaskDelay(1);
                }

                while (pressure1_kPa > 0.2f) {
                    vTaskDelay(pdMS_TO_TICKS(10));
                }

                chunk_target = current_valve_position;
                while (current_valve_position > 0) {
                    chunk_target -= STEP_CHUNK;
                    if (chunk_target < 0) chunk_target = 0;
                    move_valve_absolute(chunk_target, 100);
                    vTaskDelay(1);
                }

                set_servo_angle(90.0f);
                reg.servo_state = SERVO_NEUTRAL;
                reg.ramp.final_setpoint = 0.0f;
                reg.ramp.ramp_setpoint = 0.0f;
                reg.ramp.ramp_active = false;
                setpoint_kPa = 0.0f;
                reg.pid.integral = 0.0f;
                reg.pid.vent_integral = 0.0f;
                reg.pid.prev_error = 0.0f;
                reg.pid.derivative_filtered = 0.0f;
                reg.stuck_counter_pos = 0;
                reg.stuck_counter_neg = 0;
                reg.state = REG_STATE_IDLE;
                is_homing = false;
                ESP_LOGI("PID", "Homing завершён.");
                continue;
            }

            case REG_STATE_RAMPING:
            case REG_STATE_HOLDING:
                break;
        }

        // Обновление ramp (только в RAMPING)
        if (reg.state == REG_STATE_RAMPING) {
            update_ramp(&reg, pressure1_kPa);
            if (!reg.ramp.ramp_active) {
                reg.state = REG_STATE_HOLDING;
                set_pressure_filter_k(0.05f);
                ESP_LOGI("PID", "Ramp завершён, переход в HOLDING");
            }
        }

        // Измерения
        float current_pressure = pressure1_kPa;
        float error = reg.ramp.ramp_setpoint - current_pressure;

        float raw_rate = (current_pressure - previous_pressure) / reg.pid.dt;
        previous_pressure = current_pressure;
        //filtered_rate = 0.8f * filtered_rate + 0.2f * raw_rate;
        filtered_rate = 0.9f * filtered_rate + 0.1f * raw_rate;

        float derivative = (error - reg.pid.prev_error) / reg.pid.dt;
        reg.pid.prev_error = error;
        reg.pid.derivative_filtered = reg.pid.derivative_alpha * reg.pid.derivative_filtered +
                                      (1.0f - reg.pid.derivative_alpha) * derivative;

        // ПИД-вычисления
        bool is_venting = (reg.servo_state == SERVO_VENTING);
        float output = calculate_pid(&reg.pid, error, reg.pid.derivative_filtered, is_venting);
        int32_t target_raw = (int32_t)output;

        if (target_raw > 10000) target_raw = 10000;
        if (target_raw < 0) target_raw = 0;

        target_position = target_raw;

        // Серво-автомат (только в HOLDING)
        if (reg.state == REG_STATE_HOLDING) {
            update_servo_state(&reg, error);
        }

        // Дожим в нейтрали (только в HOLDING)
        if (reg.state == REG_STATE_HOLDING && reg.servo_state == SERVO_NEUTRAL) {
            apply_neutral_dwell(&reg, error, &target_position);
        }

        // Ограничение шага и движение
        int32_t step = target_position - current_valve_position;
        if (step > reg.max_step) target_position = current_valve_position + reg.max_step;
        else if (step < -reg.max_step) target_position = current_valve_position - reg.max_step;

        if (reg.state == REG_STATE_HOLDING && reg.servo_state == SERVO_NEUTRAL) {
            if (step > 200) target_position = current_valve_position + (step > 0 ? 200 : -200);
            else if (step < -200) target_position = current_valve_position - 200;
        }

        move_valve_absolute(target_position, 100);

        // Логирование раз в 500 мс
        uint32_t now_ms = esp_timer_get_time() / 1000;
        if (now_ms - last_log_time_ms >= LOG_PERIOD_MS) {
            ESP_LOGI("PID", "Pres=%.2f kPa | Final=%.1f | Ramp=%.1f | Err=%.2f | Rate=%.2f kPa/s | State=%d | Pos=%ld",
                     current_pressure, reg.ramp.final_setpoint, reg.ramp.ramp_setpoint, error, filtered_rate, reg.state, (long int)current_valve_position);
            last_log_time_ms = now_ms;
        }

        vTaskDelay(pdMS_TO_TICKS(20));
    }
}