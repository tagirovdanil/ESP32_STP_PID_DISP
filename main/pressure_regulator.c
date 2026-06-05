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

//static const char *TAG = "control_APP";
// Инициализация аппаратного ШИМ для Серво


extern TFT_t dev;
extern FontxFile fx16[2];



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


void regulator_init(PressureRegulator* reg, float dt) {
    memset(reg, 0, sizeof(*reg));
    
    // PID
    reg->pid.Kp = 40.0f;
    reg->pid.Ki = 25.0f;
    reg->pid.Kd = 10.0f;
    reg->pid.dt = dt;
    reg->pid.derivative_alpha = 0.7f;   // было 0.7/0.3
    reg->pid.kp_scale_high = 1.0f;
    reg->pid.kp_scale_low  = 0.5f;
    reg->pid.ki_scale_high = 1.0f;
    reg->pid.ki_scale_low  = 0.7f;
    reg->pid.integral_max = 800.0f;
    reg->pid.integral_min = -800.0f;
    
    // пороги (твои)
    reg->final_charge_on_th  = 0.05f;
    reg->final_charge_off_th = 0.01f;
    reg->vent_on_th          = -0.05f;
    reg->vent_off_th         = -0.01f;
    
    // ограничения
    reg->max_neutral_pos = 15000;
    reg->max_step = 1000;
    
    // начальное положение сервы
    reg->state = REG_STATE_IDLE; // стартуем в плоожении ни чего не далаю. 
    reg->servo_state = SERVO_NEUTRAL;
    reg->prev_state = REG_STATE_IDLE;
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
    
    if (r->ramp_setpoint > r->final_setpoint) {
        // Сброс
        if (distance > 50.0f)      speed_kPa_s = 100.0f;
        else if (distance > 10.0f) speed_kPa_s = 10.0f;
        else if (distance > 5.0f)  speed_kPa_s = 0.2f;
        else if (distance > 2.0f)  speed_kPa_s = 0.05f;
        else                       speed_kPa_s = 0.01f;
    } else {
        // Набор
        if (distance > 0.1f * r->final_setpoint) {
            speed_kPa_s = 0.1f * r->final_setpoint;
        } else if (distance > 0.02f * r->final_setpoint) {
            speed_kPa_s = 0.02f * r->final_setpoint;
        } else if (distance > 2.0f) {
            speed_kPa_s = 2.0f;
        } else {
            speed_kPa_s = 1.0f;
        }
    }
    
    // Адаптация: не даём ramp сильно обгонять давление при сбросе
    if (reg->servo_state == SERVO_VENTING && r->ramp_setpoint < current_pressure - 20.0f) {
        speed_kPa_s = 0.0f;
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
            if (raw_output > 80000.0f && error < 0) {
                pid->vent_integral -= error * pid->dt; // откат
            }
            float output = proportional + ki_eff * (-pid->vent_integral) + pid->Kd * derivative;
            if (output > 80000.0f) output = 80000.0f;
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
        if (output > 80000.0f) {
            if (error > 0) pid->integral -= (output - 80000.0f) / ki_eff * 0.1f;
            output = 80000.0f;
        } else if (output < -80000.0f) {
            if (error < 0) pid->integral -= (output + 80000.0f) / ki_eff * 0.1f;
            output = -80000.0f;
        }
        
        if (output < 0.0f) output = 0.0f;
        
        // Накапливаем интеграл (только если не в насыщении)
        if ((output < 80000.0f || error <= 0) && (output > 0.0f || error >= 0)) {
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
    if (*target > reg->max_neutral_pos) *target = reg->max_neutral_pos;
    
    if (error > 0.01f && error < 0.1f) {
        reg->stuck_counter_pos++;
        if (reg->stuck_counter_pos > 25) {
            *target += 50;
            if (*target > reg->max_neutral_pos) *target = reg->max_neutral_pos;
            reg->stuck_counter_pos = 0;
        }
    } else {
        reg->stuck_counter_pos = 0;
    }
    
    if (error < -0.01f && error > -0.1f) {
        reg->stuck_counter_neg++;
        if (reg->stuck_counter_neg > 25) {
            *target -= 50;
            if (*target < 0) *target = 0;
            reg->stuck_counter_neg = 0;
        }
    } else {
        reg->stuck_counter_neg = 0;
    }
}


//############################################################################################################
//############################################################################################################
//############################################################################################################

void pid_regulator_task(void *pvParameters) {
    PressureRegulator reg;
    regulator_init(&reg, 0.03f);

    float previous_pressure = pressure1_kPa;
    float filtered_rate = 0.0f;
    int32_t target_position = 0;

    while (1) {
                if (requested_reg_state != REG_STATE_IDLE && requested_reg_state != reg.state) {
            // Пришла команда извне
            if (requested_reg_state == REG_STATE_HOMING) {
                // Запускаем homing
                is_homing = true;
            } else {
                // Переходим в указанное состояние
                reg.state = requested_reg_state;
                // Если это IDLE – сбросим всё
                if (requested_reg_state == REG_STATE_IDLE) {
                    reg.ramp.final_setpoint = 0.0f;
                    reg.ramp.ramp_setpoint = 0.0f;
                    reg.ramp.ramp_active = false;
                    setpoint_kPa = 0.0f;
                    // сброс интеграторов
                    reg.pid.integral = 0.0f;
                    reg.pid.vent_integral = 0.0f;
                    reg.pid.prev_error = 0.0f;
                    reg.pid.derivative_filtered = 0.0f;
                    reg.stuck_counter_pos = 0;
                    reg.stuck_counter_neg = 0;
                    // серво в нейтраль и клапан закрыт
                    if (reg.servo_state != SERVO_NEUTRAL) {
                        set_servo_angle(90.0f);
                        reg.servo_state = SERVO_NEUTRAL;
                    }
                    // позиция клапана будет установлена в 0 в кейсе IDLE
                }
            }
            requested_reg_state = REG_STATE_IDLE; // сбросим запрос
        }
                
        if (is_calibrating) {
            vTaskDelay(pdMS_TO_TICKS(50));
            continue;
        }

        // -------------------- Обработка команды HOMING --------------------
        if (is_homing) {
            reg.state = REG_STATE_HOMING;
        }

        // -------------------- Новая уставка --------------------
        if (setpoint_kPa != reg.ramp.final_setpoint && setpoint_kPa > 0.01f) {
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

        // -------------------- Главный автомат состояний --------------------
        switch (reg.state) {
            case REG_STATE_IDLE: {
                // Полное бездействие
                if (reg.servo_state != SERVO_NEUTRAL) {
                    set_servo_angle(90.0f);
                    reg.servo_state = SERVO_NEUTRAL;
                }
                target_position = 0;
                if (current_valve_position != 0) {
                    move_valve_absolute(0, 20);
                }
                // сброс накоплений
                reg.pid.integral = 0.0f;
                reg.pid.vent_integral = 0.0f;
                reg.pid.derivative_filtered = 0.0f;
                reg.pid.prev_error = 0.0f;
                reg.stuck_counter_pos = 0;
                reg.stuck_counter_neg = 0;
                previous_pressure = pressure1_kPa;
                filtered_rate = 0.0f;

                printf("PID: Idle | Pres=%.2f kPa | Pos=%ld\n", pressure1_kPa, (long int)current_valve_position);
                vTaskDelay(pdMS_TO_TICKS(20));
                continue;
            }

            case REG_STATE_HOLD: {
                // Полная остановка без сброса параметров
                if (reg.servo_state != SERVO_NEUTRAL) {
                    set_servo_angle(90.0f);
                    reg.servo_state = SERVO_NEUTRAL;
                }
                target_position = 0;
                if (current_valve_position != 0) {
                    move_valve_absolute(0, 20);
                }
                // Не сбрасываем интеграторы и ramp!
                printf("PID: Hold | Pres=%.2f kPa | Pos=%ld\n", pressure1_kPa, (long int)current_valve_position);
                vTaskDelay(pdMS_TO_TICKS(20));
                continue;
            }

            case REG_STATE_HOMING: {
                // Штатный сброс давления (как в оригинале)
                ESP_LOGW("PID", "Homing: сброс давления...");
                set_servo_angle(0.0f);
                reg.servo_state = SERVO_VENTING;

                int32_t chunk_target = current_valve_position;
                const int32_t STEP_CHUNK = 2000;
                while (current_valve_position < 100000) {
                    chunk_target += STEP_CHUNK;
                    if (chunk_target > 100000) chunk_target = 100000;
                    move_valve_absolute(chunk_target, 190);
                    vTaskDelay(1);
                }

                while (pressure1_kPa > 0.2f) {
                    vTaskDelay(pdMS_TO_TICKS(10));
                }

                chunk_target = current_valve_position;
                while (current_valve_position > 0) {
                    chunk_target -= STEP_CHUNK;
                    if (chunk_target < 0) chunk_target = 0;
                    move_valve_absolute(chunk_target, 190);
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
                // Основной рабочий блок – расчёты выполняются ниже
                break;
        }

        // -------------------- Обновление ramp (только в RAMPING) --------------------
        if (reg.state == REG_STATE_RAMPING) {
            update_ramp(&reg, pressure1_kPa);
            if (!reg.ramp.ramp_active) {
                reg.state = REG_STATE_HOLDING;
                ESP_LOGI("PID", "Ramp завершён, переход в HOLDING");
            }
        }

        // -------------------- Измерения --------------------
        float current_pressure = pressure1_kPa;
        float error = reg.ramp.ramp_setpoint - current_pressure;

        float raw_rate = (current_pressure - previous_pressure) / reg.pid.dt;
        previous_pressure = current_pressure;
        filtered_rate = 0.8f * filtered_rate + 0.2f * raw_rate;

        float derivative = (error - reg.pid.prev_error) / reg.pid.dt;
        reg.pid.prev_error = error;
        reg.pid.derivative_filtered = reg.pid.derivative_alpha * reg.pid.derivative_filtered +
                                      (1.0f - reg.pid.derivative_alpha) * derivative;

        // -------------------- ПИД-вычисления --------------------
        bool is_venting = (reg.servo_state == SERVO_VENTING);
        float output = calculate_pid(&reg.pid, error, reg.pid.derivative_filtered, is_venting);
        int32_t target_raw = (int32_t)output;

        if (target_raw > 80000) target_raw = 80000;
        if (target_raw < 0) target_raw = 0;

        target_position = target_raw;

        // -------------------- Серво-автомат (только в HOLDING) --------------------
        if (reg.state == REG_STATE_HOLDING) {
            update_servo_state(&reg, error);
        }

        // -------------------- Дожим в нейтрали (только в HOLDING) --------------------
        if (reg.state == REG_STATE_HOLDING && reg.servo_state == SERVO_NEUTRAL) {
            apply_neutral_dwell(&reg, error, &target_position);
        }

        // -------------------- Ограничение шага и движение --------------------
        int32_t step = target_position - current_valve_position;
        if (step > reg.max_step) target_position = current_valve_position + reg.max_step;
        else if (step < -reg.max_step) target_position = current_valve_position - reg.max_step;

        move_valve_absolute(target_position, 20);

        printf("PID: Pres=%.2f kPa | Final=%.1f | Ramp=%.1f | Err=%.2f | Rate=%.2f kPa/s | State=%d | Pos=%ld\n",
               current_pressure, reg.ramp.final_setpoint, reg.ramp.ramp_setpoint, error, filtered_rate, reg.state, (long int)current_valve_position);

        vTaskDelay(pdMS_TO_TICKS(20));
    }
}