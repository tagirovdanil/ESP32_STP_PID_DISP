#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "esp_spiffs.h"
#include "control.h"  
#include "esp_timer.h"
#include "driver/ledc.h" // Драйвер ШИМ для Сервопривода
#include "esp_log.h"
#include <math.h>
#include "st7789.h"  // Обязательно подключаем для работы с экраном
#include "fontx.h"
#include "driver/uart.h"
#include "stdbool.h"

//static const char *TAG = "control_APP";
// Инициализация аппаратного ШИМ для Серво

volatile float setpoint_kPa = 0.0f;
volatile float pressure1_kPa = 0.0f;
static float previous_pressure = 0.0f;
static bool nyrok_done = false; 
extern TFT_t dev;
extern FontxFile fx16[2];
extern bool is_calibrating;
volatile bool is_homing = false;
static float filtered_rate = 0.0f;

TaskHandle_t pid_task_handle = NULL; // Хэндл для контроля запущенной ПИД-задачи

bool performAdvancedZeroCalibration(float offset_kPa, uart_port_t uart_num);

// Глобальная переменная текущего абсолютного положения вентиля (в шагах)
int32_t current_valve_position = 0; 

void start_pressure_homing(void) {
    if (!is_homing) {
        ESP_LOGW("CONTROL", "Запущен принудительный сброс давления (Homing)!");
        is_homing = true;
    }
}

// Служебная функция аппаратного сброса драйвера после аварии упора
static void reset_motor_driver(void) {
    gpio_set_level(PIN_ENABLE, 1); // Активируем вход Enable для сброса ошибки
    vTaskDelay(pdMS_TO_TICKS(50));  // Держим 50 мс
    gpio_set_level(PIN_ENABLE, 0); // Отпускаем, мотор снова активен
    vTaskDelay(pdMS_TO_TICKS(100)); // Даем драйверу прийти в себя
}

void calibrate_valve_home(void) {
    ESP_LOGI("HOMING", "Ожидание стабилизации питания мотора...");
    vTaskDelay(pdMS_TO_TICKS(500)); 

    ESP_LOGI("HOMING", "Запуск МОНОЛИТНОЙ ручной калибровки без пауз...");
    is_calibrating = true; 

    // Выводим надпись на экран ОДИН РАЗ до начала движения вала, чтобы не тормозить процесс!
    dev._font_fill = true;
    dev._font_fill_color = BLACK;
    char home_screen_buf[32];
    snprintf(home_screen_buf, sizeof(home_screen_buf), "VALVE HOMING...    ");
    lcdDrawString(&dev, fx16, 85, 220, (uint8_t*)home_screen_buf, GREEN);
    snprintf(home_screen_buf, sizeof(home_screen_buf), "PLEASE WAIT        ");
    lcdDrawString(&dev, fx16, 105, 220, (uint8_t*)home_screen_buf, YELLOW);
    lcdDrawFinish(&dev); // Рисуем заставку ОДИН РАЗ [INDEX]

    // Сбрасываем стартовую ошибку драйвера мотора
    gpio_set_level(PIN_ENABLE, 1); 
    vTaskDelay(pdMS_TO_TICKS(50));  
    gpio_set_level(PIN_ENABLE, 0); 
    vTaskDelay(pdMS_TO_TICKS(100)); 

    ESP_LOGI("HOMING", "Текущий уровень на пине ALARM перед стартом: %d", gpio_get_level(PIN_ALARM));

    // Направление на ЗАКРЫТИЕ
    bool dir_close = true; 
    gpio_set_level(PIN_DIR, dir_close);

    uint32_t total_steps_done = 0;
    const uint32_t MAX_SAFETY_STEPS = 100000; // Лимит 10 оборотов
    uint32_t alarm_confirm_counter = 0;

    // СВЕРХЧИСТЫЙ ЦИКЛ: Процессор занят ТОЛЬКО генерацией импульсов
    while (1) {
        
        // Защита от наводок (дебаунс)
        if (gpio_get_level(PIN_ALARM) != 0) {
            alarm_confirm_counter++;
            if (alarm_confirm_counter >= 5) {
                break; // Физический упор найден!
            }
        } else {
            alarm_confirm_counter = 0; // Сброс помехи
        }

        // САМА ГЕНЕРАЦИЯ ИМПУЛЬСА
        gpio_set_level(PIN_STEP, 1);
        esp_rom_delay_us(20); 
        gpio_set_level(PIN_STEP, 0);
        esp_rom_delay_us(200); // 4000 Гц монолитного хода

        total_steps_done++; 

        // Чтобы не срабатывал сторожевой таймер (Watchdog) процессора, 
        // мы даем микро-отдых системе раз в 4000 шагов (это раз в 1 секунду) всего на 1 тик.
        // На слух это будет едва заметный мягкий переход, а не жесткое заикание каждые 100 мс!
        if (total_steps_done % 4000 == 0) { 
            vTaskDelay(1); 
            ESP_LOGI("HOMING", "Шагов сделано: %lu", total_steps_done);
        }

        // Защита по лимиту шагов
        if (total_steps_done >= MAX_SAFETY_STEPS) {
            ESP_LOGE("HOMING", "ОШИБКА: Физический упор не найден за %lu шагов!", MAX_SAFETY_STEPS);
            is_calibrating = false;
            return;
        }
    }

    // Сюда попадем, когда вал жестко коснулся упора
    float final_turns = (float)total_steps_done / 8000.0f;
    ESP_LOGI("HOMING", "Физический упор найден! Всего честных шагов: %lu (Оборотов: %.2f)", total_steps_done, final_turns);

    // Выводим финальный результат на экран ОДИН РАЗ после остановки вала
    dev._font_fill = true;
    dev._font_fill_color = BLACK;
    
    lcdSetFontDirection(&dev, DIRECTION0);
        
    snprintf(home_screen_buf, sizeof(home_screen_buf), "VALVE CLOSED!      ");
    lcdDrawString(&dev, fx16, 4, 25, (uint8_t*)home_screen_buf, GREEN);
    snprintf(home_screen_buf, sizeof(home_screen_buf), "TOTAL TUR: %-5.2f ", final_turns);
    lcdDrawString(&dev, fx16, 4, 45, (uint8_t*)home_screen_buf, YELLOW);
    lcdDrawFinish(&dev);

    lcdSetFontDirection(&dev, DIRECTION270);
    // Сбрасываем ошибку заклинивания на моторе
    gpio_set_level(PIN_ENABLE, 1);
    vTaskDelay(pdMS_TO_TICKS(50));  
    gpio_set_level(PIN_ENABLE, 0);  
    vTaskDelay(pdMS_TO_TICKS(100)); 

    current_valve_position = 0; 
    is_calibrating = false; // Возвращаем управление экраном основному циклу main.c
    ESP_LOGI("HOMING", "Абсолютный ноль успешно установлен.");
    // Очищаем экран черным (или WHITE, смотря какой у вас фон)
    //lcdFillScreen(&dev, BLACK); 
}

void init_servo(void) {
    // 1. Конфигурируем аппаратный Таймер 0 для Сервопривода (50 Гц, 14 бит)
    ledc_timer_config_t ledc_timer = {
        .speed_mode       = LEDC_LOW_SPEED_MODE, // Низкоскоростной режим
        .timer_num        = LEDC_TIMER_0,        // Используем Таймер 0
        .duty_resolution  = LEDC_TIMER_14_BIT,   // 14-битное разрешение (0-16383)
        .freq_hz          = 50,                  // Стандартная частота для сервоприводов 50 Гц
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ledc_timer_config(&ledc_timer);

    // 2. Привязываем Канал 0 к вашей ноге PIN_SERVO
    ledc_channel_config_t ledc_channel = {
        .speed_mode     = LEDC_LOW_SPEED_MODE,
        .channel        = LEDC_CHANNEL_0,      // Используем Канал 0
        .timer_sel      = LEDC_TIMER_0,
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = PIN_SERVO,           // Ваша нога сервопривода
        .duty           = 0,                   // Изначально выключен
        .hpoint         = 0
    };
    ledc_channel_config(&ledc_channel);
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

float dt;
float Kp;
float Ki;
float Kd;

// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


void pid_regulator_task(void *pvParameters) {
    float integral = 0.0f;
    float previous_error = 0.0f;
    int32_t target_position = 0;
    uint32_t stuck_counter_pos = 0;
    uint32_t stuck_counter_neg = 0;

    dt = 0.02f;

    Kp = 80.0f;
    Ki = 40.0f;   // можно увеличить до 80 для более быстрого сброса/набора
    Kd = 5.0f;

    float derivative_filtered = 0.0f;

    // RAMP
    float final_setpoint = 0.0f;      // конечная цель
    float ramp_setpoint = 0.0f;       // текущая уставка для ПИД
    bool ramp_active = false;         // идёт ли движение к цели

    // Серво
    enum ServoState { NEUTRAL, CHARGING, VENTING };
    enum ServoState servo_state = NEUTRAL;

    static float prev_error_cross = 0.0f;   // для сброса интеграла при переходе через 0

    // Пороги для финального режима (после ramp)
    const float FINAL_CHARGE_ON_THRESHOLD  = 0.5f;
    const float FINAL_CHARGE_OFF_THRESHOLD = 0.1f;
    const float VENT_ON_THRESHOLD          = -10.0f;
    const float VENT_OFF_THRESHOLD         = -0.5f;

    set_servo_angle(90.0f);
    previous_pressure = pressure1_kPa;
    filtered_rate = 0.0f;

    ESP_LOGI("PID", "ПИД-регулятор (единая версия) запущен.");

    while (1) {
        if (is_calibrating) {
            vTaskDelay(pdMS_TO_TICKS(50));
            continue;
        }

        // ======================= СБРОС ДАВЛЕНИЯ (HOMING) =======================
        if (is_homing) {
            ESP_LOGW("PID", "Штатный сброс давления...");
            set_servo_angle(0.0f);
            servo_state = VENTING;

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
            servo_state = NEUTRAL;
            final_setpoint = 0.0f;
            ramp_setpoint = 0.0f;
            ramp_active = false;
            setpoint_kPa = 0.0f;
            integral = 0.0f;
            previous_error = 0.0f;
            derivative_filtered = 0.0f;
            stuck_counter_pos = 0;
            stuck_counter_neg = 0;
            prev_error_cross = 0.0f;
            is_homing = false;
            ESP_LOGI("PID", "Сброс завершён.");
            continue;
        }

        // ======================= НОВАЯ УСТАВКА =======================
        if (setpoint_kPa != final_setpoint && setpoint_kPa > 0.01f) {
            final_setpoint = setpoint_kPa;
            ramp_setpoint = pressure1_kPa;   // начинаем ramp с текущего давления
            ramp_active = true;

            // Полный сброс ПИД
            integral = 0.0f;
            derivative_filtered = 0.0f;
            previous_error = 0.0f;
            stuck_counter_pos = 0;
            stuck_counter_neg = 0;

            if (final_setpoint > pressure1_kPa) {
                // Набор давления
                servo_state = CHARGING;
                set_servo_angle(180.0f);
                target_position = 0;   // клапан закрыт, ПИД сам откроет
                ESP_LOGI("PID", "Старт набора до %.1f кПа", final_setpoint);
            } else {
                // Сброс давления
                servo_state = NEUTRAL;
                set_servo_angle(90.0f);
                target_position = 40000;   // широко открываем клапан для быстрого сброса
                ESP_LOGI("PID", "Старт сброса до %.1f кПа, клапан открыт", final_setpoint);
            }
        }
        if (setpoint_kPa < 0.01f) {
            final_setpoint = 0.0f;
            ramp_active = false;
        }

        // ======================= ОБНОВЛЕНИЕ RAMP-УСТАВКИ =======================
        if (ramp_active && final_setpoint > 0.01f) {
            float error_ramp = final_setpoint - ramp_setpoint;
            if (fabsf(error_ramp) < 0.01f) {
                ramp_setpoint = final_setpoint;
                ramp_active = false;   // достигли цели
                ESP_LOGI("PID", "Ramp завершён.");
            } else {
                float speed_kPa_s;
                if (ramp_setpoint > final_setpoint) {
                    // Идём ВНИЗ (сброс)
                    float distance = ramp_setpoint - final_setpoint;  // сколько ещё сбрасывать
                    if (distance > 20.0f) {
                        speed_kPa_s = 20.0f;   // быстрое снижение
                    } else if (distance > 2.0f) {
                        speed_kPa_s = 5.0f;    // умеренное
                    } else {
                        speed_kPa_s = 1.0f;    // точное
                    }
                } else {
                    // Идём ВВЕРХ (набор) – профиль с процентами
                    float percent_done = 100.0f * ramp_setpoint / final_setpoint;
                    if (percent_done < 80.0f) {
                        speed_kPa_s = 0.1f * final_setpoint;
                    } else if (percent_done < 90.0f) {
                        speed_kPa_s = 0.025f * final_setpoint;
                    } else if (percent_done < 95.0f) {
                        speed_kPa_s = 5.0f;
                    } else {
                        speed_kPa_s = 1.0f;
                    }
                }
                float step = speed_kPa_s * dt;
                if (step > fabsf(error_ramp)) step = fabsf(error_ramp);
                ramp_setpoint += (error_ramp > 0 ? step : -step);
            }
        }

        // ======================= ИЗМЕРЕНИЯ =======================
        float current_pressure = pressure1_kPa;
        float error = ramp_setpoint - current_pressure;   // ошибка для ПИД

        float raw_rate = (current_pressure - previous_pressure) / dt;
        previous_pressure = current_pressure;
        filtered_rate = 0.8f * filtered_rate + 0.2f * raw_rate;

        // ======================= НЕЛИНЕЙНЫЕ КОЭФФИЦИЕНТЫ =======================
        float abs_error = fabsf(error);
        float Kp_eff, Ki_eff;
        if (abs_error > 50.0f) {
            Kp_eff = Kp;
            Ki_eff = Ki;
        } else if (abs_error < 2.0f) {
            if (error > 0) {
                Kp_eff = Kp * 0.5f;
                Ki_eff = Ki * 0.7f;
            } else {
                Kp_eff = Kp * 0.8f;
                Ki_eff = Ki * 0.7f;
            }
        } else {
            float ratio = (abs_error - 2.0f) / 48.0f;
            Kp_eff = Kp * (0.5f + 0.5f * ratio);
            Ki_eff = Ki * (0.7f + 0.3f * ratio);
        }

        // ======================= ПИД-ВЫЧИСЛЕНИЯ =======================
        float derivative_raw = (error - previous_error) / dt;
        previous_error = error;
        derivative_filtered = 0.7f * derivative_filtered + 0.3f * derivative_raw;

        // Сброс интеграла при смене знака ошибки
        if ((error > 0 && prev_error_cross < 0) || (error < 0 && prev_error_cross > 0)) {
            integral = 0.0f;
        }

        float output = Kp_eff * error + Ki_eff * integral + Kd * derivative_filtered;
        int32_t target_raw = (int32_t)output;

        // ======================= ANTI-WINDUP =======================
        if (target_raw > 80000) target_raw = 80000;
        if (target_raw < 0) target_raw = 0;

        bool at_limit = (target_raw == 80000 && error > 0) ||
                        (target_raw == 0 && error < 0);
        if (!at_limit) {
            integral += error * dt;
            if (integral > 800.0f)  integral = 800.0f;
            if (integral < -800.0f) integral = -800.0f;
        } else {
            integral *= 0.995f;
        }

        // ======================= УПРАВЛЕНИЕ СЕРВОЙ =======================
        if (ramp_active) {
            // RAMP-фаза: серва НЕ МЕНЯЕТСЯ, кроме аварийного перелёта >50 кПа
            if (error < -50.0f && servo_state != VENTING) {
                servo_state = VENTING;
                set_servo_angle(0.0f);
                target_position = 80000;
                integral = 0.0f;
                ESP_LOGI("PID", "Аварийный сброс в ramp!");
            }
        } else {
            // ФИНАЛЬНАЯ фаза (удержание точной уставки)
            switch (servo_state) {
                case NEUTRAL:
                    if (error > FINAL_CHARGE_ON_THRESHOLD) {
                        servo_state = CHARGING;
                        set_servo_angle(180.0f);
                        ESP_LOGI("PID", "Серво: ЗАРЯДКА");
                    } else if (error < VENT_ON_THRESHOLD) {
                        servo_state = VENTING;
                        set_servo_angle(0.0f);
                        target_position = 80000;
                        integral = 0.0f;
                        ESP_LOGI("PID", "Серво: СБРОС");
                    }
                    break;

                case CHARGING:
                    if (error <= FINAL_CHARGE_OFF_THRESHOLD) {
                        servo_state = NEUTRAL;
                        set_servo_angle(90.0f);
                        ESP_LOGI("PID", "Серво: НЕЙТРАЛЬ (перекрытие)");
                    } else if (error < VENT_ON_THRESHOLD) {
                        servo_state = VENTING;
                        set_servo_angle(0.0f);
                        target_position = 80000;
                        integral = 0.0f;
                        ESP_LOGI("PID", "Серво: СБРОС из зарядки");
                    }
                    break;

                case VENTING:
                    if (error > VENT_OFF_THRESHOLD) {
                        servo_state = NEUTRAL;
                        set_servo_angle(90.0f);
                        ESP_LOGI("PID", "Серво: НЕЙТРАЛЬ после сброса");
                    }
                    break;
            }
        }

        // ======================= ЦЕЛЕВАЯ ПОЗИЦИЯ КЛАПАНА =======================
        if (fabsf(error) <= 0.01f) {
            // Мёртвая зона – цель достигнута
            target_position = 0;
            integral = 0.0f;
            derivative_filtered = 0.0f;
            stuck_counter_pos = 0;
            stuck_counter_neg = 0;
        } else {
            if (servo_state == VENTING) {
                target_position = 80000;   // полный сброс
            } else if (ramp_active && servo_state == NEUTRAL) {
                // Режим сброса в ramp: клапан открыт пропорционально ошибке
                float open = Kp_eff * fabsf(error) + Ki_eff * integral;
                int32_t raw = (int32_t)open;
                if (raw < 5000) raw = 5000;   // минимальное открытие, чтобы давление падало
                if (raw > 40000) raw = 40000; // максимальное при сбросе
                target_position = raw;
            } else {
                // Обычное управление (ramp CHARGING или финальный режим)
                target_position = target_raw;
                if (!ramp_active && servo_state == NEUTRAL && target_position > 15000) {
                    target_position = 15000;   // в финальной нейтрали не открываемся слишком сильно
                }
            }
        }

        // ======================= ДОЖИМ (только в финальной нейтрали) =======================
        if (!ramp_active && servo_state == NEUTRAL) {
            if (error > 0.01f && error < 0.5f) {
                stuck_counter_pos++;
                if (stuck_counter_pos > 25) {   // 0.5 секунды
                    target_position += 50;
                    if (target_position > 15000) target_position = 15000;
                    stuck_counter_pos = 0;
                }
            } else {
                stuck_counter_pos = 0;
            }

            if (error < -0.01f && error > -0.5f) {
                stuck_counter_neg++;
                if (stuck_counter_neg > 25) {
                    target_position -= 50;
                    if (target_position < 0) target_position = 0;
                    stuck_counter_neg = 0;
                }
            } else {
                stuck_counter_neg = 0;
            }
        }

        // ======================= ОГРАНИЧЕНИЕ ШАГА =======================
        const int32_t MAX_STEP = 3000;
        int32_t step = target_position - current_valve_position;
        if (step > MAX_STEP) {
            target_position = current_valve_position + MAX_STEP;
        } else if (step < -MAX_STEP) {
            target_position = current_valve_position - MAX_STEP;
        }

        move_valve_absolute(target_position, 20);

        printf("PID_LOOP: Pres: %.3f | Final: %.1f | Ramp: %.1f | Err: %.3f | Servo: %c | Cur: %ld | Tgt: %ld\n",
               current_pressure, final_setpoint, ramp_setpoint, error,
               (servo_state == NEUTRAL ? 'N' : (servo_state == CHARGING ? 'C' : 'V')),
               (long int)current_valve_position, (long int)target_position);

        prev_error_cross = error;
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}


// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


void init_pid_regulator(void) {
    // 1. Инициализируем ШИМ только для Сервопривода (Таймер 0 / Канал 0)
    init_servo(); 

    // 2. ИСПРАВЛЕНО: Конфигурируем ВСЕ ТРИ пина шаговика (STEP, DIR, ENABLE) как обычные выходы GPIO
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << PIN_STEP) | (1ULL << PIN_DIR) | (1ULL << PIN_ENABLE), // ТЕПЕРЬ PIN_STEP ТУТ!
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&io_conf);

    // 3. Настраиваем пин ALARM (GPIO 15) как вход с программной подтяжкой к 3.3V (инверсная логика)
    gpio_config_t alarm_conf = {
        .pin_bit_mask = (1ULL << PIN_ALARM),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE, // Обязательно подтягиваем к единице!
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&alarm_conf);

    // Сбрасываем возможные старые ошибки драйвера при включении питания
    reset_motor_driver();

    // ВЫЗЫВАЕМ КАЛИБРОВКУ ХОМИНГА (Мотор плавно и честно найдет физический ноль своими импульсами)
    calibrate_valve_home();

    // ==========================================================================
    // ЖЕЛЕЗОБЕТОННАЯ СКОРОСТНАЯ РУЧНАЯ ПРОДУВКА ПО ШАГАМ (БЕЗ ШИМ)
    // ==========================================================================
    if (pressure1_kPa > 5.0f) {
        ESP_LOGW("PURGE", "Обнаружено остаточное давление: %.1f кПа. Запуск продувки...", pressure1_kPa);
        
        // Ставим сервопривод строго в положение АТМОСФЕРЫ (0 градусов)
        set_servo_angle(0.0f); 
        vTaskDelay(pdMS_TO_TICKS(200)); // Даем серве честно переложиться
        
        // Включаем направление на ОТКРЫТИЕ иглы вверх
        gpio_set_level(PIN_DIR, false); // false - ОТКРЫТИЕ иглы вверх
        esp_rom_delay_us(50);

        ESP_LOGI("PURGE", "Выкручивание иглы строго на 100000 шагов руками...");
        current_valve_position = 0; 

        // Цикл на ОТКРЫТИЕ: Выдаем строго 100 000 физических импульсов на частоте 4000 Гц
        for (int32_t i = 0; i < 100000; i++) {
            gpio_set_level(PIN_STEP, 1);
            esp_rom_delay_us(10); 
            gpio_set_level(PIN_STEP, 0);
            esp_rom_delay_us(20); // 240 мкс пауза = мощные стабильные 4000 Гц

            current_valve_position++; // Точный инкремент абсолютной позиции
        }
        
        ESP_LOGW("PURGE", "Игла открыта на %ld шагов. Ожидание падения давления...", current_valve_position);
        
        // Ждем, пока воздух реально выйдет в атмосферу из объема 100 мл
        while (pressure1_kPa > 2.0f) {
            vTaskDelay(pdMS_TO_TICKS(100));
        }
        
        // В системе честная атмосфера. Вызываем калибровку нуля датчика
        ESP_LOGI("PURGE", "Атмосфера достигнута. Запуск аппаратного нуля датчика...");
        //performAdvancedZeroCalibration(0.0f, 1);
        vTaskDelay(pdMS_TO_TICKS(500)); 

        // ВОЗВРАТ ИГЛЫ ОБРАТНО В ПОЛОЖЕНИЕ АБСОЛЮТНОГО НУЛЯ "0"!
        ESP_LOGI("PURGE", "Возврат иглы обратно в положение абсолютного нуля (0 шагов)...");
        gpio_set_level(PIN_DIR, true); // true - ЗАКРЫТИЕ (вкручиваем обратно к седлу)
        esp_rom_delay_us(50);

        // Делаем ровно 100 000 шагов назад, возвращая вал в исходную точку
        for (int32_t i = 0; i < 100000; i++) {
            gpio_set_level(PIN_STEP, 1);
            esp_rom_delay_us(10); 
            gpio_set_level(PIN_STEP, 0);
            esp_rom_delay_us(20); // быстрая скорость

            current_valve_position--; // Уменьшаем координату строго до нуля!
        }

        // Финальная отсечка сервопривода в безопасную нейтраль
        set_servo_angle(90.0f); 
        vTaskDelay(pdMS_TO_TICKS(200));
        
        ESP_LOGI("PURGE", "Игла успешно вернулась в ноль. Координата в памяти: %ld", current_valve_position);
    } 
    else {
        ESP_LOGI("PURGE", "В системе чисто. Профилактическое обнуление сенсора...");
        vTaskDelay(pdMS_TO_TICKS(500));
        //performAdvancedZeroCalibration(0.0f, 1); //пока не надо
        
        current_valve_position = 0;
    }
}

void update_setpoint(float new_setpoint) {
    if (new_setpoint < 0.0f) new_setpoint = 0.0f;
    
    // Ограничение безопасности: максимум 2000 кПа (2.0 МПа)
    if (new_setpoint > 2000.0f) new_setpoint = 2000.0f; 
    
    setpoint_kPa = new_setpoint;
    
    // Вывод в консоль ESP для отладки
    printf("PID: Новая уставка давления принята: %.1f кПа\n", setpoint_kPa);

    // ==========================================================================
    // ДИНАМИЧЕСКИЙ ЗАПУСК РЕГУЛЯТОРА СТРОГО ПО КОМАНДЕ SET
    // ==========================================================================
    if (pid_task_handle == NULL) {
        printf("PID: Первая команда получена. Запуск фонового потока регулирования...\n");
        
        // Создаем задачу и сохраняем её указатель в pid_task_handle
        xTaskCreate(pid_regulator_task, "pid_regulator_task", 4096, NULL, 5, &pid_task_handle);
    }
    // ==========================================================================
}
