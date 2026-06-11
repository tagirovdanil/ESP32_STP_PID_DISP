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
#include "st7789.h"  // Обязательно подключаем для работы с экраном
#include "fontx.h"
#include "driver/uart.h"
#include "pressure_regulator.h"
#include "config.h" 
#include "esp_task_wdt.h"

//static const char *TAG = "control_APP";
// Инициализация аппаратного ШИМ для Серво

volatile float setpoint_kPa = 0;
volatile float pressure1_kPa = 0;
volatile bool is_homing = false;
volatile bool is_calibrating = false;             
extern TFT_t dev;
extern FontxFile fx16[2];

extern TaskHandle_t display_task_handle;
static void reset_motor_driver(void);
void calibrate_valve_home(void);

bool performAdvancedZeroCalibration(float offset_kPa, uart_port_t uart_num);

// Глобальная переменная текущего абсолютного положения вентиля (в шагах)
volatile int32_t current_valve_position = 0; 

void update_setpoint(float new_setpoint) {
    if (new_setpoint < 0.0f) new_setpoint = 0.0f;
    if (new_setpoint > 4000.0f) new_setpoint = 4000.0f;
    setpoint_kPa = new_setpoint;
    printf("PID: Новая уставка давления принята: %.1f кПа\n", setpoint_kPa);
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
    reset_motor_driver();

    ESP_LOGI("HOMING", "Текущий уровень на пине ALARM перед стартом: %d", gpio_get_level(PIN_ALARM));

    // Направление на ЗАКРЫТИЕ
    bool dir_close = true; 
    gpio_set_level(PIN_DIR, dir_close);

    uint32_t total_steps_done = 0;
    const uint32_t MAX_SAFETY_STEPS = 15000; // Лимит 15 оборотов
    uint32_t alarm_confirm_counter = 0;

    // СВЕРХЧИСТЫЙ ЦИКЛ: Процессор занят ТОЛЬКО генерацией импульсов
        while (1) {
            
            // Защита от наводок (дебаунс)
            if (gpio_get_level(PIN_ALARM) != 0) {
                alarm_confirm_counter++;
                if (alarm_confirm_counter >= 10) {
                    break; // Физический упор найден!
                }
            } else {
                alarm_confirm_counter = 0; // Сброс помехи
            }

            // САМА ГЕНЕРАЦИЯ ИМПУЛЬСА
            gpio_set_level(PIN_STEP, 1);
            esp_rom_delay_us(20); 
            gpio_set_level(PIN_STEP, 0);
            esp_rom_delay_us(500); // 4000 Гц монолитного хода

            total_steps_done++; 

            // Чтобы не срабатывал сторожевой таймер (Watchdog) процессора, 
            // мы даем микро-отдых системе раз в 4000 шагов (это раз в 1 секунду) всего на 1 тик.
            // На слух это будет едва заметный мягкий переход, а не жесткое заикание каждые 100 мс!
            if (total_steps_done % 100 == 0) { 
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
    float final_turns = (float)total_steps_done / 800.0f;
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
    reset_motor_driver();

    current_valve_position = 0; 
    is_calibrating = false; // Возвращаем управление экраном основному циклу main.c
    ESP_LOGI("HOMING", "Абсолютный ноль успешно установлен.");
    ESP_LOGI("PURGE", "После закрытия позиция: %ld", current_valve_position);
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

void hardware_setup_and_calibrate(void) {
    // 1. Инициализируем ШИМ только для Сервопривода (Таймер 0 / Канал 0)
    init_servo(); 

    // 2. Конфигурируем ВСЕ ТРИ пина шаговика (STEP, DIR, ENABLE) как обычные выходы GPIO
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
    
    UBaseType_t saved_priority = uxTaskPriorityGet(NULL);
    vTaskPrioritySet(NULL, 5); // выше UART и дисплея

    // Регистрируем текущую задачу в Task Watchdog
    esp_task_wdt_add(NULL);


    // ==========================================================================
    // ЖЕЛЕЗОБЕТОННАЯ СКОРОСТНАЯ РУЧНАЯ ПРОДУВКА ПО ШАГАМ (БЕЗ ШИМ)
    // ==========================================================================
        if (pressure1_kPa > 5.0f) {
            if (current_valve_position != 0) {
                ESP_LOGE("PURGE", "Ошибка: игла не в нуле (%ld)", current_valve_position);
                return;
            }

            if (display_task_handle != NULL) vTaskSuspend(display_task_handle);

            set_servo_angle(0.0f);
            vTaskDelay(pdMS_TO_TICKS(200));

            // Открытие иглы на полный ход
            gpio_set_level(PIN_DIR, false);
            current_valve_position = 0;
            for (int32_t i = 0; i < MAX_VALVE_STEPS; i++) {
                gpio_set_level(PIN_STEP, 1);
                esp_rom_delay_us(10);
                gpio_set_level(PIN_STEP, 0);
                esp_rom_delay_us(200);
                current_valve_position++;
                if (i % 2000 == 0) {
                    esp_task_wdt_reset();   // сброс WDT без блокировки
                }
            }

            while (pressure1_kPa > 2.0f) {
                vTaskDelay(pdMS_TO_TICKS(1000));
                esp_task_wdt_reset();
            }

            ESP_LOGI("PURGE", "Атмосфера достигнута.");
            vTaskDelay(pdMS_TO_TICKS(2000));

            if (pressure1_kPa < 1.0f) {
                performAdvancedZeroCalibration(0.0f, 1);
                ESP_LOGW("PURGE", "Датчик обнулён.");
            }

            vTaskDelay(pdMS_TO_TICKS(200));

            // Закрытие иглы обратно в ноль
            gpio_set_level(PIN_DIR, true);
            for (int32_t i = 0; i < MAX_VALVE_STEPS; i++) {
                gpio_set_level(PIN_STEP, 1);
                esp_rom_delay_us(10);
                gpio_set_level(PIN_STEP, 0);
                esp_rom_delay_us(200);
                current_valve_position--;
                if (i % 2000 == 0) {
                    esp_task_wdt_reset();
                }
            }

            current_valve_position = 0;   // гарантируем ноль
            set_servo_angle(90.0f);
            vTaskDelay(pdMS_TO_TICKS(200));

            if (display_task_handle != NULL) vTaskResume(display_task_handle);

            ESP_LOGI("PURGE", "Продувка завершена. Позиция: %ld", current_valve_position);
        }
    // Удаляем задачу из Task Watchdog
    esp_task_wdt_delete(NULL);
    vTaskPrioritySet(NULL, saved_priority);
}
