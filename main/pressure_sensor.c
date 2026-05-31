#include "pressure_sensor.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "st7789.h"
#include "esp_timer.h"
#include "fontx.h"
#include "driver/gpio.h"
#include "esp_spiffs.h"
#include "control.h"  
#include <math.h>

static const char *TAG = "SENSOR_MODULE";
// Служебные переменные для расчета кПа/сек
//static float previous_pressure = 0.0f;
//static bool nyrok_done = false; // Флаг, что один нырок в атмосферу при перелете выполнен


esp_timer_handle_t pressure_timer;

uint8_t rxBuffer1[12];
uint8_t rxIndex1 = 0;
bool frameStarted1 = false;
uint32_t sum_err = 0;
uint32_t lastPressureTime = 0;

// Реализация глобальной переменной флага калибровки
bool is_calibrating = false; 

extern void usb_uart_rx_task(void *pvParameters); 

/* ========================================================================== */
/*   ПРИВАТНЫЕ (СЛУЖЕБНЫЕ) ФУНКЦИИ — СНАРУЖИ ИХ НЕ ВИДНО                      */
/* ========================================================================== */

// Фоновая задача или функция циклического чтения данных из буфера UART
static void processIncomingData(void);

// Если dev и fx16 объявлены в main.c, даем знать файлу pressure_sensor.c о них:
extern TFT_t dev;

void display_update_task(void *pvParameters) {
    // Создаем буфер для текста прямо внутри задачи, чтобы не тащить его из main
    char screen_buffer[32]; 

    // Инициализируем пин кнопки Boot (GPIO 0) на вход с подтяжкой к питанию
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << GPIO_NUM_0),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&io_conf);

    // Бесконечный рабочий цикл задачи
    while (1) {
        // 1. Опрос кнопки Boot (Калибровка нуля)
        if (gpio_get_level(GPIO_NUM_0) == 0) {
            performAdvancedZeroCalibration(0.0f, SENSOR_UART_NUM);
        }

        // 2. Отрисовка данных, если нет калибровки
        if (!is_calibrating) {
            dev._font_fill = true;
            dev._font_fill_color = BLACK; 

            // Значение Уставки
            snprintf(screen_buffer, sizeof(screen_buffer), "%-6.1f kPa", setpoint_kPa);
            lcdDrawString(&dev, fx16, 25, 140, (uint8_t*)screen_buffer, GREEN); 

            // Значение Текущего давления
            snprintf(screen_buffer, sizeof(screen_buffer), "%-6.1f kPa", pressure1_kPa);
            lcdDrawString(&dev, fx16, 45, 140, (uint8_t*)screen_buffer, YELLOW); 

            // Количество Ошибок
            snprintf(screen_buffer, sizeof(screen_buffer), "%-5lu", sum_err);
            lcdDrawString(&dev, fx16, 65, 100, (uint8_t*)screen_buffer, RED); 

            dev._font_fill = false;
            lcdDrawFinish(&dev);
        }

        // Та самая важная задержка на 200 мс, которая дает процессору дышать
        vTaskDelay(pdMS_TO_TICKS(200));
    }
}

// Подсчет контрольной суммы XOR
uint8_t calculateChecksum(const uint8_t *data, size_t length) {
        uint8_t checksum = 0;
        for (size_t i = 0; i < length; i++) {
            checksum ^= data[i];
        }
        return checksum;
        }

void clearBufferCompletely(uart_port_t uart_num) {
            uart_flush_input(uart_num);
        }
    
void LCD_init(void){
    esp_vfs_spiffs_conf_t spiffs_conf = {
        .base_path = "/fonts", .partition_label = "storage1", .max_files = 1, .format_if_mount_failed = false 
    };
    if (esp_vfs_spiffs_register(&spiffs_conf) != ESP_OK) return;

    InitFontx(fx16, "/fonts/ILMH16XB.FNT", ""); 
    spi_master_init(&dev, PIN_MOSI, PIN_SCLK, PIN_CS, PIN_DC, PIN_RST, PIN_BL);
    
    // Передаем SCREEN_WIDTH и SCREEN_HEIGHT (Внимание на Шаг 2 ниже!)
    lcdInit(&dev, SCREEN_WIDTH, SCREEN_HEIGHT, OFFSET_X, OFFSET_Y);
    
    gpio_set_direction(PIN_BL, GPIO_MODE_OUTPUT);
    gpio_set_level(PIN_BL, 1);

    gpio_config_t btn_config = { .pin_bit_mask = (1ULL << GPIO_NUM_0), .mode = GPIO_MODE_INPUT, .pull_up_en = GPIO_PULLUP_ENABLE };
    gpio_config(&btn_config);

    lcdFillScreen(&dev, BLACK); 
    
    // СТРОГО DIRECTION0 (Текст пишется как обычно, слева направо)
    lcdSetFontDirection(&dev, DIRECTION270); 
}

// ==========================================================================
// ПРИВАТНЫЕ ПРОТОТИПЫ (Видны ТОЛЬКО внутри этого файла)
// ==========================================================================

void IRAM_ATTR timerCallback(void *arg) {
     // 1. Сначала забираем и парсим ответ на ПРЕДЫДУЩИЙ запрос
    processIncomingData();
    // 2. Отправляем НОВЫЙ запрос датчику (он ответит на него к следующему вызову таймера)
    uint8_t prCmd1[] = { 0x02, DEVICE_ADDRESS1, 0x01, 0x00, 0x00 };
    prCmd1[4] = calculateChecksum(prCmd1, 4);
    uart_write_bytes(SENSOR_UART_NUM, (const char*)prCmd1, sizeof(prCmd1));
}
// Обработчик ответа от датчика на команду калибровки/измерения
static void handlePressureResponse1(uint8_t *frame, uint8_t length);

// Обработчик успешно принятого и проверенного UART кадра
static void handleUART1ReceivedFrame(uint8_t *frame, uint8_t length);

// ==========================================================================
// РЕАЛИЗАЦИЯ ФУНКЦИЙ
// ==========================================================================
// Настройка UART1 датчика давления
void pressure_sensor_init(void) {
    uart_config_t uart_config = {
        .baud_rate = 19200, .data_bits = UART_DATA_8_BITS, .parity = UART_PARITY_ODD,
        .stop_bits = UART_STOP_BITS_1, .flow_ctrl = UART_HW_FLOWCTRL_DISABLE, .source_clk = UART_SCLK_DEFAULT
    };
    uart_driver_install(SENSOR_UART_NUM, BUF_SIZE * 2, 0, 0, NULL, 0);
    uart_param_config(SENSOR_UART_NUM, &uart_config);
    uart_set_pin(SENSOR_UART_NUM, PIN_TX1, PIN_RX1, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

    const esp_timer_create_args_t timer_args = { .callback = &timerCallback, .name = "pressure_timer" };
    esp_timer_create(&timer_args, &pressure_timer);
    esp_timer_start_periodic(pressure_timer, SERIAL_UART1_INTERVAL * 1000);

    
    ESP_LOGI(TAG, "Модуль датчика давления успешно инициализирован.");

}

bool performAdvancedZeroCalibration(float offset_kPa, uart_port_t uart_num) {
    // 1. Включаем флаг, чтобы заблокировать отрисовку экрана на время калибровки
    is_calibrating = true; 

    esp_timer_stop(pressure_timer);
    vTaskDelay(150 / portTICK_PERIOD_MS);
    clearBufferCompletely(uart_num);

    uint8_t zeroCmd[9] = { 0x02, 0x02, 0x2B, 0x04 };
    union { float value; uint8_t bytes[4]; } pUnion;
    pUnion.value = offset_kPa;
    zeroCmd[4] = pUnion.bytes[3]; zeroCmd[5] = pUnion.bytes[2];
    zeroCmd[6] = pUnion.bytes[1]; zeroCmd[7] = pUnion.bytes[0];
    zeroCmd[8] = calculateChecksum(zeroCmd, 8);

    uart_write_bytes(uart_num, (const char*)zeroCmd, sizeof(zeroCmd));

    uint8_t response[32] = {0};
    int responseIndex = 0;
    uint32_t startTime = esp_timer_get_time() / 1000;
    
    while (((esp_timer_get_time() / 1000) - startTime) < 2000 && responseIndex == 0) {
        uint8_t byte_in;
        if (uart_read_bytes(uart_num, &byte_in, 1, 0) > 0) response[responseIndex++] = byte_in;
        vTaskDelay(1);
    }

    if (responseIndex == 0) {
        esp_timer_start_periodic(pressure_timer, SERIAL_UART1_INTERVAL * 1000);
        // СБРОС ФЛАГА ПРИ ОШИБКЕ: возвращаем управление экрану
        is_calibrating = false; 
        return false;
    }

    startTime = esp_timer_get_time() / 1000;
    while (((esp_timer_get_time() / 1000) - startTime) < 150) {
        uint8_t byte_in;
        if (uart_read_bytes(uart_num, &byte_in, 1, 0) > 0) {
            response[responseIndex++] = byte_in;
            startTime = esp_timer_get_time() / 1000;
            if (responseIndex >= sizeof(response)) break;
        }
        vTaskDelay(1);
    }

    esp_timer_start_periodic(pressure_timer, SERIAL_UART1_INTERVAL * 1000);
    
    // 2. СБРОС ФЛАГА ПРИ УСПЕХЕ: калибровка завершена, экран снова оживает
    is_calibrating = false; 
    return true;
}

// Приватный обработчик ответа (обязательно пишется static)
static void handlePressureResponse1(uint8_t *frame, uint8_t length) {
    union { uint8_t bytes[4]; float value; } fval;
    fval.bytes[0] = frame[10]; fval.bytes[1] = frame[9];
    fval.bytes[2] = frame[8];  fval.bytes[3] = frame[7];

    uint32_t raw_value;
    memcpy(&raw_value, &fval.value, 4);
    const uint32_t OVF_PATTERN = 0x7F800000;
    if ((raw_value == OVF_PATTERN) || isnan(fval.value) || fval.value > 3000.0f || fval.value < -100.0f) {
        sum_err++;
        return;
    }

    pressure1_kPa = fval.value;
    lastPressureTime = esp_timer_get_time() / 1000;
}

// Приватный обработчик кадра (обязательно пишется static)
static void handleUART1ReceivedFrame(uint8_t *frame, uint8_t length) {
    if (length < 3) return;
    if (frame[0] != ACK || frame[1] != DEVICE_ADDRESS1) return;

    uint8_t calcChecksum = calculateChecksum(frame, length - 1);
    if (calcChecksum != frame[length - 1]) {
        sum_err++;
        return;
    }

    if (frame[2] == 0x01 && length >= 12) { 
        handlePressureResponse1(frame, 12);
    }
    // Анализ заголовков, CRC и вызов handlePressureResponse1
}

// Приватная функция чтения UART (обязательно пишется static)
static void processIncomingData(void) {

    // Вызов uart_read_bytes и сборка кадров
    uint8_t incomingByte;
    while (uart_read_bytes(SENSOR_UART_NUM, &incomingByte, 1, 0) > 0) {
        if (!frameStarted1 && incomingByte == ACK) {
            frameStarted1 = true;
            rxIndex1 = 0;
            rxBuffer1[rxIndex1++] = incomingByte;
            continue;
        }
        if (frameStarted1 && rxIndex1 < 12) {
            rxBuffer1[rxIndex1++] = incomingByte;
        }
        if (rxIndex1 == 12) {
            handleUART1ReceivedFrame(rxBuffer1, 12);
            frameStarted1 = false;
            rxIndex1 = 0;
        }
    }
}

// Перед самой функцией нужно объявить таску, чтобы xTaskCreate понимал, что это такое
extern void usb_uart_rx_task(void *pvParameters); 

void pressure_ui_and_usb_init(TFT_t *p_dev) {
    // ==========================================================================
    // ИНИЦИАЛИЗАЦИЯ ДРАЙВЕРА USB UART (UART_NUM_0) ДЛЯ ПРИЕМА КОМАНД С ПК
    // ==========================================================================
    uart_config_t uart_config = {
        .baud_rate = 115200,                  // Стандартная скорость монитора ESP-IDF
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    // Конфигурируем параметры порта
    uart_param_config(UART_NUM_0, &uart_config);
    
    // Устанавливаем стандартные пины (GPIO 1 и GPIO 3 используются для USB по умолчанию)
    uart_set_pin(UART_NUM_0, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    
    // Устанавливаем драйвер и выделяем буфер под прием (например, 512 байт)
    uart_driver_install(UART_NUM_0, 512, 0, 0, NULL, 0);
    // ==========================================================================

    // Теперь создание задачи пройдет успешно и без ошибок драйвера!
    xTaskCreate(usb_uart_rx_task, "usb_uart_rx_task", 3072, NULL, 4, NULL);

    ESP_LOGI("INIT", "Все системы запущены.");

    // Рисуем тонкую серую рамку по всему периметру экрана 135x240
    // Так как p_dev это указатель, используем его напрямую (без знака &)
    lcdDrawRect(p_dev, 0, 0, 134, 239, BLUE);

    // Рисуем горизонтальную линию под показаниями давления
    lcdDrawLine(p_dev, 0, 50, 134, 50, BLUE);
    
    // Включаем прозрачный фон для статического текста
    lcdUnsetFontFill(p_dev);

    // Рисуем неизменяемые подписи (передаем разыменованный указатель на шрифт *p_fx16)
    lcdDrawString(p_dev, fx16, 25, 220, (uint8_t*)"TARGET:", GREEN); 
    lcdDrawString(p_dev, fx16, 45, 220, (uint8_t*)"P1_OUT:", YELLOW); 
    lcdDrawString(p_dev, fx16, 65, 220, (uint8_t*)"CRC ERRORS:", RED); 

    // Рисуем заставку «PLEASE WAIT» или рамки, если они нужны
    lcdDrawFinish(p_dev); 
    
    // Запускаем задачу обновления экрана автоматически!
    // Выделяем ей 3072 байт стека и приоритет 3 (чуть ниже, чем у UART)
    xTaskCreate(display_update_task, "display_update_task", 3072, NULL, 3, NULL);
}