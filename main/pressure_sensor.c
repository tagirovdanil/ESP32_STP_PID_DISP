#include "pressure_sensor.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "st7789.h"
#include "esp_timer.h"
#include "fontx.h"
#include "driver/gpio.h"
#include "esp_spiffs.h"
#include "config.h"
#include "compressor_control.h"
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

// ===== Датчик №2 (независимая шина UART2): свои буферы, счётчики и таймер =====
esp_timer_handle_t pressure_timer2;
volatile float pressure2_kPa = 0;   // показания второго датчика (extern в .h)
uint8_t rxBuffer2[12];
uint8_t rxIndex2 = 0;
bool frameStarted2 = false;
uint32_t sum_err2 = 0;              // CRC-ошибки второго датчика
uint32_t lastPressureTime2 = 0;

/* ========================================================================== */
/*   ПРИВАТНЫЕ (СЛУЖЕБНЫЕ) ФУНКЦИИ — СНАРУЖИ ИХ НЕ ВИДНО                      */
/* ========================================================================== */

// Фоновая задача или функция циклического чтения данных из буфера UART
static void processIncomingData(void);

// Если dev и fx16 объявлены в main.c, даем знать файлу pressure_sensor.c о них:
extern TFT_t dev;

void display_update_task(void *pvParameters) {
#if SAFE_NO_PINS
    // Заглушка: местный экран и кнопка BOOT (GPIO0) не опрашиваются.
    (void)pvParameters;
    ESP_LOGW(TAG, "SAFE: display_update_task отключена (экран не инициализирован)");
    vTaskDelete(NULL);
#endif
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

            // P1 (1-я ступень), МПа
            snprintf(screen_buffer, sizeof(screen_buffer), "%.2f MPa", pressure1_kPa / 1000.0f);
            lcdDrawString(&dev, fx16, 25, 140, (uint8_t*)screen_buffer, YELLOW);

            // P2 (2-я ступень), МПа
            snprintf(screen_buffer, sizeof(screen_buffer), "%.2f MPa", pressure2_kPa / 1000.0f);
            lcdDrawString(&dev, fx16, 45, 140, (uint8_t*)screen_buffer, CYAN);

            // Уставка P2, МПа
            snprintf(screen_buffer, sizeof(screen_buffer), "%.2f MPa", p2_set_kPa / 1000.0f);
            lcdDrawString(&dev, fx16, 65, 140, (uint8_t*)screen_buffer, GREEN);

            // K1/K2 — подписи в правой колонке (как P1/P2/SET), каждая на своей
            // строке; красным — компрессор включён, белым — выключен.
            dev._font_fill = true;
            dev._font_fill_color = BLACK;
            snprintf(screen_buffer, sizeof(screen_buffer), "K1");
            lcdDrawString(&dev, fx16, 85, 220, (uint8_t*)screen_buffer, comp1_on ? RED : WHITE);
            snprintf(screen_buffer, sizeof(screen_buffer), "K2");
            lcdDrawString(&dev, fx16, 105, 220, (uint8_t*)screen_buffer, comp2_on ? RED : WHITE);

            // Режим/авария — в колонке значений (строка 85): сначала стираем, потом пишем
            snprintf(screen_buffer, sizeof(screen_buffer), "        ");
            lcdDrawString(&dev, fx16, 85, 140, (uint8_t*)screen_buffer, BLACK);
            int alm = compressor_get_alarm();
            if (alm != 0) {
                snprintf(screen_buffer, sizeof(screen_buffer), "AL %d", alm);
                lcdDrawString(&dev, fx16, 85, 140, (uint8_t*)screen_buffer, RED);
            } else {
                snprintf(screen_buffer, sizeof(screen_buffer), "%-4s",
                         compressor_is_running() ? "RUN" : "STOP");
                lcdDrawString(&dev, fx16, 85, 140, (uint8_t*)screen_buffer,
                              compressor_is_running() ? GREEN : WHITE);
            }

            dev._font_fill = false;
            lcdDrawFinish(&dev);
        }

        // Логирование показаний раз в ~1 с (каждый 5-й цикл по 200 мс)
        static uint32_t sensor_log_cnt = 0;
        if ((++sensor_log_cnt % 5) == 0) {
            ESP_LOGI("SENSOR", "P1=%.2f P2=%.2f MPa | SET=%.2f | ALM=%d K1=%d K2=%d",
                     pressure1_kPa / 1000.0f, pressure2_kPa / 1000.0f,
                     p2_set_kPa / 1000.0f, compressor_get_alarm(),
                     comp1_on ? 1 : 0, comp2_on ? 1 : 0);
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
#if SAFE_NO_PINS
    // Заглушка: SPI и выводы LCD (GPIO18/19/5/16/23/4) не инициализируются.
    ESP_LOGW(TAG, "SAFE: LCD_init пропущен — ST7789 и его пины отключены");
    return;
#endif
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

// ===== Датчик №2 (UART2): прототипы =====
// NB: IRAM_ATTR только в определении (не в прототипе), иначе GCC ругается на
// конфликт секций ".iram1.1"/".iram1.2".
void timerCallback2(void *arg);
static void handlePressureResponse2(uint8_t *frame, uint8_t length);
static void handleUART2ReceivedFrame(uint8_t *frame, uint8_t length);
static void processIncomingData2(void);

// ==========================================================================
// РЕАЛИЗАЦИЯ ФУНКЦИЙ
// ==========================================================================
// Настройка UART1 датчика давления
void pressure_sensor_init(void) {
#if SAFE_NO_PINS
    // Заглушка: UART1 (TX=GPIO27, RX=GPIO26) не занимаем — выводы «в воздухе».
    ESP_LOGW(TAG, "SAFE: UART1 датчика №1 не настраивается (TX=GPIO%d, RX=GPIO%d)",
             PIN_TX1, PIN_RX1);
#else
    uart_config_t uart_config = {
        .baud_rate = 19200, .data_bits = UART_DATA_8_BITS, .parity = UART_PARITY_ODD,
        .stop_bits = UART_STOP_BITS_1, .flow_ctrl = UART_HW_FLOWCTRL_DISABLE, .source_clk = UART_SCLK_DEFAULT
    };
    uart_driver_install(SENSOR_UART_NUM, BUF_SIZE * 2, 0, 0, NULL, 0);
    uart_param_config(SENSOR_UART_NUM, &uart_config);
    uart_set_pin(SENSOR_UART_NUM, PIN_TX1, PIN_RX1, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
#endif

    const esp_timer_create_args_t timer_args = { .callback = &timerCallback, .name = "pressure_timer" };
    esp_timer_create(&timer_args, &pressure_timer);
#if !SAFE_NO_PINS
    esp_timer_start_periodic(pressure_timer, SERIAL_UART1_INTERVAL * 1000);
#else
    ESP_LOGW(TAG, "SAFE: опрос датчика №1 (каждые %d мс) не запущен", SERIAL_UART1_INTERVAL);
#endif

    
    ESP_LOGI(TAG, "Модуль датчика давления №1 успешно инициализирован.");

    // Инициализируем и датчик №2 (UART2) — отдельная независимая шина
    pressure_sensor2_init();
}

// Настройка UART2 датчика давления №2 (отдельная независимая шина).
// У обоих датчиков одинаковый адрес, поэтому второй сидит на своём UART2.
void pressure_sensor2_init(void) {
#if SAFE_NO_PINS
    // Заглушка: UART2 (TX=GPIO33, RX=GPIO25) не занимаем — выводы «в воздухе».
    ESP_LOGW(TAG, "SAFE: UART2 датчика №2 не настраивается (TX=GPIO%d, RX=GPIO%d)",
             PIN_TX2, PIN_RX2);
#else
    uart_config_t uart_config = {
        .baud_rate = 19200, .data_bits = UART_DATA_8_BITS, .parity = UART_PARITY_ODD,
        .stop_bits = UART_STOP_BITS_1, .flow_ctrl = UART_HW_FLOWCTRL_DISABLE, .source_clk = UART_SCLK_DEFAULT
    };
    uart_driver_install(SENSOR2_UART_NUM, BUF_SIZE * 2, 0, 0, NULL, 0);
    uart_param_config(SENSOR2_UART_NUM, &uart_config);
    uart_set_pin(SENSOR2_UART_NUM, PIN_TX2, PIN_RX2, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
#endif

    const esp_timer_create_args_t timer_args = { .callback = &timerCallback2, .name = "pressure_timer2" };
    esp_timer_create(&timer_args, &pressure_timer2);
#if !SAFE_NO_PINS
    esp_timer_start_periodic(pressure_timer2, SERIAL_UART2_INTERVAL * 1000);
#else
    ESP_LOGW(TAG, "SAFE: опрос датчика №2 (каждые %d мс) не запущен", SERIAL_UART2_INTERVAL);
#endif

    ESP_LOGI(TAG, "Датчик давления №2 (UART2: RX=%d, TX=%d) инициализирован.", PIN_RX2, PIN_TX2);
}

bool performAdvancedZeroCalibration(float offset_kPa, uart_port_t uart_num) {
#if SAFE_NO_PINS
    // Заглушка: калибровка невозможна — UART датчика не поднят (ничего не дёргаем).
    (void)offset_kPa;
    (void)uart_num;
    is_calibrating = false;
    ESP_LOGW(TAG, "SAFE: калибровка нуля пропущена");
    return false;
#endif
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
    if ((raw_value == OVF_PATTERN) || isnan(fval.value) || fval.value > PRESSURE_MAX_KPA || fval.value < -100.0f) {
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
        // Валидация заголовка (портировано из старого проекта):
        //  frame[3] — длина данных, всегда 0x07 (единица изм. + 4 байта float +
        //             статус устройства + статус команды);
        //  frame[5] — статус команды, 0x00 = OK.
        // XOR-чексумма выше ловит не всё: при рассинхронизации сборщик может
        // начать «кадр» с 0x06 из ДАННЫХ, и случайное окно проходит XOR с
        // вероятностью 1/256 (так в регулятор пролез отсчёт ~118 кПа при
        // реальных 200). Эти два байта режут ложные кадры ещё в ~65536 раз.
        if (frame[3] != 0x07 || frame[5] != 0x00) {
            sum_err++;
            ESP_LOGW("SENSOR", "Отброшен кадр с битым заголовком: dlen=0x%02X cmd_status=0x%02X",
                     frame[3], frame[5]);
            return;
        }
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

// ==========================================================================
//  ДАТЧИК №2 (UART2, независимая шина)
//  Полностью повторяет логику датчика №1, но на своём UART2 и со своими
//  буферами/счётчиками — два одинаковых по адресу датчика (0x02) на разных
//  шинах отвечают только своему UART, поэтому между собой не конфликтуют.
// ==========================================================================
void IRAM_ATTR timerCallback2(void *arg) {
    // 1. Сначала забираем и парсим ответ на ПРЕДЫДУЩИЙ запрос
    processIncomingData2();
    // 2. Отправляем НОВЫЙ запрос датчику №2 (он ответит к следующему тику таймера)
    uint8_t prCmd2[] = { 0x02, DEVICE_ADDRESS1, 0x01, 0x00, 0x00 };
    prCmd2[4] = calculateChecksum(prCmd2, 4);
    uart_write_bytes(SENSOR2_UART_NUM, (const char*)prCmd2, sizeof(prCmd2));
}

// Приватный обработчик ответа датчика №2
static void handlePressureResponse2(uint8_t *frame, uint8_t length) {
    union { uint8_t bytes[4]; float value; } fval;
    fval.bytes[0] = frame[10]; fval.bytes[1] = frame[9];
    fval.bytes[2] = frame[8];  fval.bytes[3] = frame[7];

    uint32_t raw_value;
    memcpy(&raw_value, &fval.value, 4);
    const uint32_t OVF_PATTERN = 0x7F800000;
    if ((raw_value == OVF_PATTERN) || isnan(fval.value) || fval.value > PRESSURE_MAX_KPA || fval.value < -100.0f) {
        sum_err2++;
        return;
    }

    pressure2_kPa = fval.value;
    lastPressureTime2 = esp_timer_get_time() / 1000;
}

// Приватный обработчик кадра датчика №2
static void handleUART2ReceivedFrame(uint8_t *frame, uint8_t length) {
    if (length < 3) return;
    if (frame[0] != ACK || frame[1] != DEVICE_ADDRESS1) return;

    uint8_t calcChecksum = calculateChecksum(frame, length - 1);
    if (calcChecksum != frame[length - 1]) {
        sum_err2++;
        return;
    }

    if (frame[2] == 0x01 && length >= 12) {
        // Та же валидация заголовка, что и у датчика №1 (см. handleUART1ReceivedFrame)
        if (frame[3] != 0x07 || frame[5] != 0x00) {
            sum_err2++;
            ESP_LOGW("SENSOR2", "Датчик №2: отброшен кадр с битым заголовком: dlen=0x%02X cmd_status=0x%02X",
                     frame[3], frame[5]);
            return;
        }
        handlePressureResponse2(frame, 12);
    }
}

// Приватная функция чтения UART2 и сборки кадров датчика №2
static void processIncomingData2(void) {
    uint8_t incomingByte;
    while (uart_read_bytes(SENSOR2_UART_NUM, &incomingByte, 1, 0) > 0) {
        if (!frameStarted2 && incomingByte == ACK) {
            frameStarted2 = true;
            rxIndex2 = 0;
            rxBuffer2[rxIndex2++] = incomingByte;
            continue;
        }
        if (frameStarted2 && rxIndex2 < 12) {
            rxBuffer2[rxIndex2++] = incomingByte;
        }
        if (rxIndex2 == 12) {
            handleUART2ReceivedFrame(rxBuffer2, 12);
            frameStarted2 = false;
            rxIndex2 = 0;
        }
    }
}

// ==========================================================================
//  Отправка состояния на внешний дисплей по UART0 (TX=GPIO21) в формате NMEA:
//      $SLSD,<P1 МПа>,<P2 МПа>,<K1>,<K2>,<alarm>*<HH>\r\n
//  Дисплей шлёт нам $SLSS/$SLSC обратно (RX=GPIO22) — их разбирает
//  display_rx_task в main.c.
// ==========================================================================
static void display_link_task(void *arg) {
    char body[96];
    char out[128];
    while (1) {
        int bn = snprintf(body, sizeof(body), "SLSD,%.2f,%.2f,%d,%d,%d",
                          pressure1_kPa / 1000.0f,
                          pressure2_kPa / 1000.0f,
                          comp1_on ? 1 : 0,
                          comp2_on ? 1 : 0,
                          compressor_get_alarm());
        uint8_t cs = 0;
        for (int i = 0; i < bn; i++) cs ^= (uint8_t)body[i];
        int n = snprintf(out, sizeof(out), "$%s*%02X\r\n", body, (unsigned)cs);
        uart_write_bytes(DISPLAY_UART_NUM, out, n);
        vTaskDelay(pdMS_TO_TICKS(DISPLAY_SEND_MS));
    }
}

void pressure_ui_and_usb_init(TFT_t *p_dev) {
#if SAFE_NO_PINS
    // Заглушка: UART0 (TX=GPIO21/RX=GPIO22) не занимаем, ST7789 не поднимаем,
    // задачи отрисовки и отправки $SLSD не создаём — ни один вывод не дёргается.
    (void)p_dev;
    (void)display_link_task;
    ESP_LOGW("INIT", "SAFE: UART0 панели (TX=GPIO%d, RX=GPIO%d) и экран отключены",
             PIN_DISP_TX, PIN_DISP_RX);
    return;
#endif

    // ==========================================================================
    // UART0: порт данных на внешний дисплей.
    // Пины переназначены с GPIO1/3 (USB-консоль) на GPIO21 (TX) / GPIO22 (RX).
    // Прошивка НЕ ломается: ROM-загрузчик общается по GPIO1/3 по умолчанию.
    // ==========================================================================
    uart_config_t uart_config = {
        .baud_rate = DISPLAY_UART_BAUD,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    uart_driver_install(DISPLAY_UART_NUM, 512, 512, 0, NULL, 0);
    uart_param_config(DISPLAY_UART_NUM, &uart_config);
    uart_set_pin(DISPLAY_UART_NUM, PIN_DISP_TX, PIN_DISP_RX, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

    ESP_LOGI("INIT", "UART0 -> внешний дисплей (TX=GPIO%d, RX=GPIO%d)", PIN_DISP_TX, PIN_DISP_RX);

    // (рамки/разделители убраны — экран чистый, красивости добавим позже)
    
    // Включаем прозрачный фон для статического текста
    lcdUnsetFontFill(p_dev);

    // Рисуем неизменяемые подписи (передаем разыменованный указатель на шрифт *p_fx16)
    lcdDrawString(p_dev, fx16, 25, 220, (uint8_t*)"P1:", YELLOW);
    lcdDrawString(p_dev, fx16, 45, 220, (uint8_t*)"P2:", CYAN);
    lcdDrawString(p_dev, fx16, 65, 220, (uint8_t*)"SET:", GREEN);

    // Рисуем заставку «PLEASE WAIT» или рамки, если они нужны
    lcdDrawFinish(p_dev);
    
    // Задача локального экрана TTGO
    // Выделяем ей 3072 байт стека и приоритет 3
    xTaskCreate(display_update_task, "display_update_task", 3072, NULL, 3, NULL);
    // Задача передачи данных на внешний большой дисплей
    xTaskCreate(display_link_task, "display_link", 3072, NULL, 4, NULL);
}