#include "pressure_sensor.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "driver/uart.h"
#include "config.h"
#include "compressor_control.h"
#include <math.h>

static const char *TAG = "SENSOR_MODULE";
// Служебные переменные для расчета кПа/сек
//static float previous_pressure = 0.0f;
//static bool nyrok_done = false; // Флаг, что один нырок в атмосферу при перелете выполнен


esp_timer_handle_t pressure_timer;

// ===== ОБЩАЯ ШИНА UART1: оба датчика (0x01 и 0x02) на одних проводах =====
// Буфер сборки кадра один: принадлежность кадра определяется по frame[1] —
// байту адреса устройства. Оба датчика отвечают кадром ровно 12 байт.
uint8_t rxBuffer1[12];
uint8_t rxIndex1 = 0;
bool frameStarted1 = false;
uint32_t sum_err = 0;               // ошибки кадра датчика №1 (адрес 0x01)
uint32_t sum_err2 = 0;              // ошибки кадра датчика №2 (адрес 0x02)
uint32_t lastPressureTime = 0;
uint32_t lastPressureTime2 = 0;
volatile float pressure2_kPa = 0;   // показания второго датчика (extern в .h)

// Адрес, которому уйдёт СЛЕДУЮЩИЙ запрос: 0x01 -> 0x02 -> 0x01 -> ...
static uint8_t poll_addr = DEVICE_ADDRESS1;

/* ========================================================================== */
/*   ПРИВАТНЫЕ (СЛУЖЕБНЫЕ) ФУНКЦИИ — СНАРУЖИ ИХ НЕ ВИДНО                      */
/* ========================================================================== */

// Фоновая задача или функция циклического чтения данных из буфера UART
static void processIncomingData(void);

// Локального экрана на этой плате НЕТ (ST7789/шрифты вырезаны).
// Всё общение — UART0 (консоль), UART2 (панель) и Wi-Fi/m531.

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
    
// ==========================================================================
// ПРИВАТНЫЕ ПРОТОТИПЫ (Видны ТОЛЬКО внутри этого файла)
// ==========================================================================

// Тик общего опроса шины UART1 (период — SENSOR_POLL_INTERVAL_MS).
// Сначала вычитываем и разбираем ответ на ПРЕДЫДУЩИЙ запрос, затем отправляем
// запрос СЛЕДУЮЩЕМУ адресу. Так запросы к 0x01 и 0x02 чередуются, а пауза
// между ними равна периоду таймера (по умолчанию 50 мс).
static void timerCallback(void *arg) {
    (void)arg;
    processIncomingData();

    uint8_t prCmd[] = { 0x02, poll_addr, 0x01, 0x00, 0x00 };
    prCmd[4] = calculateChecksum(prCmd, 4);
    uart_write_bytes(SENSOR_UART_NUM, (const char*)prCmd, sizeof(prCmd));

    poll_addr = (poll_addr == DEVICE_ADDRESS1) ? DEVICE_ADDRESS2 : DEVICE_ADDRESS1;
}

// Разбор принятого кадра: адрес в кадре решает, какому датчику он принадлежит
static void handleSensorFrame(uint8_t *frame, uint8_t length);
// Обработчики показаний конкретного датчика
static void handlePressureResponse1(uint8_t *frame, uint8_t length);
static void handlePressureResponse2(uint8_t *frame, uint8_t length);

// ==========================================================================
// РЕАЛИЗАЦИЯ ФУНКЦИЙ
// ==========================================================================
// Настройка ОБЩЕЙ шины UART1: на ней два датчика с адресами 0x01 и 0x02.
// Опрос по кругу (round-robin) с периодом SENSOR_POLL_INTERVAL_MS.
void pressure_sensor_init(void) {
#if SAFE_NO_PINS
    // Заглушка: UART1 (TX=GPIO27, RX=GPIO26) не занимаем — выводы «в воздухе».
    ESP_LOGW(TAG, "SAFE: UART1 датчиков не настраивается (TX=GPIO%d, RX=GPIO%d)",
             PIN_TX1, PIN_RX1);
#else
    uart_config_t uart_config = {
        .baud_rate = 19200, .data_bits = UART_DATA_8_BITS, .parity = UART_PARITY_ODD,
        .stop_bits = UART_STOP_BITS_1, .flow_ctrl = UART_HW_FLOWCTRL_DISABLE, .source_clk = UART_SCLK_DEFAULT
    };
    uart_driver_install(SENSOR_UART_NUM, BUF_SIZE * 2, 0, 0, NULL, 0);
    uart_param_config(SENSOR_UART_NUM, &uart_config);
    uart_set_pin(SENSOR_UART_NUM, PIN_TX1, PIN_RX1, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

    ESP_LOGI(TAG, "Общая шина UART1 (RX=%d, TX=%d): опрос 0x%02X/0x%02X каждые %d мс",
             PIN_RX1, PIN_TX1, DEVICE_ADDRESS1, DEVICE_ADDRESS2, SENSOR_POLL_INTERVAL_MS);
#endif

    const esp_timer_create_args_t timer_args = { .callback = &timerCallback, .name = "sensor_poll" };
    esp_timer_create(&timer_args, &pressure_timer);
#if !SAFE_NO_PINS
    poll_addr = DEVICE_ADDRESS1;
    esp_timer_start_periodic(pressure_timer, SENSOR_POLL_INTERVAL_MS * 1000);
#else
    ESP_LOGW(TAG, "SAFE: опрос датчиков (каждые %d мс) не запущен", SENSOR_POLL_INTERVAL_MS);
#endif

    ESP_LOGI(TAG, "Модуль датчиков давления инициализирован (общая шина UART1).");
}

// Калибровка нуля датчика на общей шине. device_addr — адрес датчика (0x01/0x02):
// команда уходит на тот же UART1, но конкретному устройству.
bool performAdvancedZeroCalibration(float offset_kPa, uart_port_t uart_num, uint8_t device_addr) {
#if SAFE_NO_PINS
    // Заглушка: калибровка невозможна — UART датчиков не поднят (ничего не дёргаем).
    (void)offset_kPa;
    (void)uart_num;
    (void)device_addr;
    is_calibrating = false;
    ESP_LOGW(TAG, "SAFE: калибровка нуля пропущена");
    return false;
#endif
    // 1. Включаем флаг, чтобы заблокировать отрисовку экрана на время калибровки
    is_calibrating = true; 

    esp_timer_stop(pressure_timer);
    vTaskDelay(150 / portTICK_PERIOD_MS);
    clearBufferCompletely(uart_num);

    uint8_t zeroCmd[9] = { 0x02, device_addr, 0x2B, 0x04 };
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
        // Возобновляем опрос с адреса 0x01, иначе round-robin может «застрять»
        poll_addr = DEVICE_ADDRESS1;
        esp_timer_start_periodic(pressure_timer, SENSOR_POLL_INTERVAL_MS * 1000);
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

    // Возобновляем опрос с адреса 0x01
    poll_addr = DEVICE_ADDRESS1;
    esp_timer_start_periodic(pressure_timer, SENSOR_POLL_INTERVAL_MS * 1000);
    
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
    // Отрицательные показания допустимы (атмосферный дрейф нуля): нижний
    // предел берём единый с автоматом — PRESSURE_MIN_KPA, а не «0 кПа».
    const uint32_t OVF_PATTERN = 0x7F800000;
    if ((raw_value == OVF_PATTERN) || isnan(fval.value) ||
        fval.value > PRESSURE_MAX_KPA || fval.value < PRESSURE_MIN_KPA) {
        sum_err++;
        return;
    }

    pressure1_kPa = fval.value;
    lastPressureTime = esp_timer_get_time() / 1000;
}

// Приватный обработчик показаний датчика №2 (адрес 0x02)
static void handlePressureResponse2(uint8_t *frame, uint8_t length) {
    (void)length;
    union { uint8_t bytes[4]; float value; } fval;
    fval.bytes[0] = frame[10]; fval.bytes[1] = frame[9];
    fval.bytes[2] = frame[8];  fval.bytes[3] = frame[7];

    uint32_t raw_value;
    memcpy(&raw_value, &fval.value, 4);
    // Отрицательные показания допустимы (атмосферный дрейф нуля): нижний
    // предел берём единый с автоматом — PRESSURE_MIN_KPA, а не «0 кПа».
    const uint32_t OVF_PATTERN = 0x7F800000;
    if ((raw_value == OVF_PATTERN) || isnan(fval.value) ||
        fval.value > PRESSURE_MAX_KPA || fval.value < PRESSURE_MIN_KPA) {
        sum_err2++;
        return;
    }

    pressure2_kPa = fval.value;
    lastPressureTime2 = esp_timer_get_time() / 1000;
}

// ==========================================================================
//  ОБЩИЙ РАЗБОР КАДРОВ С ШИНЫ UART1
//  Кадр: [0]=ACK(0x06) [1]=адрес [2]=команда(0x01) [3]=длина данных(0x07)
//        [5]=статус команды(0x00) ... [11]=XOR
//  Принадлежность определяет frame[1]: 0x01 -> P1, 0x02 -> P2.
// ==========================================================================
static void handleSensorFrame(uint8_t *frame, uint8_t length) {
    if (length < 3 || frame[0] != ACK) return;

    const uint8_t addr = frame[1];
    uint32_t *err_cnt;

    if (addr == DEVICE_ADDRESS1) {
        err_cnt = &sum_err;
    } else if (addr == DEVICE_ADDRESS2) {
        err_cnt = &sum_err2;
    } else {
        // На этой шине других устройств нет — кадр битый/рассинхронизация
        sum_err++;
        return;
    }

    if (calculateChecksum(frame, length - 1) != frame[length - 1]) {
        (*err_cnt)++;
        return;
    }

    if (frame[2] != 0x01 || length < 12) return;

    // Валидация заголовка (портировано из старого проекта):
    //  frame[3] — длина данных, всегда 0x07 (единица изм. + 4 байта float +
    //             статус устройства + статус команды);
    //  frame[5] — статус команды, 0x00 = OK.
    // XOR-чексумма выше ловит не всё: при рассинхронизации сборщик может
    // начать «кадр» с 0x06 из ДАННЫХ, и случайное окно проходит XOR с
    // вероятностью 1/256 (так в регулятор пролез отсчёт ~118 кПа при
    // реальных 200). Эти два байта режут ложные кадры ещё в ~65536 раз.
    if (frame[3] != 0x07 || frame[5] != 0x00) {
        (*err_cnt)++;
        ESP_LOGW("SENSOR", "Датчик 0x%02X: отброшен кадр с битым заголовком: dlen=0x%02X cmd_status=0x%02X",
                 addr, frame[3], frame[5]);
        return;
    }

    if (addr == DEVICE_ADDRESS1) {
        handlePressureResponse1(frame, 12);
    } else {
        handlePressureResponse2(frame, 12);
    }
}

// Приватная функция чтения общей шины UART1 и сборки кадров.
// Буфер один: оба датчика отвечают кадром ровно 12 байт, начиная с ACK.
static void processIncomingData(void) {
    uint8_t incomingByte;
    while (uart_read_bytes(SENSOR_UART_NUM, &incomingByte, 1, 0) > 0) {
        if (!frameStarted1) {
            if (incomingByte != ACK) continue;   // ищем начало кадра
            frameStarted1 = true;
            rxIndex1 = 0;
        }
        rxBuffer1[rxIndex1++] = incomingByte;
        if (rxIndex1 >= 12) {
            handleSensorFrame(rxBuffer1, 12);
            frameStarted1 = false;
            rxIndex1 = 0;
        }
    }
}

// ==========================================================================
//  Отправка состояния на внешний дисплей по UART2 (TX=GPIO33) в формате NMEA.
//  Расширенный кадр (обратно совместим: панель читает 5 полей, хвост игнорирует):
//      $SLSD,<P1>,<P2>,<K1>,<K2>,<alarm>,<K3>,<K4>,<state>*<HH>
//  K3 — фактическое состояние реле охлаждения (в т.ч. по выдержке 60 с),
//  K4 — резерв (штатно 0), state — COMP_STATE_STOP/RUN/ALARM/SERVICE/PAUSE2.
//  Панель шлёт нам $SLSS/$SLSC обратно (RX=GPIO25) — их разбирает
//  display_rx_task в main.c. Выводы 21/22 отданы реле K1/K2, реле K3/K4 —
//  17/2, поэтому линия панели на 33/25.
// ==========================================================================
// Один кадр $SLSD. Вынесено отдельно, чтобы после обнуления датчиков ($SLSC,ZERO)
// сразу отдать панели свежие нулевые значения, не дожидаясь очередных 200 мс.
static void display_send_status_frame(void) {
    char body[96];
    char out[128];

    // NO_LINK: панель молчит дольше DISPLAY_LINK_MS -> state + 10.
    // Управление при этом НЕ прекращается: панель — только HMI.
    uint32_t now = (uint32_t)(esp_timer_get_time() / 1000);
    bool panel_link = ((uint32_t)(now - panel_last_rx_ms) < DISPLAY_LINK_MS);
    int state = compressor_get_state();
    if (!panel_link) state += 10;

    int bn = snprintf(body, sizeof(body), "SLSD,%.2f,%.2f,%d,%d,%d,%d,%d,%d",
                      pressure1_kPa / 1000.0f,
                      pressure2_kPa / 1000.0f,
                      comp1_on ? 1 : 0,
                      comp2_on ? 1 : 0,
                      compressor_get_alarm(),
                      comp3_on ? 1 : 0,
                      comp4_on ? 1 : 0,
                      state);
    uint8_t cs = 0;
    for (int i = 0; i < bn; i++) cs ^= (uint8_t)body[i];
    int n = snprintf(out, sizeof(out), "$%s*%02X\r\n", body, (unsigned)cs);
    uart_write_bytes(DISPLAY_UART_NUM, out, n);
}

void display_send_status_now(void) {
#if !SAFE_NO_PINS
    display_send_status_frame();
#endif
}

// Передача на панель включается только после стартовой паузы и инициализации:
// так панель не получает кадры, пока сама ещё грузится. Приём при этом уже идёт.
static volatile bool display_tx_enabled = false;

void pressure_display_link_start(void) {
    display_tx_enabled = true;
    ESP_LOGI("PANEL", "передача $SLSD на панель разрешена (UART2 TX=GPIO%d)",
             PIN_DISP_TX);
}

// Попросить у панели уставку: $SLSC,GET -> панель отвечает $SLSS,<Set>,<Start>.
// Нужно на случай, когда панель отправила $SLSS раньше, чем мастер был готов
// его принять (или уставка на панели менялась до включения мастера).
static void display_send_get_request(void) {
    static const char body[] = "SLSC,GET";
    uint8_t cs = 0;
    for (const char *p = body; *p; p++) cs ^= (uint8_t)*p;

    char out[32];
    int n = snprintf(out, sizeof(out), "$%s*%02X\r\n", body, (unsigned)cs);
    uart_write_bytes(DISPLAY_UART_NUM, out, n);
}

void display_request_setpoint_now(void) {
#if SAFE_NO_PINS
    return;
#else
    // Не чаще 1 раза в 500 мс: команды панели могут идти пачкой (ZERO,1 + START).
    static uint32_t last_req_ms = 0;
    uint32_t now = (uint32_t)(esp_timer_get_time() / 1000);
    if (last_req_ms != 0 && (uint32_t)(now - last_req_ms) < 500u) return;
    last_req_ms = now;

    display_send_get_request();

    // Лог не чаще раза в 10 с, чтобы не заливать консоль.
    static uint32_t last_log_ms = 0;
    if (last_log_ms == 0 || (uint32_t)(now - last_log_ms) >= 10000u) {
        ESP_LOGW("PANEL", "прошу уставку у панели: $SLSC,GET (нужен ответ $SLSS)");
        last_log_ms = now;
    }
#endif
}

static void display_link_task(void *arg) {
    (void)arg;
    uint32_t last_get_ms = 0;
    bool get_warned = false;

    while (1) {
        if (!display_tx_enabled) {
            vTaskDelay(pdMS_TO_TICKS(DISPLAY_SEND_MS));
            continue;
        }

        display_send_status_frame();

        // Уставки от панели ещё не было — раз в 2 с просим её прислать.
        uint32_t now = (uint32_t)(esp_timer_get_time() / 1000);
        if (!compressor_has_panel_setpoint() &&
            (uint32_t)(now - last_get_ms) >= 2000u) {
            display_send_get_request();
            if (!get_warned) {
                ESP_LOGW("PANEL", "уставка от панели не приходила — прошу её кадром "
                         "$SLSC,GET (панель должна ответить $SLSS)");
                get_warned = true;
            }
            last_get_ms = now;
        }

        vTaskDelay(pdMS_TO_TICKS(DISPLAY_SEND_MS));
    }
}

// UART2 — единственная линия к панели (GPIO33 TX / GPIO25 RX).
// Локального экрана нет: SPI/ST7789/шрифты/spiffs больше не поднимаются.
void pressure_display_uart_init(void) {
#if SAFE_NO_PINS
    (void)display_link_task;
    ESP_LOGW("INIT", "SAFE: UART2 панели (TX=GPIO%d, RX=GPIO%d) отключён",
             PIN_DISP_TX, PIN_DISP_RX);
    return;
#endif

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

    ESP_LOGI("INIT", "UART2 -> панель (TX=GPIO%d, RX=GPIO%d)", PIN_DISP_TX, PIN_DISP_RX);

    // Задача отправки $SLSD на панель
    xTaskCreate(display_link_task, "display_link", 3072, NULL, 4, NULL);
}