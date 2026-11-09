#include <stdio.h>
#include <string.h>
#include <strings.h>     // strncasecmp для точного сравнения команд
#include <stdlib.h>
#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "driver/uart.h"
#include "esp_mac.h"          // Для чтения заводского MAC (серийного номера) из efuse

#include "pressure_sensor.h"  // Датчики давления + UART2 к панели
#include "config.h"           // Глобалы: давления, уставка P2, порт дисплея
#include "compressor_control.h"
// ВНИМАНИЕ: NVS на этой плате не используется (уставку хранит дисплейный
// контроллер, он же присылает её кадром $SLSS). Уставка живёт только в RAM.

#define KPA_PER_MPA 1000.0f

// ==========================================================================
// ВЫВОД ОТВЕТА: только внешней панели по UART2 (Wi-Fi в этой прошивке нет).
// Панель шлёт текстовые и NMEA-команды, ответы читает сама.
// ==========================================================================
static void respond(const char *msg) {
#if !SAFE_NO_PINS
    uart_write_bytes(DISPLAY_UART_NUM, msg, strlen(msg));
#else
    // Заглушка: UART2 (GPIO33/25) не установлен — в порт не пишем.
    (void)msg;
#endif
}

// ==========================================================================
// XOR-контрольная сумма NMEA: XOR байтов между '$' и '*' (не включая их).
// ==========================================================================
static uint8_t nmea_checksum(const char *line) {
    uint8_t cs = 0;
    const char *p = line;
    if (*p == '$') p++;   // по протоколу '$' не входит в XOR
    while (*p && *p != '*' && *p != '\r' && *p != '\n') {
        cs ^= (uint8_t)*p;
        p++;
    }
    return cs;
}

// Отправка NMEA-кадра по телу (без '$' и '*HH'): сам считает XOR.
static void send_nmea_body(const char *body) {
    uint8_t cs = 0;
    for (const char *p = body; *p; p++) cs ^= (uint8_t)*p;
    char out[96];
    snprintf(out, sizeof(out), "$%s*%02X\r\n", body, (unsigned)cs);
    respond(out);
}

// Подтверждение команды панели: $SLSA,<CMD>,OK|ERR*HH
static void send_ack(const char *cmd, bool ok) {
    char body[32];
    snprintf(body, sizeof(body), "SLSA,%s,%s", cmd, ok ? "OK" : "ERR");
    send_nmea_body(body);
}

// Ответ на $SLSC,GET: текущая уставка и порог (источник истины — мастер)
static void send_setpoint_frame(void) {
    char body[40];
    snprintf(body, sizeof(body), "SLSS,%.2f,%.2f",
             p2_set_kPa / KPA_PER_MPA, p2_start_kPa / KPA_PER_MPA);
    send_nmea_body(body);
}

// ==========================================================================
// Разбор кадра от панели (PANEL_PROTOCOL.md §4):
//   $SLSS,<P2_Set>,<P2_Start>          — уставка и порог повторного включения
//   $SLSC,START | STOP | RESET | GET
//   $SLSC,OUT,<K1>,<K2>,<R3>,<R4>      — сервисный тест выходов
//   $SLSC,TH,<S1_NP>,<S1_VP>,<S2_NP>,<S2_VP> — пороги ступеней (кПа)
// На каждую команду отвечаем $SLSA,<CMD>,OK|ERR (кроме GET — он получает $SLSS).
// Любая отбраковка пишется в лог с причиной — это главный инструмент наладки.
// ==========================================================================

// Обнуление модуля давления — единая реализация для $SLSC,ZERO,<n> и CLI "ZERO [n]".
// Повторно проверяем |P| < ZERO_MAX_ABS_KPA: дисплей не гарантия (датчик мог
// «поплыть» между кадрами). Обнуляется только выбранный модуль.
static bool zero_pressure_channel(int n) {
    if (n != 1 && n != 2) {
        ESP_LOGW("ZERO", "неверный номер модуля %d (нужно 1 или 2)", n);
        return false;
    }
    uint8_t addr = (n == 2) ? DEVICE_ADDRESS2 : DEVICE_ADDRESS1;
    float p = (n == 2) ? pressure2_kPa : pressure1_kPa;

    if (p > ZERO_MAX_ABS_KPA || p < -ZERO_MAX_ABS_KPA) {
        ESP_LOGW("ZERO", "P%d отклонено: |P|=%.3f МПа (нужно |P| < %.3f МПа)",
                 n, p / KPA_PER_MPA, ZERO_MAX_ABS_KPA / KPA_PER_MPA);
        return false;
    }

    ESP_LOGW("ZERO", "P%d: обнуляю датчик 0x%02X (P=%.3f МПа)", n, addr, p / KPA_PER_MPA);
    bool ok = performAdvancedZeroCalibration(0.0f, SENSOR_UART_NUM, addr);
    ESP_LOGW("ZERO", "P%d: %s", n, ok ? "OK" : "ОШИБКА (модуль не ответил)");

    if (ok) display_send_status_now();   // панель сразу видит нулевое давление
    return ok;
}

// $SLSS,<P2_Set МПа>[,<P2_Start МПа>]
static bool handle_slss(char *rest) {
    panel_slss_count++;   // видно в STATE как slss=N
    char *end = NULL;
    float set_mpa = strtof(rest, &end);
    if (end == rest) {
        ESP_LOGW("NMEA", "SLSS: не число в '%s'", rest);
        send_ack("SLSS", false);
        return false;
    }

    float start_mpa = 0.0f;
    bool has_start = false;
    if (*end == ',') {
        char *e2 = NULL;
        float v = strtof(end + 1, &e2);
        if (e2 != end + 1) {
            start_mpa = v;
            has_start = true;
        }
    }

    float set_kpa = set_mpa * KPA_PER_MPA;
    float start_kpa;
    if (has_start) {
        start_kpa = start_mpa * KPA_PER_MPA;
    } else {
        // Старый формат: P2_Start = MAX(1.0; P2_Set - 0.4) с учётом зазора
        start_kpa = set_kpa - P2_START_DROP_KPA;
        if (start_kpa < P_SYNC_SET_KPA) start_kpa = P_SYNC_SET_KPA;
        if (start_kpa > set_kpa - P2_MIN_GAP_KPA) start_kpa = set_kpa - P2_MIN_GAP_KPA;
        if (start_kpa < P2_START_MIN_KPA) start_kpa = P2_START_MIN_KPA;
    }

    // Защита от перепутанных полей: некоторые версии панели шлют
    // $SLSS,<P2_Start>,<P2_Set> (нижняя ручка первой). Если прямой порядок
    // не проходит по зазору, а обратный корректен — меняем местами с WARN.
    if (has_start && start_kpa > set_kpa - P2_MIN_GAP_KPA &&
        set_kpa > start_kpa + P2_MIN_GAP_KPA &&
        start_kpa >= P2_START_MIN_KPA && set_kpa <= P2_SET_MAX_KPA) {
        ESP_LOGW("NMEA", "SLSS: поля перепутаны (%.2f, %.2f) — меняю местами",
                 set_kpa / KPA_PER_MPA, start_kpa / KPA_PER_MPA);
        float tmp = set_kpa;
        set_kpa = start_kpa;
        start_kpa = tmp;
    }

    float applied_set = 0.0f, applied_start = 0.0f;
    if (!compressor_set_pressure_task_kpa(set_kpa, start_kpa, &applied_set, &applied_start)) {
        ESP_LOGW("NMEA", "SLSS: отклонено '%s'", rest);
        send_ack("SLSS", false);
        return false;
    }

    panel_slss_last_ms = (uint32_t)(esp_timer_get_time() / 1000);

    ESP_LOGI("NMEA", "SLSS%s: SET %.2f -> %.2f МПа, START %.2f -> %.2f МПа",
             has_start ? "" : " (1 поле)",
             set_kpa / KPA_PER_MPA, applied_set / KPA_PER_MPA,
             start_kpa / KPA_PER_MPA, applied_start / KPA_PER_MPA);
    ESP_LOGW("NMEA", "APPLY set=%.2f start=%.2f МПа (src=UART $SLSS)",
             applied_set / KPA_PER_MPA, applied_start / KPA_PER_MPA);
    send_ack("SLSS", true);
    return true;
}

// $SLSC,<CMD>[...]
static bool handle_slsc(char *rest) {
    if (strncasecmp(rest, "START", 5) == 0 && rest[5] == '\0') {
        bool ok = compressor_cmd_start();
        send_ack("START", ok);
        // Связь работает (команда дошла) — но уставки ещё не было: просим сразу,
        // не дожидаясь очередного 2-секундного опроса.
        if (!compressor_has_panel_setpoint()) display_request_setpoint_now();
        return ok;
    }
    if (strncasecmp(rest, "STOP", 4) == 0 && rest[4] == '\0') {
        compressor_cmd_stop();
        send_ack("STOP", true);
        return true;
    }
    if (strncasecmp(rest, "RESET", 5) == 0 && rest[5] == '\0') {
        compressor_cmd_reset();
        send_ack("RESET", true);
        return true;
    }
    if (strncasecmp(rest, "GET", 3) == 0 && (rest[3] == '\0' || rest[3] == ',')) {
        // Панель после своей перезагрузки спрашивает уставку у мастера
        send_setpoint_frame();
        return true;
    }

    // $SLSC,ZERO,<n> — обнуление модуля давления (n=1 -> P1, n=2 -> P2)
    if (strncasecmp(rest, "ZERO", 4) == 0) {
        int n = 0;
        if (sscanf(rest + 4, ",%d", &n) != 1 || (n != 1 && n != 2)) {
            ESP_LOGW("NMEA", "ZERO: нужно ZERO,1 или ZERO,2, принято '%s'", rest);
            send_ack("ZERO", false);
            return false;
        }
        bool ok = zero_pressure_channel(n);
        send_ack("ZERO", ok);
        return ok;
    }

    // $SLSC,OUT,<K1>,<K2>,<R3>,<R4> — только тест/диагностика в SERVICE
    if (strncasecmp(rest, "OUT,", 4) == 0) {
        int v[4] = {0};
        int n = sscanf(rest + 4, "%d,%d,%d,%d", &v[0], &v[1], &v[2], &v[3]);
        bool ok = (n == 4);
        for (int i = 0; i < 4; i++) {
            if (v[i] != 0 && v[i] != 1) ok = false;
        }
        if (!ok) {
            ESP_LOGW("NMEA", "OUT: нужно 4 поля 0/1, принято '%s'", rest);
            send_ack("OUT", false);
            return false;
        }
        bool accepted = compressor_test_outputs(v[0], v[1], v[2], v[3]);
        send_ack("OUT", accepted);
        return accepted;
    }

    // $SLSC,TH,<S1_NP>,<S1_VP>,<S2_NP>,<S2_VP> — пороги ступеней, кПа
    if (strncasecmp(rest, "TH,", 3) == 0) {
        panel_th_count++;   // видно в STATE как th=N
        float np1 = 0, vp1 = 0, np2 = 0, vp2 = 0;
        if (sscanf(rest + 3, "%f,%f,%f,%f", &np1, &vp1, &np2, &vp2) != 4) {
            ESP_LOGW("NMEA", "TH: нужно 4 числа, принято '%s'", rest);
            send_ack("TH", false);
            return false;
        }
        // TH справочный: уставку не перебивает (источник истины — $SLSS).
        bool ok = compressor_check_thresholds_kpa(np1, vp1, np2, vp2);

        // РЕЗЕРВНЫЙ путь получения уставки: в кадре TH поля S2_NP/S2_VP — это
        // копии нижней/верхней ручки панели. Если панель по какой-то причине
        // не шлёт $SLSS, но шлёт TH (S2 = ручки), берём уставку оттуда.
        // Только пока ни одного $SLSS не было — TH не должен перебивать $SLSS.
        if (ok && !compressor_has_panel_setpoint()) {
            float applied_set = 0.0f, applied_start = 0.0f;
            if (compressor_set_pressure_task_kpa(vp2, np2, &applied_set, &applied_start)) {
                panel_slss_last_ms = (uint32_t)(esp_timer_get_time() / 1000);
                ESP_LOGW("NMEA",
                         "TH: кадра $SLSS не было — беру уставку из S2 кадра TH: "
                         "set=%.2f start=%.2f МПа (src=UART $SLSC,TH)",
                         applied_set / KPA_PER_MPA, applied_start / KPA_PER_MPA);
            }
        }

        send_ack("TH", ok);
        return ok;
    }

    ESP_LOGW("NMEA", "SLSC: неизвестная команда '%s'", rest);
    return false;
}

static bool parse_display_nmea(char *line) {
    if (line[0] != '$') {
        ESP_LOGW("NMEA", "нет '$' в начале: '%s'", line);
        return false;
    }
    char *star = strchr(line, '*');
    if (star == NULL) {
        ESP_LOGW("NMEA", "нет '*' (чексуммы): '%s'", line);
        return false;
    }

    // контрольная сумма (считаем до отрезания '*')
    uint8_t cs = nmea_checksum(line);
    unsigned int cs_rx = 0;
    if (sscanf(star + 1, "%02x", &cs_rx) != 1) {
        ESP_LOGW("NMEA", "чексумма не читается: '%s'", line);
        return false;
    }
    if (cs_rx != cs) {
        ESP_LOGW("NMEA", "чексумма НЕ совпала: принято %02X, расчёт %02X ('%s')", cs_rx, cs, line);
        return false;
    }
    *star = '\0';   // дальше работаем только с телом кадра

    char type[5] = {0};
    char *body = line + 1;
    if (sscanf(body, "%4[^,]", type) != 1) {
        ESP_LOGW("NMEA", "не удалось прочитать тип: '%s'", line);
        return false;
    }
    char *rest = strchr(body, ',');
    if (rest == NULL) {
        ESP_LOGW("NMEA", "тип '%s' без запятой: '%s'", type, line);
        return false;
    }
    rest++; // после первой запятой

    if (strcmp(type, "SLSS") == 0) return handle_slss(rest);
    if (strcmp(type, "SLSC") == 0) return handle_slsc(rest);

    ESP_LOGW("NMEA", "неизвестный тип '%s' ('%s')", type, line);
    return false;
}

// ==========================================================================
// ТЕКСТОВЫЕ КОМАНДЫ (WiFi и не-NMEA строки с дисплея)
// Формат: START / STOP / RESET / SET <МПа> / STAT / SN / ZERO / DEMO / DEMOSTOP
// ==========================================================================

// Точное совпадение команды: без учёта регистра, с допуском пробелов и \r\n
// по краям. Нужно, чтобы "DEMOSTOP" не принимался за "DEMO", а случайный
// текст, содержащий буквы DEMO (например, эхо наших же ответов), не запускал
// демо реле.
static bool cmd_match(const char *str, const char *word) {
    if (str == NULL || word == NULL) return false;
    while (*str == ' ' || *str == '\t') str++;
    size_t n = strlen(word);
    if (strncasecmp(str, word, n) != 0) return false;
    const char *tail = str + n;
    while (*tail == ' ' || *tail == '\t') tail++;
    return (*tail == '\0' || *tail == '\r' || *tail == '\n');
}

static void handle_command(char *str) {

    // PING — простая проверка, что команды доходят и ответ уходит
    if (cmd_match(str, "PING")) {
        respond("\r\n>> OK PING\r\n");
        return;
    }

    if (strstr(str, "sn") || strstr(str, "SN")) {
        uint8_t mac[6];
        if (esp_efuse_mac_get_default(mac) != ESP_OK) {
            respond("\r\n>> SN: UNKNOWN\r\n");
            return;
        }
        char msg[48];
        snprintf(msg, sizeof(msg), "\r\n>> SN: %02X%02X%02X%02X%02X%02X\r\n",
                 mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
        respond(msg);
        return;
    }

    if (strstr(str, "stat") || strstr(str, "STAT")) {
        char msg[220];
        snprintf(msg, sizeof(msg),
                 "\r\n>> P1=%.2f P2=%.2f SET=%.2f ST=%.2f MPa | K1=%s K2=%s K3=%s K4=%s | ALM=%d RUN=%s | DEMO=%s\r\n",
                 pressure1_kPa / KPA_PER_MPA, pressure2_kPa / KPA_PER_MPA,
                 p2_set_kPa / KPA_PER_MPA, p2_start_kPa / KPA_PER_MPA,
                 comp1_on ? "ON" : "OFF", comp2_on ? "ON" : "OFF",
                 comp3_on ? "ON" : "OFF", comp4_on ? "ON" : "OFF",
                 compressor_get_alarm(),
                 compressor_is_running() ? "ON" : "OFF",
                 compressor_relay_demo_active() ? "ON" : "OFF");
        respond(msg);
        return;
    }

    if (strstr(str, "start") || strstr(str, "START")) {
        compressor_cmd_start();
        if (!compressor_has_panel_setpoint()) display_request_setpoint_now();
        respond(compressor_is_running() ? "\r\n>> RUNNING\r\n" : "\r\n>> [ERR] alarm active, do RESET first\r\n");
        return;
    }

    if (strstr(str, "stop") || strstr(str, "STOP")) {
        compressor_cmd_stop();
        respond("\r\n>> STOPPED\r\n");
        return;
    }

    if (strstr(str, "reset") || strstr(str, "RESET")) {
        compressor_cmd_reset();
        respond("\r\n>> ALARM RESET\r\n");
        return;
    }

    // DEMOSTOP — остановить непрерывный демо-цикл реле (проверяется ДО "DEMO")
    if (cmd_match(str, "DEMOSTOP")) {
        if (compressor_relay_demo_active()) {
            compressor_relay_demo_stop();
            respond("\r\n>> RELAY DEMO: stopping, all relays will be OFF\r\n");
        } else {
            respond("\r\n>> [ERR] relay demo is not running\r\n");
        }
        return;
    }

    // DEMO — непрерывный цикл: K1->K2->K3->K4 ON, затем K4->K3->K2->K1 OFF.
    // Только точное совпадение всей строки: START/STOP и прочие команды
    // сюда не попадают, а "DEMOSTOP" отсекается предыдущей проверкой.
    if (cmd_match(str, "DEMO")) {
        if (compressor_relay_demo_active()) {
            respond("\r\n>> [ERR] relay demo already running (use DEMOSTOP or STOP)\r\n");
        } else {
            compressor_relay_demo_start();
            respond("\r\n>> RELAY DEMO: cycle K1->K2->K3->K4 ON, then reverse OFF (DEMOSTOP to stop)\r\n");
        }
        return;
    }

    // SET <МПа> — уставка P2; P2_Start пересчитывается по алгоритму
    char *ps = strstr(str, "set ");
    if (ps == NULL) ps = strstr(str, "SET ");
    if (ps != NULL) {
        float mpa = strtof(ps + 4, NULL);
        float applied = compressor_set_setpoint_kpa(mpa * KPA_PER_MPA);
        char msg[96];
        snprintf(msg, sizeof(msg), "\r\n>> P2 target set to %.2f MPa (restart %.2f MPa)\r\n",
                 applied / KPA_PER_MPA, p2_start_kPa / KPA_PER_MPA);
        respond(msg);
        return;
    }

    if (strstr(str, "zero") || strstr(str, "ZERO")) {
        // ZERO / ZERO 1 -> P1, ZERO 2 -> P2. Та же реализация, что у $SLSC,ZERO,<n>.
        int n = 1;
        const char *pz = strstr(str, "zero");
        if (pz == NULL) pz = strstr(str, "ZERO");
        if (pz != NULL) {
            const char *pa = pz + 4;                 // после слова ZERO
            while (*pa == ' ' || *pa == '\t') pa++;  // пропускаем пробелы
            if (*pa == '2') n = 2;
        }

        char msg[64];
        snprintf(msg, sizeof(msg), "\r\n>> Zeroing pressure sensor P%d...\r\n", n);
        respond(msg);
        bool ok = zero_pressure_channel(n);
        snprintf(msg, sizeof(msg), ok ? "\r\n>> [OK] Sensor P%d zeroed\r\n"
                                      : "\r\n>> [ERR] Sensor P%d zero failed\r\n", n);
        respond(msg);
        return;
    }

    respond("\r\n>> Usage: START | STOP | RESET | SET <МПа> | STAT | SN | ZERO [1|2] | DEMO | DEMOSTOP | PING\r\n");
}

// ==========================================================================
// Задача приёма с внешнего дисплея (UART2, RX=GPIO25): строки NMEA $SLSS/$SLSC
// или текстовые команды. Строка между \r\n. Длина — как у панели (256 байт).
// ==========================================================================
// Готовность всей периферии (датчики, реле, автомат). До этого приём UART2
// уже работает, но из кадров применяется только $SLSS — он безопасен.
static volatile bool app_ready = false;

static void display_rx_task(void *pvParameters) {
    char line[DISPLAY_RX_MAX];
    int  idx = 0;

    uint32_t rx_bytes = 0, rx_frames = 0;
    bool     rx_ever = false;    // был ли хоть один кадр с момента включения
    uint32_t last_report = (uint32_t)(esp_timer_get_time() / 1000);

    // Разбивка принятых строк по типам за окно 30 с — чтобы одним взглядом
    // видеть, шлёт ли панель уставку или только PING.
    uint32_t n_ping = 0, n_slss = 0, n_slsc = 0, n_other = 0;
    uint32_t last_types = last_report;

    while (1) {
        uint8_t b;
        int n = uart_read_bytes(DISPLAY_UART_NUM, &b, 1, pdMS_TO_TICKS(20));
        if (n > 0) {
            // Отметка «панель жива» для флага NO_LINK в state кадра $SLSD
            panel_last_rx_ms = (uint32_t)(esp_timer_get_time() / 1000);
            rx_bytes++;
            if (b == '\n') {
                if (idx > 0) {
                    // убрать возможный \r в конце
                    if (line[idx - 1] == '\r') line[idx - 1] = '\0';
                    else                       line[idx] = '\0';
                    idx = 0;
                    rx_frames++;
                    rx_ever = true;
                    panel_rx_frames_total++;   // видно в STATE как rx=N

                    // Тип кадра для сводки
                    if (strncmp(line, "PING", 4) == 0)            n_ping++;
                    else if (strncmp(line + 1, "SLSS", 4) == 0)    n_slss++;
                    else if (strncmp(line + 1, "SLSC", 4) == 0)    n_slsc++;
                    else                                          n_other++;

                    // Полный дамп принятой строки: видно, что реально шлёт панель
                    ESP_LOGI("PANEL", "RX: '%s'", line);

                    if (line[0] == '$') {
                        // До окончания инициализации применяем ТОЛЬКО уставку:
                        // кадр $SLSS не трогает реле и датчики, поэтому его можно
                        // принять даже во время стартовой паузы (панель отправляет
                        // уставку при своём включении — раньше, чем мастер готов).
                        bool is_setpoint = (strncmp(line + 1, "SLSS,", 5) == 0);
                        if (app_ready || is_setpoint) {
                            parse_display_nmea(line);
                        } else {
                            ESP_LOGW("PANEL",
                                     "кадр до завершения инициализации пропущен: '%.40s'",
                                     line);
                        }
                        // Пришёл любой $SLSC, а уставки всё ещё нет — сразу
                        // просим её у панели (не ждём 2-секундного опроса).
                        if (app_ready && !compressor_has_panel_setpoint() &&
                            strncmp(line + 1, "SLSC,", 5) == 0) {
                            display_request_setpoint_now();
                        }
                    } else if (line[0] == '>') {
                        // эхо наших же ответов (">> ...") — командами не считаем
                        ESP_LOGD("PANEL", "echo ignored");
                    } else if (line[0] != '\0') {
                        if (app_ready) {
                            handle_command(line);
                        } else {
                            ESP_LOGW("PANEL",
                                     "команда до завершения инициализации пропущена: '%.40s'",
                                     line);
                        }
                    }
                }
            } else if (b == '\r') {
                // игнорируем, кадр заканчивается по \n
            } else if (b == '$') {
                // Ресинхронизация: кадр начинается с '$'. Если перед ним успел
                // накопиться мусор (потерянный \n, помеха), выбрасываем его,
                // иначе кадр уйдёт в handle_command() вместе с мусорным префиксом.
                if (idx > 1) {
                    ESP_LOGW("PANEL", "ресинхронизация: отброшено %d байт без конца строки", idx);
                }
                line[0] = '$';
                idx = 1;
            } else if (idx < (int)sizeof(line) - 1) {
                line[idx++] = (char)b;
            } else {
                // строка без '\n' длиннее буфера — сбрасываем, чтобы не склеивать кадры
                ESP_LOGW("PANEL", "строка > %d символов без \\n — сброс", (int)sizeof(line) - 1);
                idx = 0;
            }
        }

        // Диагностика линии RX раз в 5 секунд: приходят ли вообще байты от панели.
        // 0 байт -> проблема с проводом/пином/скоростью; байты есть, строк нет —
        // панель не завершает кадр '\n' или другая скорость.
        uint32_t now = (uint32_t)(esp_timer_get_time() / 1000);
        if ((uint32_t)(now - last_report) >= 5000u) {
            if (rx_bytes == 0 && !rx_ever) {
                // Ни одного байта с включения — это физика/передача панели.
                ESP_LOGW("PANEL",
                         "UART2 RX: с включения НИ ОДНОГО байта от панели. Проверьте TX панели -> GPIO%d, "
                         "GND, 115200. Нажмите кнопку на панели и сравните rx= в строке STATE",
                         PIN_DISP_RX);
            } else if (rx_bytes == 0) {
                // Панель событийная: молчит, когда нет действий. Это НЕ ошибка.
                ESP_LOGI("PANEL",
                         "RX тишина 5 с (панель шлёт только по действиям), всего принято %u кадров",
                         (unsigned)panel_rx_frames_total);
            } else if (rx_frames == 0) {
                ESP_LOGW("PANEL",
                         "UART2 RX: %u байт, но ни одной строки (нет '\\n'? другая скорость?)",
                         (unsigned)rx_bytes);
                // Hex-дамп накопленного: видно инверсию/скорость/мусор на линии
                char hex[3 * 32 + 1];
                int  lim = idx < 32 ? idx : 32;
                int  p = 0;
                hex[0] = '\0';
                for (int i = 0; i < lim && p < (int)sizeof(hex) - 4; i++) {
                    p += snprintf(hex + p, sizeof(hex) - (size_t)p, "%02X ", (unsigned)line[i] & 0xFFu);
                }
                ESP_LOGW("PANEL", "RX hex (%d байт): %s", lim, hex);
            } else {
                ESP_LOGI("PANEL", "UART2 RX: %u байт, %u строк за 5 с",
                         (unsigned)rx_bytes, (unsigned)rx_frames);
            }
            rx_bytes = 0;
            rx_frames = 0;
            last_report = now;
        }

        // Сводка по типам кадров раз в 30 с: PING / SLSS / SLSC / прочее.
        // Если PING растёт, а SLSS=0 — панель уставку не отправляет вообще.
        if ((uint32_t)(now - last_types) >= 30000u) {
            ESP_LOGI("PANEL", "RX 30 с по типам: PING=%u SLSS=%u SLSC=%u прочие=%u",
                     (unsigned)n_ping, (unsigned)n_slss,
                     (unsigned)n_slsc, (unsigned)n_other);
            n_ping = n_slss = n_slsc = n_other = 0;
            last_types = now;
        }
    }
    vTaskDelete(NULL);
}

// ==========================================================================
// Консольный ввод по UART0 (монитор). Строки читаются из stdin и идут в тот же
// handle_command(), что и команды с TCP/дисплея: DEMO, STAT, ZERO 2, SET 2.5 ...
// ==========================================================================
static void console_cmd_task(void *pvParameters) {
    (void)pvParameters;
    char line[128];

    while (1) {
        if (fgets(line, sizeof(line), stdin) != NULL) {
            size_t n = strlen(line);
            while (n > 0 && (line[n - 1] == '\n' || line[n - 1] == '\r')) line[--n] = '\0';
            if (n > 0) {
                ESP_LOGI("CONSOLE", "> %s", line);
                handle_command(line);
            }
        } else {
            vTaskDelay(pdMS_TO_TICKS(50));
        }
    }
}

// ==========================================================================
// ТОЧКА ВХОДА
// ==========================================================================
void app_main(void) {

    // NVS на этой плате НЕТ: уставку хранит дисплейный контроллер и присылает
    // её кадром $SLSS. До первого кадра работаем на умолчаниях из config.c
    // (4.00/3.60 МПа) — в логе будет WARN «уставка от панели не приходила».
    ESP_LOGW("INIT", "NVS отключён: уставка только в RAM, источник — $SLSS от панели "
             "(умолчания Set=%.2f Start=%.2f МПа)",
             P2_SET_DEFAULT_KPA / 1000.0f, P2_START_DEFAULT_KPA / 1000.0f);

    // UART2 поднимаем СРАЗУ, до стартовой паузы: панель отправляет $SLSS при
    // собственном включении (и при отпускании ручки), и если мастер ещё «спит»,
    // кадр терялся — мастер потом работал на умолчаниях. Теперь драйвер копит
    // байты в RX-буфере, а задача приёма применяет $SLSS даже во время паузы.
    // Передача $SLSD на панель включается позже — pressure_display_link_start().
    pressure_display_uart_init();
#if !SAFE_NO_PINS
    xTaskCreate(display_rx_task, "display_rx", 3072, NULL, 6, NULL);
#else
    // Заглушка: UART2/RX (GPIO25) не настроен, приём от панели не запускаем.
    (void)display_rx_task;
    ESP_LOGW("INIT", "SAFE: приём с внешнего дисплея (UART2 RX=GPIO%d) отключён", PIN_DISP_RX);
#endif

    // ==========================================================================
    // Стартовая пауза: пока она идёт, прошивка НЕ трогает силовые выводы —
    // реле не сконфигурированы, датчики не опрашиваются. UART2 при этом уже
    // слушает панель (см. выше), но $SLSD на панель ещё не отправляется.
    // Длительность — STARTUP_DELAY_MS в config.h.
    // ==========================================================================
#if STARTUP_DELAY_MS > 0
    ESP_LOGW("INIT", "Стартовая пауза %d мс: силовые выводы не трогаем (дисплей стартует)",
             STARTUP_DELAY_MS);
    vTaskDelay(pdMS_TO_TICKS(STARTUP_DELAY_MS));
    ESP_LOGW("INIT", "Пауза окончена, начинаю инициализацию периферии");
#endif

    // Локального экрана нет — только UART2 (TX33/RX25) к панели
    pressure_sensor_init();         // общая шина UART1: датчики 0x01 и 0x02 по кругу

    compressor_control_init();      // реле K1..K4 (GPIO2/17/21/22) — выключены
    compressor_control_start();     // автомат двухступенчатой установки

    // Теперь можно говорить панели состояние: $SLSD каждые 200 мс
    pressure_display_link_start();
    app_ready = true;

    // Ввод команд из монитора (UART0): DEMO запускает проверку реле
    xTaskCreate(console_cmd_task, "console_cmd", 3072, NULL, 4, NULL);

    // Wi-Fi в этой прошивке нет: связь с панелью — только UART2.
    // Все рабочие задачи уже созданы, главному циклу делать нечего.
    ESP_LOGI("INIT", "Инициализация завершена: датчики, реле, UART2, команды");
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
