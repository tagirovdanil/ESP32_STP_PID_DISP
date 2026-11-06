# Задача агенту: TTGO-контроллер принимает команды от m531 по Wi-Fi

Репозиторий: `ESP32_STP_PID_DISP`, ветка **`uart-ttgo`** (последний коммит e80039b,
«алгоритм работает_гистерезис…»). Таргет `esp32` (WROOM, без PSRAM), ESP-IDF 5.5.3.

## 0. Что нужно получить

Есть первая панель **m531** (большой HMI на ESP32-S3, отдельный проект, **уже доработан, его
не трогать**). Оператор пишет в USB-терминале m531:

```
send_panel2 SET 1.6
```

m531 отправляет по Wi-Fi строку `SET 1.6` нашей плате (TTGO). TTGO должна выполнить её
**ровно так, как если бы её набрали в мониторе UART0**, то есть через существующий
`handle_command()` в `main/main.c`, и вернуть m531 текст ответа плюс маркер конца `<END>`.
Это касается любой команды терминала: `SET <МПа>`, `STAT`, `START`, `STOP`, `RESET`, `ZERO [1|2]`,
`SN`, `DEMO`, `DEMOSTOP`. Плюс добавить отладочную команду **`PING`**.

`LOCK_SCREEN` / `UNLOCK_SCREEN` (были в старом образце `esp32-2`) **не нужны**, не добавлять.

## 1. Как устроена связь (сторона m531 уже готова)

| | m531 (панель 1) | TTGO (эта плата) |
|---|---|---|
| Wi-Fi | точка доступа `Metran531`, пароль `12345678`, канал 1, адрес **192.168.4.1/24** | **станция** (STA), получает 192.168.4.x по DHCP |
| TCP | сервер, порт **3333** | **клиент**, сам делает `connect(192.168.4.1:3333)` |
| Команды | отправляет | исполняет и отвечает |

TCP двунаправленный: TTGO подключается к серверу m531 как обычный клиент и дальше просто ждёт
строки в `recv()`. Свой сервер для этого не нужен, а m531 не нужно знать IP платы.

Протокол (строки ASCII, конец строки `\n`, `\r` игнорируется):

```
TTGO → m531   "HELLO PANEL2\n"        сразу после каждого connect(): «я вторая панель, а не ПК»
m531 → TTGO   "SET 1.6\n"             команда (ровно то, что оператор набрал после send_panel2)
TTGO → m531   "\r\n>> P2 target set to 1.60 MPa (restart 1.20 MPa)\r\n"   ответ, строк сколько угодно
TTGO → m531   "<END>\r\n"             конец ответа, ОБЯЗАТЕЛЕН на каждую команду
```

Что важно знать о поведении m531:

- Без `HELLO PANEL2` m531 считает подключившегося обычным ПК: его строки исполняет как свои
  команды, а сама ему ничего не шлёт. Поэтому HELLO отправлять **после каждого** переподключения.
- После HELLO m531 строки от TTGO **не исполняет**. Это ответы: она копит их до `<END>` и
  показывает плашкой «Панель 2: …» внизу своего экрана (и пишет в лог `WTSRV: панель2 → "…"`).
  Без `<END>` плашка не появится.
- Пустые строки m531 отбрасывает, так что ведущий `\r\n` в ответах `handle_command()` не мешает.
- Одна строка ответа не длиннее **127 символов** (буфер m531 128 байт), длиннее обрежется.
  Плашка показывает примерно 150 символов, полный текст есть в логе m531.
- m531 сама **ничего не пишет** этому сокету, кроме команд: ни `<END>`, ни ошибок, ни эха.
  Исключение — отказ при переполнении слотов (до HELLO): `Error: too many clients` + `<END>`.
  Такое не исполнять.
- Слот «второй панели» у m531 **один**: новое HELLO закрывает предыдущее соединение с HELLO.
  Значит, `HELLO PANEL2` должно слать только одно устройство в сети.
- Wi-Fi на m531 **выключен по умолчанию** и включается тапом по иконке. Точка доступа может
  появляться и пропадать в любой момент, поэтому TTGO должна переподключаться бесконечно.
  При выключении Wi-Fi m531 закрывает соединение сама (FIN). При пропаже питания закрытия не
  будет, нужен keepalive.

## 2. Что уже есть в TTGO и что из этого следует

- `main/wifi_tcp.cpp/.h`: Wi-Fi в режиме **AP** (`ESP32_PID`/`12345678`, канал 1,
  **192.168.4.1**) плюс TCP-сервер :3333 для телефона. Строки клиентов кладутся в очередь и
  исполняются в главном цикле `app_main` через `wifi_tcp_poll_command()` → `handle_command()`.
  Там же уже есть режим **STA** (`WIFI_TCP_MODE_AP 0` → `wifi_init_sta()`), его и берём за основу.
- `main/main.c`: `handle_command(char *str)` — текстовые команды; `respond(msg)` рассылает ответ
  всем TCP-клиентам (`wifi_tcp_send`) и в UART2 внешнему дисплею. `handle_command()` вызывается
  из трёх задач: `console_cmd_task` (UART0), `display_rx_task` (UART2) и главного цикла (Wi-Fi).
- `sdkconfig` в `.gitignore`, `sdkconfig.defaults` нет. **Всё делать в коде**, без зависимости
  от новых опций menuconfig.

**Решение по своей точке доступа: убрать.** Своя AP `ESP32_PID` больше не нужна, телефон к TTGO
напрямую не подключается. Плата работает **только станцией** (`WIFI_MODE_STA`) в сети m531.
Это заодно снимает конфликт подсетей: своя AP была на `192.168.4.1/24`, ровно там же, где m531,
и при AP+STA `connect(192.168.4.1)` ушёл бы в собственный интерфейс.

## 3. Архитектура решения

1. `wifi_tcp.cpp`: только **STA**, подключение к `Metran531` с бесконечными повторами.
   Своя AP `ESP32_PID` не поднимается.
2. Новый модуль **`main/m531_link.c/.h`** (C, не C++): TCP-клиент к m531. Ждёт IP на STA,
   подключается, шлёт HELLO, собирает строки в очередь. Wi-Fi сам **не поднимает**.
3. `main/main.c`: главный цикл забирает команды из `m531_link_poll()` и выполняет тем же
   `handle_command()`. На время выполнения `respond()` дополнительно пишет ответ в сокет m531,
   после команды уходит `<END>`. Команды исполняются в той же задаче, что и Wi-Fi-команды
   из `wifi_tcp`, поэтому новых гонок нет.
4. `PING` → ответ `>> OK PING`.

## 4. Изменения по файлам

### 4.1 `main/wifi_tcp.h`

- `WIFI_TCP_MODE_AP` по умолчанию **0** (STA). Код AP (`wifi_init_softap()`, `WIFI_TCP_AP_*`)
  можно удалить или оставить мёртвым под `#if`, но по умолчанию AP не поднимается.
- Параметры станции — сеть m531:

```c
// Сеть m531 (панель 1). Обязаны совпадать с wifi_tcp_server.h проекта m531
// (WTSRV_AP_SSID / WTSRV_AP_PASS / WTSRV_AP_CHANNEL).
#define WIFI_TCP_STA_SSID          "Metran531"
#define WIFI_TCP_STA_PASS          "12345678"
#define WIFI_TCP_STA_CHANNEL       1        // канал AP m531: сканируем только его
#define WIFI_TCP_STA_RECONNECT_MS  3000     // пауза между попытками найти AP m531
```

- Поправить шапку-комментарий: плата — станция в сети m531, своей AP нет.

### 4.2 `main/wifi_tcp.cpp`

1. `wifi_init_sta()` доработать (это C++: `= {}` вместо designated initializers):

```cpp
#include "esp_timer.h"

static esp_timer_handle_t s_sta_retry_timer = NULL;
static void sta_retry_cb(void*) { esp_wifi_connect(); }

static void wifi_init_sta(void)
{
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    esp_wifi_set_storage(WIFI_STORAGE_RAM);         // конфиг Wi-Fi во флеш не пишем

    esp_timer_create_args_t targs = {};
    targs.callback = &sta_retry_cb;
    targs.name     = "sta_retry";
    ESP_ERROR_CHECK(esp_timer_create(&targs, &s_sta_retry_timer));

    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL));

    wifi_config_t wc = {};
    strncpy((char*)wc.sta.ssid,     WIFI_TCP_STA_SSID, sizeof(wc.sta.ssid));
    strncpy((char*)wc.sta.password, WIFI_TCP_STA_PASS, sizeof(wc.sta.password));
    wc.sta.channel            = WIFI_TCP_STA_CHANNEL;
    wc.sta.scan_method        = WIFI_FAST_SCAN;
    wc.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wc));
    ESP_ERROR_CHECK(esp_wifi_start());
    esp_wifi_set_ps(WIFI_PS_NONE);                  // иначе команда ждала бы до ~300 мс

    ESP_LOGI(TAG, "STA: ищу точку доступа m531 \"%s\"", WIFI_TCP_STA_SSID);
}
```

2. `wifi_event_handler`, ветка `WIFI_EVENT_STA_DISCONNECTED`: **не** звать `esp_wifi_connect()`
   сразу, а `esp_timer_start_once(s_sta_retry_timer, WIFI_TCP_STA_RECONNECT_MS * 1000ULL)`
   (ошибку «таймер уже запущен» игнорировать). Wi-Fi на m531 выключен по умолчанию, и пока
   его не включат, это событие приходит постоянно. Мгновенный переподключ означает
   непрерывный скан эфира и сплошной WARN в логе. WARN «отключились» печатать только при
   первом разрыве после успешного подключения (или не чаще раза в 30 с).
3. TCP-сервер (`server_task`) можно оставить: он слушает `INADDR_ANY`, в сети m531 до него
   можно достучаться с ПК по IP платы из лога (`STA: получен IP 192.168.4.x`), это удобно для
   отладки. Без своей AP он больше ничему не мешает. Если проще — отключить, на задачу это
   не влияет.

### 4.3 Новый модуль `main/m531_link.h` / `main/m531_link.c`

Сделан по образцу проверенного модуля `wifi_link.c` (проект esp32-2), но **без собственной
инициализации Wi-Fi**: Wi-Fi поднимает `wifi_tcp_init()`.

`m531_link.h`:

```c
#pragma once
#include <stdbool.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

// Обязаны совпадать с wifi_tcp_server.h проекта m531.
#define M531_SERVER_IP     "192.168.4.1"   // адрес SoftAP m531
#define M531_SERVER_PORT   3333            // = WTSRV_PORT
#define M531_HELLO         "HELLO PANEL2"  // = WTSRV_PEER_HELLO
#define M531_END_MARKER    "<END>"         // = WTSRV_END_MARKER
#define M531_MSG_MAX       128             // = PEER_MSG_MAX у m531
#define M531_RX_QUEUE_LEN  8
#define M531_RECONNECT_MS  2000

// Запустить задачу связи. Звать ПОСЛЕ wifi_tcp_init() (нужны event loop и STA-netif).
void m531_link_init(void);

// Забрать одну команду от m531 (без \r\n). false = команд нет. Звать из главного цикла.
bool m531_link_poll(char *out, size_t out_size);

// Отправить текст m531 как есть; если не кончается на '\n' — допишет "\r\n".
// false = связи нет.
bool m531_link_write(const char *text);

// Маркер конца ответа "<END>\r\n". Слать после КАЖДОЙ команды.
bool m531_link_end(void);

// Есть ли TCP-соединение с m531 (для индикации на экране / в STAT).
bool m531_link_connected(void);

#ifdef __cplusplus
}
#endif
```

`m531_link.c` (основа: можно брать как есть):

```c
#include "m531_link.h"

#include <string.h>
#include <errno.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"
#include "freertos/semphr.h"

#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "lwip/sockets.h"

static const char *TAG = "M531";

#define GOT_IP_BIT  BIT0

typedef struct { char s[M531_MSG_MAX]; } link_msg_t;

static QueueHandle_t      s_rxq    = NULL;
static EventGroupHandle_t s_eg     = NULL;   // GOT_IP_BIT = мы в сети m531
static SemaphoreHandle_t  s_tx_mtx = NULL;   // охраняет s_sock и send()
static int                s_sock   = -1;

static void on_event(void *arg, esp_event_base_t base, int32_t id, void *data)
{
    if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED) {
        xEventGroupClearBits(s_eg, GOT_IP_BIT);
    } else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *e = (ip_event_got_ip_t *)data;
        ESP_LOGI(TAG, "в сети m531, наш IP " IPSTR, IP2STR(&e->ip_info.ip));
        xEventGroupSetBits(s_eg, GOT_IP_BIT);
    }
}

static int link_connect(void)
{
    struct sockaddr_in a = {0};
    a.sin_family = AF_INET;
    a.sin_port   = htons(M531_SERVER_PORT);
    inet_pton(AF_INET, M531_SERVER_IP, &a.sin_addr);

    int fd = socket(AF_INET, SOCK_STREAM, IPPROTO_IP);
    if (fd < 0) { ESP_LOGE(TAG, "socket() errno=%d", errno); return -1; }
    if (connect(fd, (struct sockaddr *)&a, sizeof(a)) != 0) {
        // AP уже есть, а сервер m531 ещё не слушает — обычное дело, повторим.
        ESP_LOGW(TAG, "connect(%s:%d) errno=%d", M531_SERVER_IP, M531_SERVER_PORT, errno);
        close(fd);
        return -1;
    }
    struct timeval rtv = { .tv_sec = 0, .tv_usec = 200 * 1000 };  // recv просыпается раз в 200 мс
    setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &rtv, sizeof(rtv));
    struct timeval stv = { .tv_sec = 0, .tv_usec = 300 * 1000 };  // главный цикл не виснет на send
    setsockopt(fd, SOL_SOCKET, SO_SNDTIMEO, &stv, sizeof(stv));

    // Keepalive с явными таймингами: у m531 пропало питание — заметим за ~11 с.
    int ka = 1, idle = 5, intvl = 2, cnt = 3;
    setsockopt(fd, SOL_SOCKET,  SO_KEEPALIVE,  &ka,    sizeof(ka));
    setsockopt(fd, IPPROTO_TCP, TCP_KEEPIDLE,  &idle,  sizeof(idle));
    setsockopt(fd, IPPROTO_TCP, TCP_KEEPINTVL, &intvl, sizeof(intvl));
    setsockopt(fd, IPPROTO_TCP, TCP_KEEPCNT,   &cnt,   sizeof(cnt));
    return fd;
}

static bool link_write_raw(const char *s)
{
    bool ok = false;
    xSemaphoreTake(s_tx_mtx, portMAX_DELAY);
    if (s_sock >= 0) {
        size_t len = strlen(s), off = 0;
        ok = true;
        while (off < len) {
            int n = send(s_sock, s + off, len - off, 0);
            if (n <= 0) { ESP_LOGW(TAG, "send() errno=%d", errno); ok = false; break; }
            off += (size_t)n;
        }
    }
    xSemaphoreGive(s_tx_mtx);
    return ok;
}

static void rx_push(const char *line)
{
    // Служебные строки сервера и эхо ответов — не команды.
    if (strcmp(line, M531_END_MARKER) == 0 || strncmp(line, "Error:", 6) == 0 || line[0] == '>') {
        ESP_LOGW(TAG, "не команда, пропускаю: \"%s\"", line);
        return;
    }
    link_msg_t m;
    strlcpy(m.s, line, sizeof(m.s));
    if (xQueueSend(s_rxq, &m, 0) != pdTRUE)
        ESP_LOGW(TAG, "очередь полна, \"%s\" потеряна", line);
}

static void link_task(void *arg)
{
    char rx[128];
    char line[M531_MSG_MAX];

    while (1) {
        xEventGroupWaitBits(s_eg, GOT_IP_BIT, pdFALSE, pdTRUE, portMAX_DELAY);

        int fd = link_connect();
        if (fd < 0) { vTaskDelay(pdMS_TO_TICKS(M531_RECONNECT_MS)); continue; }

        xSemaphoreTake(s_tx_mtx, portMAX_DELAY);
        s_sock = fd;
        xSemaphoreGive(s_tx_mtx);
        size_t len = 0;
        bool overflow = false;

        link_write_raw(M531_HELLO "\n");          // без HELLO m531 не считает нас панелью 2
        ESP_LOGI(TAG, "на связи с m531 (sock=%d)", fd);

        while (xEventGroupGetBits(s_eg) & GOT_IP_BIT) {
            int n = recv(fd, rx, sizeof(rx), 0);
            if (n == 0) { ESP_LOGI(TAG, "m531 закрыла соединение"); break; }
            if (n < 0) {
                if (errno == EAGAIN || errno == EWOULDBLOCK) continue;
                ESP_LOGW(TAG, "recv() errno=%d", errno);
                break;
            }
            for (int i = 0; i < n; i++) {
                char ch = rx[i];
                if (ch == '\r') continue;
                if (ch != '\n') {
                    if (len < sizeof(line) - 1) line[len++] = ch; else overflow = true;
                    continue;
                }
                line[len] = '\0';
                if (overflow)     ESP_LOGW(TAG, "слишком длинная строка отброшена");
                else if (len > 0) rx_push(line);
                len = 0;
                overflow = false;
            }
        }

        // Сначала убрать сокет из-под m531_link_write (мьютекс дождётся отправки), потом закрыть.
        xSemaphoreTake(s_tx_mtx, portMAX_DELAY);
        s_sock = -1;
        xSemaphoreGive(s_tx_mtx);
        close(fd);
        ESP_LOGW(TAG, "связь с m531 потеряна, повтор через %d мс", M531_RECONNECT_MS);
        vTaskDelay(pdMS_TO_TICKS(M531_RECONNECT_MS));
    }
}

void m531_link_init(void)
{
    s_rxq    = xQueueCreate(M531_RX_QUEUE_LEN, sizeof(link_msg_t));
    s_eg     = xEventGroupCreate();
    s_tx_mtx = xSemaphoreCreateMutex();

    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, WIFI_EVENT_STA_DISCONNECTED, on_event, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT,   IP_EVENT_STA_GOT_IP,         on_event, NULL));

    // IP мог прийти раньше регистрации обработчика — проверить текущее состояние.
    esp_netif_t *sta = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
    esp_netif_ip_info_t ip;
    if (sta && esp_netif_get_ip_info(sta, &ip) == ESP_OK && ip.ip.addr != 0)
        xEventGroupSetBits(s_eg, GOT_IP_BIT);

    xTaskCreate(link_task, "m531_link", 4096, NULL, 5, NULL);
}

bool m531_link_poll(char *out, size_t out_size)
{
    if (!s_rxq || !out || out_size == 0) return false;
    link_msg_t m;
    if (xQueueReceive(s_rxq, &m, 0) != pdTRUE) return false;
    strlcpy(out, m.s, out_size);
    return true;
}

bool m531_link_write(const char *text)
{
    if (!text) return false;
    size_t n = strlen(text);
    bool ok = link_write_raw(text);
    if (ok && (n == 0 || text[n - 1] != '\n')) ok = link_write_raw("\r\n");
    return ok;
}

bool m531_link_end(void)      { return link_write_raw(M531_END_MARKER "\r\n"); }
bool m531_link_connected(void) { return s_sock >= 0; }
```

### 4.4 `main/main.c`

1. `#include "m531_link.h"`.
2. Маршрутизация ответа. `respond()` вызывается из трёх задач, поэтому в m531 пишем **только**
   ответ команды, которую главный цикл выполняет по запросу m531:

```c
static TaskHandle_t  s_main_task     = NULL;    // задача app_main (главный цикл)
static volatile bool s_reply_to_m531 = false;   // ставит/снимает только главный цикл

static void respond(const char *msg) {
    wifi_tcp_send(msg);
#if !SAFE_NO_PINS
    uart_write_bytes(DISPLAY_UART_NUM, msg, strlen(msg));
#endif
    // Ответ на команду от m531 — ещё и ей. Проверка задачи: параллельно respond()
    // зовут display_rx_task и console_cmd_task, их вывод m531 не нужен.
    if (s_reply_to_m531 && xTaskGetCurrentTaskHandle() == s_main_task)
        m531_link_write(msg);
}
```

   Рассылку в UART2 и TCP-клиентам `wifi_tcp` оставить как была: так ведут себя и остальные
   команды.
3. `PING` в самом начале `handle_command()` через уже существующий точный `cmd_match()`:

```c
    if (cmd_match(str, "PING")) {           // отладка связи (m531: send_panel2 PING)
        respond("\r\n>> OK PING\r\n");
        return;
    }
```

   Строку usage в конце `handle_command()` дополнить `| PING`.
4. `app_main`: после `wifi_tcp_init()`:

```c
    s_main_task = xTaskGetCurrentTaskHandle();
    m531_link_init();                        // TCP-клиент к m531 (Wi-Fi уже поднят)

    char wcmd[256];
    while (1) {
        while (wifi_tcp_poll_command(wcmd, sizeof(wcmd))) {
            handle_command(wcmd);
        }
        // Команды от m531 — тем же handle_command, как набранные в терминале.
        while (m531_link_poll(wcmd, sizeof(wcmd))) {
            ESP_LOGI("M531", "команда: '%s'", wcmd);
            s_reply_to_m531 = true;
            handle_command(wcmd);
            s_reply_to_m531 = false;
            m531_link_end();                 // <END> обязателен: по нему m531 показывает ответ
        }
        vTaskDelay(pdMS_TO_TICKS(20));
    }
```

`handle_command()` **не менять** ради этой задачи: команда от m531 должна вести себя ровно как
набранная в терминале. Например, `SET 1.6` → `compressor_set_setpoint_kpa(1600)` →
`P2_Start = 1.20` → сохранение в NVS → ответ `>> P2 target set to 1.60 MPa (restart 1.20 MPa)`.

### 4.5 Сборка и документация

- `main/CMakeLists.txt`: добавить `"m531_link.c"` в `SRCS`. Нужные компоненты (`esp_wifi`,
  `esp_netif`, `esp_event`, `lwip`, `esp_timer`) в `PRIV_REQUIRES` уже есть.
- `СПРАВКА_ПОРТЫ.md`, строка «WiFi» в §1: своей AP `ESP32_PID` больше нет; плата —
  станция в сети `Metran531` (m531) и принимает команды от неё. Добавить короткий раздел
  «Связь с m531» с протоколом из §1 этого документа и проверкой из §6.
- В комментарии-шапке `wifi_tcp.h` поправить описание режима (только STA в сети m531).

## 5. Чего не делать

- Не трогать NMEA-обмен с внешним дисплеем по UART2 (`$SLSS`/`$SLSC`/`$SLSD`), алгоритм
  компрессоров, датчики.
- Не исполнять команды m531 в сетевой задаче `m531_link`: только через очередь в главном цикле.
- Не поднимать Wi-Fi второй раз в `m531_link.c` (никаких `esp_wifi_init`/`esp_netif_init` там).
- Не поднимать свою AP (`ESP32_PID`) и не включать `WIFI_MODE_APSTA` (см. §2).
- Не отвечать m531 ничем «от себя» вне ответа на команду: каждая строка после HELLO — это
  ответ, она попадёт на экран m531.

## 6. Проверка (приёмка)

Сборка: `idf.py build` без ошибок и новых warning'ов в изменённых файлах.

| # | Действие | Ожидание |
|---|---|---|
| 1 | Прошить TTGO, Wi-Fi на m531 выключен | Лог TTGO спокойный: STA пробует раз в ~3 с, без спама; сети `ESP32_PID` в эфире нет |
| 2 | На m531 тапнуть иконку Wi-Fi | Через несколько секунд в логе TTGO `M531: в сети m531, наш IP 192.168.4.x`, затем `на связи с m531`; в логе m531 `WTSRV: вторая панель на связи` |
| 3 | В USB-мониторе m531: `send_panel2 PING` | m531 отвечает `OK SEND_PANEL2`; лог TTGO `M531: команда: 'PING'`; на экране m531 плашка «Панель 2: >> OK PING» |
| 4 | `send_panel2 SET 1.6` | На TTGO уставка 1.60, повторный пуск 1.20 (видно в `STAT` и на экране TTGO); плашка на m531 `>> P2 target set to 1.60 MPa (restart 1.20 MPa)` |
| 5 | `send_panel2 STAT` | Плашка с P1/P2/SET/K1..K4 |
| 6 | `send_panel2 XYZ` | Плашка с usage-строкой (ответ и `<END>` есть даже на неизвестную команду) |
| 7 | Во время `send_panel2 STAT` набрать `STAT` в мониторе UART0 TTGO | На m531 приходит только ответ на её команду; ответ терминалу TTGO в m531 не попадает |
| 8 | Выключить Wi-Fi на m531, включить снова | TTGO переподключается сама, `send_panel2 PING` снова работает |
| 9 | Выдернуть питание m531 | TTGO замечает обрыв за ~11 с (keepalive), потом ищет AP; компрессорная логика не останавливается |
| 10 | Перезагрузить TTGO при работающей m531 | После старта переподключается, m531 пишет «вторая панель на связи», старый слот закрыт |

Проверять нужно с реальной m531 (её прошивка с командой `send_panel2` уже готова): точку доступа
с адресом ровно 192.168.4.1 обычный ПК не поднимет (хот-спот Windows раздаёт 192.168.137.x).
