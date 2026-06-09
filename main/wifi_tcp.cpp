#include "wifi_tcp.h"

#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include <errno.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"

#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "nvs_flash.h"

#include "lwip/sockets.h"
#include "lwip/netdb.h"
#include "lwip/tcp.h"      // TCP_KEEPIDLE / TCP_KEEPINTVL / TCP_KEEPCNT

static const char* TAG = "WIFI_TCP";

#define CMD_QUEUE_LEN   16
#define CMD_MAX_LEN     256
#define RX_CHUNK        512

// --- TCP keepalive: мёртвый клиент будет обнаружен за ~11 с -------------------
#define KEEPALIVE_IDLE_S    5   // начать пробы после 5 с тишины
#define KEEPALIVE_INTVL_S   2   // повторять пробы каждые 2 с
#define KEEPALIVE_COUNT     3   // 3 неответа = мёртв

typedef struct { char data[CMD_MAX_LEN]; } cmd_msg_t;

// --- Очередь принятых команд (заберёт основной цикл) --------------------------
static QueueHandle_t s_cmd_queue = NULL;

// --- Список активных клиентов + mutex ----------------------------------------
static int s_client_socks[WIFI_TCP_MAX_CLIENTS];
static SemaphoreHandle_t s_clients_mutex = NULL;

static void clients_init(void) {
    for (int i = 0; i < WIFI_TCP_MAX_CLIENTS; i++) s_client_socks[i] = -1;
    s_clients_mutex = xSemaphoreCreateMutex();
}

static bool clients_add(int sock) {
    bool ok = false;
    xSemaphoreTake(s_clients_mutex, portMAX_DELAY);
    for (int i = 0; i < WIFI_TCP_MAX_CLIENTS; i++) {
        if (s_client_socks[i] == -1) { s_client_socks[i] = sock; ok = true; break; }
    }
    xSemaphoreGive(s_clients_mutex);
    return ok;
}

static void clients_remove(int sock) {
    xSemaphoreTake(s_clients_mutex, portMAX_DELAY);
    for (int i = 0; i < WIFI_TCP_MAX_CLIENTS; i++) {
        if (s_client_socks[i] == sock) { s_client_socks[i] = -1; break; }
    }
    xSemaphoreGive(s_clients_mutex);
}

// Вытесняет одного клиента из списка и возвращает его сокет (или -1).
// Сам сокет НЕ закрывает — это сделает его собственный client_task,
// когда recv проснётся из-за shutdown().
static int clients_evict_one(void) {
    int victim = -1;
    xSemaphoreTake(s_clients_mutex, portMAX_DELAY);
    for (int i = 0; i < WIFI_TCP_MAX_CLIENTS; i++) {
        if (s_client_socks[i] != -1) {
            victim = s_client_socks[i];
            s_client_socks[i] = -1;
            break;
        }
    }
    xSemaphoreGive(s_clients_mutex);
    return victim;
}

// ============================ WiFi события ===================================
static void wifi_event_handler(void* arg, esp_event_base_t base,
                               int32_t id, void* data)
{
    if (base == WIFI_EVENT) {
        switch (id) {
            case WIFI_EVENT_STA_START:
                esp_wifi_connect();
                break;
            case WIFI_EVENT_STA_DISCONNECTED:
                ESP_LOGW(TAG, "STA: отключились, переподключаемся...");
                esp_wifi_connect();
                break;
            case WIFI_EVENT_AP_STACONNECTED:
                ESP_LOGI(TAG, "AP: клиент подключился");
                break;
            case WIFI_EVENT_AP_STADISCONNECTED:
                ESP_LOGI(TAG, "AP: клиент отключился");
                break;
        }
    } else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* e = (ip_event_got_ip_t*)data;
        ESP_LOGI(TAG, "STA: получен IP " IPSTR ", порт сервера %d",
                 IP2STR(&e->ip_info.ip), WIFI_TCP_PORT);
    }
}

// ============================ WiFi init ======================================
static void wifi_init_softap(void)
{
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_ap();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_event_handler_register(
        WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL));

    wifi_config_t wc = {};
    strncpy((char*)wc.ap.ssid,     WIFI_TCP_AP_SSID, sizeof(wc.ap.ssid));
    strncpy((char*)wc.ap.password, WIFI_TCP_AP_PASS, sizeof(wc.ap.password));
    wc.ap.ssid_len       = strlen(WIFI_TCP_AP_SSID);
    wc.ap.channel        = WIFI_TCP_AP_CHAN;
    wc.ap.max_connection = 4;
    wc.ap.authmode       = (strlen(WIFI_TCP_AP_PASS) >= 8)
                           ? WIFI_AUTH_WPA2_PSK : WIFI_AUTH_OPEN;

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wc));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "AP поднят: SSID=\"%s\" pass=\"%s\"  IP=192.168.4.1  порт=%d",
             WIFI_TCP_AP_SSID, WIFI_TCP_AP_PASS, WIFI_TCP_PORT);
}

static void wifi_init_sta(void)
{
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_event_handler_register(
        WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(
        IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL));

    wifi_config_t wc = {};
    strncpy((char*)wc.sta.ssid,     WIFI_TCP_STA_SSID, sizeof(wc.sta.ssid));
    strncpy((char*)wc.sta.password, WIFI_TCP_STA_PASS, sizeof(wc.sta.password));

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wc));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "STA стартовал, подключаюсь к \"%s\"...", WIFI_TCP_STA_SSID);
}

// Отправить накопленную строку в очередь команд.
static void flush_line(char* line, size_t& line_len)
{
    if (line_len == 0) return;
    line[line_len] = '\0';
    cmd_msg_t m;
    strncpy(m.data, line, CMD_MAX_LEN - 1);
    m.data[CMD_MAX_LEN - 1] = '\0';
    ESP_LOGI(TAG, "RX cmd: \"%s\"", m.data);
    if (xQueueSend(s_cmd_queue, &m, 0) != pdTRUE) {
        ESP_LOGW(TAG, "RX-очередь переполнена, потеря команды");
    }
    line_len = 0;
}

// ============================ TCP клиент (на каждое соединение свой таск) =====
static void client_task(void* arg)
{
    int sock = (int)(intptr_t)arg;

    // Таймаут на recv — чтобы можно было исполнять команды, пришедшие БЕЗ \n
    // (многие TCP-клиенты на Android по умолчанию перевод строки не добавляют).
    struct timeval tv;
    tv.tv_sec  = 0;
    tv.tv_usec = 200 * 1000;            // 200 мс
    setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

    char   rx[RX_CHUNK];
    char   line[CMD_MAX_LEN];
    size_t line_len = 0;

    while (true) {
        int n = recv(sock, rx, sizeof(rx) - 1, 0);

        if (n == 0) {
            ESP_LOGI(TAG, "Клиент sock=%d закрыл соединение", sock);
            break;
        }
        if (n < 0) {
            if (errno == EAGAIN || errno == EWOULDBLOCK) {
                // Тайм-аут: новых байт нет. Если что-то лежит в буфере без \n —
                // считаем это законченной командой и шлём в очередь.
                flush_line(line, line_len);
                continue;
            }
            // ENOTCONN/ECONNRESET/ETIMEDOUT/EBADF — keepalive или вытеснение
            ESP_LOGI(TAG, "Клиент sock=%d ошибка recv errno=%d", sock, errno);
            break;
        }

        ESP_LOGD(TAG, "RX %d байт", n);

        // Собираем строки по '\n', '\r' игнорируем.
        for (int i = 0; i < n; i++) {
            char c = rx[i];
            if (c == '\r') continue;
            if (c == '\n') {
                flush_line(line, line_len);
            } else if (line_len < CMD_MAX_LEN - 1) {
                line[line_len++] = c;
            } else {
                ESP_LOGW(TAG, "Слишком длинная строка, отброшена");
                line_len = 0;
            }
        }
    }

    // clients_remove — no-op, если нас уже вытеснили из списка
    clients_remove(sock);
    shutdown(sock, SHUT_RDWR);
    close(sock);
    vTaskDelete(NULL);
}

// ============================ TCP сервер (accept loop) ========================
static void server_task(void* arg)
{
    int listen_sock = socket(AF_INET, SOCK_STREAM, IPPROTO_IP);
    if (listen_sock < 0) {
        ESP_LOGE(TAG, "socket() errno=%d", errno);
        vTaskDelete(NULL); return;
    }

    int yes = 1;
    setsockopt(listen_sock, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(yes));

    struct sockaddr_in addr = {};
    addr.sin_family      = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_ANY);
    addr.sin_port        = htons(WIFI_TCP_PORT);

    if (bind(listen_sock, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
        ESP_LOGE(TAG, "bind() errno=%d", errno);
        close(listen_sock); vTaskDelete(NULL); return;
    }
    if (listen(listen_sock, WIFI_TCP_MAX_CLIENTS) < 0) {
        ESP_LOGE(TAG, "listen() errno=%d", errno);
        close(listen_sock); vTaskDelete(NULL); return;
    }

    ESP_LOGI(TAG, "TCP сервер слушает порт %d", WIFI_TCP_PORT);

    while (true) {
        struct sockaddr_in cli; socklen_t clen = sizeof(cli);
        int s = accept(listen_sock, (struct sockaddr*)&cli, &clen);
        if (s < 0) {
            ESP_LOGE(TAG, "accept() errno=%d", errno);
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        // --- Keepalive: мёртвые клиенты будут отваливаться сами за ~11 с -------
        int ka = 1;
        int ka_idle  = KEEPALIVE_IDLE_S;
        int ka_intvl = KEEPALIVE_INTVL_S;
        int ka_count = KEEPALIVE_COUNT;
        setsockopt(s, SOL_SOCKET, SO_KEEPALIVE,  &ka,       sizeof(ka));
        setsockopt(s, IPPROTO_TCP, TCP_KEEPIDLE,  &ka_idle,  sizeof(ka_idle));
        setsockopt(s, IPPROTO_TCP, TCP_KEEPINTVL, &ka_intvl, sizeof(ka_intvl));
        setsockopt(s, IPPROTO_TCP, TCP_KEEPCNT,   &ka_count, sizeof(ka_count));

        char ip[16];
        inet_ntoa_r(cli.sin_addr, ip, sizeof(ip));
        ESP_LOGI(TAG, "Новый клиент %s:%d (sock=%d)",
                 ip, ntohs(cli.sin_port), s);

        // --- Регистрация в списке. Если мест нет — вытесняем самого первого ---
        if (!clients_add(s)) {
            int victim = clients_evict_one();
            if (victim >= 0) {
                ESP_LOGW(TAG, "Лимит клиентов, выкидываю sock=%d", victim);
                // shutdown разбудит recv в чужом task'е, он сам почистит сокет
                shutdown(victim, SHUT_RDWR);
            }
            if (!clients_add(s)) {
                // Невероятно, но на всякий случай
                ESP_LOGE(TAG, "Не удалось зарегистрировать sock=%d, закрываю", s);
                shutdown(s, SHUT_RDWR);
                close(s);
                continue;
            }
        }

        // --- Запускаем обработчик соединения ----------------------------------
        BaseType_t ok = xTaskCreate(client_task, "tcp_cli", 6144,
                                    (void*)(intptr_t)s, 5, NULL);
        if (ok != pdPASS) {
            ESP_LOGE(TAG, "xTaskCreate fail для sock=%d (мало RAM)", s);
            clients_remove(s);
            shutdown(s, SHUT_RDWR);
            close(s);
        }
    }
}

// ============================ Публичные функции ==============================
void wifi_tcp_init(void)
{
    // NVS нужен esp_wifi для хранения калибровки.
    esp_err_t r = nvs_flash_init();
    if (r == ESP_ERR_NVS_NO_FREE_PAGES || r == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        r = nvs_flash_init();
    }
    ESP_ERROR_CHECK(r);

    s_cmd_queue = xQueueCreate(CMD_QUEUE_LEN, sizeof(cmd_msg_t));
    clients_init();

#if WIFI_TCP_MODE_AP
    wifi_init_softap();
#else
    wifi_init_sta();
#endif

    xTaskCreate(server_task, "tcp_srv", 4096, NULL, 5, NULL);
}

void wifi_tcp_send(const char* msg)
{
    if (!msg || !s_clients_mutex) return;
    size_t len = strlen(msg);
    bool   add_nl = (len == 0) || (msg[len - 1] != '\n');

    xSemaphoreTake(s_clients_mutex, portMAX_DELAY);
    for (int i = 0; i < WIFI_TCP_MAX_CLIENTS; i++) {
        int s = s_client_socks[i];
        if (s == -1) continue;
        // MSG_DONTWAIT — чтобы основной цикл не вис, если клиент тормозит.
        // Ошибки send игнорируем: упавший клиент закроется в своём recv-цикле.
        if (len > 0) send(s, msg, len, MSG_DONTWAIT);
        if (add_nl)  send(s, "\n", 1, MSG_DONTWAIT);
    }
    xSemaphoreGive(s_clients_mutex);
}

void wifi_tcp_sendf(const char* fmt, ...)
{
    char buf[256];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    wifi_tcp_send(buf);
}

bool wifi_tcp_poll_command(char* out, size_t out_size)
{
    if (!s_cmd_queue || !out || out_size == 0) return false;
    cmd_msg_t m;
    if (xQueueReceive(s_cmd_queue, &m, 0) != pdTRUE) return false;
    strncpy(out, m.data, out_size - 1);
    out[out_size - 1] = '\0';
    return true;
}
