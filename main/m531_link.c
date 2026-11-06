#include "m531_link.h"

#include <stdio.h>
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
#include "lwip/tcp.h"      // TCP_KEEPIDLE / TCP_KEEPINTVL / TCP_KEEPCNT

/*
 * ============================================================================
 *  m531_link.c — TCP-клиент к панели m531 (вторая панель по протоколу m531)
 * ============================================================================
 *  Wi-Fi поднимает wifi_tcp_init() в режиме STA. Здесь только TCP:
 *    - ждём получения IP в сети m531;
 *    - connect(192.168.4.1:3333), сразу шлём "HELLO PANEL2\n";
 *    - принимаем строки, отдаём их в очередь (исполняет главный цикл);
 *    - keepalive ~11 c: пропажу питания m531 замечаем без FIN.
 * ============================================================================
 */

static const char *TAG = "M531";

#define GOT_IP_BIT  BIT0

typedef struct { char s[M531_MSG_MAX]; } link_msg_t;

static QueueHandle_t      s_rxq    = NULL;
static EventGroupHandle_t s_eg     = NULL;   // GOT_IP_BIT = мы в сети m531
static SemaphoreHandle_t  s_tx_mtx = NULL;   // охраняет s_sock и send()
static int                s_sock   = -1;

static void on_event(void *arg, esp_event_base_t base, int32_t id, void *data)
{
    (void)arg;
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
    struct sockaddr_in a;
    memset(&a, 0, sizeof(a));
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
    snprintf(m.s, sizeof(m.s), "%s", line);
    if (xQueueSend(s_rxq, &m, 0) != pdTRUE)
        ESP_LOGW(TAG, "очередь полна, \"%s\" потеряна", line);
}

static void link_task(void *arg)
{
    (void)arg;
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
    snprintf(out, out_size, "%s", m.s);
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
