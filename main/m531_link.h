#pragma once
#include <stdbool.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

// ============================================================================
//  m531_link — TCP-КЛИЕНТ к панели m531 (Wi-Fi STA, сеть Metran531).
// ============================================================================
//  Плата подключается к серверу m531 (192.168.4.1:3333) и после каждого
//  connect() отправляет "HELLO PANEL2" — без этого m531 не считает её второй
//  панелью. Команды от m531 ("SET 1.6", "STAT", ...) кладутся в очередь и
//  исполняются в главном цикле через handle_command().
//  Wi-Fi здесь НЕ поднимается: его инициализирует wifi_tcp_init().
// ============================================================================

// Должны совпадать с wifi_tcp_server.h проекта m531.
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
