#include <stdint.h>
#include <stddef.h>
#include "nvs_flash.h"
#include "nvs.h"
#include "esp_log.h"
#include "config.h"
#include "settings.h"

/*
 * ============================================================================
 *  settings.c — уставка в NVS (namespace "sls")
 * ============================================================================
 *  Храним ровно два значения: P2_Set и P2_Start. Пишутся одним блобом, чтобы
 *  чтение не могло вернуть «половину» новой уставки.
 *
 *  Порогов ступеней в NVS нет (panel2_task_overshoot.md §5.3):
 *    - S1_NP/S1_VP фиксированы алгоритмом (600/1000 кПа);
 *    - S2_NP/S2_VP — это и есть P2_Start/P2_Set из $SLSS;
 *    - кадр $SLSC,TH справочный и настройкой не является.
 *  Устаревшие ключи np1/vp1/np2/vp2 (старые прошивки) при старте удаляются,
 *  а старый формат блоба отбрасывается по номеру версии.
 * ============================================================================ */

#define SLS_NVS_NAMESPACE "sls"
#define SLS_NVS_KEY       "p_task"
#define SLS_NVS_VERSION   3u

static const char *TAG = "SETTINGS";

// Ключи-пороги из старых прошивок: после обновления их быть не должно.
static const char *LEGACY_KEYS[] = { "np1", "vp1", "np2", "vp2" };

typedef struct {
    uint32_t version;
    float    p2_set_kpa;
    float    p2_start_kpa;
} sls_settings_t;

static bool s_nvs_ready = false;

static bool nvs_ensure(void) {
    if (s_nvs_ready) return true;

    // nvs_flash_init() идемпотентен: wifi_tcp_init() вызывает его позже ещё раз.
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_LOGW(TAG, "NVS требует очистки (%s) — стираю", esp_err_to_name(err));
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "nvs_flash_init: %s", esp_err_to_name(err));
        return false;
    }
    s_nvs_ready = true;
    return true;
}

// Удаление устаревших ключей с порогами ступеней
static void nvs_drop_legacy_keys(void) {
    nvs_handle_t h;
    if (nvs_open(SLS_NVS_NAMESPACE, NVS_READWRITE, &h) != ESP_OK) return;

    bool erased = false;
    for (size_t i = 0; i < sizeof(LEGACY_KEYS) / sizeof(LEGACY_KEYS[0]); i++) {
        if (nvs_erase_key(h, LEGACY_KEYS[i]) == ESP_OK) {
            ESP_LOGW(TAG, "Удалён устаревший ключ NVS '%s' (порогов ступеней больше нет)",
                     LEGACY_KEYS[i]);
            erased = true;
        }
    }
    if (erased) nvs_commit(h);
    nvs_close(h);
}

void settings_init(void) {
    if (!nvs_ensure()) return;

    nvs_drop_legacy_keys();

    nvs_handle_t h;
    if (nvs_open(SLS_NVS_NAMESPACE, NVS_READONLY, &h) != ESP_OK) {
        ESP_LOGW(TAG, "APPLY set=%.2f start=%.2f МПа (src=DEFAULT: $SLSS ещё не приходил)",
                 p2_set_kPa / 1000.0f, p2_start_kPa / 1000.0f);
        return;
    }

    sls_settings_t st = {0};
    size_t len = sizeof(st);
    esp_err_t err = nvs_get_blob(h, SLS_NVS_KEY, &st, &len);
    nvs_close(h);

    if (err != ESP_OK || len != sizeof(st) || st.version != SLS_NVS_VERSION) {
        ESP_LOGW(TAG, "Запись уставки отсутствует/устарела (%s) — значения по умолчанию: "
                 "set=%.2f start=%.2f МПа",
                 esp_err_to_name(err), p2_set_kPa / 1000.0f, p2_start_kPa / 1000.0f);
        return;
    }

    // Диапазоны проверяем и здесь: в NVS могло остаться что-то от старой прошивки.
    bool ok = (st.p2_set_kpa >= P2_SET_MIN_KPA && st.p2_set_kpa <= P2_SET_MAX_KPA &&
               st.p2_start_kpa >= P2_START_MIN_KPA &&
               st.p2_start_kpa <= st.p2_set_kpa - P2_MIN_GAP_KPA + 0.5f);
    if (!ok) {
        ESP_LOGW(TAG, "В NVS некорректная уставка (set=%.2f start=%.2f МПа) — "
                 "оставляю значения по умолчанию",
                 st.p2_set_kpa / 1000.0f, st.p2_start_kpa / 1000.0f);
        return;
    }

    p2_set_kPa   = st.p2_set_kpa;
    p2_start_kPa = st.p2_start_kpa;

    ESP_LOGW(TAG, "APPLY set=%.2f start=%.2f МПа (src=NVS)",
             p2_set_kPa / 1000.0f, p2_start_kPa / 1000.0f);
}

static void settings_store(void) {
    if (!nvs_ensure()) return;

    // Не пишем в NVS одно и то же: панель может повторять кадр $SLSS.
    static sls_settings_t last_saved;
    static bool have_last_saved = false;

    sls_settings_t cur = {
        .version      = SLS_NVS_VERSION,
        .p2_set_kpa   = p2_set_kPa,
        .p2_start_kpa = p2_start_kPa,
    };
    if (have_last_saved &&
        cur.p2_set_kpa == last_saved.p2_set_kpa &&
        cur.p2_start_kpa == last_saved.p2_start_kpa) {
        return;   // значения не изменились — флеш не трогаем
    }

    nvs_handle_t h;
    esp_err_t err = nvs_open(SLS_NVS_NAMESPACE, NVS_READWRITE, &h);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "nvs_open: %s", esp_err_to_name(err));
        return;
    }

    err = nvs_set_blob(h, SLS_NVS_KEY, &cur, sizeof(cur));
    if (err == ESP_OK) err = nvs_commit(h);
    nvs_close(h);

    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Не удалось сохранить уставку: %s", esp_err_to_name(err));
    } else {
        last_saved = cur;
        have_last_saved = true;
    }
}

void settings_save_pressure_task(void) {
    settings_store();
}
