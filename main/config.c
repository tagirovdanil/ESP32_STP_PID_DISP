#include <stdint.h>
#include "config.h"

/*
 * ============================================================================
 *  config.c — общие глобальные переменные
 * ============================================================================
 *  pressure1_kPa / pressure2_kPa обновляют таймеры в pressure_sensor.c.
 *  p2_set_kPa (уставка P2, кПа) и p2_start_kPa (порог повторного включения K2)
 *  задаются только кадром $SLSS от панели или командой SET в консоли/UART2.
 *  NVS на этой плате НЕТ: значения живут в RAM и после перезагрузки
 *  возвращаются к умолчаниям P2_SET_DEFAULT_KPA / P2_START_DEFAULT_KPA.
 */

// Давления обновляют таймеры в pressure_sensor.c. pressure1_kPa определён здесь,
// pressure2_kPa — в pressure_sensor.c (чтобы не было дубликата).
volatile float pressure1_kPa = 0;        // давление P1, кПа
volatile float p2_set_kPa = P2_SET_DEFAULT_KPA;     // уставка P2, кПа
volatile float p2_start_kPa = P2_START_DEFAULT_KPA; // повторное включение K2, кПа
volatile uint32_t panel_last_rx_ms = 0;  // последний кадр от панели (0 = не было)
volatile uint32_t panel_rx_frames_total = 0; // всего строк от панели с включения
volatile uint32_t panel_slss_count = 0;      // сколько принято $SLSS
volatile uint32_t panel_th_count = 0;        // сколько принято $SLSC,TH
volatile uint32_t panel_slss_last_ms = 0;    // когда уставка применена (мс)
volatile bool  is_calibrating = false;   // пауза на время калибровки нуля датчика
