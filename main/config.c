#include <stdint.h>
#include "config.h"

/*
 * ============================================================================
 *  config.c — общие глобальные переменные
 * ============================================================================
 *  pressure1_kPa / pressure2_kPa обновляют таймеры в pressure_sensor.c.
 *  p2_set_kPa (уставка P2, кПа) и p2_start_kPa (порог повторного включения K2)
 *  задаются с панели ($SLSS,<P2_Set>,<P2_Start>) или по WiFi/WiFi-команде SET.
 */

// Давления обновляют таймеры в pressure_sensor.c. pressure1_kPa определён здесь,
// pressure2_kPa — в pressure_sensor.c (чтобы не было дубликата).
volatile float pressure1_kPa = 0;        // давление P1, кПа
volatile float p2_set_kPa = P2_SET_DEFAULT_KPA;     // уставка P2, кПа
volatile float p2_start_kPa = P2_START_DEFAULT_KPA; // повторное включение K2, кПа
volatile uint32_t panel_last_rx_ms = 0;  // последний кадр от панели (0 = не было)
volatile bool  is_calibrating = false;   // пауза на время калибровки нуля датчика
