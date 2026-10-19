#include <stdint.h>
#include "config.h"

/*
 * ============================================================================
 *  config.c — общие глобальные переменные
 * ============================================================================
 *  pressure1_kPa / pressure2_kPa обновляют таймеры в pressure_sensor.c.
 *  p2_set_kPa (уставка P2, кПа) задаётся с дисплея/WiFi, по умолчанию 4,0 МПа.
 */

// Давления обновляют таймеры в pressure_sensor.c. pressure1_kPa определён здесь,
// pressure2_kPa — в pressure_sensor.c (чтобы не было дубликата).
volatile float pressure1_kPa = 0;        // давление P1, кПа
volatile float p2_set_kPa = P2_SET_DEFAULT_KPA;  // уставка P2, кПа (4,0 МПа)
volatile bool  is_calibrating = false;   // пауза на время калибровки нуля датчика
