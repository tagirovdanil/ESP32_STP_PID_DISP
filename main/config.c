#include <stdio.h>
#include <stdint.h>
#include "config.h"

/*
 * ============================================================================
 *  config.c — глобальные переменные двухканального компрессорного контроллера
 * ============================================================================
 *  Здесь лежат только общие переменные (давления задаёт датчиковый модуль,
 *  уставки — команды/пользователь). Само управление реле компрессоров — в
 *  compressor_control.c.
 */

volatile float pressure1_kPa = 0;   // давление канала 1 — обновляет UART1-таймер
volatile float setpoint1_kPa = 0;   // уставка канала 1
volatile float setpoint2_kPa = 0;   // уставка канала 2
volatile bool  is_calibrating = false; // пауза экрана на время калибровки нуля датчика

// Давление канала 2 определено в pressure_sensor.c (там его читает UART2-таймер)
extern volatile float pressure2_kPa;

// Установка уставки канала (1 или 2) с ограничением диапазона [0 .. SETPOINT_MAX_KPA].
void update_setpoint(uint8_t channel, float new_setpoint) {
    if (new_setpoint < 0.0f)             new_setpoint = 0.0f;
    if (new_setpoint > SETPOINT_MAX_KPA) new_setpoint = SETPOINT_MAX_KPA;

    if (channel == 1)      setpoint1_kPa = new_setpoint;
    else if (channel == 2) setpoint2_kPa = new_setpoint;
    else                   return;

    printf("COMPR: Уставка канала %u = %.1f кПа\n", (unsigned)channel, new_setpoint);
}
