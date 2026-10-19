#ifndef COMPRESSOR_CONTROL_H
#define COMPRESSOR_CONTROL_H

#include <stdbool.h>
#include <stdint.h>

/*
 * ============================================================================
 *  Управление двухступенчатой компрессорной установкой
 * ============================================================================
 *  Алгоритм по алгоритм.txt, аварии и обмен с дисплеем по protocol.txt.
 *  Давления в кПа (1 МПа = 1000 кПа).
 */

// Коды аварий (совпадают с полем alarm протокола $SLSD)
enum {
    ALARM_NONE   = 0,   // нет аварии
    ALARM_ESTOP  = 1,   // аварийный останов / аппаратная защита
    ALARM_SENSOR = 2,   // неисправность датчика давления
    ALARM_STAGE2 = 3,   // вторая ступень не создаёт давление
    ALARM_STAGE1 = 4    // первая ступень не создаёт давление
};

// Текущие состояния реле (для экрана/$SLSD)
extern volatile bool comp1_on;
extern volatile bool comp2_on;
extern volatile int  current_alarm;      // код аварии 0..4

// Инициализация пинов реле (все выключены) — вызывается до compressor_control_start().
void compressor_control_init(void);

// Создать задачу управления (тик ~50 мс).
void compressor_control_start(void);

// ---- Управление (Пуск/Стоп/Сброс) ----
void compressor_cmd_start(void);        // Пуск установки
void compressor_cmd_stop(void);         // Останов (штатный)
void compressor_cmd_reset(void);        // Сброс аварии

// Установка уставки P2 в кПа (диапазон 1,0..4,0 МПа). Возвращает применённое значение.
float compressor_set_setpoint_kpa(float v);

// Вспомогательные для экрана/протокола
bool compressor_is_running(void);
int  compressor_get_alarm(void);

#endif // COMPRESSOR_CONTROL_H
