#ifndef COMPRESSOR_CONTROL_H
#define COMPRESSOR_CONTROL_H

#include <stdbool.h>

// Текущие состояния реле компрессоров (для экрана/лога/команды статуса)
extern volatile bool comp1_on;
extern volatile bool comp2_on;

// Сконфигурировать пины реле как выходы и гарантированно выключить компрессоры.
void compressor_control_init(void);

// Создать задачу управления двумя компрессорами (гистерезис по уставкам).
void compressor_control_start(void);

#endif // COMPRESSOR_CONTROL_H
