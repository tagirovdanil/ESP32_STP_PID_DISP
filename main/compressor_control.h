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
    ALARM_NONE      = 0,   // нет аварии
    ALARM_ESTOP     = 1,   // аварийный останов / аппаратная защита
    ALARM_SENSOR    = 2,   // неисправность датчика давления
    ALARM_STAGE2    = 3,   // вторая ступень не создаёт давление
    ALARM_STAGE1    = 4,   // первая ступень не создаёт давление
    ALARM_OVERSHOOT = 5    // перепуск: P2 >= P2_Set + P2_OVERSHOOT_KPA
};

// Коды режима (поле state протокола $SLSD)
enum {
    COMP_STATE_STOP    = 0,   // остановлен, ждём START
    COMP_STATE_RUN     = 1,   // работает
    COMP_STATE_ALARM   = 2,   // авария (нужен RESET)
    COMP_STATE_SERVICE = 3,   // сервисный тест выходов ($SLSC,OUT)
    COMP_STATE_PAUSE2  = 4    // пауза повторного запуска 2-й ступени
};

// Текущие состояния реле (для экрана/$SLSD/лога)
extern volatile bool comp1_on;
extern volatile bool comp2_on;
extern volatile bool comp3_on;           // R3: реле охлаждения (GPIO17)
extern volatile bool comp4_on;           // R4: резерв (GPIO22)
extern volatile int  current_alarm;      // код аварии 0..4

// Инициализация пинов реле (все выключены) — вызывается до compressor_control_start().
void compressor_control_init(void);

// Создать задачу управления (тик ~50 мс).
void compressor_control_start(void);

// ---- Управление (Пуск/Стоп/Сброс) ----
// true — команда принята и применена, false — отклонена (авария/тест/условия).
bool compressor_cmd_start(void);        // Пуск установки
void compressor_cmd_stop(void);         // Останов (штатный, всегда исполняется)
void compressor_cmd_reset(void);        // Сброс аварии

// ---- Уставка и порог повторного включения ($SLSS) ----
// Атомарно применяет P2_Set и P2_Start (кПа). Проверяет диапазоны панели:
//   P2_Set   = P2_SET_MIN_KPA .. P2_SET_MAX_KPA (350..4200 кПа)
//   P2_Start = P2_START_MIN_KPA .. P2_Set - P2_MIN_GAP_KPA
// Применяемая уставка дополнительно ограничивается P2_SET_APPLY_MAX_KPA.
// false — значения недопустимы, ничего не изменено.
// applied_set/applied_start (если не NULL) получают фактически применённые значения.
bool compressor_set_pressure_task_kpa(float set_kpa, float start_kpa,
                                      float *applied_set, float *applied_start);

// Уставка из текстовой команды SET <МПа>: P2_Start пересчитывается по алгоритму.
// Возвращает применённое значение уставки (кПа).
float compressor_set_setpoint_kpa(float v);

// ---- Пороги ступеней из $SLSC,TH (кПа) — ТОЛЬКО СПРАВОЧНО ----
// Кадр ничего не меняет: уставка/порог повторного включения берутся только из
// $SLSS, пороги 1-й ступени фиксированы (600/1000 кПа). Функция проверяет
// корректность значений и сверяет их с текущими; false — кадр некорректен.
bool compressor_check_thresholds_kpa(float s1_np, float s1_vp, float s2_np, float s2_vp);

// ---- Сервисный тест выходов ($SLSC,OUT) ----
// Разрешён только в STOP, без аварий, при P1 и P2 < SERVICE_MAX_P_KPA.
// K1/K2 — с таймаутом T_TEST_COMP_MS, R4 — с автоотключением T_TEST_R4_MS.
// false — сервисный режим недоступен, выходы не тронуты.
bool compressor_test_outputs(bool k1, bool k2, bool r3, bool r4);
void compressor_test_cancel(void);          // снять тест (START/STOP/авария/потеря связи)
bool compressor_test_active(void);

// ---- Демо-проверка реле (K1..K4) ----
// Останавливает установку и запускает НЕПРЕРЫВНЫЙ цикл:
//   K1 ON -> K2 ON -> K3 ON -> K4 ON -> K4 OFF -> K3 OFF -> K2 OFF -> K1 OFF
// с паузой RELAY_DEMO_HOLD_MS между шагами; состояние пишется в лог.
// Во время демо автомат не трогает выходы. Повторный запуск игнорируется.
void compressor_relay_demo_start(void);
void compressor_relay_demo_stop(void);   // корректно завершить демо (все реле OFF)
bool compressor_relay_demo_active(void);

// Вспомогательные для экрана/протокола
bool compressor_is_running(void);
int  compressor_get_alarm(void);
int  compressor_get_state(void);         // COMP_STATE_* для поля state в $SLSD

#endif // COMPRESSOR_CONTROL_H
