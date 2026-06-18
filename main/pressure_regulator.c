#include <stdio.h>
#include <string.h>        // memset
#include <stdlib.h>        // labs
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "esp_spiffs.h"
#include "esp_timer.h"
#include "driver/ledc.h"   // Драйвер ШИМ для сервопривода
#include <math.h>
#include "stdbool.h"
#include "driver/uart.h"

#include "pressure_regulator.h"
#include "st7789.h"
#include "fontx.h"
#include "config.h"

// ============================================================================
//  Внешние объекты/глобалы (определены в config.c / st7789.c)
// ============================================================================
extern TFT_t dev;
extern FontxFile fx16[2];
extern TaskHandle_t display_task_handle;

// «нет запроса» по умолчанию; команды idle/abort/home кладут сюда своё значение
volatile RegulatorState requested_reg_state = REG_STATE_NONE;

// Пауза между импульсами шага в РЕГУЛЯТОРЕ (мкс). Период импульса ≈ 10 + это.
// ВАЖНО: было 20 (~33 кГц) — NEMA17 со старта столько не тянет и стоит на месте,
// хотя программа считает шаги. Хоминг работает на 500 (~1.9 кГц), reset на 190
// (~5 кГц). 400 -> ~2.4 кГц — заведомо в рабочем диапазоне. Если хочешь быстрее,
// уменьшай это число постепенно и смотри, чтобы мотор не срывался в свист.
#define VALVE_STEP_US   400

// Защита фильтров от глюков датчика. Битый UART-кадр иногда раскодируется в
// «валидное» давление (проходит проверку диапазона в pressure_sensor.c), и один
// такой отсчёт при alpha=0.8 надолго отравляет EMA (P_filt проваливался до ~184
// на несколько секунд и ложно гнал серво в подкачку). В RUNNING давление
// физически не меняется быстрее ~100–150 кПа/с, поэтому мгновенную dP/dt выше
// PRESS_GLITCH_RATE считаем аномалией и отсчёт игнорируем. Но не более
// PRESS_GLITCH_MAX подряд — иначе при реальном длительном сдвиге датчика залипнем.
#define PRESS_GLITCH_RATE   400.0f
#define PRESS_GLITCH_MAX    10

// ============================================================================
//  ПУБЛИЧНЫЕ ВСПОМОГАТЕЛЬНЫЕ ФУНКЦИИ ПРИВОДОВ (используются и из config.c)
// ============================================================================

// Принудительный физический сброс давления (команда reset). Выставляет флаг,
// сам сброс делает задача регулятора в ветке REG_STATE_HOMING.
void start_pressure_homing(void) {
    if (!is_homing) {
        ESP_LOGW("CONTROL", "Запущен принудительный сброс давления (Homing)!");
        is_homing = true;
    }
}

// Запрос смены состояния извне (из обработчика команд).
void regulator_request_state(RegulatorState new_state) {
    requested_reg_state = new_state;
}

// Угол сервопривода 0..180 -> ШИМ. 0 = сброс, 90 = нейтраль, 180 = подача.
void set_servo_angle(float angle) {
    if (angle < 0.0f)   angle = 0.0f;
    if (angle > 180.0f) angle = 180.0f;

    uint32_t us   = SERVO_MIN_US + (uint32_t)((angle / 180.0f) * (SERVO_MAX_US - SERVO_MIN_US));
    uint32_t duty = (us * 16383) / 20000;   // 14-битный ШИМ, период 20000 мкс (50 Гц)

    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, duty);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
}

// Перемещение иглы в абсолютную координату (в шагах), руками, без аппаратного ШИМ.
// speed_us — пауза между импульсами (чем меньше, тем быстрее).
void move_valve_absolute(int32_t target_position, uint32_t speed_us) {
    if (target_position < 0)               target_position = 0;
    if (target_position > MAX_VALVE_STEPS) target_position = MAX_VALVE_STEPS;
    if (current_valve_position == target_position) return;

    int32_t steps_to_move = labs(target_position - current_valve_position);
    bool    dir = (target_position > current_valve_position) ? false : true; // false=открыть, true=закрыть
    gpio_set_level(PIN_DIR, dir);
    esp_rom_delay_us(5);

    for (int32_t i = 0; i < steps_to_move; i++) {
        gpio_set_level(PIN_STEP, 1);
        esp_rom_delay_us(10);
        gpio_set_level(PIN_STEP, 0);
        esp_rom_delay_us(speed_us);
        if (dir == false) current_valve_position++;
        else              current_valve_position--;
    }
}

// ============================================================================
//  ИНИЦИАЛИЗАЦИЯ РЕГУЛЯТОРА
//  Все коэффициенты ФИКСИРОВАННЫЕ — настраиваются один раз. Значения ниже —
//  стартовые/безопасные, под конкретное железо их нужно подобрать (см. подсказки).
// ============================================================================
void regulator_init(PressureRegulator* reg) {
    memset(reg, 0, sizeof(*reg));

    reg->state           = REG_STATE_IDLE;
    reg->servo_state     = SERVO_NEUTRAL;
    reg->active_setpoint = 0.0f;

    // --- измерение скорости ---
    reg->rate_filter_alpha = 0.8f;   // сильное сглаживание: dP/dt очень шумная
    // Фильтр давления для ПОРОГОВОГО решения в HOLD (bang-bang серво). Сырое
    // давление шумит на уровне sensor_noise_delta, и одиночный выброс между
    // редкими логами латчил серво. EMA по давлению гасит такие спайки.
    // 0.8 ≈ среднее по ~9 отсчётам, лаг ~200 мс (в HOLD это некритично).
    reg->press_filter_alpha = 0.8f;

    // --- ВНУТРЕННИЙ PI по скорости (дальняя зона) ---
    // Выход — открытие иглы в шагах. Подбирать так:
    //   1) задать постоянную желаемую скорость (например 20 кПа/с) на тесте,
    //   2) поднимать rate_ki, пока контур уверенно выходит на эту скорость,
    //   3) rate_kp добавить чуть-чуть для гашения отклонений (но не до дрожания).
    reg->rate_kp           = 10.0f;
    reg->rate_ki           = 20.0f;
    reg->rate_integral     = 0.0f;
    reg->rate_integral_max = 200.0f; // страховочный потолок. Настоящий анти-windup — условная
                                     // интеграция в rate_control_step (I не копится, пока выход
                                     // на упоре valve_max), так что реально I живёт не выше
                                     // (valve_max - floor)/ki ≈ 113 и до этого капа не доходит.

    // --- порог «дошли» / ползучая скорость у цели ---
    // Макс. дрожание датчика в покое (по логам ~0.2 кПа). Двойная роль:
    //  - |error| ниже этого значения -> считаем, что ДОШЛИ: серво в нейтраль, держим;
    //  - в полосе error 0..2 это же значение = желаемая скорость подхода (ползём медленно).
    // Больше -> раньше останавливаемся (меньше перелёт, грубее точность); меньше -> точнее.
    reg->sensor_noise_delta = 0.22f;
    reg->sensor_noise_delta_filt = 0.03f;

    // --- лимиты привода (в шагах) ---
    reg->valve_max      = MAX_VALVE_STEPS; // полный ход иглы
    // ВАЖНО (подобрать под железо!): открытие, ниже которого игла физически не
    // пропускает газ (мёртвый ход / cracking). По логам поток начинался ~4500 и
    // держался вниз до ~3600 -> ставим floor около нижней границы потока. Регулятор
    // никогда не опускает иглу ниже floor во время работы, поэтому уходит мёртвое
    // время ~3 с на старте и тонкая зона реально может дать поток.
    reg->valve_flow_floor = 0;// 3600 * 0.9; // на всякий случай умножил на 0.8, чтоб наверняка 0 был
                                        // (он же — парковка иглы в HOLD: запечатано, но близко к зоне потока)

    // --- HOLD: МИКРОДОЗЫ ---
    // У цели контур по скорости не работает: нужные 0.01-0.02 кПа/с не видны в
    // dP/dt (шум ±0.5 кПа/с при отсчётах 50 мс), а PI с преднатягом давал поток
    // 1-3 кПа/с и пинг-понг НАБОР<->СБРОС. Поэтому в HOLD игла стоит на
    // ФИКСИРОВАННОМ дозирующем открытии step_holding_*, а раз в dose_period_us
    // смотрим, сколько давление РЕАЛЬНО прошло к цели за окно, и чуть трогаем
    // открытие:
    //   прошло < dose_dp_slow    -> +dose_trim      (слишком медленно / не туда)
    //   прошло > dose_dp_runaway -> -dose_trim_big  (разогнались)
    //   прошло > dose_dp_fast    -> -dose_trim      (чуть быстрее нужного)
    // Полоса dose_dp_slow..dose_dp_fast — «хорошо», открытие не трогаем.
    reg->hold_enter_err  = 0.8f;        // |error| <= этого -> PID выключается, дальше HOLD-струйка
    reg->hold_exit_err   = 5.0f;        // в HOLD |err_filt| больше -> микродозой не вытянуть, назад в RATE качать.
                                        // >>hold_enter_err: гистерезис, чтоб не дёргалось RATE<->HOLD у цели
    reg->dose_step_back  = 30;          // вход в HOLD набором: СТАРТ ПЕРЕБОРА поиска = позиция RATE минус это
                                        // (было 60; на высоком давлении порог ВЫШЕ позиции RATE, отступ вниз
                                        //  только удлинял перебор -> 30. Подгон в floor страхует от перелёта)
                                        // (было 200 для старой иглы; у новой порог ~позиции RATE, нужен
                                        //  малый отступ — перебор стартует чуть ниже порога и быстро доходит)
    reg->dose_period_us  = 5000000ULL;  // окно проверки прогресса дозы: 5 с
    reg->dose_dp_slow    = 0.05f;       // кПа за окно
    reg->dose_dp_VERYslow    = -0.04f;  // прогресс хуже этого (явная утечка/мимо за окно) -> крупный добор
                                        // (было -0.05: утечку -0.047 не ловило, падало в «медленно» +2)
    reg->dose_dp_fast    = 0.10f;
    reg->dose_dp_runaway = 0.20f;
    reg->dose_trim       = 2;          // шаги иглы     // было 10
    reg->dose_trim_big   = 5;    // было 10; «большой» шаг при дрейфе <dose_dp_VERYslow и при разгоне >runaway.
                                 // На 1000 кПа +10 переливал (280->290 -> подскок до 1000.18) -> 5
    reg->dose_trim_very_big   = 10; // было 100, но решил поставить 50, так как на 100 бывает перескакивает     // было 50
    reg->dose_big_min_err = 0.5f;       // п.1: ближе 0.5 кПа к цели (по СЫРОМУ давлению) big/very_big не применяем
    // п.2: кольцо лагов мультиоконной проверки HOLD/FINE (как окно поиска порога, но на 5с-масштаб)
    reg->dose_hist_dt_us = 1250000ULL;  // 1.25 с -> лаги 1.25/2.5/3.75/5.0 с
    reg->dose_hist_idx   = 0;
    reg->dose_hist_count = 0;
    reg->dose_hist_t0_us = 0;
    reg->step_holding_charge = reg->valve_flow_floor;  // безопасный дефолт; засеется при входе в HOLD
    reg->step_holding_vent   = reg->valve_flow_floor;
    reg->dose_charge_calibrated = false; // флаги ставятся при входе в HOLD: направление подхода
    reg->dose_vent_calibrated   = false; // калибровано, противоположное разгоняется с floor по +trim_big

    // --- БЫСТРЫЙ ПОИСК ПОРОГА ПОТОКА (идея 3) ---
    // Перебор иглой вверх до первого потока вместо медленного +dose_trim за 5 с.
    // Числа: +1 шаг раз в 0.2 с (=5 шаг/с), окно детекции 1 с, поток = ΔP>=0.05
    // кПа за окно, после нахождения откат на 10 (лаг детекции ~ 1шаг*10 за окно),
    // пауза 2 с в floor. Перебор медленный для большого мёртвого хода — поднимай
    // search_step, если порог высоко по шагам.
    reg->dose_searching        = false;
    reg->search_phase          = 0;
    reg->search_step_period_us = 200000ULL;   // 0.2 с
    reg->search_step           = 1;           // мелкий шаг = точная локализация порога
    reg->search_hist_dt_us     = 250000ULL;   // чекпойнт раз в 0.25 с -> лаги 0.25..1.0 с
    reg->search_hist_idx       = 0;
    reg->search_hist_count     = 0;
    reg->search_hist_t0_us     = 0;
    reg->search_dp_flow        = 0.05f;       // P_filt выросло на это над лагом = поток (>шум)
    reg->search_warmup_rate    = 0.10f;       // |rate_filt|<0.1 кПа/с = давление устаканилось
    reg->search_warmup_max_us  = 2500000ULL;  // но подгон не дольше 2.5 с
    reg->search_settle_t0_us   = 0;           // таймер фазы Подгон
    reg->search_found_pos      = 0;
    reg->search_target_back    = 1.0f;        // финиш фазы 2 на (цель - 1.0): хвост доводит без перелёта +
                                              // калибровка фиксируется РАНЬШЕ внешней проверки «дошли» (см. .h)
    reg->search_done_back      = 10;          // после финиша держать иглу на (найденная - 10), а не на самой
                                              // флоу-позиции: мягкий добор хвоста без перелёта (0 = на найденной)

    // --- ВТОРОЙ (ТОЧНЫЙ) ХОЛДИНГ: равновесное приоткрытие вместо подкачек ---
    // После выхода на точку НАБОРОМ (доза подобрана делом) серво НЕ печатаем,
    // оставляем в НАБОРЕ: игла встаёт на step_holding_charge - fine_seed_back и
    // раз в fine_period_us двигается на +-fine_trim по знаку сползания давления
    // за окно (росло -> прикрыть, падало -> приоткрыть) — ищем открытие, где
    // приток через иглу равен утечке. Окно с |dP| < fine_eq_band = «равновесие»:
    // открытие запоминается в fine_eq_pos, игла стоит на нём БЕЗ подкачек.
    // Если при этом стоим МИМО цели (поиск съел давление: |err_filt| >
    // fine_corr_err) — само равновесие мягко двигаем на fine_trim в сторону
    // цели. Печатать и докачивать просадку обычным HOLD нельзя: пока объём
    // запечатан, магистраль успевает измениться и равновесие протухает
    // (проверено: найденные 3393 после докачки уже лили).
    // Выход на точку СБРОСОМ в точный холдинг НЕ ведёт: печатаем, ждём
    // естественного стравливания вниз, обычный эпизод НАБОРА подбирает дозу и
    // выводит на точку — только тогда включается точный холдинг (hold_fine_step).
    reg->hold_fine_enable = true;        // КОНФИГ: false = старое поведение (печать + эпизоды подкачки)
    reg->fine_seed_back   = 2;           // ПЕРВЫЙ вход в FINE: игла = step_holding_charge минус это.
                                         // Дозовая позиция чуть ВЫШЕ равновесия -> небольшой откат вниз
                                         // (было 20 — FINE потом долго полз вверх; 0 = садиться ровно на дозу)
    reg->fine_relearn_step = 2;          // повторный вход: ±2 к позиции прошлого выхода по стороне выброса
    reg->fine_trim        = 1;           // +-шаг подстройки открытия за окно
    reg->fine_period_us   = 5000000ULL;  // окно оценки знака скорости: 5 с
    reg->fine_eq_band     = 0.01f;       // |dP| за окно меньше этого = «стоим», равновесие найдено
    reg->fine_eq_found    = false;       // сбрасывается на новой уставке (равновесие там другое)
    reg->fine_corr_err    = 0.05f;       // мёртвая зона цели; меньше — дёргаемся от шума P_filt (~±0.02)
    reg->fine_dev_guard   = 0.00f;       // было 0.05, потом 0.01 поставил - на 500 вышло, теперь пробую 0.00 q2: ниже цели на 0.01+ не прикрываем (−), выше на 0.01+ не приоткрываем (+)
    reg->fine_trim_fast   = 8;           // пока давление ещё активно движется — крупный шаг (вместо fine_trim=2)
    reg->fine_hold_change = 0.5f;        // |ΔP_filt| за ~5 с >= этого = «ещё движусь»: не отдаём в HOLD
    reg->fine_rate_t0_us  = 0;           // 0 -> трекер fine_change сам инициализируется на 1-м тике RUNNING
    // макс. сдвиг иглы за тик. При VALVE_STEP_US=400 один тик блокирует задачу
    // примерно на max_step*0.41 мс, поэтому держим небольшим (60 -> ~25 мс/тик,
    // слю ~2400 шаг/с, полный ход ~4 с). Если игла открывается слишком медленно —
    // увеличивай вместе с контролем времени тика.
    reg->max_step       = 60;
}

// ============================================================================
//  ВНЕШНИЙ СЛОЙ: таблица «ошибка по давлению -> желаемая скорость dP/dt»
//  Знак результата = направление (плюс — набирать, минус — стравливать).
//  Это профиль приближения: далеко быстро, ближе медленнее. Полосы дискретные
//  (как ты задумал); при желании потом легко заменить на непрерывную k*error.
//  В полосе 0..2 кПа скорость = near_rate (передаём sensor_noise_delta): у цели
//  ползём со скоростью на уровне шума датчика. Отдельной FINE-фазы больше нет.
// ============================================================================
static float desired_rate_from_error(float error, float near_rate, float very_near_rate) {
    float e = fabsf(error);
    float rate;
    if      (e > 500.0f) rate = 100.0f;
    else if (e > 250.0f) rate = 50.0f;
    else if (e >  80.0f) rate = 20.0f;
    else if (e >  30.0f) rate = 10.0f;
    else if (e >   16.0f) rate = 3.0f;
    else if (e >   5.0f) rate = 1.0f;        // полоса 2..8 кПа
    else if (e >   1.0)                 rate = near_rate;   // полоса 0..2 кПа: ползём со скоростью шума датчика
    else rate = very_near_rate;
    return (error >= 0.0f) ? rate : -rate;
}

// ============================================================================
//  ВНУТРЕННИЙ PI ПО СКОРОСТИ: держит реальную dP/dt равной желаемой, выход —
//  открытие иглы (шаги). Работает ТОЛЬКО в фазе подхода (RATE); в HOLD не
//  используется — там микродозы (см. hold_dose_step).
// ============================================================================
static int32_t rate_control_step(PressureRegulator* reg, float desired, float dt) {
    // реальная скорость «в сторону цели» (сравниваем по модулю желаемой)
    float measured_toward = (desired > 0) ? reg->filtered_rate : -reg->filtered_rate;
    float rate_err = fabsf(desired) - measured_toward;   // >0 — едем медленнее нужного

    // АВАРИЙНЫЙ ТОРМОЗ: едем к цели в 3+ раза быстрее заказа И превышение
    // ощутимое (>1.5 кПа/с — отсечка шума: у цели заказ 0.03-0.22, а шум
    // скорости ±0.5, без второй проверки тормоз дёргался бы от выбросов).
    // Штатная размотка PI закрывает иглу лишь со скоростью ki*|rate_err| шаг/с
    // (перебор 5 кПа/с -> ~300 шаг/с), а при низком абс. давлении до «медленной»
    // позиции тысячи шагов — не успевает (заезд к 50 кПа, проскок и удар
    // магистрали). Поэтому закрываемся на полном слю (max_step за тик), а
    // интегратор синхронизируем с фактической позицией, иначе после отпускания
    // тормоза PI вернул бы иглу обратно вверх.
    float koef_for_big_error = (setpoint_kPa > 200) ? 4.5 : 3; // было - float koef_for_big_error = (setpoint_kPa > 200) ? 3.5 : 2;
    if (measured_toward > koef_for_big_error * fabsf(desired) && (measured_toward - fabsf(desired)) > 0.2f) {  // было > 1.5f, не смогло замедлится на 50
        int32_t out_brake = current_valve_position - reg->max_step;
        if (out_brake < reg->valve_flow_floor) out_brake = reg->valve_flow_floor;
        reg->rate_integral = ((float)out_brake - (float)reg->valve_flow_floor
                              - reg->rate_kp * rate_err) / reg->rate_ki;
        if (reg->rate_integral < 0.0f)                   reg->rate_integral = 0.0f;
        if (reg->rate_integral > reg->rate_integral_max) reg->rate_integral = reg->rate_integral_max;
        return out_brake;
    }

    // АНТИ-WINDUP (условная интеграция): если выход уже упёрся в valve_max и
    // ошибка требует открываться дальше (rate_err > 0) — интегратор НЕ копим:
    // такой приказ игла исполнить не может, а накопленный «фантом» потом
    // секунды разматывается, пока игла стоит на упоре (заезд к 50: pos=10000
    // держался 8 c). Размотку (rate_err < 0) разрешаем всегда — это выход из
    // насыщения. Нижний край защищён существующим клампом I >= 0.
    float out_now = (float)reg->valve_flow_floor + reg->rate_kp * rate_err
                  + reg->rate_ki * reg->rate_integral;
    if (!(out_now >= (float)reg->valve_max && rate_err > 0.0f))
        reg->rate_integral += rate_err * dt;
    if (reg->rate_integral < 0.0f)                   reg->rate_integral = 0.0f;
    if (reg->rate_integral > reg->rate_integral_max) reg->rate_integral = reg->rate_integral_max;

    // открытие иглы = floor + Kp*ошибка_скорости + Ki*интеграл
    float out = (float)reg->valve_flow_floor + reg->rate_kp * rate_err + reg->rate_ki * reg->rate_integral;
    if (out < (float)reg->valve_flow_floor) out = (float)reg->valve_flow_floor;
    if (out > (float)reg->valve_max)        out = (float)reg->valve_max;
    return (int32_t)out;
}

// Добавляет чекпойнт P_filt в кольцо лагов раз в dose_hist_dt_us (п.2). Кольцо
// общее для дозы (HOLD) и FINE — фазы не пересекаются.
static void dose_hist_tick(PressureRegulator* reg, uint64_t now_us) {
    if (now_us - reg->dose_hist_t0_us >= reg->dose_hist_dt_us) {
        reg->dose_hist_t0_us = now_us;
        reg->dose_hist[reg->dose_hist_idx] = reg->filtered_pressure;
        reg->dose_hist_idx = (reg->dose_hist_idx + 1) % DOSE_HIST_N;
        if (reg->dose_hist_count < DOSE_HIST_N) reg->dose_hist_count++;
    }
}

// Сброс окна проверки дозы (на входе в HOLD и на старте каждого эпизода), чтобы
// прогресс мерился от начала текущего эпизода, а не от прошлого. Заодно чистит
// кольцо лагов (п.2): после смены фазы / защитного залпа лаги считаем заново.
static void dose_window_reset(PressureRegulator* reg, uint64_t now_us) {
    reg->dose_t0_us = now_us;
    reg->dose_p0    = reg->filtered_pressure;
    reg->dose_hist_count = 0;
    reg->dose_hist_idx   = 0;
    reg->dose_hist_t0_us = now_us;
}
//
// ============================================================================
//  МИКРОДОЗЫ В HOLD: игла стоит на фиксированном открытии step_holding_*; раз в
//  dose_period_us сверяем фактический прогресс давления К ЦЕЛИ за окно и
//  подстраиваем открытие (+trim / -trim / -trim_big). Никакого dP/dt — только
//  само давление за 5 с, его видно сквозь шум. Возвращает позицию иглы на тик.
// ============================================================================
static int32_t hold_dose_step(PressureRegulator* reg, uint64_t now_us, bool charging, float pressure_raw) {
    int32_t* pos   = charging ? &reg->step_holding_charge    : &reg->step_holding_vent;
    bool*    calib = charging ? &reg->dose_charge_calibrated : &reg->dose_vent_calibrated;

    // п.1: близко к цели (по СЫРОМУ давлению) крупный шаг переливает -> big/very_big
    // заменяем обычным dose_trim. Далеко от цели — штатные крупные шаги.
    bool    far      = fabsf(reg->active_setpoint - pressure_raw) >= reg->dose_big_min_err;
    int32_t big      = far ? reg->dose_trim_big      : reg->dose_trim;
    int32_t very_big = far ? reg->dose_trim_very_big : reg->dose_trim;

    // п.2: кольцо чекпойнтов P_filt -> лаги 1.25..5с (статичные пороги, как в поиске).
    dose_hist_tick(reg, now_us);

    // --- ЗАЩИТНЫЕ / CLOSE направления — по ЛЮБОМУ лагу СРАЗУ (не ждём 5с) ---
    //   разгон      (прогресс к цели за какой-то лаг > runaway) -> прикрыть big
    //   чуть быстро (прогресс за какой-то лаг > fast)           -> прикрыть dose_trim
    //   утечка/мимо (прогресс за какой-то лаг < VERYslow)       -> открыть big/very_big
    // ВАЖНО: «слишком МЕДЛЕННО» (toward < slow) тут НЕТ — это проверка «НЕ МЕНЬШЕ X»,
    // а на коротком лаге даже нормальный подход даёт <slow (времени мало) -> ложно
    // «медленно» -> перелив. Минимум-прогресс судим только по полному 5с окну ниже.
    // Короткий лаг ловит резкий выброс, длинный — пологий (как search_dp_flow).
    // После залпа сбрасываем окно+кольцо -> ждём отклик, без повторного выстрела.
    if (reg->dose_hist_count > 0) {
        float toward_max = -1e9f, toward_min = 1e9f;
        for (int i = 0; i < reg->dose_hist_count; i++) {
            float toward_i = charging ? (reg->filtered_pressure - reg->dose_hist[i])
                                      : (reg->dose_hist[i] - reg->filtered_pressure);
            if (toward_i > toward_max) toward_max = toward_i;
            if (toward_i < toward_min) toward_min = toward_i;
        }
        int32_t old = *pos;
        bool fired = true;
        if      (toward_max > reg->dose_dp_runaway)  *pos -= big;                      // где-то сильно разогнались
        else if (toward_max > reg->dose_dp_fast)     *pos -= reg->dose_trim;           // q1.3: где-то чуть быстро -> прикрыть по ЛЮБОМУ лагу
        else if (toward_min < reg->dose_dp_VERYslow) *pos += *calib ? big : very_big;  // где-то течёт/мимо
        else                                         fired = false;
        if (fired) {
            if (*pos < reg->valve_flow_floor) *pos = reg->valve_flow_floor;
            if (*pos > reg->valve_max)        *pos = reg->valve_max;
            if (*pos != old)
                ESP_LOGI("PID", "HOLD: доза %s ЗАЩИТА %ld -> %ld (toward_max=%+.3f toward_min=%+.3f по лагам<=%.2fс)",
                         charging ? "НАБОР" : "СБРОС", (long)old, (long)*pos, toward_max, toward_min,
                         (float)(reg->dose_hist_dt_us * DOSE_HIST_N) / 1000000.0f);
            dose_window_reset(reg, now_us);
            return *pos;
        }
    }

    // --- ОСНОВНОЕ ОКНО 5с (длинный лаг): «медленно / хорошо / чуть быстро» ---
    if (now_us - reg->dose_t0_us >= reg->dose_period_us) {
        float dp     = reg->filtered_pressure - reg->dose_p0;  // изменение давления за окно
        float toward = charging ? dp : -dp;                    // прогресс в сторону цели
        int32_t old  = *pos;

        // Некалиброванное направление (засеяно floor'ом при входе в HOLD) впервые
        // дало нормальный поток -> доза найдена, дальше обычная тонкая подстройка.
        if (!*calib && toward >= reg->dose_dp_slow) {
            *calib = true;
            ESP_LOGI("PID", "HOLD: доза %s откалибрована на %ld (прогресс %.3f за окно)",
                     charging ? "НАБОР" : "СБРОС", (long)*pos, toward);
        }

        if      (toward < reg->dose_dp_VERYslow) *pos += *calib ? big : very_big;  // явно вниз/мимо (резерв длинного окна)
        else if (toward < reg->dose_dp_slow)     *pos += *calib ? reg->dose_trim   // штатный добор
                                                               : very_big;         // разгон с floor: ищем порог потока быстрее
        else if (toward > reg->dose_dp_runaway)  *pos -= big;                      // сильно разогнались (резерв длинного окна)
        else if (toward > reg->dose_dp_fast)     *pos -= reg->dose_trim;           // чуть быстрее нужного
        if (*pos < reg->valve_flow_floor) *pos = reg->valve_flow_floor;
        if (*pos > reg->valve_max)        *pos = reg->valve_max;
        if (*pos != old)
            ESP_LOGI("PID", "HOLD: доза %s %ld -> %ld (прогресс %.3f: P_filt %.3f->%.3f за 5с)",
                     charging ? "НАБОР" : "СБРОС", (long)old, (long)*pos, toward,
                     reg->dose_p0, reg->filtered_pressure);
        reg->dose_t0_us = now_us;
        reg->dose_p0    = reg->filtered_pressure;
    }
    return *pos;
}

// ============================================================================
//  БЫСТРЫЙ ПОИСК ПОРОГА ПОТОКА (идея 3). Пока доза НАБОРА не калибрована (порог
//  иглы неизвестен), вместо медленного +dose_trim за 5 с от floor перебираем иглой
//  вверх по search_step раз в search_step_period_us. Детекция потока — по
//  нескольким лагам: P_filt поднялось на search_dp_flow над ЛЮБЫМ из чекпойнтов
//  (кольцо раз в search_hist_dt_us, лаги 0.25..1.0с) -> поток пошёл. Тогда иглу НЕ
//  роняем в floor, а ДЕРЖИМ на найденной позиции и едем вверх до цели; дошли ->
//  калибровка засчитана, в дозу/удержание с этой же позиции.
//  Возвращает позицию иглы.
// ============================================================================
static int32_t hold_search_step(PressureRegulator* reg, uint64_t now_us) {
    // --- инициализация на входе в поиск: фаза Подгон ---
    if (!reg->dose_searching) {
        reg->dose_searching      = true;
        reg->search_phase        = 0;                       // 0 = Подгон фильтра
        // Перебор стартуем НЕ с floor, а с подсказки RATE (seed уже лежит в
        // step_holding_charge = позиция RATE минус dose_step_back). У этой иглы
        // порог высоко по шагам (~позиция RATE), перебор с floor занял бы десятки
        // секунд; с seed — несколько. Кламп в [floor, valve_max] на всякий.
        if (reg->step_holding_charge < reg->valve_flow_floor) reg->step_holding_charge = reg->valve_flow_floor;
        if (reg->step_holding_charge > reg->valve_max)        reg->step_holding_charge = reg->valve_max;
        reg->search_settle_t0_us = now_us;                  // таймер фазы (тут — Подгон)
        ESP_LOGI("PID", "HOLD: ПОИСК ПОРОГА — вход, фаза ПОДГОНА УГЛА (игла в floor, ждём P_filt; перебор начнётся с %ld). P_filt=%.2f rate_filt=%.2f",
                 (long)reg->step_holding_charge, reg->filtered_pressure, reg->filtered_rate);
    }

    // --- фаза 0 Подгон: после RATE фильтр P_filt ОТСТАЁТ от raw и слюит вверх.
    //     rate_filt считается по RAW (а raw после закрытия иглы стабилен почти сразу),
    //     поэтому ждём |rate_filt| мал = «raw стоит», затем ПРИБИВАЕМ P_filt к raw и
    //     только тогда перебираем. Без прибивки остаточный слю P_filt (rate_filt уже мал,
    //     а P_filt ещё ползёт — это РАЗНЫЕ EMA) принимается за поток: ловили «поток на
    //     221», а реально 266. ---
    if (reg->search_phase == 0) {
        bool settled = fabsf(reg->filtered_rate) < reg->search_warmup_rate;
        bool timeout = (now_us - reg->search_settle_t0_us) >= reg->search_warmup_max_us;
        if (settled || timeout) {
            reg->filtered_pressure = reg->prev_pressure;    // прибить P_filt к raw: убрать лаг EMA
            reg->search_phase      = 1;                     // -> ПЕРЕБОР
            reg->search_step_t0_us = now_us;
            reg->search_hist_t0_us = now_us;
            reg->search_hist_idx   = 0;
            reg->search_hist_count = 0;
            ESP_LOGI("PID", "HOLD: ПОИСК ПОРОГА — старт ПЕРЕБОРА с %ld (+%ld за %.2fс, %s). P_filt прибит к raw=%.2f",
                     (long)reg->step_holding_charge, (long)reg->search_step,
                     (float)reg->search_step_period_us / 1000000.0f,
                     settled ? "raw стабилен" : "таймаут подгона",
                     reg->filtered_pressure);
            // В самом начале перебора давление падает (скорость отрицательная) ->
            // запечатанный объём травит мимо иглы. Кричим капсом для диагностики.
            if (reg->filtered_rate < 0.0f)
                ESP_LOGW("PID", "УТЕЧКА: СКОРОСТЬ: %.3f кПа/с ШАГ: %ld",
                         reg->filtered_rate, (long)reg->step_holding_charge);
        }
        return reg->valve_flow_floor;
    }

    // --- фаза 2 ЕДЕМ ДО ЦЕЛИ: поток найден -> НЕ роняем в floor, ДЕРЖИМ иглу на
    //     найденной позиции и даём давлению дорасти до цели; дошли -> в дозу с этой
    //     же (текучей) позиции. Раньше тут была пауза в floor (давление утекало) +
    //     доза доползала обратно к порогу — это и был самый долгий кусок. ---
    if (reg->search_phase == 2) {
        float err_filt = reg->active_setpoint - reg->filtered_pressure;
        if (err_filt <= reg->search_target_back ) {          // доехали почти до цели (цель - search_target_back):
                                                            // остаток доведёт хвост давления, а калибровка
                                                            // фиксируется ЗДЕСЬ — раньше внешней проверки точной
                                                            // цели в SERVO_CHARGING (иначе перебор повторится)
            // Держим НЕ на самой флоу-позиции (там поток активен, хвоста search_target_back
            // хватает с перелётом), а на search_done_back шагов НИЖЕ: поток слабее -> остаток
            // давление добирает мягко, ниже порога потока доза догонит +dose_trim. Клампим в floor.
            int32_t hold_pos = reg->search_found_pos - reg->search_done_back;
            if (hold_pos < reg->valve_flow_floor) hold_pos = reg->valve_flow_floor;
            reg->step_holding_charge    = hold_pos;       // с него держим (и сюда же сядет первый вход FINE)
            reg->dose_charge_calibrated = true;
            reg->dose_searching         = false;
            dose_window_reset(reg, now_us);
            ESP_LOGI("PID", "HOLD: ПОИСК ПОРОГА — финиш на (цель-%.2f) на игле %ld -> держим %ld (-%ld), калибровка зафиксирована",
                     reg->search_target_back, (long)reg->search_found_pos, (long)hold_pos, (long)reg->search_done_back);
            return hold_pos;
        }
        // страховка от застоя: если за ~1 с давление не подросло (поток на найденной
        // позиции слабее утечки / ложный детект) — приоткрыть ещё, иначе зависнем ниже цели.
        if (now_us - reg->search_step_t0_us >= 1000000ULL) {
            if (reg->filtered_pressure < reg->dose_p0 + 0.02f) {
                reg->search_found_pos += reg->search_step;
                if (reg->search_found_pos > reg->valve_max) reg->search_found_pos = reg->valve_max;
                ESP_LOGI("PID", "HOLD: ПОИСК ПОРОГА — давление стоит, приоткрыл до %ld (едем до цели)",
                         (long)reg->search_found_pos);
            }
            reg->search_step_t0_us = now_us;
            reg->dose_p0           = reg->filtered_pressure;
        }
        return reg->search_found_pos;               // держим иглу, давление растёт к цели
    }

    // --- фаза 1 ПЕРЕБОР: поток = P_filt поднялось на search_dp_flow над ЛЮБЫМ из
    //     недавних чекпойнтов (лаги 0.25..1.0с). Проверяем КАЖДЫЙ тик -> реагируем
    //     почти сразу, без ожидания закрытия одного длинного окна (раньше окно 1с
    //     давало ΔP=0.27 и перелёт). ---
    float max_rise = 0.0f;
    for (int i = 0; i < reg->search_hist_count; i++) {
        float rise = reg->filtered_pressure - reg->search_hist[i];
        if (rise > max_rise) max_rise = rise;
    }
    if (max_rise >= reg->search_dp_flow) {
        reg->search_found_pos  = reg->step_holding_charge;
        reg->search_phase      = 2;                            // -> ЕДЕМ ДО ЦЕЛИ (не floor!)
        reg->search_step_t0_us = now_us;                       // таймер страховки от застоя в фазе 2
        reg->dose_p0           = reg->filtered_pressure;       // опорное P_filt для проверки роста
        ESP_LOGI("PID", "HOLD: ПОИСК ПОРОГА — ПОТОК на игле %ld (P_filt +%.3f над лагом <=%.2fс) -> едем до цели",
                 (long)reg->search_found_pos, max_rise,
                 (float)(reg->search_hist_dt_us * SEARCH_HIST_N) / 1000000.0f);
        return reg->search_found_pos;
    }
    // новый чекпойнт раз в search_hist_dt_us (кольцо, глубина SEARCH_HIST_N)
    if (now_us - reg->search_hist_t0_us >= reg->search_hist_dt_us) {
        reg->search_hist_t0_us = now_us;
        reg->search_hist[reg->search_hist_idx] = reg->filtered_pressure;
        reg->search_hist_idx = (reg->search_hist_idx + 1) % SEARCH_HIST_N;
        if (reg->search_hist_count < SEARCH_HIST_N) reg->search_hist_count++;
    }

    // --- перебор: приращение иглы раз в период ---
    if (now_us - reg->search_step_t0_us >= reg->search_step_period_us) {
        reg->search_step_t0_us    = now_us;
        reg->step_holding_charge += reg->search_step;
        if (reg->step_holding_charge >= reg->valve_max) {
            // дошли до упора без потока -> калибруемся на максимуме (лучшее усилие)
            reg->step_holding_charge    = reg->valve_max;
            reg->dose_charge_calibrated = true;
            reg->dose_searching         = false;
            dose_window_reset(reg, now_us);
            ESP_LOGW("PID", "HOLD: ПОИСК ПОРОГА — перебор дошёл до valve_max=%ld без потока, калибровка на упоре",
                     (long)reg->valve_max);
        }
    }
    return reg->step_holding_charge;
}

// q2. FINE: дрейф-шаг иглы с защитой «не толкать прочь от цели». Падает (drift<0)
// -> приоткрыть (+trim), растёт -> прикрыть (−trim). НО если давление уже ниже цели
// на fine_dev_guard и больше — НЕ прикрываем; если выше на столько же — НЕ
// приоткрываем (естественный дрейф К цели не глушим). error_filt = цель − P_filt
// (>0 = ниже цели). Возвращает фактический сдвиг: +trim / −trim / 0 (подавлено).
// sgn: НАБОР=+1 (открытие иглы гонит P ВВЕРХ), СБРОС=−1 (открытие гонит P ВНИЗ).
// Сначала считаем нужное ВОЗДЕЙСТВИЕ на давление phys (упало -> вверх, выросло ->
// вниз) и режем его защитой «не прочь от цели», потом переводим в шаг ИГЛЫ умножением
// на sgn: на наборе шаг=phys, на сбросе зеркальный (−phys).
static int32_t fine_drift_step(PressureRegulator* reg, float drift, float error_filt, int32_t trim, int sgn) {
    int32_t phys = (drift < 0.0f) ? trim : -trim;                 // знак нужного ВОЗДЕЙСТВИЯ на давление
    if (phys < 0 && error_filt >=  reg->fine_dev_guard) phys = 0;  // ниже цели -> не толкаем вниз
    if (phys > 0 && error_filt <= -reg->fine_dev_guard) phys = 0;  // выше цели -> не толкаем вверх
    return sgn * phys;                                            // НАБОР: шаг=phys; СБРОС: зеркально
}

// ============================================================================
//  ТОЧНЫЙ ХОЛДИНГ: серво постоянно в НАБОРЕ, игла стоит на равновесном
//  приоткрытии (приток компенсирует утечку). Раз в fine_period_us смотрим, куда
//  сползло отфильтрованное давление за окно, и двигаем открытие на fine_trim
//  ПРОТИВ знака: росло -> прикрыть, падало -> приоткрыть. dP за окно — это та же
//  средняя скорость, только сквозь шум её видно (мгновенная dP/dt у цели тонет).
//  Окно с |dP| < fine_eq_band = «стоим»: открытие запоминается в fine_eq_pos.
//  Стоим, но МИМО цели (|err_filt| > fine_corr_err) — само равновесие мягко
//  двигаем на fine_trim в сторону цели (P ниже -> приоткрыть, выше -> прикрыть).
//  Докачивать просадку печатью/обычным HOLD нельзя: пока объём
//  запечатан, магистраль меняется и равновесие протухает.
//  Окно переиспользует dose_t0_us/dose_p0 — с эпизодами оно не пересекается.
// ============================================================================
bool first_in_fine = false;
static int32_t hold_fine_step(PressureRegulator* reg, uint64_t now_us, float error_filt, int32_t trim) {
    // Направление берём из серво (в FINE оно не меняется): НАБОР -> +1, СБРОС -> −1.
    // Через sgn зеркалим знак «игла->давление»: на сбросе открытие иглы гонит P ВНИЗ,
    // поэтому «выросло (тепловой подскок) -> приоткрыть сброс, упало -> прикрыть».
    int sgn = (reg->servo_state == SERVO_VENTING) ? -1 : +1;
    // п.2: то же кольцо лагов, что и в дозе (FINE и HOLD-доза не пересекаются).
    dose_hist_tick(reg, now_us);

    // --- ДРЕЙФ — по ЛЮБОМУ лагу СРАЗУ (не ждём 5с): берём лаг с макс. |dP|;
    //     |dP| >= fine_eq_band (статичный порог) -> корректируем приоткрытие.
    //     Падает -> приоткрыть, растёт -> прикрыть. После шага сброс окна+кольца.
    //     «Стоим/равновесие» (тихо на всех лагах целое окно) уходит в 5с-ветку ниже.
    if (reg->dose_hist_count > 0) {
        float drift = 0.0f;                                   // знаковый dP лага с макс. |dP|
        for (int i = 0; i < reg->dose_hist_count; i++) {
            float dp_i = reg->filtered_pressure - reg->dose_hist[i];
            if (fabsf(dp_i) > fabsf(drift)) drift = dp_i;
        }
        if (fabsf(drift) >= reg->fine_eq_band) {
            int32_t step = fine_drift_step(reg, drift, error_filt, trim, sgn);  // q2: дрейф со «стоп прочь от цели» (sgn зеркалит СБРОС)
            if (step != 0) {
                int32_t old = reg->fine_pos;
                reg->fine_pos += step;
                if (reg->fine_pos < reg->valve_flow_floor) reg->fine_pos = reg->valve_flow_floor;
                if (reg->fine_pos > reg->valve_max)        reg->fine_pos = reg->valve_max;
                if (reg->fine_pos != old)
                    ESP_LOGI("PID", "FINE: дрейф %ld -> %ld (dP=%+.3f по лагу<=%.2fс, шаг %+ld)",
                             (long)old, (long)reg->fine_pos, drift,
                             (float)(reg->dose_hist_dt_us * DOSE_HIST_N) / 1000000.0f, (long)step);
                dose_window_reset(reg, now_us);
                return reg->fine_pos;
            }
            // step==0: дрейф ведёт К цели -> не толкаем прочь, держим иглу, даём
            // давлению дойти само. Окно/кольцо НЕ сбрасываем -> идём в 5с-ветку ниже.
        }
    }

    // --- ОСНОВНОЕ ОКНО 5с (длинный лаг): равновесие / стоим мимо цели ---
    if (now_us - reg->dose_t0_us >= reg->fine_period_us) {
        float dp = reg->filtered_pressure - reg->dose_p0;   // знак = знак средней dP/dt за окно
        int32_t old = reg->fine_pos;

        if (fabsf(dp) < reg->fine_eq_band) {
            // «стоим» — текущее открытие и есть равновесное, запоминаем
            reg->fine_eq_pos = reg->fine_pos;
            if (!reg->fine_eq_found) {
                reg->fine_eq_found = true;
                ESP_LOGI("PID", "FINE: равновесие найдено, игла %ld (dP=%+.3f)",
                         (long)reg->fine_eq_pos, dp);
            }
            if (fabsf(error_filt) > reg->fine_corr_err) {
                // Стоим (приток=утечка), но равновесие НЕ на цели -> МЯГКО сдвигаем его
                // к цели на fine_trim. Раньше тут был рывок до цели и обратно (на 20
                // шагов): при 1000 кПа это перелив 273->293 -> подскок до 1000.26 и
                // срыв в СБРОС (см. лог). И возврат на то же равновесие = снова просадка
                // (цикл). Маленький сдвиг равновесия не перебрасывает и реально центрирует.
                // phys = нужное ВОЗДЕЙСТВИЕ (ниже цели -> вверх +, выше -> вниз −); шаг
                // иглы = sgn*phys (на сбросе зеркально: ниже цели -> ПРИКРЫТЬ сброс).
                int32_t corr = sgn * ((error_filt > 0.0f) ? reg->fine_trim : -reg->fine_trim);
                reg->fine_pos += corr;
                if (reg->fine_pos < reg->valve_flow_floor) reg->fine_pos = reg->valve_flow_floor;
                if (reg->fine_pos > reg->valve_max)        reg->fine_pos = reg->valve_max;
                ESP_LOGI("PID", "FINE: стоим мимо цели (err_filt=%+.2f) -> сдвиг равновесия на %+ld -> %ld",
                         error_filt, (long)corr,
                         (long)reg->fine_pos);
            } else {
                ESP_LOGI("PID", "FINE: стоим, игла %ld (dP=%+.3f кПа за окно)",
                         (long)reg->fine_pos, dp);
            }
        } else {
            reg->fine_pos += fine_drift_step(reg, dp, error_filt, trim, sgn);  // q2: дрейф со «стоп прочь от цели» (sgn зеркалит СБРОС)
            if (reg->fine_pos < reg->valve_flow_floor) reg->fine_pos = reg->valve_flow_floor;
            if (reg->fine_pos > reg->valve_max)        reg->fine_pos = reg->valve_max;
            if (reg->fine_pos != old)
                ESP_LOGI("PID", "FINE: игла %ld -> %ld (dP=%+.3f за окно, шаг %ld, скорость %+.4f кПа/с)",
                         (long)old, (long)reg->fine_pos, dp, (long)trim,
                         dp / ((float)reg->fine_period_us / 1000000.0f));
        }
        reg->dose_t0_us = now_us;
        reg->dose_p0    = reg->filtered_pressure;
    }
    return reg->fine_pos;
}

// Ставит серво в нужное положение, но только если оно изменилось
// (чтобы не дёргать ШИМ каждый тик).
static void apply_servo(PressureRegulator* reg, ServoState s) {
    if (reg->servo_state == s) return;
    reg->servo_state = s;
    switch (s) {
        case SERVO_CHARGING: set_servo_angle(0.0f); break;
        case SERVO_VENTING:  set_servo_angle(180.0f);   break;
        case SERVO_NEUTRAL:  set_servo_angle(90.0f);  break;
    }
}

// Обнуляет интегратор контура скорости и состояние холда (при новой цели / сбросе).
static void reset_controllers(PressureRegulator* reg) {
    reg->rate_integral = 0.0f;
    reg->holding       = false;
    reg->fine_holding  = false;
    reg->fine_eq_found = false;   // на новой уставке равновесное открытие другое
    reg->fine_rate_t0_us = 0;     // сброс трекера fine_change (на новой уставке история не нужна)
    reg->dose_searching  = false; // начатый поиск порога потока к новой уставке не относится
    reg->fine_visited    = false; // переучивание FINE (±5 от прошлой позиции) — на новой уставке с чистого листа
    first_in_fine        = false; // недоигранная пауза входа в FINE к новой уставке не относится
}
                                                      //и только потом ждать минимально необходимую скорость (которая должна быть больше чем sensor_noise_delta)

// ============================================================================
//  ТЕСТ УТЕЧКИ (команда leak). Накачивает объём (серво НАБОР + игла настежь 10000),
//  ждёт 2 с; запечатывает (игла 0, серво нейтраль), ждёт 10 с устаканиться; за
//  окно 5 с мерит, сколько давления потерялось -> печатает скорость утечки кПа/с;
//  в конце стравливает всё. Блокирующая процедура, крутится в задаче регулятора
//  (REG_STATE_LEAK) -> единолично владеет иглой/серво. Иглу гоняем кусками с
//  vTaskDelay, чтобы длинный ход не морил сторожевой таймер.
// ============================================================================
static void leak(PressureRegulator* reg) {
    const int32_t  LEAK_OPEN    = 10000; // игла настежь (= MAX_VALVE_STEPS)
    const int32_t  STEP_CHUNK   = 2000;  // ход кусками — кормим watchdog
    const uint32_t LEAK_STEP_US = 190;   // скорость хода иглы (как в homing). На штатных
                                         // VALVE_STEP_US=400 полный ход 10000 шагов ~4с в
                                         // КАЖДУЮ сторону -> накачка+печать растягивались на
                                         // ~10с. 190 (~5кГц) homing тянет без срыва.
    int32_t chunk;

    ESP_LOGW("PID", "LEAK: тест утечки — НАЧАЛО (P=%.2f)", pressure1_kPa);

    // -- 1. Накачка: серво в НАБОР, игла настежь, ждём 2 с --
    apply_servo(reg, SERVO_CHARGING);
    chunk = current_valve_position;
    while (current_valve_position < LEAK_OPEN) {
        chunk += STEP_CHUNK;
        if (chunk > LEAK_OPEN) chunk = LEAK_OPEN;
        move_valve_absolute(chunk, LEAK_STEP_US);
        vTaskDelay(1);
    }
    ESP_LOGI("LEAK", "LEAK: игла настежь %ld, накачка 0.1с", (long)current_valve_position);
    vTaskDelay(pdMS_TO_TICKS(100));

    // -- 2. Запечатать: игла 0, серво нейтраль, ждём 10 с устаканиться --
    chunk = current_valve_position;
    while (current_valve_position > 0) {
        chunk -= STEP_CHUNK;
        if (chunk < 0) chunk = 0;
        move_valve_absolute(chunk, LEAK_STEP_US);
        vTaskDelay(1);
    }
    apply_servo(reg, SERVO_NEUTRAL);
    ESP_LOGI("LEAK", "LEAK: запечатано (игла 0, серво нейтраль), устаканиваемся 30с (P=%.2f)", pressure1_kPa);
    vTaskDelay(pdMS_TO_TICKS(10000));
    ESP_LOGI("LEAK", "давление щас, устаканиваемся (P=%.2f)", pressure1_kPa);
    vTaskDelay(pdMS_TO_TICKS(10000));
    ESP_LOGI("LEAK", "давление щас, устаканиваемся (P=%.2f)", pressure1_kPa);
    for(int i = 0; i < 20; i++){
        vTaskDelay(pdMS_TO_TICKS(500));
        ESP_LOGI("LEAK", "давление щас, считаем утечку (P=%.2f)", pressure1_kPa);
    }
    


    // -- 3. Окно 5 с: сколько давления потеряли --
    float p_start = pressure1_kPa;
    vTaskDelay(pdMS_TO_TICKS(5000));
    float p_end = pressure1_kPa;
    float lost  = p_start - p_end;      // >0 = давление упало (утечка)
    ESP_LOGW("PID", "LEAK: за 5с потеряно %.3f кПа -> УТЕЧКА %.4f кПа/с (P %.2f -> %.2f)",
             lost, lost / 5.0f, p_start, p_end);

    // -- 4. Стравить всё: серво СБРОС, игла настежь, ждём падения, закрыть --
    ESP_LOGI("PID", "LEAK: стравливаем всё...");
    apply_servo(reg, SERVO_VENTING);
    chunk = current_valve_position;
    while (current_valve_position < LEAK_OPEN) {
        chunk += STEP_CHUNK;
        if (chunk > LEAK_OPEN) chunk = LEAK_OPEN;
        move_valve_absolute(chunk, LEAK_STEP_US);
        vTaskDelay(1);
    }
    for (int guard = 0; pressure1_kPa > 0.2f && guard < 3000; guard++)
        vTaskDelay(pdMS_TO_TICKS(10));   // ждём падения давления, но не вечно (~30с макс)
    chunk = current_valve_position;
    while (current_valve_position > 0) {
        chunk -= STEP_CHUNK;
        if (chunk < 0) chunk = 0;
        move_valve_absolute(chunk, LEAK_STEP_US);
        vTaskDelay(1);
    }
    apply_servo(reg, SERVO_NEUTRAL);
    ESP_LOGW("PID", "LEAK: тест утечки — КОНЕЦ (P=%.2f)", pressure1_kPa);
}

// ============================================================================
//  ГЛАВНАЯ ЗАДАЧА РЕГУЛЯТОРА
// ============================================================================
uint64_t fix_time_for_delay = 0;
void pid_regulator_task(void *pvParameters) {
    PressureRegulator reg;
    regulator_init(&reg);

    reg.prev_pressure     = pressure1_kPa;
    reg.filtered_pressure = pressure1_kPa;   // стартуем фильтр с текущего давления, а не с 0
    reg.last_pressure_us  = esp_timer_get_time();

    uint64_t last_us      = esp_timer_get_time();
    uint32_t last_log_ms  = 0;
    const uint32_t LOG_PERIOD_MS = 500;

    while (1) {

        // -------- 0. Реальный шаг цикла dt (точная скорость + честные I-члены) --------
        uint64_t now_us = esp_timer_get_time();
        float dt = (float)(now_us - last_us) / 1000000.0f;
        last_us = now_us;
        if (dt < 0.005f) dt = 0.005f;   // защита от слишком малого/большого dt
        if (dt > 0.1f)   dt = 0.1f;

        // -------- 1. Обработка внешних запросов (idle/abort, homing) --------
        RegulatorState req = requested_reg_state;
        if (req == REG_STATE_IDLE) {
            reg.state = REG_STATE_IDLE;
            setpoint_kPa        = 0.0f;
            reg.active_setpoint = 0.0f;
            reset_controllers(&reg);
            requested_reg_state = REG_STATE_NONE;
        } else if (req == REG_STATE_HOMING) {
            is_homing = true;
            requested_reg_state = REG_STATE_NONE;
        } else if (req == REG_STATE_LEAK) {
            reg.state = REG_STATE_LEAK;          // одноразовый тест утечки (см. case ниже)
            requested_reg_state = REG_STATE_NONE;
        }

        // -------- 2. Пока идёт калибровка экрана/нуля — не трогаем привод --------
        if (is_calibrating) {
            vTaskDelay(pdMS_TO_TICKS(50));
            continue;
        }

        // -------- 3. Запрос физического сброса давления --------
        if (is_homing) reg.state = REG_STATE_HOMING;

        // -------- 4. Появилась новая уставка -> переходим в активное регулирование --------
        if (setpoint_kPa != reg.active_setpoint && reg.state != REG_STATE_HOMING && reg.state != REG_STATE_LEAK) {
            reg.active_setpoint = setpoint_kPa;
            reset_controllers(&reg);
            reg.state = REG_STATE_RUNNING;
            // Микро-фикс: стартуем иглу сразу с valve_flow_floor (край потока), а не
            // ползём от 0 через мёртвый ход. Если уже выше floor (меняем уставку на
            // ходу) — не опускаем.
            if (current_valve_position < reg.valve_flow_floor)
                move_valve_absolute(reg.valve_flow_floor, VALVE_STEP_US);
        }

        // -------- 5. Измерения: давление и его (отфильтрованная) скорость --------
        // ВАЖНО: датчик обновляет pressure1_kPa лишь раз в ~50 мс, а цикл крутится
        // быстрее. Если делить на dt цикла каждый тик, то на тиках, где отсчёт ещё не
        // обновился, получаем (P-P)/dt = 0, а на следующем — двойной скачок (через
        // раз rate=0). Поэтому скорость пересчитываем ТОЛЬКО когда пришёл свежий
        // отсчёт (значение реально изменилось), и по реальному времени с прошлого.
        float pressure = pressure1_kPa;
        if (fabsf(pressure-reg.prev_pressure) > 0.001f ) { // сравниваем если на одинаковых показаниях датчик дает чуть разное
            float dt_p = (float)(now_us - reg.last_pressure_us) / 1000000.0f;
            if (dt_p < 0.001f) dt_p = 0.001f;
            float raw_rate = (pressure - reg.prev_pressure) / dt_p;   // кПа/с по свежему отсчёту

            if (fabsf(raw_rate) > PRESS_GLITCH_RATE && reg.glitch_skips < PRESS_GLITCH_MAX) {
                // Аномальный скачок -> битый отсчёт. Игнорируем целиком: фильтры,
                // prev_pressure и время НЕ трогаем (dt_p сам подрастёт), а для error
                // этого тика берём последнее хорошее давление.
                reg.glitch_skips++;
                pressure = reg.prev_pressure;
            } else {
                reg.glitch_skips = 0;
                reg.raw_rate = raw_rate;                              // сохраняем сырую dP/dt для лога
                reg.filtered_rate = reg.rate_filter_alpha * reg.filtered_rate
                                  + (1.0f - reg.rate_filter_alpha) * raw_rate;
                reg.filtered_pressure = reg.press_filter_alpha * reg.filtered_pressure
                                      + (1.0f - reg.press_filter_alpha) * pressure;   // EMA давления (для порога HOLD)
                reg.prev_pressure    = pressure;
                reg.last_pressure_us = now_us;
            }
        }
        // если отсчёт не обновился — держим прежнюю filtered_rate (без ложных нулей)

        float error      = reg.active_setpoint - pressure;              // сырое: фаза RATE и вход в HOLD
        float error_filt = reg.active_setpoint - reg.filtered_pressure; // сглаженное: bang-bang серво в HOLD

        // ======================= АВТОМАТ СОСТОЯНИЙ =======================
        switch (reg.state) {

            // ---------- ПОКОЙ ----------
            case REG_STATE_IDLE: {
                apply_servo(&reg, SERVO_NEUTRAL);
                if (current_valve_position != 0) move_valve_absolute(0, VALVE_STEP_US);
                reset_controllers(&reg);
                if (esp_timer_get_time()/1000 - last_log_ms > LOG_PERIOD_MS) {
                    ESP_LOGI("PID", "Idle | P=%.2f kPa | Pos=%ld",
                             pressure, (long)current_valve_position);
                    last_log_ms = esp_timer_get_time()/1000;
                }
                vTaskDelay(pdMS_TO_TICKS(20));
                continue;
            }

            // ---------- ФИЗИЧЕСКИЙ СБРОС ДАВЛЕНИЯ ----------
            case REG_STATE_HOMING: {
                ESP_LOGW("PID", "Homing: сброс давления...");
                apply_servo(&reg, SERVO_VENTING);

                // Открываем иглу большими кусками до упора
                int32_t chunk = current_valve_position;
                const int32_t STEP_CHUNK = 2000;
                while (current_valve_position < MAX_VALVE_STEPS) {
                    chunk += STEP_CHUNK;
                    if (chunk > MAX_VALVE_STEPS) chunk = MAX_VALVE_STEPS;
                    move_valve_absolute(chunk, 190);
                    vTaskDelay(1);
                }
                // Ждём, пока давление реально упадёт
                while (pressure1_kPa > 0.2f) vTaskDelay(pdMS_TO_TICKS(10));
                // Закрываем иглу обратно
                chunk = current_valve_position;
                while (current_valve_position > 0) {
                    chunk -= STEP_CHUNK;
                    if (chunk < 0) chunk = 0;
                    move_valve_absolute(chunk, 190);
                    vTaskDelay(1);
                }

                apply_servo(&reg, SERVO_NEUTRAL);
                setpoint_kPa        = 0.0f;
                reg.active_setpoint = 0.0f;
                reset_controllers(&reg);
                reg.state = REG_STATE_IDLE;
                is_homing = false;
                ESP_LOGI("PID", "Homing завершён.");
                continue;
            }

            // ---------- ТЕСТ УТЕЧКИ ----------
            case REG_STATE_LEAK: {
                leak(&reg);                       // блокирующая процедура; в конце всё стравлено
                setpoint_kPa        = 0.0f;
                reg.active_setpoint = 0.0f;
                reset_controllers(&reg);
                reg.state = REG_STATE_IDLE;
                continue;
            }

            // ---------- АКТИВНОЕ РЕГУЛИРОВАНИЕ ----------
            case REG_STATE_RUNNING:
                break;  // основная логика ниже

            default:
                reg.state = REG_STATE_IDLE;
                continue;
        }

        // ======================= ЛОГИКА REG_STATE_RUNNING =======================
        // Две фазы:
        //  RATE — ПОДХОД: игла по скорости (внутренний PI), серво задаёт направление.
        //         Как только |error| <= hold_enter_err (0.5) — PID выключается
        //         НАВСЕГДА (до новой уставки), дальше только HOLD-микродозы.
        //  HOLD — в покое объём ЗАПЕЧАТАН: игла на valve_flow_floor, серво в нейтрали,
        //         газ из трубки серво->игла не сочится в объём. Коррекция-эпизод:
        //         |error_filt| > порога -> серво в нужную сторону, игла на
        //         ФИКСИРОВАННОМ дозирующем открытии step_holding_* (микродоза, без
        //         PI — скорости у цели не видны измерению dP/dt). Раз в 5 с открытие
        //         подстраивается по фактическому прогрессу давления (hold_dose_step).
        //         error_filt дошёл до порога печати -> игла на парковку, серво в нейтраль.
        //  FINE — точный холдинг (конфиг hold_fine_enable): когда НАБОРОМ вышли на
        //         точку, не печатаем — серво остаётся в НАБОРЕ, игла на
        //         step_holding_charge - fine_seed_back, раз в fine_period_us
        //         +-fine_trim по знаку сползания давления. Окно с |dP| <
        //         fine_eq_band = равновесие: открытие запоминается, игла стоит
        //         на нём без подкачек; стоим мимо цели — само равновесие мягко
        //         двигаем на fine_trim в сторону цели (hold_fine_step). Увод за
        //         hold_enter_err -> обратно в HOLD (эпизоды).
        // -------- Трекер «насколько P_filt ушло за последние ~5 с» (гейт FINE->HOLD) --------
        // Два чекпойнта по fine_period_us/2: сравниваем текущее P_filt с отсчётами
        // 2.5..5 с назад, берём больший сдвиг. По нему фаза FINE решает, давление
        // ещё активно движется или уже успокоилось. Крутится во всём RUNNING (и в
        // RATE/HOLD), чтобы на входе в FINE трекер уже нёс свежую историю.
        if (reg.fine_rate_t0_us == 0) {                  // первый тик / после смены уставки
            reg.fine_rate_t0_us  = now_us;
            reg.fine_rate_p_cur  = reg.filtered_pressure;
            reg.fine_rate_p_prev = reg.filtered_pressure;
        }
        if (now_us - reg.fine_rate_t0_us >= reg.fine_period_us / 2) {
            reg.fine_rate_t0_us  = now_us;
            reg.fine_rate_p_prev = reg.fine_rate_p_cur;
            reg.fine_rate_p_cur  = reg.filtered_pressure;
        }
        float ch_cur  = fabsf(reg.filtered_pressure - reg.fine_rate_p_cur);
        float ch_prev = fabsf(reg.filtered_pressure - reg.fine_rate_p_prev);
        reg.fine_change = (ch_cur > ch_prev) ? ch_cur : ch_prev;

        int32_t target_valve = current_valve_position;   // по умолчанию — стоим
        const char* zone;

        // Возврат HOLD->RATE, если ошибка уехала далеко (ложный вход на спайке или
        // сильная просадка): микродозой 17 кПа не наберёшь. Гистерезис: вошли в HOLD
        // при |error|<=hold_enter_err(0.5), выходим при |err_filt|>hold_exit_err(5).
        // На возврате сбрасываем калибровку — на другой высоте порог потока другой,
        // RATE накачает и поиск переищет уже у цели.
        if (reg.holding && fabsf(error_filt) > reg.hold_exit_err) {
            ESP_LOGW("PID", "HOLD->RATE: err_filt=%.2f > %.2f — далеко от цели, качаем RATE заново",
                     error_filt, reg.hold_exit_err);
            reg.holding                = false;
            reg.fine_holding           = false;
            reg.dose_searching         = false;
            reg.dose_charge_calibrated = false;
            reg.dose_vent_calibrated   = false;
            reg.fine_visited           = false;  // большой выброс — переучивание FINE начинаем заново
            reg.rate_integral          = 0.0f;   // чистый старт контура скорости
        }

        if (!reg.holding) {
            // ---- ФАЗА ПОДХОДА (RATE): рулим ШАГОВИКОМ по скорости ----
            zone = "RATE";

            if (fabsf(error) <= reg.hold_enter_err && fabsf(error_filt) <= reg.hold_enter_err) {
                // И сырое, И фильтр у цели -> это НЕ транзиентный спайк (видели заброс
                // сырого до 1000.49 при реальных P_filt=988 -> ложный вход в HOLD за 17
                // кПа от цели, потом 40 с поиска впустую). На спайке P_filt далеко -> ждём.
                // Подошли на 0.5 -> PID больше не работает. Прикрываем иглу на
                // dose_step_back от последнего открытия — это доза направления,
                // КОТОРЫМ ПОДХОДИЛИ (его перепад проверен делом). Противоположному
                // направлению ставим floor: перепад у него другой (бывает в 40 раз
                // больше), унаследованная доза стреляет — набор на 50 кПа со
                // сбросовой дозы 4455 дал +14 кПа за секунду. С floor оно потом
                // разгоняется по +dose_trim_big за окно до первого потока
                // (см. hold_dose_step), дальше штатные +-dose_trim.
                reg.holding = true;
                int32_t seed = current_valve_position - reg.dose_step_back;
                if (seed < reg.valve_flow_floor) seed = reg.valve_flow_floor;
                if (reg.servo_state == SERVO_VENTING) {        // подходили сбросом
                    reg.step_holding_vent      = seed;
                    reg.dose_vent_calibrated   = true;
                    reg.step_holding_charge    = reg.valve_flow_floor;
                    reg.dose_charge_calibrated = false;
                } else {                                       // подходили набором
                    reg.step_holding_charge    = seed;       // = старт перебора поиска порога (подсказка RATE)
                    // Доверять seed как готовой дозе нельзя: у этой иглы порог потока
                    // ~ позиции RATE (RATE до него и доводил), а seed = RATE - dose_step_back
                    // НИЖЕ порога -> сразу медленная доза +2/5с через мёртвую зону (видели
                    // 259->59 -> ползёт обратно ~250 с). Поэтому ВСЕГДА запускаем быстрый
                    // поиск порога (идея 3): он переберет от seed вверх и найдёт реальный порог.
                    reg.dose_charge_calibrated = false;
                    reg.step_holding_vent      = reg.valve_flow_floor;
                    reg.dose_vent_calibrated   = false;
                }
                dose_window_reset(&reg, now_us);
                target_valve = seed;              // серво НЕ трогаем — доводим в текущую сторону
                // (набор -> дальше быстрый ПОИСК ПОРОГА от seed; сброс -> доводим струйкой)
                ESP_LOGI("PID", "HOLD: вход, игла %ld -> %ld (seed). P=%.2f",
                         (long)current_valve_position, (long)seed, pressure);
            } else {
                float desired = desired_rate_from_error(error_filt, reg.sensor_noise_delta * 1.5, 0.15); // знаковая желаемая скорость  // 0.22 и 0.03 для 25мпа // *1.5 взял когда аварийное понижение лонастроил, потому что пофакут скорость нужна по хорошему чтобы побольше чем шум иначе почти не растет (чень долго растет)
                // было в конце reg.sensor_noise_delta_filt * 1.5, но решил поставить 0.15 чтоб росло хоть как то
                apply_servo(&reg, (desired > 0) ? SERVO_CHARGING : SERVO_VENTING);
                target_valve = rate_control_step(&reg, desired, dt);
            }
        } else if (reg.fine_holding) {
            // ---- ТОЧНЫЙ ХОЛДИНГ: серво стоит в НАБОРЕ, игла на равновесном
            //      приоткрытии, раз в окно +-fine_trim по знаку скорости ----
            zone = "FINE";

            if (first_in_fine) {
            // Пауза 1с в floor при входе в FINE (доза -> floor -> fine_pos): глушим
            // перелив дозы, чтобы войти в равновесие без остаточного разгона давления.
            // Серво остаётся в НАБОРЕ; floor ниже порога потока -> приток ~ноль, P оседает.
                if (now_us - fix_time_for_delay >= 1000000ULL) {   // 1 с прошла
                    first_in_fine = false;
                    dose_window_reset(&reg, now_us);               // окно равновесия — с чистого листа после паузы
                    target_valve = reg.fine_pos;
                    ESP_LOGI("PID", "FINE: пауза 1с в floor окончена -> игла на %ld", (long)reg.fine_pos);
                } else {
                    target_valve = reg.valve_flow_floor;           // стоим в floor, ждём
                }
            } else {
            // Гейт FINE->HOLD с защитой от дёрганья у порога 0.1. Раньше любое
            // |err_filt| > 0.1 печатало -> HOLD-эпизод -> снова FINE -> опять
            // проскок 0.1 пока равновесие не устаканилось = колебания. Теперь
            // смотрим ещё и на fine_change (сколько P_filt прошло за ~5 с):
            //   ещё активно движемся (>= fine_hold_change) -> ВСЕГДА остаёмся в
            //       FINE (ранний проскок 0.1 при незастывшем равновесии — не
            //       провал) и подстраиваем крупным шагом fine_trim_fast;
            //   успокоились и |err_filt| > 0.1 -> тонкая подстройка не вытянет,
            //       отдаём обычному HOLD (эпизоды), FINE включится сам после НАБОРА;
            //   успокоились и |err_filt| <= 0.1 -> остаёмся, штатный fine_trim.
            bool fast = (reg.fine_change >= reg.fine_hold_change);
            if (!fast && fabsf(error_filt) > 0.1) {
                // СБРОС-FINE: держать против утечки нечем (она тоже вниз) -> просто отдаём в
                // обычный HOLD ниже; charge-переучивание (fine_visited/last_pos/мид-доза)
                // пропускаем — это позиции НАБОРА (иной перепад на игле). НАБОР-FINE учится как раньше.
                if (reg.servo_state != SERVO_VENTING) {
                // Учимся на ПОВТОРНЫЙ вход: запоминаем последнюю позицию FINE и сторону
                // выброса. Выбросило ВНИЗ (P<цель, err_filt>0) -> в равновесии надо ОТКРЫТЬ
                // больше (+fine_relearn_step); ВВЕРХ (P>цель) -> прикрыть (−). Следующий
                // вход сядет на fine_last_pos+adj, а не всегда на «доза-fine_seed_back».
                reg.fine_last_pos = reg.fine_pos;
                reg.fine_seed_adj = (error_filt > 0.0f) ? reg.fine_relearn_step : -reg.fine_relearn_step;
                reg.fine_visited  = true;
                ESP_LOGW("PID", "FINE: запомнил позицию %ld, выброс %s -> следующий вход %+ld",
                         (long)reg.fine_last_pos,
                         (error_filt > 0.0f ? "вниз (<цель)" : "вверх (>цель)"),
                         (long)reg.fine_seed_adj);
                // Эпизод НАБОРА (если давление просело) откроет иглу на step_holding_charge —
                // но это ГРУБАЯ доза, она ВЫШЕ равновесия (дозе надо переливать, чтобы P дошло
                // до цели), а FINE держался на fine_pos НИЖЕ равновесия (там подтекало). Прыжок
                // сразу на грубую дозу перелетает (видели 258->274 -> P 1000.14). Садим дозу в
                // середину между ними — это ~равновесие, эпизод подкачает мягко.
                if (error_filt > 0.0f)
                    reg.step_holding_charge = (reg.fine_pos + reg.step_holding_charge) / 2;
                }   // конец «только НАБОР» (переучивание/мид-доза). Ниже — общий выход в HOLD.
                reg.fine_holding  = false;
                apply_servo(&reg, SERVO_NEUTRAL);
                target_valve = reg.valve_flow_floor;
                ESP_LOGW("PID", "FINE: err_filt=%.2f change=%.2f/5с — выход в обычный HOLD (доза -> %ld)",
                         error_filt, reg.fine_change, (long)reg.step_holding_charge);
            } else {
                int32_t trim = fast ? reg.fine_trim_fast : reg.fine_trim;
                target_valve = hold_fine_step(&reg, now_us, error_filt, trim);
            }
            }  // конец else (пауза first_in_fine не активна) — штатный FINE
        } else {
            // ---- ФАЗА ХОЛДА: покой запечатан, коррекции — микродозы ----
            zone = "HOLD";

            switch (reg.servo_state) {
                case SERVO_NEUTRAL: {
                    target_valve = reg.valve_flow_floor;     // запечатано: игла закрыта
                    ServoState dir = SERVO_NEUTRAL;
                    if      (error_filt >  0.1f) dir = SERVO_CHARGING; // упало -> набрать     // пробую фильт * 2 // маловато, решил 0.1 поставить
                    else if (error_filt < -0.1f) dir = SERVO_VENTING;  // выросло -> стравить
                    if (dir != SERVO_NEUTRAL) {
                        apply_servo(&reg, dir);
                        dose_window_reset(&reg, now_us);
                        target_valve = (dir == SERVO_CHARGING) ? reg.step_holding_charge
                                                               : reg.step_holding_vent;
                        ESP_LOGI("PID", "HOLD: эпизод %s, игла на %ld, err_filt=%.2f",
                                 (dir == SERVO_CHARGING) ? "НАБОР" : "СБРОС",
                                 (long)target_valve, error_filt);
                    }
                    break;
                }

                case SERVO_CHARGING:
                    if (error_filt <= reg.sensor_noise_delta_filt) {             // дошли РОВНО до цели  // поменял 0 на reg.sensor_noise_delta_filt (0.03) для компенсации перелета. Не уверен что надо именно эту переменную, но в целом как будто она подходит
                        
                        reg.dose_searching = false;   // цель достигнута — быстрый поиск (если шёл) прекращаем
                        if (reg.hold_fine_enable && reg.dose_charge_calibrated) {
                            // НАБОРОМ вышли на точку и доза проверена делом ->
                            // вместо печати точный холдинг: серво ОСТАЁТСЯ в
                            // НАБОРЕ. Если равновесие уже найдено — игла сразу
                            // на него; если нет — чуть ниже дозы и ищем
                            // (+-fine_trim раз в окно по знаку скорости).
                            reg.fine_holding  = true;
                            if (reg.fine_visited) {
                                // уже были в FINE этой уставкой -> ±fine_relearn_step от
                                // последней позиции по стороне прошлого выброса (учимся)
                                reg.fine_pos = reg.fine_last_pos + reg.fine_seed_adj;
                            } else {
                                // первый вход: дозовая/флоу-позиция step_holding_charge ВЫШЕ равновесия
                                // (доза переливает, чтобы ДОСТАТЬ цель), поэтому садимся на fine_seed_back
                                // шагов НИЖЕ — ближе к точке, где приток = утечке. fine_seed_back держим
                                // МАЛЕНЬКИМ (~4): большой откат (было 20) FINE потом долго отыгрывал вверх.
                                reg.fine_pos = reg.step_holding_charge - reg.fine_seed_back;
                            }
                            if (reg.fine_pos < reg.valve_flow_floor) reg.fine_pos = reg.valve_flow_floor;
                            if (reg.fine_pos > reg.valve_max)        reg.fine_pos = reg.valve_max;
                            dose_window_reset(&reg, now_us);

                            ESP_LOGI("PID", "FINE: вход, игла %ld (%s), серво остаётся в НАБОРЕ. P=%.2f",
                                     (long)reg.fine_pos,
                                     reg.fine_visited ? "±переучивание от прошлой"
                                                      : "первый вход: на найденную флоу-позицию",
                                     pressure);
                            //target_valve = reg.fine_pos;
                            target_valve = reg.valve_flow_floor;
                            fix_time_for_delay = now_us; // фиксируем время для задержки в 1с перед тем как перейти к fine_pos
                            first_in_fine = true; // это сделано чтоб секунду постоять (270 - 0 - 250 вот так сделать)
                        } else {
                            apply_servo(&reg, SERVO_NEUTRAL);   // печатаем как раньше
                            target_valve = reg.valve_flow_floor;
                        }
                    } else if (!reg.dose_charge_calibrated) {
                        // порог потока ещё неизвестен -> быстрый перебор к нему (идея 3),
                        // а не медленный +dose_trim за 5 с через весь мёртвый ход
                        zone = "SEARCH";
                        target_valve = hold_search_step(&reg, now_us);
                    } else {
                        target_valve = hold_dose_step(&reg, now_us, true, pressure);
                    }
                    break;

                case SERVO_VENTING:
                    if (error_filt >= -reg.sensor_noise_delta_filt) {            // дошли РОВНО до цели
                        reg.dose_searching = false;   // цель достигнута
                        if (reg.hold_fine_enable && reg.dose_vent_calibrated) {
                            // СБРОСОМ вышли на точку и доза проверена делом -> вместо печати
                            // ТОЧНЫЙ ХОЛДИНГ, серво ОСТАЁТСЯ в СБРОСЕ (servo=2). hold_fine_step
                            // берёт знак из servo_state и зеркалит: тут открытие иглы СТРАВЛИВАЕТ,
                            // поэтому «выросло (тепловой подскок) -> приоткрыть, упало -> прикрыть».
                            // Держать против УТЕЧКИ тут нечем (она тоже вниз): просадка ниже цели
                            // уйдёт в обычный HOLD -> эпизод НАБОРА (гейт FINE->HOLD при |err|>0.1).
                            reg.fine_holding = true;
                            // первый вход: садимся на fine_seed_back НИЖЕ дозы (меньше сброса ->
                            // не проскочим вниз). Переучивание (fine_visited) у СБРОСА не
                            // используем — те позиции относятся к НАБОРУ (иной перепад на игле).
                            reg.fine_pos = reg.step_holding_vent - reg.fine_seed_back;
                            if (reg.fine_pos < reg.valve_flow_floor) reg.fine_pos = reg.valve_flow_floor;
                            if (reg.fine_pos > reg.valve_max)        reg.fine_pos = reg.valve_max;
                            dose_window_reset(&reg, now_us);
                            ESP_LOGI("PID", "FINE(СБРОС): вход, игла %ld, серво остаётся в СБРОСЕ. P=%.2f",
                                     (long)reg.fine_pos, pressure);
                            target_valve = reg.valve_flow_floor;
                            fix_time_for_delay = now_us;   // пауза 1с в floor перед fine_pos (как у НАБОРА)
                            first_in_fine = true;
                        } else {
                            apply_servo(&reg, SERVO_NEUTRAL);   // печатаем как раньше
                            target_valve = reg.valve_flow_floor;
                        }
                    } else {
                        target_valve = hold_dose_step(&reg, now_us, false, pressure);
                    }
                    break;
            }
        }

        // -------- Ограничение скорости движения иглы и собственно перемещение --------
        int32_t step = target_valve - current_valve_position;
        if (step >  reg.max_step) target_valve = current_valve_position + reg.max_step;
        if (step < -reg.max_step) target_valve = current_valve_position - reg.max_step;

        move_valve_absolute(target_valve, VALVE_STEP_US);

        // -------- Периодический лог --------
        uint32_t now_ms = esp_timer_get_time() / 1000;
        if (now_ms - last_log_ms >= LOG_PERIOD_MS) {
            ESP_LOGI("PID",
                "%s | P=%.2f P_filt=%.2f set=%.1f err=%.2f rate=%.2f rate_filt=%.2f kPa/s | servo=%d pos=%ld ch5=%.2f",
                zone, pressure, reg.filtered_pressure, reg.active_setpoint, error, reg.raw_rate, reg.filtered_rate,
                reg.servo_state, (long)current_valve_position, reg.fine_change);
            last_log_ms = now_ms;
        }

        vTaskDelay(pdMS_TO_TICKS(20));
    }
}
