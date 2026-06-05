#pragma once

#include <stdbool.h>
#include <stdint.h>



// Структуры
typedef struct {
    float Kp, Ki, Kd;
    float dt;
    float integral;
    float prev_error;
    float derivative_filtered;
    float derivative_alpha;
    float kp_scale_high, kp_scale_low;
    float ki_scale_high, ki_scale_low;
    float integral_max, integral_min;
    float prev_error_cross;
    float vent_integral;      // интегратор для сброса
} PIDController;

typedef struct {
    float final_setpoint;
    float ramp_setpoint;
    bool   ramp_active;
} RampController;

typedef enum {
    SERVO_NEUTRAL,
    SERVO_CHARGING,
    SERVO_VENTING
} ServoState;

typedef enum {
    REG_STATE_IDLE,
    REG_STATE_HOMING,
    REG_STATE_RAMPING,
    REG_STATE_HOLDING,
    REG_STATE_HOLD
} RegulatorState;

typedef struct {
    PIDController pid;
    RampController ramp;
    RegulatorState state; 
    RegulatorState prev_state;
    ServoState servo_state;
    
    uint32_t stuck_counter_pos;
    uint32_t stuck_counter_neg;
    
    float final_charge_on_th;
    float final_charge_off_th;
    float vent_on_th;
    float vent_off_th;
    
    int32_t max_neutral_pos;
    int32_t max_step;
} PressureRegulator;


extern volatile RegulatorState requested_reg_state;   // запрос извне
void regulator_request_state(RegulatorState new_state);

// Прототипы функций
void regulator_init(PressureRegulator* reg, float dt);
void update_ramp(PressureRegulator* reg, float current_pressure);
float calculate_pid(PIDController* pid, float error, float derivative, bool is_venting);
void update_servo_state(PressureRegulator* reg, float error);
void apply_neutral_dwell(PressureRegulator* reg, float error, int32_t* target);
void pid_regulator_task(void *pvParameters);
void regulator_start_task(void);