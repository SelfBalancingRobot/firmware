/*
 * dc_motor.h
 *
 *  Created on: Mar 22, 2026
 *      Author: ozker
 */

#ifndef INC_DC_MOTOR_H_
#define INC_DC_MOTOR_H_

#include "tim.h"

typedef struct{
    TIM_HandleTypeDef *pwm_tim;
    uint32_t pwm_channel_A;
    uint32_t pwm_channel_B;

    TIM_HandleTypeDef *encoder_tim;
    int32_t encoder_prev_count;
    int32_t encoder_curr_count;
    uint32_t encoder_resolution;

    float measured_rpm;
    float target_rpm;
    float max_rpm;
    float ramped_rpm;
    float ramp_step;

    float regulator_Kp;
    float regulator_Ki;
    float integral;
    int sign;
}motor_t;

void motor_init(motor_t *motor);
void motor_measure_rpm(motor_t *motor, float Ts);
void motor_set_rpm(motor_t *motor, float rpm, float Ts);
void motor_set_raw_pwm(motor_t *motor, uint32_t duty, uint32_t channel);
void motor_set_signed_pwm(motor_t *motor, float pwm);
#endif /* INC_DC_MOTOR_H_ */
