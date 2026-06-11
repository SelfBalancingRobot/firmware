/*
 * dc_motor.c
 *
 *  Created on: Mar 22, 2026
 *      Author: ozker
 */
#include "dc_motor.h"

static void pwm_set_duty(motor_t *motor, uint32_t duty, uint32_t channel){
    if(duty > 100){
        duty = 100;
    }
    uint32_t arr = __HAL_TIM_GET_AUTORELOAD(motor->pwm_tim);
    uint32_t ccr = ((uint64_t)(arr + 1) * duty) / 100;
    __HAL_TIM_SET_COMPARE(motor->pwm_tim, channel, ccr);
}
static int32_t encoder_get_cnt(motor_t *motor){
    return (int32_t)(__HAL_TIM_GET_COUNTER(motor->encoder_tim));
}

static int32_t encoder_get_delta_cnt(motor_t *motor){
    motor->encoder_prev_count = motor->encoder_curr_count;
    motor->encoder_curr_count = encoder_get_cnt(motor);
    int32_t delta = motor->encoder_curr_count - motor->encoder_prev_count;

    int32_t arr = __HAL_TIM_GET_AUTORELOAD(motor->encoder_tim);
    int32_t half = arr / 2;

    if (delta > half){
        delta -= arr + 1;
    }
    if (delta < -half){
         delta += arr + 1;
    }
    return delta;
}

static void motor_update_ramp(motor_t *motor){
	if(motor->target_rpm > motor->ramped_rpm + motor->ramp_step){
		motor->ramped_rpm += motor->ramp_step;
	}else if(motor->target_rpm < motor->ramped_rpm - motor->ramp_step){
		motor->ramped_rpm -= motor->ramp_step;
	}else{
		motor->ramped_rpm = motor->target_rpm;
	}
}

void motor_measure_rpm(motor_t *motor, float Ts){
    int32_t delta = encoder_get_delta_cnt(motor);
    float revolutions = (float)delta / motor->encoder_resolution;
    motor->measured_rpm = motor->sign * (revolutions / Ts) * 30.0f;
}

void motor_init(motor_t *motor){
    if (motor == NULL || motor->pwm_tim == NULL || motor->encoder_tim == NULL) {
        return;
    }
    if(motor->sign < 0){
    	motor->sign = -1;
    }else{
    	motor->sign = 1;
    }

    __HAL_TIM_SET_AUTORELOAD(motor->encoder_tim, 0xFFFF);
    __HAL_TIM_SET_COUNTER(motor->encoder_tim, 0);
    HAL_TIM_Encoder_Start(motor->encoder_tim, TIM_CHANNEL_ALL);
    
    pwm_set_duty(motor, 0, motor->pwm_channel_A);
    pwm_set_duty(motor, 0, motor->pwm_channel_B);
    HAL_TIM_PWM_Start(motor->pwm_tim, motor->pwm_channel_A);
    HAL_TIM_PWM_Start(motor->pwm_tim, motor->pwm_channel_B);
}

void motor_set_rpm(motor_t *motor, float rpm, float Ts){

    if(rpm > motor->max_rpm){
        rpm = motor->max_rpm;
    }else if(rpm < -motor->max_rpm){
        rpm = -motor->max_rpm;
    }
    motor->target_rpm = rpm;
    motor_update_ramp(motor);
    motor_measure_rpm(motor, Ts);
    float error = motor->ramped_rpm - motor->measured_rpm;
    motor->integral += error * Ts;
    float u = (motor->regulator_Kp * error) + (motor->regulator_Ki * motor->integral);
    if(u > 100.0f){
        u = 100.0f;
        motor->integral -= error * Ts;
    }else if(u < -100.0f){
        u = -100.0f;
        motor->integral -= error * Ts;
    }
    if(u > 0){
        uint32_t duty = (uint32_t)(u + 0.5f);
        pwm_set_duty(motor, 0, motor->pwm_channel_B);
        pwm_set_duty(motor, duty, motor->pwm_channel_A);
    }else{
        uint32_t duty = (uint32_t)(-u + 0.5f);
        pwm_set_duty(motor, 0, motor->pwm_channel_A);
        pwm_set_duty(motor, duty, motor->pwm_channel_B);
    }
}

void motor_set_raw_pwm(motor_t *motor, uint32_t duty, uint32_t channel){
    pwm_set_duty(motor, duty, channel);
}

void motor_set_signed_pwm(motor_t *motor, float pwm){
    if(pwm > 0){
        uint32_t duty = (uint32_t)(pwm + 0.5f);
        motor_set_raw_pwm(motor, 0, motor->pwm_channel_B);
        motor_set_raw_pwm(motor, duty, motor->pwm_channel_A);

    }else{
        uint32_t duty = (uint32_t)(-pwm + 0.5f);
        motor_set_raw_pwm(motor, 0, motor->pwm_channel_A);
        motor_set_raw_pwm(motor, duty, motor->pwm_channel_B);
    }
}
