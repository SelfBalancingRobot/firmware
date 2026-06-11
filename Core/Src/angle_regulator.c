/*
 * angle_regulator.c
 *
 *  Created on: Jun 11, 2026
 *      Author: ozker
 */

#include "angle_regulator.h"

float angle_pid_count_pwm(angle_pid_t *pid, float gx, float target_angle, float current_angle, float Ts_s){
	float error = target_angle - current_angle;
	pid->integral += error * Ts_s;
	float derivative = -gx;
	pid->prev_error = error;
	float target_pwm = pid->Kp * error + pid->Ki * pid->integral + pid->Kd * derivative;
	if(target_pwm > 100.0f){
		target_pwm = 100.0f;
		pid->integral -= error * Ts_s;
	}else if(target_pwm < -100.0f){
		target_pwm = -100.0f;
		pid->integral -= error * Ts_s;
	}
	return target_pwm;
}
