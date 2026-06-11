/*
 * velocity_regulator.c
 *
 *  Created on: Jun 11, 2026
 *      Author: ozker
 */
#include "velocity_regulator.h"

float velocity_pid_count_angle(velocity_regulator_t *vel_pid, float target_vel, float current_vel, float Ts_s){
	float error = target_vel - current_vel;
	vel_pid->integral += error * Ts_s;
	float derivative = (error - vel_pid->prev_error) / Ts_s;
	vel_pid->prev_error = error;
	float target_angle = vel_pid->Kp * error + vel_pid->Ki * vel_pid->integral + vel_pid->Kd * derivative;
	if(target_angle > 8.0f){
		target_angle = 8.0f;
		vel_pid->integral -= error * Ts_s;
	}else if(target_angle < -8.0f){
		target_angle = -8.0f;
		vel_pid->integral -= error * Ts_s;
	}
	return target_angle;
}
