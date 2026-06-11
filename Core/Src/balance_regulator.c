/*
 * balance_regulator.c
 *
 *  Created on: May 20, 2026
 *      Author: ozker
 */

#include "balance_regulator.h"

float balance_count_target_rpm(balance_pid_t *pid, float current_angle, float Ts){

	float error = pid-> target_angle - current_angle;
	pid->integral += error * Ts;
	float derivative = (error - pid->prev_error) / Ts;
	pid->prev_error = error;
	float target_rpm = (pid->Kp * error) + (pid->Ki * pid->integral) + (pid->Kd * derivative);
	if(target_rpm < pid->rpm_death_band && target_rpm > -pid->rpm_death_band){
		target_rpm = 0;
		pid->integral -= error * Ts;
	}else if(target_rpm > 200.0f){
		target_rpm = 200.0f;
		pid->integral -= error * Ts;
	}else if (target_rpm < -200.0f){
		target_rpm = -200.0f;
		pid->integral -= error * Ts;
	}
	return target_rpm;
}
