/*
 * balance_regulator.h
 *
 *  Created on: May 20, 2026
 *      Author: ozker
 */

#ifndef INC_BALANCE_REGULATOR_H_
#define INC_BALANCE_REGULATOR_H_


typedef struct {
	float target_angle;
	float Kp;
	float Ki;
	float Kd;
	float integral;
	float prev_error;
}balance_pid_t;

float balance_count_target_rpm(balance_pid_t *pid, float current_angle, float Ts);

#endif /* INC_BALANCE_REGULATOR_H_ */
