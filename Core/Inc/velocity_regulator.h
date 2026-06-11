/*
 * velocity_regulator.h
 *
 *  Created on: Jun 11, 2026
 *      Author: ozker
 */

#ifndef INC_VELOCITY_REGULATOR_H_
#define INC_VELOCITY_REGULATOR_H_

typedef struct {
	float Kp;
	float Ki;
	float Kd;
	float integral;
	float prev_error;
}velocity_regulator_t;

float velocity_pid_count_angle(velocity_regulator_t *vel_pid, float target_vel, float current_vel, float Ts_s);

#endif /* INC_VELOCITY_REGULATOR_H_ */
