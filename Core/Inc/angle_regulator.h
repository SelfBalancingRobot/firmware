/*
 * angle_regulator.h
 *
 *  Created on: Jun 11, 2026
 *      Author: ozker
 */

#ifndef INC_ANGLE_REGULATOR_H_
#define INC_ANGLE_REGULATOR_H_

typedef struct {
	float Kp;
	float Ki;
	float Kd;
	float integral;
	float prev_error;

}angle_pid_t;

float angle_pid_count_pwm(angle_pid_t *pid, float gx, float target_angle, float current_angle, float Ts_s);


#endif /* INC_ANGLE_REGULATOR_H_ */
