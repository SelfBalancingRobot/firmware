/*
 * pid_parser.h
 *
 *  Created on: May 22, 2026
 *      Author: ozker
 */

#ifndef INC_COMMAND_PARSER_H_
#define INC_COMMAND_PARSER_H_

#include "dc_motor.h"
#include "balance_regulator.h"
#include "MPU6050.h"
#include "angle_regulator.h"
#include "velocity_regulator.h"

typedef enum{
	BALANCE,
	CALIBRATION
}robot_mode_t;

typedef struct{
	robot_mode_t *mode;
	bool *send_imu_offsets_flag;
	bool *send_pid_flag;
	float *angle;
	velocity_regulator_t *vel_pid;
	angle_pid_t *ang_pid;
	motor_t *motor1;
	motor_t *motor2;
	mpu_kalman_t *kalman;
}command_context_t;

void parse_command(command_context_t *command_context, char *msg);

#endif /* INC_COMMAND_PARSER_H_ */
