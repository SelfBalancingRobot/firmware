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

typedef enum{
	BALANCE,
	MOTOR_CALIBRATION
}robot_mode_t;

typedef struct{
	robot_mode_t *mode;
	bool *send_imu_offsets_flag;
	float *rpm;
	motor_t *motor1;
	motor_t *motor2;
	balance_pid_t *balance_pid;
	mpu_kalman_t *kalman;
}command_context_t;

void parse_command(command_context_t *command_context, char *msg);

#endif /* INC_COMMAND_PARSER_H_ */
