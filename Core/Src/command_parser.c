/*
 * pid_parser.c
 *
 *  Created on: May 22, 2026
 *      Author: ozker
 */
#include "command_parser.h"
#include <string.h>
#include <stdlib.h>



static void pid_command_process(angle_pid_t *angle_pid, velocity_regulator_t *velocity_pid, bool *flag, char *msg){
	if (strlen(msg) < 6){
		return;
	}

	char subcommand = msg[3];
	char term = msg[4];

	float value = strtof(&msg[5], NULL);

	if(subcommand == 'a'){
		switch(term){
		case 'p':
			angle_pid->Kp = value;
			break;
		case 'i':
			angle_pid->Ki = value;
			break;
		case 'd':
			angle_pid->Kd = value;
			break;
		default:
			break;
		}
	}
	else if(subcommand == 'v'){
		switch(term){
		case 'p':
			velocity_pid->Kp = value;
			break;
		case 'i':
			velocity_pid->Ki = value;
			break;
		case 'd':
			velocity_pid->Kd = value;
			break;
		default:
			break;
		}
	}
	else if(subcommand == 'f'){
		*flag = true;
	}
}

static void mode_command_process(robot_mode_t *mode_p, char *msg){
	if(strlen(msg) < 5){
		return;
	}
	if(msg[4] == 'b'){
		*mode_p = BALANCE;
	}else if(msg[4] == 'c'){
		*mode_p = CALIBRATION;
	}
}

static void imu_command_process(bool *offsets_flag, char *msg){
	if(strlen(msg) < 4){
		return;
	}
	if(msg[3] == 'o'){
		*offsets_flag = true;
	}
}

static void step_command_process(float *step, char *msg){
	if(strlen(msg) < 5){
		return;
	}
	*step = strtof(&msg[4], NULL);
}

static void angle_command_process(float *calib_ang, char *msg){
	if(strlen(msg) < 4){
		return;
	}
	*calib_ang = strtof(&msg[3], NULL);
}


static void kalman_command_process(mpu_kalman_t *kalman, char *msg){
	if(strlen(msg) < 6){
		return;
	}
	if(strncmp(&msg[3], "rm", 2) == 0){
		kalman->R_measure = strtof(&msg[5], NULL);
	}
	else if(strncmp(&msg[3], "qa", 2) == 0){
		kalman->Q_angle = strtof(&msg[5], NULL);
	}
	else if(strncmp(&msg[3], "qb", 2) == 0){
		kalman->Q_bias = strtof(&msg[5], NULL);
	}
}


void parse_command(command_context_t *command_context, char *msg){
	if(strncmp(msg, "pid", 3) == 0){
		pid_command_process(command_context->ang_pid, command_context->vel_pid, command_context->send_pid_flag, msg);
	}else if(strncmp(msg, "mode", 4) == 0){
		mode_command_process(command_context->mode, msg);
	}else if(strncmp(msg, "imu", 3) == 0){
		imu_command_process(command_context->send_imu_offsets_flag, msg);
	}else if(strncmp(msg, "step", 4) == 0){
		step_command_process(&command_context->motor1->ramp_step, msg);
		step_command_process(&command_context->motor2->ramp_step, msg);
	}else if(strncmp(msg, "kal", 3) == 0){
		kalman_command_process(command_context->kalman, msg);
	}else if(strncmp(msg, "ang", 3) == 0){
		angle_command_process(command_context->angle, msg);
	}
}

