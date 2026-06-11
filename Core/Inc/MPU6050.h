/*
 * MPU6050.h
 *
 *  Created on: Feb 28, 2026
 *      Author: ozker
 */

#ifndef INC_MPU6050_H_
#define INC_MPU6050_H_
#include "stdbool.h"
#include "i2c.h"

typedef struct {
	float Q_angle;
	float Q_bias;
	float R_measure;

	float angle;
	float bias;

	float P[2][2];

	bool initialized;
}mpu_kalman_t;


bool mpu_who_am_i(void);
void mpu_init(I2C_HandleTypeDef *hi2c);

void mpu_get_accel(float *accel_x_ptr, float *accel_y_ptr, float *accel_z_ptr);
void mpu_get_gyro(float *gyro_x_ptr, float *gyro_y_ptr, float *gyro_z_ptr);

void mpu_count_gyro_offset(void);
void mpu_get_gyro_offset(float *gx_ptr, float *gy_ptr, float *gz_ptr);

void mpu_count_accel_offset();
void mpu_get_accel_offset(float *ax_ptr, float *ay_ptr, float *az_ptr);

float mpu_count_angle_complementary(float ax, float az, float gy, float dt_s);

float mpu_count_angle_Kalman(mpu_kalman_t *kalman, float ax, float az, float gy, float dt_s);
void mpu_reset_Kalman(mpu_kalman_t *kalman, float ax, float az);


#endif /* INC_MPU6050_H_ */
