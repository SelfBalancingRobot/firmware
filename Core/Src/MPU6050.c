/*
 * MPU6050.c
 *
 *  Created on: Feb 28, 2026
 *      Author: ozker
 */

#include "MPU6050.h"
#include "MPU6050_regs.h"
#include "math.h"
#include "my_delay.h"


static I2C_HandleTypeDef *my_i2c = NULL;
static float accel_divider = 16384.0f;
static float gyro_divider = 131.0f;

static float gyro_x_offset = 0.0f;
static float gyro_y_offset = 0.0f;
static float gyro_z_offset = 0.0f;

static float accel_x_offset = 0.0f;
static float accel_y_offset = 0.0f;
static float accel_z_offset = 0.0f;

static float filtered_angle_deg = 0.0f;

static void mpu_read_reg(uint8_t reg_addr, uint8_t* result_ptr, uint8_t result_size){
    HAL_I2C_Mem_Read(my_i2c, MPU6050_ADDR, reg_addr, 1, result_ptr, result_size, HAL_MAX_DELAY);
}

static void mpu_write_reg(uint8_t reg_addr, uint8_t write_val){
    HAL_I2C_Mem_Write(my_i2c, MPU6050_ADDR, reg_addr, 1, &write_val, 1, HAL_MAX_DELAY);
}

bool mpu_who_am_i(void){
    uint8_t result;
    mpu_read_reg(MPU6050_WHO_AM_I, &result, sizeof(result));
    if(result == 0x68){
        return true;
    }
    return false;
}

void mpu_init(I2C_HandleTypeDef *hi2c){
    my_i2c = hi2c;
    mpu_dlpf_t dlpf = DLPF_BW_42HZ;
    mpu_gyro_t gyro = GYRO_RANGE_500DPS;
    mpu_accel_t accel = ACCEL_RANGE_4G;
    mpu_write_reg(MPU6050_POWER_MANAGMENT1, WAKE_UP);
    mpu_write_reg(MPU6050_CONFIG, dlpf);
    mpu_write_reg(MPU6050_GYRO_CONFIG, gyro);
    mpu_write_reg(MPU6050_ACCEL_CONFIG, accel);

    switch(gyro){
        case GYRO_RANGE_250DPS:
            gyro_divider = 131.0f;
            break;
        case GYRO_RANGE_500DPS:
            gyro_divider = 65.5f;
            break;
        case GYRO_RANGE_1000DPS:
            gyro_divider = 32.8f;
            break;
        case GYRO_RANGE_2000DPS:
            gyro_divider = 16.4f;
            break;
        default: break;
    }
    switch(accel){
        case ACCEL_RANGE_2G:
            accel_divider = 16384.0f;
            break;
        case ACCEL_RANGE_4G:
            accel_divider = 8192.0f;
            break;
        case ACCEL_RANGE_8G:
            accel_divider = 4096.0f;
            break;
        case ACCEL_RANGE_16G:
            accel_divider = 2048.0f;
            break;
        default: break;
    }
}

static void mpu_read_accel_raw(int16_t *accel_x_raw_ptr, int16_t *accel_y_raw_ptr, int16_t *accel_z_raw_ptr){
    uint8_t raw_read[6] = {0};
    mpu_read_reg(MPU6050_ACCEL_X_OUT_H, raw_read, sizeof(raw_read));
    *accel_x_raw_ptr = ((int16_t)raw_read[0] << 8) | raw_read[1];
    *accel_y_raw_ptr = ((int16_t)raw_read[2] << 8) | raw_read[3];
    *accel_z_raw_ptr = ((int16_t)raw_read[4] << 8) | raw_read[5]; 
}

static void mpu_read_gyro_raw(int16_t *gyro_x_raw_ptr, int16_t *gyro_y_raw_ptr, int16_t *gyro_z_raw_ptr){
    uint8_t raw_read[6] = {0};
    mpu_read_reg(MPU6050_GYRO_X_OUT_H, raw_read, sizeof(raw_read));
    *gyro_x_raw_ptr = ((int16_t)raw_read[0] << 8) | raw_read[1];
    *gyro_y_raw_ptr = ((int16_t)raw_read[2] << 8) | raw_read[3];
    *gyro_z_raw_ptr = ((int16_t)raw_read[4] << 8) | raw_read[5];
}

void mpu_get_accel(float *accel_x_ptr, float *accel_y_ptr, float *accel_z_ptr){
    int16_t accel_x_raw = 0;
    int16_t accel_y_raw = 0;
    int16_t accel_z_raw = 0;
    mpu_read_accel_raw(&accel_x_raw, &accel_y_raw, &accel_z_raw);

    *accel_x_ptr = (accel_x_raw / accel_divider) - accel_x_offset;
    *accel_y_ptr = (accel_y_raw / accel_divider) - accel_y_offset;
    *accel_z_ptr = (accel_z_raw / accel_divider) - accel_z_offset;
}

void mpu_get_gyro(float *gyro_x_ptr, float *gyro_y_ptr, float *gyro_z_ptr){
    int16_t gyro_x_raw = 0;
    int16_t gyro_y_raw = 0;
    int16_t gyro_z_raw = 0;
    mpu_read_gyro_raw(&gyro_x_raw, &gyro_y_raw, &gyro_z_raw);

    *gyro_x_ptr = gyro_x_raw / gyro_divider - gyro_x_offset;
    *gyro_y_ptr = gyro_y_raw / gyro_divider - gyro_y_offset;
    *gyro_z_ptr = gyro_z_raw / gyro_divider - gyro_z_offset;
}


void mpu_count_gyro_offset(void){
    gyro_x_offset = 0;
    gyro_y_offset = 0;
    gyro_z_offset = 0;

    float gx, gy, gz;
    float sum_x = 0.0f;
    float sum_y = 0.0f;
    float sum_z = 0.0f;
    uint16_t num_samples = 500;

    for(uint16_t i = 0; i<num_samples; i++){
        mpu_get_gyro(&gx, &gy, &gz);
        sum_x += gx;
        sum_y += gy;
        sum_z += gz;
        myDelay(8);

    }
    gyro_x_offset = sum_x / num_samples;
    gyro_y_offset = sum_y / num_samples;
    gyro_z_offset = sum_z / num_samples;
}

void mpu_get_gyro_offset(float *gx_ptr, float *gy_ptr, float *gz_ptr){
    *gx_ptr = gyro_x_offset;
    *gy_ptr = gyro_y_offset;
    *gz_ptr = gyro_z_offset;
}

void mpu_count_accel_offset(){
    accel_x_offset = 0;
    accel_y_offset = 0;
    accel_z_offset = 0;

    float ax, ay, az;
    float sum_x = 0.0f;
    float sum_y = 0.0f;
    float sum_z = 0.0f;
    uint16_t num_samples = 500;
    for(uint16_t i = 0; i < num_samples; i++){
        mpu_get_accel(&ax, &ay, &az);
        sum_x += ax;
        sum_y += ay;
        sum_z += az;
        myDelay(8);

    }
    accel_x_offset = sum_x / num_samples;
    accel_y_offset = sum_y / num_samples;
    accel_z_offset = (sum_z / num_samples) - 1.0f;
}

void mpu_get_accel_offset(float *ax_ptr, float *ay_ptr, float *az_ptr){
    *ax_ptr = accel_x_offset;
    *ay_ptr = accel_y_offset;
    *az_ptr = accel_z_offset;
}

float mpu_count_angle(float ax, float az, float gy, float dt_s){

    float accel_angle_deg = atan2(ax, az) * (180.0f / 3.141592f);

    filtered_angle_deg = 0.88f * (filtered_angle_deg + (gy * dt_s)) + 0.12f * accel_angle_deg;

    return filtered_angle_deg;
}

