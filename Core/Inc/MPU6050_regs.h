/*
 * MPU6050_regs.h
 *
 *  Created on: Feb 28, 2026
 *      Author: ozker
 */

#ifndef INC_MPU6050_REGS_H_
#define INC_MPU6050_REGS_H_

typedef enum {
    DLPF_BW_256HZ = 0x00,
    DLPF_BW_188HZ = 0x01,
    DLPF_BW_98HZ  = 0x02,
    DLPF_BW_42HZ  = 0x03,
    DLPF_BW_20HZ  = 0x04,
    DLPF_BW_10HZ  = 0x05,
    DLPF_BW_5HZ   = 0x06
}mpu_dlpf_t;

/*250DPS oznacza zakres od -250 do 250 stopni na sekunde, czyli zakres wynosi 500 stopni na sekunde.
Analogicznie dla innych zakresów*/
typedef enum {
    GYRO_RANGE_250DPS  = 0x00,
    GYRO_RANGE_500DPS  = 0x08,
    GYRO_RANGE_1000DPS = 0x10,
    GYRO_RANGE_2000DPS = 0x18
}mpu_gyro_t;

/*2G oznacza, ze zakres akcelerometru wynosi od -2g do 2g czyli tak na prawde 4g.
Analogicznie dla innych zakresow*/
typedef enum {
    ACCEL_RANGE_2G  = 0x00,
    ACCEL_RANGE_4G  = 0x08,
    ACCEL_RANGE_8G  = 0x10,
    ACCEL_RANGE_16G = 0x18
}mpu_accel_t;

#define MPU6050_ADDR 0xD0
#define MPU6050_WHO_AM_I 0x75
#define MPU6050_CONFIG 0x1A
#define MPU6050_POWER_MANAGMENT1 0x6B
#define MPU6050_GYRO_CONFIG 0x1B
#define MPU6050_ACCEL_CONFIG 0x1C
#define MPU6050_GYRO_X_OUT_H 0x43
#define MPU6050_GYRO_Y_OUT_H 0x45
#define MPU6050_GYRO_Z_OUT_H 0x47
#define MPU6050_ACCEL_X_OUT_H 0x3B
#define MPU6050_ACCEL_Y_OUT_H 0x3D
#define MPU6050_ACCEL_Z_OUT_H 0x3F

#define WAKE_UP 0x00





#endif /* INC_MPU6050_REGS_H_ */
