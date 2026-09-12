#ifndef MPU6050_H
#define MPU6050_H

#include "main.h"
#include <stdint.h>

#define DEVICE_ADDRESS      0x68

#define REG_SMPLRT_DIV       0x19
#define REG_CONFIG           0x1A
#define REG_GYRO_CONFIG      0x1B
#define REG_ACCEL_CONFIG     0x1C
#define REG_ACCEL_XOUT_H     0x3B
#define REG_PWR_MGMT_1       0x6B

#define PWR_MGMT_1            REG_PWR_MGMT_1
#define CONFIG_GYRO           REG_GYRO_CONFIG
#define CONFIG_ACC            REG_ACCEL_CONFIG

#define GYRO_250              0x00
#define GYRO_500              0x08
#define GYRO_1000             0x10
#define GYRO_2000             0x18

#define ACC_2G                0x00
#define ACC_4G                0x08
#define ACC_8G                0x10
#define ACC_16G               0x18


#define DLPF_184HZ_2MS              0x01



typedef enum
{
    MPU6050_OK = 0,
    MPU6050_ERROR_I2C,
} MPU6050_Status;


typedef struct
{
    float Q_angle;
    float Q_bias;
    float R_measure;

    float angle;
    float bias;
    float rate;

    float P[2][2];
} Kalman_t;


MPU6050_Status mpu6050_read(void);

float mpu6050_get_angle(float dt);

void mpu6050_reset_filter(void);

#endif /* MPU6050_H */
