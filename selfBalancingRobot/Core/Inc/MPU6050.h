#ifndef INC_MPU6050_H_
#define INC_MPU6050_H_

#include <stdint.h>

#define DEVICE_ADDRESS  0x68

// MPU6050 registers
#define PWR_MGMT_1      0x6B
#define CONFIG_GYRO     0x1B
#define CONFIG_ACC      0x1C
#define DATA_ACC        0x3B
#define DATA_GYRO       0x43

// Gyro full-scale settings
#define GYRO_250        0x00
#define GYRO_500        0x08
#define GYRO_1000       0x10
#define GYRO_2000       0x18

// Accelerometer full-scale settings
#define ACC_2G          0x00
#define ACC_4G          0x08
#define ACC_8G          0x10
#define ACC_16G         0x18

// Extern raw data
extern int16_t accel_x;
extern int16_t accel_y;
extern int16_t accel_z;

extern int16_t gyro_x;
extern int16_t gyro_y;
extern int16_t gyro_z;

// Public API
void  mpu6050_init(void);
void  mpu6050_calibrate(void);
void  mpu6050_read(void);
float mpu6050_get_angle(float dt);

#endif /* INC_MPU6050_H_ */
