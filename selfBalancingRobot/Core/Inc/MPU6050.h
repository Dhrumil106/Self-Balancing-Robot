#ifndef INC_MPU6050_H_
#define INC_MPU6050_H_

#include <stdint.h>

#define DEVICE_ADDRESS  0x68

#define PWR_MGMT_1      0x6B
#define CONFIG_GYRO     0x1B
#define CONFIG_ACC      0x1C

#define GYRO_250        0x00
#define GYRO_500        0x08
#define GYRO_1000       0x10
#define GYRO_2000       0x18

#define ACC_2G          0x00
#define ACC_4G          0x08
#define ACC_8G          0x10
#define ACC_16G         0x18

extern int16_t accel_x;
extern int16_t accel_y;
extern int16_t accel_z;
extern int16_t gyro_x;
extern int16_t gyro_y;
extern int16_t gyro_z;

extern float acc_x_g_debug;
extern float acc_y_g_debug;
extern float acc_z_g_debug;
extern float gyro_y_dps_debug;
extern float acc_angle_debug;
extern float angle_debug;

void  mpu6050_init(void);
void  mpu6050_read(void);
float mpu6050_get_angle(float dt);

#endif
