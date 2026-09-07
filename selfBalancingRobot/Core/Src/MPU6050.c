#include "MPU6050.h"
#include "main.h"
#include <math.h>

extern I2C_HandleTypeDef hi2c1;


#define RAD_TO_DEG      57.2957795f
#define COMP_ALPHA      0.98f
#define GYRO_DEADBAND   1.0f
#define CALIB_SAMPLES   500

int16_t accel_x = 0;
int16_t accel_y = 0;
int16_t accel_z = 0;

int16_t gyro_x  = 0;
int16_t gyro_y  = 0;
int16_t gyro_z  = 0;

uint8_t accel_config_read = 0;
uint8_t gyro_config_read  = 0;


float acc_x_g_debug    = 0.0f;
float acc_y_g_debug    = 0.0f;
float acc_z_g_debug    = 0.0f;
float gyro_y_dps_debug = 0.0f;
float acc_angle_debug  = 0.0f;
float angle_debug      = 0.0f;


static int16_t ax_offset = 0;
static int16_t ay_offset = 0;
static int16_t az_offset = 0;
static int16_t gy_offset = 0;


static void i2c_error_handler(const char *context)
{
    (void)context;
    while (1);
}


void mpu6050_init(void)
{
    uint8_t temp_data;


    temp_data = 0x00;
    if (HAL_I2C_Mem_Write(&hi2c1, DEVICE_ADDRESS << 1, PWR_MGMT_1, 1,
                           &temp_data, 1, 100) != HAL_OK)
        i2c_error_handler("wake");

    HAL_Delay(100);

    temp_data = 0x03;
    if (HAL_I2C_Mem_Write(&hi2c1, DEVICE_ADDRESS << 1, 0x1A, 1,
                           &temp_data, 1, 100) != HAL_OK)
        i2c_error_handler("dlpf");

    temp_data = GYRO_500;
    if (HAL_I2C_Mem_Write(&hi2c1, DEVICE_ADDRESS << 1, CONFIG_GYRO, 1,
                           &temp_data, 1, 100) != HAL_OK)
        i2c_error_handler("gyro cfg");

    temp_data = ACC_4G;
    if (HAL_I2C_Mem_Write(&hi2c1, DEVICE_ADDRESS << 1, CONFIG_ACC, 1,
                           &temp_data, 1, 100) != HAL_OK)
        i2c_error_handler("accel cfg");

    HAL_Delay(100);

    if (HAL_I2C_Mem_Read(&hi2c1, DEVICE_ADDRESS << 1, CONFIG_ACC, 1,
                          &accel_config_read, 1, 100) != HAL_OK)
        i2c_error_handler("accel readback");

    if (HAL_I2C_Mem_Read(&hi2c1, DEVICE_ADDRESS << 1, CONFIG_GYRO, 1,
                          &gyro_config_read, 1, 100) != HAL_OK)
        i2c_error_handler("gyro readback");
}


void mpu6050_read(void)
{
    uint8_t buf[14];

    if (HAL_I2C_Mem_Read(&hi2c1, DEVICE_ADDRESS << 1, 0x3B, 1, buf, 14, 10) != HAL_OK)
    {
        __HAL_RCC_I2C1_FORCE_RESET();
        HAL_Delay(1);
        __HAL_RCC_I2C1_RELEASE_RESET();

        HAL_I2C_Init(&hi2c1);

        uint8_t temp_data = 0x00;
        HAL_I2C_Mem_Write(&hi2c1, DEVICE_ADDRESS << 1, PWR_MGMT_1, 1, &temp_data, 1, 10);

        return;
    }

    accel_x = (int16_t)(buf[0] << 8 | buf[1]) - ax_offset;
    accel_y = (int16_t)(buf[2] << 8 | buf[3]) - ay_offset;
    accel_z = (int16_t)(buf[4] << 8 | buf[5]) - az_offset;

    gyro_x  = (int16_t)(buf[8]  << 8 | buf[9]);
    gyro_y  = (int16_t)(buf[10] << 8 | buf[11]);
    gyro_z  = (int16_t)(buf[12] << 8 | buf[13]);
}


float mpu6050_get_angle(float dt)
{
    static float angle = 0.0f;

    acc_x_g_debug    = accel_x / 8192.0f;
    acc_y_g_debug    = accel_y / 8192.0f;
    acc_z_g_debug    = accel_z / 8192.0f;

    gyro_y_dps_debug = (gyro_y - gy_offset) / 65.5f;

    float gyro_rate = gyro_y_dps_debug;
    if (fabsf(gyro_rate) < GYRO_DEADBAND)
        gyro_rate = 0.0f;

    acc_angle_debug = atan2f(acc_x_g_debug, acc_z_g_debug) * RAD_TO_DEG;

    angle = COMP_ALPHA * (angle + gyro_rate * dt)
          + (1.0f - COMP_ALPHA) * acc_angle_debug;

    angle_debug = angle;
    return angle;
}
