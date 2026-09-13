#include "MPU6050.h"
#include "main.h"
#include <math.h>

extern I2C_HandleTypeDef hi2c1;

#define RAD_TO_DEG      57.2957795f

#define Q_ANGLE         0.001f
#define Q_BIAS          0.003f
#define R_MEASURE       0.03f

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

typedef struct {
    float angle;
    float bias;
    float P[2][2];
} KalmanState;

static KalmanState kalman = {
    .angle   = 0.0f,
    .bias    = 0.0f,
    .P       = {{0.0f, 0.0f},
                {0.0f, 0.0f}}
};

static void i2c_error_handler(const char *context)
{
    (void)context;
    while (1);
}

static float kalman_update(float acc_angle, float gyro_rate, float dt)
{

    kalman.angle += dt * (gyro_rate - kalman.bias);

    kalman.P[0][0] += dt * (dt * kalman.P[1][1]
                     - kalman.P[0][1]
                     - kalman.P[1][0]
                     + Q_ANGLE);
    kalman.P[0][1] -= dt * kalman.P[1][1];
    kalman.P[1][0] -= dt * kalman.P[1][1];
    kalman.P[1][1] += Q_BIAS * dt;

    float S = kalman.P[0][0] + R_MEASURE;
    float K[2];
    K[0] = kalman.P[0][0] / S;
    K[1] = kalman.P[1][0] / S;

    float y = acc_angle - kalman.angle;
    kalman.angle += K[0] * y;
    kalman.bias  += K[1] * y;

    float P00_temp = kalman.P[0][0];
    float P01_temp = kalman.P[0][1];
    kalman.P[0][0] -= K[0] * P00_temp;
    kalman.P[0][1] -= K[0] * P01_temp;
    kalman.P[1][0] -= K[1] * P00_temp;
    kalman.P[1][1] -= K[1] * P01_temp;

    return kalman.angle;
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

    uint8_t buf[6];
    HAL_I2C_Mem_Read(&hi2c1, DEVICE_ADDRESS << 1, 0x3B, 1, buf, 6, 100);
    float ax = (int16_t)(buf[0] << 8 | buf[1]) / 8192.0f;
    float az = (int16_t)(buf[4] << 8 | buf[5]) / 8192.0f;
    kalman.angle = atan2f(ax, az) * RAD_TO_DEG;
}

void mpu6050_read(void)
{
    uint8_t buf[14];

    if (HAL_I2C_Mem_Read(&hi2c1, DEVICE_ADDRESS << 1, 0x3B, 1,
                          buf, 14, 100) != HAL_OK)
        i2c_error_handler("burst read");

    accel_x = (int16_t)(buf[0] << 8 | buf[1]);
    accel_y = (int16_t)(buf[2] << 8 | buf[3]);
    accel_z = (int16_t)(buf[4] << 8 | buf[5]);

    gyro_x  = (int16_t)(buf[8]  << 8 | buf[9]);
    gyro_y  = (int16_t)(buf[10] << 8 | buf[11]);
    gyro_z  = (int16_t)(buf[12] << 8 | buf[13]);
}

float mpu6050_get_angle(float dt)
{
    acc_x_g_debug    = accel_x / 8192.0f;
    acc_y_g_debug    = accel_y / 8192.0f;
    acc_z_g_debug    = accel_z / 8192.0f;
    gyro_y_dps_debug = gyro_y  / 65.5f;

    acc_angle_debug  = atan2f(acc_x_g_debug, acc_z_g_debug) * RAD_TO_DEG;

    float angle = kalman_update(acc_angle_debug, gyro_y_dps_debug, dt);
    angle_debug = angle;

    return angle;
}
