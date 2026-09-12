#include "MPU6050.h"
#include "main.h"
#include <math.h>

extern I2C_HandleTypeDef hi2c1;

#define RAD_TO_DEG          57.2957795f
#define GYRO_DEADBAND_DPS   1.0f

#define ACC_SENS_4G         8192.0f
#define GYRO_SENS_500       65.5f

#define I2C_TIMEOUT_MS      10

static int16_t accel_x = 0;
static int16_t accel_y = 0;
static int16_t accel_z = 0;

static int16_t gyro_x  = 0;
static int16_t gyro_y  = 0;
static int16_t gyro_z  = 0;

static float gyro_y_bias_dps = 0.0f;

volatile float acc_x_g_debug    = 0.0f;
volatile float acc_y_g_debug    = 0.0f;
volatile float acc_z_g_debug    = 0.0f;
volatile float gyro_y_dps_debug = 0.0f;
volatile float acc_angle_debug  = 0.0f;
volatile float angle_debug      = 0.0f;

static Kalman_t pitch_kf;

static MPU6050_Status i2c_write_reg(uint8_t reg, uint8_t value)
{
    if (HAL_I2C_Mem_Write(&hi2c1, DEVICE_ADDRESS << 1, reg, 1,
                           &value, 1, I2C_TIMEOUT_MS) != HAL_OK)
    {
        return MPU6050_ERROR_I2C;
    }
    return MPU6050_OK;
}

static MPU6050_Status i2c_read_regs(uint8_t reg, uint8_t *buf, uint16_t len)
{
    if (HAL_I2C_Mem_Read(&hi2c1, DEVICE_ADDRESS << 1, reg, 1,
                          buf, len, I2C_TIMEOUT_MS) != HAL_OK)
    {
        return MPU6050_ERROR_I2C;
    }
    return MPU6050_OK;
}

static void Kalman_Init(Kalman_t *kf)
{
    kf->Q_angle   = 0.001f;
    kf->Q_bias    = 0.003f;
    kf->R_measure = 0.03f;

    kf->angle = 0.0f;
    kf->bias  = 0.0f;
    kf->rate  = 0.0f;

    kf->P[0][0] = 0.0f;
    kf->P[0][1] = 0.0f;
    kf->P[1][0] = 0.0f;
    kf->P[1][1] = 0.0f;
}

static float Kalman_GetAngle(Kalman_t *kf, float newAngle, float newRate, float dt)
{

    kf->rate   = newRate - kf->bias;
    kf->angle += dt * kf->rate;

    kf->P[0][0] += dt * (dt * kf->P[1][1] - kf->P[0][1] - kf->P[1][0] + kf->Q_angle);
    kf->P[0][1] -= dt * kf->P[1][1];
    kf->P[1][0] -= dt * kf->P[1][1];
    kf->P[1][1] += kf->Q_bias * dt;

    float S = kf->P[0][0] + kf->R_measure;
    float K0 = kf->P[0][0] / S;
    float K1 = kf->P[1][0] / S;

    float y = newAngle - kf->angle;

    kf->angle += K0 * y;
    kf->bias  += K1 * y;

    float P00_temp = kf->P[0][0];
    float P01_temp = kf->P[0][1];

    kf->P[0][0] -= K0 * P00_temp;
    kf->P[0][1] -= K0 * P01_temp;
    kf->P[1][0] -= K1 * P00_temp;
    kf->P[1][1] -= K1 * P01_temp;

    return kf->angle;
}


MPU6050_Status mpu6050_init(void)
{
    MPU6050_Status status;

    status = i2c_write_reg(REG_PWR_MGMT_1, 0x00);
    if (status != MPU6050_OK) return status;
    HAL_Delay(100);

    status = i2c_write_reg(REG_CONFIG, DLPF_184HZ_2MS);
    if (status != MPU6050_OK) return status;

    status = i2c_write_reg(REG_GYRO_CONFIG, GYRO_500);
    if (status != MPU6050_OK) return status;

    status = i2c_write_reg(REG_ACCEL_CONFIG, ACC_4G);
    if (status != MPU6050_OK) return status;

    HAL_Delay(10);


    uint8_t accel_config_read = 0;
    uint8_t gyro_config_read  = 0;

    status = i2c_read_regs(REG_ACCEL_CONFIG, &accel_config_read, 1);
    if (status != MPU6050_OK) return status;

    status = i2c_read_regs(REG_GYRO_CONFIG, &gyro_config_read, 1);
    if (status != MPU6050_OK) return status;

    if (accel_config_read != ACC_4G || gyro_config_read != GYRO_500)
    {
        return MPU6050_ERROR_I2C;
    }

    Kalman_Init(&pitch_kf);
    gyro_y_bias_dps = 0.0f;

    return MPU6050_OK;
}


MPU6050_Status mpu6050_calibrate_gyro(uint16_t num_samples)
{
    if (num_samples == 0) num_samples = 1;

    float sum = 0.0f;

    for (uint16_t i = 0; i < num_samples; i++)
    {
        MPU6050_Status status = mpu6050_read();
        if (status != MPU6050_OK) return status;

        sum += gyro_y / GYRO_SENS_500;
        HAL_Delay(2);
    }

    gyro_y_bias_dps = sum / (float)num_samples;
    return MPU6050_OK;
}


MPU6050_Status mpu6050_read(void)
{
    uint8_t buf[14];

    MPU6050_Status status = i2c_read_regs(REG_ACCEL_XOUT_H, buf, 14);
    if (status != MPU6050_OK)
    {
        return status;
    }

    accel_x = (int16_t)(buf[0]  << 8 | buf[1]);
    accel_y = (int16_t)(buf[2]  << 8 | buf[3]);
    accel_z = (int16_t)(buf[4]  << 8 | buf[5]);

    gyro_x  = (int16_t)(buf[8]  << 8 | buf[9]);
    gyro_y  = (int16_t)(buf[10] << 8 | buf[11]);
    gyro_z  = (int16_t)(buf[12] << 8 | buf[13]);

    return MPU6050_OK;
}

float mpu6050_get_angle(float dt)
{
    if (dt <= 0.0f) dt = 0.002f;

    acc_x_g_debug = accel_x / ACC_SENS_4G;
    acc_y_g_debug = accel_y / ACC_SENS_4G;
    acc_z_g_debug = accel_z / ACC_SENS_4G;

    gyro_y_dps_debug = (gyro_y / GYRO_SENS_500) - gyro_y_bias_dps;

    float gyro_rate = gyro_y_dps_debug;
    if (fabsf(gyro_rate) < GYRO_DEADBAND_DPS)
    {
        gyro_rate = 0.0f;
    }

    acc_angle_debug = atan2f(acc_x_g_debug, acc_z_g_debug) * RAD_TO_DEG;

    angle_debug = Kalman_GetAngle(&pitch_kf, acc_angle_debug, gyro_rate, dt);

    return angle_debug;
}


void mpu6050_reset_filter(void)
{
    Kalman_Init(&pitch_kf);
}
