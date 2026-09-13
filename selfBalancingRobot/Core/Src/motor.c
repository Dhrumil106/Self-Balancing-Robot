#include "motor.h"
#include <stdio.h>

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern UART_HandleTypeDef huart1;

#define MAX_SPEED   1000
#define MIN_SPEED   20
#define MOTOR1_TRIM 1.0f
#define MOTOR2_TRIM 1.0f


#define MOTOR1_IN1_CHANNEL TIM_CHANNEL_3
#define MOTOR1_IN2_CHANNEL TIM_CHANNEL_4

#define MOTOR2_IN1_CHANNEL TIM_CHANNEL_1
#define MOTOR2_IN2_CHANNEL TIM_CHANNEL_2

volatile int16_t speed_1_debug = 0;
volatile int16_t speed_2_debug = 0;


static void set_motor1(int16_t speed)
{
    if (speed > 0)
    {
        __HAL_TIM_SET_COMPARE(&htim2, MOTOR1_IN1_CHANNEL, (uint16_t)speed);
        __HAL_TIM_SET_COMPARE(&htim2, MOTOR1_IN2_CHANNEL, 0);
    }
    else if (speed < 0)
    {
        __HAL_TIM_SET_COMPARE(&htim2, MOTOR1_IN1_CHANNEL, 0);
        __HAL_TIM_SET_COMPARE(&htim2, MOTOR1_IN2_CHANNEL, (uint16_t)(-speed));
    }
    else
    {
        // COAST: both inputs low
        __HAL_TIM_SET_COMPARE(&htim2, MOTOR1_IN1_CHANNEL, 0);
        __HAL_TIM_SET_COMPARE(&htim2, MOTOR1_IN2_CHANNEL, 0);
    }
}

static void set_motor2(int16_t speed)
{
    if (speed > 0)
    {
        __HAL_TIM_SET_COMPARE(&htim2, MOTOR2_IN1_CHANNEL, (uint16_t)speed);
        __HAL_TIM_SET_COMPARE(&htim2, MOTOR2_IN2_CHANNEL, 0);
    }
    else if (speed < 0)
    {
        __HAL_TIM_SET_COMPARE(&htim2, MOTOR2_IN1_CHANNEL, 0);
        __HAL_TIM_SET_COMPARE(&htim2, MOTOR2_IN2_CHANNEL, (uint16_t)(-speed));
    }
    else
    {

        __HAL_TIM_SET_COMPARE(&htim2, MOTOR2_IN1_CHANNEL, 0);
        __HAL_TIM_SET_COMPARE(&htim2, MOTOR2_IN2_CHANNEL, 0);
    }
}


void Motor_Init(void)
{

    HAL_TIM_PWM_Start(&htim2, MOTOR1_IN1_CHANNEL);
    HAL_TIM_PWM_Start(&htim2, MOTOR1_IN2_CHANNEL);
    HAL_TIM_PWM_Start(&htim2, MOTOR2_IN1_CHANNEL);
    HAL_TIM_PWM_Start(&htim2, MOTOR2_IN2_CHANNEL);

    // Start both encoder timers
    HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);

    Motor_Stop();
    Motor_Enable();
}

void Motor_Enable(void)
{
    HAL_TIM_PWM_Start(&htim2, MOTOR1_IN1_CHANNEL);
    HAL_TIM_PWM_Start(&htim2, MOTOR1_IN2_CHANNEL);
    HAL_TIM_PWM_Start(&htim2, MOTOR2_IN1_CHANNEL);
    HAL_TIM_PWM_Start(&htim2, MOTOR2_IN2_CHANNEL);
}

void Motor_Disable(void)
{
    Motor_Stop();
    HAL_TIM_PWM_Stop(&htim2, MOTOR1_IN1_CHANNEL);
    HAL_TIM_PWM_Stop(&htim2, MOTOR1_IN2_CHANNEL);
    HAL_TIM_PWM_Stop(&htim2, MOTOR2_IN1_CHANNEL);
    HAL_TIM_PWM_Stop(&htim2, MOTOR2_IN2_CHANNEL);
}

void Motor_Stop(void)
{
    set_motor1(0);
    set_motor2(0);
}

void Motor_Set(int16_t balance_speed, int16_t turn_speed)
{
    int16_t speed_2 = balance_speed + turn_speed;
    int16_t speed_1 = balance_speed - turn_speed;


    if (speed_2 > 0) speed_2 += MIN_SPEED;
    if (speed_2 < 0) speed_2 -= MIN_SPEED;

    if (speed_1 > 0) speed_1 += MIN_SPEED;
    if (speed_1 < 0) speed_1 -= MIN_SPEED;

    speed_2 = (int16_t)(speed_2 * MOTOR2_TRIM);
    speed_1 = (int16_t)(speed_1 * MOTOR1_TRIM);

    if (speed_2 >  MAX_SPEED) speed_2 =  MAX_SPEED;
    if (speed_2 < -MAX_SPEED) speed_2 = -MAX_SPEED;
    if (speed_1 >  MAX_SPEED) speed_1 =  MAX_SPEED;
    if (speed_1 < -MAX_SPEED) speed_1 = -MAX_SPEED;

    speed_2_debug = speed_2;
    speed_1_debug = speed_1;

    set_motor2(speed_2);
    set_motor1(speed_1);
}

int16_t Encoder1_GetDelta(void)
{
    int16_t current_count = (int16_t)__HAL_TIM_GET_COUNTER(&htim1);
    static int16_t last_count = 0;

    int16_t delta = current_count - last_count;
    last_count = current_count;

    return current_count;
}

int16_t Encoder2_GetDelta(void)
{
    int16_t current_count = (int16_t)__HAL_TIM_GET_COUNTER(&htim3);
    static int16_t last_count = 0;

    int16_t delta = current_count - last_count;
    last_count = current_count;

    return current_count;
}
