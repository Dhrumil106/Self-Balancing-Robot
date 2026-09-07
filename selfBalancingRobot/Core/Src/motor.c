#include "motor.h"
#include <stdio.h>

extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern UART_HandleTypeDef huart1;

#define MAX_SPEED   1000
#define MIN_SPEED   20
#define MOTOR_A_TRIM 1.0f
#define MOTOR_B_TRIM 1.0f

// --- Motor A
#define MOTOR_A_IN1_PORT    GPIOA
#define MOTOR_A_IN1_PIN     GPIO_PIN_4
#define MOTOR_A_IN2_PORT    GPIOA
#define MOTOR_A_IN2_PIN     GPIO_PIN_5
#define MOTOR_A_CHANNEL     TIM_CHANNEL_1

// --- Motor B (Right) ---
#define MOTOR_B_IN1_PORT    GPIOA
#define MOTOR_B_IN1_PIN     GPIO_PIN_2
#define MOTOR_B_IN2_PORT    GPIOA
#define MOTOR_B_IN2_PIN     GPIO_PIN_3
#define MOTOR_B_CHANNEL     TIM_CHANNEL_2

// --- Driver Standby ---
#define MOTOR_STBY_PORT     GPIOB
#define MOTOR_STBY_PIN      GPIO_PIN_0

volatile int16_t speed_a_debug = 0;
volatile int16_t speed_b_debug = 0;


static void set_motor_a(int16_t speed)
{
    if (speed > 0)
    {
        HAL_GPIO_WritePin(MOTOR_A_IN1_PORT, MOTOR_A_IN1_PIN, GPIO_PIN_SET);
        HAL_GPIO_WritePin(MOTOR_A_IN2_PORT, MOTOR_A_IN2_PIN, GPIO_PIN_RESET);
        __HAL_TIM_SET_COMPARE(&htim2, MOTOR_A_CHANNEL, (uint16_t)speed);
    }
    else if (speed < 0)
    {
        HAL_GPIO_WritePin(MOTOR_A_IN1_PORT, MOTOR_A_IN1_PIN, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(MOTOR_A_IN2_PORT, MOTOR_A_IN2_PIN, GPIO_PIN_SET);
        __HAL_TIM_SET_COMPARE(&htim2, MOTOR_A_CHANNEL, (uint16_t)(-speed));
    }
    else
    {
        HAL_GPIO_WritePin(MOTOR_A_IN1_PORT, MOTOR_A_IN1_PIN, GPIO_PIN_SET);
        HAL_GPIO_WritePin(MOTOR_A_IN2_PORT, MOTOR_A_IN2_PIN, GPIO_PIN_SET);
        __HAL_TIM_SET_COMPARE(&htim2, MOTOR_A_CHANNEL, 0);
    }
}

static void set_motor_b(int16_t speed)
{
    if (speed > 0)
    {
        HAL_GPIO_WritePin(MOTOR_B_IN1_PORT, MOTOR_B_IN1_PIN, GPIO_PIN_SET);
        HAL_GPIO_WritePin(MOTOR_B_IN2_PORT, MOTOR_B_IN2_PIN, GPIO_PIN_RESET);
        __HAL_TIM_SET_COMPARE(&htim2, MOTOR_B_CHANNEL, (uint16_t)speed);
    }
    else if (speed < 0)
    {
        HAL_GPIO_WritePin(MOTOR_B_IN1_PORT, MOTOR_B_IN1_PIN, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(MOTOR_B_IN2_PORT, MOTOR_B_IN2_PIN, GPIO_PIN_SET);
        __HAL_TIM_SET_COMPARE(&htim2, MOTOR_B_CHANNEL, (uint16_t)(-speed));
    }
    else
    {
        HAL_GPIO_WritePin(MOTOR_B_IN1_PORT, MOTOR_B_IN1_PIN, GPIO_PIN_SET);
        HAL_GPIO_WritePin(MOTOR_B_IN2_PORT, MOTOR_B_IN2_PIN, GPIO_PIN_SET);
        __HAL_TIM_SET_COMPARE(&htim2, MOTOR_B_CHANNEL, 0);
    }
}


void Motor_Init(void)
{
    // Start PWM timers
    HAL_TIM_PWM_Start(&htim2, MOTOR_A_CHANNEL);
    HAL_TIM_PWM_Start(&htim2, MOTOR_B_CHANNEL);

    // Start Encoder timer
    HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
    HAL_GPIO_WritePin(MOTOR_STBY_PORT, MOTOR_STBY_PIN, GPIO_PIN_SET);
    Motor_Stop();
    Motor_Enable();
}

void Motor_Enable(void)
{
    HAL_GPIO_WritePin(MOTOR_STBY_PORT, MOTOR_STBY_PIN, GPIO_PIN_SET);
}

void Motor_Disable(void)
{
    HAL_GPIO_WritePin(MOTOR_STBY_PORT, MOTOR_STBY_PIN, GPIO_PIN_RESET);
    Motor_Stop();
}

void Motor_Stop(void)
{
    set_motor_a(0);
    set_motor_b(0);
}
void Motor_Set(int16_t balance_speed, int16_t turn_speed)
{
    int16_t speed_a = balance_speed + turn_speed;
    int16_t speed_b = balance_speed - turn_speed;


    if (speed_a > 0 && speed_a < MIN_SPEED) speed_a = MIN_SPEED;
    if (speed_a < 0 && speed_a > -MIN_SPEED) speed_a = -MIN_SPEED;


    speed_a = (int16_t)(speed_a * MOTOR_A_TRIM);
    speed_b = (int16_t)(speed_b * MOTOR_B_TRIM);


    if (speed_a >  MAX_SPEED) speed_a =  MAX_SPEED;
    if (speed_a < -MAX_SPEED) speed_a = -MAX_SPEED;
    if (speed_b >  MAX_SPEED) speed_b =  MAX_SPEED;
    if (speed_b < -MAX_SPEED) speed_b = -MAX_SPEED;

    speed_b_debug = speed_b;
    speed_a_debug = speed_a;

    set_motor_a(speed_a);
    set_motor_b(-speed_b);
}


int16_t Encoder_GetDelta(void)
{
    int16_t current_count = (int16_t)__HAL_TIM_GET_COUNTER(&htim3);
    static int16_t last_count = 0;

    int16_t delta = current_count - last_count;
    last_count = current_count;


    return current_count;
}
