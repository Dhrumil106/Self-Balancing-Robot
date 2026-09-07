#ifndef MOTOR_H
#define MOTOR_H

#include "main.h"   // Pulls in STM32 HAL and GPIO pin definitions
#include <stdint.h>

/**
 * @brief Initializes the PWM timers for both motors and ensures they start stopped.
 */
void Motor_Init(void);

/**
 * @brief Sets the speed for both motors equally.
 *
 * @param speed Range from -1000 (full reverse) to +1000 (full forward).
 */
void Motor_Set(int16_t speed, int16_t turn_speed);

/**
 * @brief Actively brakes both motors by driving both inputs HIGH.
 */
void Motor_Stop(void);
int16_t Encoder_GetDelta();
void Motor_Enable();
void Motor_Disable();
#endif /* MOTOR_H */
