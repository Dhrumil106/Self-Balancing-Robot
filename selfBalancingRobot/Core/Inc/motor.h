#ifndef MOTOR_H
#define MOTOR_H

#include "main.h"
#include <stdint.h>

void Motor_Init(void);
void Motor_Enable(void);
void Motor_Disable(void);
void Motor_Stop(void);


void Motor_Set(int16_t balance_speed, int16_t turn_speed);

int16_t Encoder1_GetDelta(void);
int16_t Encoder2_GetDelta(void);

#endif /* MOTOR_H */

