/*
 * PID.h
 *
 *  Created on: Jul 24, 2026
 *      Author: dhrum
 */

#ifndef INC_PID_H_
#define INC_PID_H_

typedef struct
{

	float sampling_time;
	float kp;
	float ki;
	float kd;

	float proportional;
	float integral;
	float derivative;
	float prev_error;
	float motor_output;
} PIDController;

void PIDController_Init(PIDController *controller);
void PIDController_Update(PIDController *controller, float setPoint, float angle);

#endif /* INC_PID_H_ */
