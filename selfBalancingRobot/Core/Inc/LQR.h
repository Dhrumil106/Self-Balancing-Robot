/*
 * LQR.h
 *
 *  Created on: Jul 27, 2026
 *      Author: dhrum
 */

#ifndef INC_LQR_H_
#define INC_LQR_H_

typedef struct
{
	// User-defined
	float dt; // based on PWM signals and timers configured
	float k1;
	float k2;
	float k3;
	float k4;

	float position;
	float velocity;
	float angle;
	float angular_speed;
	float motor_output;
} LQRController;

#endif /* INC_LQR_H_ */
