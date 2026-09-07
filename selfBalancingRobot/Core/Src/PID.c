#include "PID.h"
#include <math.h>


volatile float motor_output_debug = 0.0f;


void PIDController_Init(PIDController *controller)
{
    controller->integral    = 0.0f;
    controller->derivative  = 0.0f;
    controller->prev_error  = 0.0f;
    controller->motor_output = 0.0f;
}

void PIDController_Update(PIDController *controller, float setPoint, float angle)
{

    if (controller->sampling_time <= 0.0f)
    {
        controller->motor_output = 0.0f;
        motor_output_debug = 0.0f;
        return;
    }

    float error = setPoint - angle;

    // Proportional
    float proportional = controller->kp * error;
    controller->proportional = proportional;

    // Integral
    controller->integral += error * controller->sampling_time;

    float max_integral = (controller->ki != 0.0f) ? (300.0f / controller->ki) : 1e6f;

    if (controller->integral > max_integral) {
        controller->integral = max_integral;
    }
    else if (controller->integral < -max_integral) {
        controller->integral = -max_integral;
    }

    float integral = controller->integral * controller->ki;

    // Derivative
    controller->derivative = (error - controller->prev_error) / controller->sampling_time;
    float derivative = controller->derivative * controller->kd;

    controller->motor_output = proportional + integral + derivative;
    motor_output_debug = controller->motor_output;

    controller->prev_error = error;
}
