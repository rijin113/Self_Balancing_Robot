/*
 * pid_controller.h
 *
 *  Created on: Oct 11, 2023
 *      Author: Rijin
 */

#ifndef INC_PID_CONTROLLER_H_
#define INC_PID_CONTROLLER_H_

typedef struct
{
	float kp;
	float ki;
	float kd;

	float error;

	float proportional;
	float integration;
	float differentiation;

	float output_1; // Left Motor (Will change)
	float output_2; // Right Motor (will change)

	float sampling_time; // based on PWM signals and timers configured

} PIDController;

void start_controlLoop(PIDController * controller);
void update_controlLoop(PIDController * controller, float setpoint, float mpu_reading);

#endif /* INC_PID_CONTROLLER_H_ */
