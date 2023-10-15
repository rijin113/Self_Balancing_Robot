/*
 * pid_controller.h
 *
 *  Created on: Oct 11, 2023
 *      Author: Rijin
 */

#ifndef INC_PID_H_
#define INC_PID_H_

typedef struct
{
	// User-defined
	float sampling_time; // based on PWM signals and timers configured
	float kp;
	float ki;
	float kd;

	float integral;
	float derivative;

	float prev_error;

//	float left_motor_output; // Left Motor (Will change) // PWM Duty Cycle
//	float right_motor_output; // Right Motor (will change) // PWM Duty Cycle
	float motor_output;

} PIDController;

void PIDController_Init(PIDController * controller);
void PIDController_Update(PIDController * controller, float setpoint, float imu_reading);

#endif /* INC_PID_H_ */
