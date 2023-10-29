/*
 * state_machine.c
 *
 *  Created on: Oct. 15, 2023
 *      Author: Rijin
 */

#include "state_machine.h"
#include "math.h"
#include "main.h"

/* IDLE state waits for a user button press.
   If button is pressed, robot goes into START state.
   If button is not pressed, it stays idle. */
Robot_State robot_idle_state(PIDController * controller, TIM_HandleTypeDef * timer)
{
	__HAL_TIM_SET_COMPARE(timer, TIM_CHANNEL_3, 0);
	__HAL_TIM_SET_COMPARE(timer, TIM_CHANNEL_4, 0);
	return ROBOT_IDLE;
}

/* START state determines which state the robot enters next
   based on the pitch angle. */
Robot_State robot_start_state(accel_data * accel)
{
	if((accel->pitch_angle) < 0)
	{
		return ROBOT_BACKWARD;
	}
	else if((accel->pitch_angle) > 0)
	{
		return ROBOT_FORWARD;
	}
	else
	{
		return ROBOT_START;
	}
}

/* ROBOT_FORWARD state means the robot is currently leaning forwards.
   To offset this change, it changes the motor direction so
   the robot goes backwards to get back to its upright position. */
Robot_State robot_forward_state(accel_data * accel, int * motor_toggle)
{
	if(accel->pitch_angle > 0 && *(motor_toggle) == 0)
	{
		// Change direction of Left Motor
		HAL_GPIO_TogglePin(GPIOA, PA9_D8_OUT_Pin);
		HAL_GPIO_TogglePin(PC7_D9_OUT_GPIO_Port, PC7_D9_OUT_Pin);

		// Change direction of Right Motor
		HAL_GPIO_TogglePin(GPIOA, PA8_D7_OUT_Pin);
		HAL_GPIO_TogglePin(PB10_D6_OUT_GPIO_Port, PB10_D6_OUT_Pin);

		*(motor_toggle) = 1;
	}

	/* Transition to ROBOT_BACKWARD state if pitch angle becomes negative
	   This means the robot is leaning backwards now. */
	if(accel->pitch_angle < 0)
	{
		return ROBOT_BACKWARD;
	}
	else
	{
		return ROBOT_FORWARD;
	}
}

/* ROBOT_BACKWARD state means the robot is currently leaning backwards.
   To offset this change, it changes the motor direction so
   the robot goes forwards to get back to its upright position.*/
Robot_State robot_backward_state(accel_data * accel, int * motor_toggle)
{
	if(accel->pitch_angle < 0 && *(motor_toggle) == 1)
	{
		// Change direction of Left Motor
		HAL_GPIO_TogglePin(GPIOA, PA9_D8_OUT_Pin);
		HAL_GPIO_TogglePin(PC7_D9_OUT_GPIO_Port, PC7_D9_OUT_Pin);

		// Change direction of Right Motor
		HAL_GPIO_TogglePin(GPIOA, PA8_D7_OUT_Pin);
		HAL_GPIO_TogglePin(PB10_D6_OUT_GPIO_Port, PB10_D6_OUT_Pin);

		*(motor_toggle) = 0;
	}

	/* Transition to ROBOT_FORWARD state if pitch angle becomes positive
	   This means the robot is leaning forwards now. */
	if(accel->pitch_angle > 0)
	{
		return ROBOT_FORWARD;
	}
	else
	{
		return ROBOT_BACKWARD;
	}
}

/* This state is to make the robot stop. The duty cycle of
   the signals become zero at this point and the robot returns
   to IDLE state. */
Robot_State robot_stopped_state(TIM_HandleTypeDef * timer)
{
	__HAL_TIM_SET_COMPARE(timer, TIM_CHANNEL_3, 0);
	__HAL_TIM_SET_COMPARE(timer, TIM_CHANNEL_4, 0);
	return ROBOT_IDLE;
}

/* Optional State Implementation */
Robot_State robot_balanced_state()
{
	return ROBOT_BALANCED;
}

