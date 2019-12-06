/**
 * PID controller implementation.
*/

#include "pid.h"

/*
 * Calculates the error (sepoint - measure) for PID controller
 */
uint16_t calcPIDError(uint16_t measure, uint16_t steering_left, uint16_t steering_right)
{
	uint16_t setpoint = 2048;
	if (steering_right > 400) {
		setpoint += (steering_right >> 1);
	}
	else if (steering_left > 400) {
		setpoint -= (steering_left >> 1);
	}

	return -measure + setpoint;
}

/*
 * Main PID controller function, calculates the error and gives the output
 */
double calcPIDOutput(uint16_t measure, uint16_t steering_left, uint16_t steering_right)
{
	double error = calcPIDError(measure, steering_left, steering_right);
	int elapsed_time = xTaskGetTickCount() - last_tick_count;

	cumulative_error += error * elapsed_time;
	double rate_error = (error - last_output) / (double)(elapsed_time);

	double output = KP * error + KI * cumulative_error + KD * rate_error;

	last_output = output;

	return output;
}

/*
 * Initialize variables for PID controller
 */
void initPID()
{
	last_output = last_tick_count = -1;
	cumulative_error = 0;
}

/*
 * Update private tick counter
 */
void updatePIDTickCount()
{
	last_tick_count = xTaskGetTickCount();
}
