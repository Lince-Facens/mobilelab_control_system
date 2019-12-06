/**
 * PID controller definition.
*/

/* Includes */
#include "FreeRTOS.h"

/* Private defines */
#define KP 6
#define KI 0
#define KD 1

/* Private variables */
double cumulative_error, last_output;
int last_tick_count;

/* Public functions */
uint16_t calcPIDError(uint16_t measure, uint16_t steering_left, uint16_t steering_right);
double calcPIDOutput(uint16_t measure, uint16_t steering_left, uint16_t steering_right);
void initPID();
void updatePIDTickCount();
