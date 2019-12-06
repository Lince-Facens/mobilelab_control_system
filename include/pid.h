/**
 * PID controller definition.
 *
 * Usage example:
 *
 * initPID();
 * while(1) {
 *     output = calcPIDOutput( measure, s_l, s_r );
 *     ...
 *     updatePIDTickCount();
 * }
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

/* Private functions */
uint16_t calcPIDError(uint16_t measure, uint16_t steering_left, uint16_t steering_right);

/* Public functions */
double calcPIDOutput(uint16_t measure, uint16_t steering_left, uint16_t steering_right);
void initPID();
void updatePIDTickCount();
