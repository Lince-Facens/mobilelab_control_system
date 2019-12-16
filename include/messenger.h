#include <stm32f10x.h>
#include "peripherals.h"

#define MESSAGE_DEBUG 0 // Whether it'll print out the messages

#define MESSAGE_OUTPUT_CHANNEL USART1 // The channel where the messages will be sent out

#define MESSAGE_STEERING_MAX 4095
#define MESSAGE_STEERING_THRESHOLD 0.1

#define MESSAGE_ACCELERATION_MAX 4095
#define MESSAGE_ACCELERATION_THRESHOLD 0.1

/**
 * Sends the actuators message formated as "s0000a0000"
 */
void sendActuatorsMessage(uint16_t steeringRight, uint16_t steeringLeft, uint16_t acceleration, uint8_t accelerationReversed);

