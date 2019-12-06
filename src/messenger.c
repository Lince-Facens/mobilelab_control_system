/**
 * Message Transmitter
 * Sends ASCII messages through USART peripheral ports
 */

#include <messenger.h>
#include <stm32f10x.h>

#define MESSAGE_STEERING_CENTER (MESSAGE_STEERING_MAX / 2)
#define MESSAGE_STEERING_MIN (MESSAGE_STEERING_MAX * MESSAGE_STEERING_THRESHOLD)
#define MESSAGE_ACCELERATION_MIN (MESSAGE_ACCELERATION_MAX * MESSAGE_ACCELERATION_THRESHOLD)

#if MESSAGE_DEBUG

#include <stdio.h>
#define sendCharMessage putchar

#else

void sendCharMessage(char c)
{
	USART_SendData(MESSAGE_OUTPUT_CHANNEL, c);
	while (USART_GetFlagStatus(MESSAGE_OUTPUT_CHANNEL, USART_FLAG_TXE) == RESET);
}

#endif

void sendValueMessage(char prefix, uint16_t value)
{
	sendCharMessage(prefix);
	sendCharMessage('0' + (value / 1000));
	sendCharMessage('0' + (value / 100) % 10);
	sendCharMessage('0' + (value / 10) % 10);
	sendCharMessage('0' + value % 10);
}

void sendActuatorsMessage(uint16_t brake, uint16_t steeringRight, uint16_t steeringLeft, uint16_t acceleration)
{
	uint16_t steering = MESSAGE_STEERING_CENTER;

	if (steeringLeft > MESSAGE_STEERING_MIN) {
		steering -= steeringLeft / 2;
	} else if (steeringRight > MESSAGE_STEERING_MIN) {
		steering += steeringRight / 2;
	}

	if (acceleration <= MESSAGE_ACCELERATION_MIN) {
		acceleration = 0;
	}

	sendValueMessage('s', steering);
	sendValueMessage('b', brake);
	sendValueMessage('a', acceleration);
	sendCharMessage('\n');
}
