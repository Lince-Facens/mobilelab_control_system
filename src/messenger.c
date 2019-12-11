/**
 * Message Transmitter
 * Sends ASCII messages through USART peripheral ports
 */

#include <messenger.h>
#include <stm32f10x.h>

#define MESSAGE_STEERING_CENTER (MESSAGE_STEERING_MAX / 2)
#define MESSAGE_STEERING_MIN (MESSAGE_STEERING_MAX * MESSAGE_STEERING_THRESHOLD)
#define MESSAGE_ACCELERATION_MIN (MESSAGE_ACCELERATION_MAX * MESSAGE_ACCELERATION_THRESHOLD)
#define MESSAGE_REVERSE_ACCELERATION_MIN (MESSAGE_REVERSE_ACCELERATION_MAX * MESSAGE_REVERSE_ACCELERATION_THRESHOLD)

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

void sendActuatorsMessage(uint16_t reverse_acceleration, uint16_t steeringRight, uint16_t steeringLeft, uint16_t acceleration)
{
	uint16_t steering = MESSAGE_STEERING_CENTER;
	uint8_t  reverse_acceleration_flag = GPIO_ReadInputDataBit(REVERSE_ACCELERATION_FLAG_PORT, REVERSE_ACCELERATION_FLAG_PIN);

	if (steeringLeft > MESSAGE_STEERING_MIN) {
		steering -= steeringLeft / 2;
	} else if (steeringRight > MESSAGE_STEERING_MIN) {
		steering += steeringRight / 2;
	}

	if (acceleration <= MESSAGE_ACCELERATION_MIN || reverse_acceleration_flag ) {
		acceleration = 0;
	}

	if (reverse_acceleration <= MESSAGE_REVERSE_ACCELERATION_MIN || !reverse_acceleration_flag) {
		reverse_acceleration = 0;
	}

	sendValueMessage('s', steering);
	sendValueMessage('r', reverse_acceleration);
	sendValueMessage('a', acceleration);
	sendCharMessage('\n');
}
