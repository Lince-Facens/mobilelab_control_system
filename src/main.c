/*
 * FreeRTOS Coding Standard and Style Guide:
 * http://www.freertos.org/FreeRTOS-Coding-Standard-and-Style-Guide.html
 */

/* Kernel includes. */
#include "FreeRTOS.h"
#include "task.h"

/* STM32 Library includes. */
#include "stm32f10x.h"
#include "stm32f10x_adc.h"
#include "stm32f10x_dma.h"

/* System includes */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* Private includes */
#include "pid.h"
#include "peripherals.h"
#include "messenger.h"

/* Donkeycar or mobile robotics approach defines */
#define DONKEY
// #define ROBOTICS

#define MAX_STEERING 4095
#define STEERING_THRESHOLD 0.1
#define MAX_ACCELERATION 4095
#define ACCELERATION_THRESHOLD 0.1

#define STEERING_THRESHOLD_VALUE (MAX_STEERING * STEERING_THRESHOLD)
#define ACCELERATION_THRESHOLD_VALUE (MAX_ACCELERATION * ACCELERATION_THRESHOLD)


/* Priorities at which the tasks are created. */
#define receiveActuators_TASK_PRIORITY				(tskIDLE_PRIORITY + 2)
#define transmitSensors_TASK_PRIORITY				(tskIDLE_PRIORITY + 1)
#define transmitPWM_TASK_PRIORITY					(tskIDLE_PRIORITY + 1)

#define RxBufferSize   5

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/
#define countof(a)   (sizeof(a) / sizeof(*(a)))

/* Private variables ---------------------------------------------------------*/
uint8_t RxBuffer[RxBufferSize];
uint8_t ActuatorsMessageRx[] = "s:000b:000a:000";
uint16_t steering_actuator;
uint16_t brake_actuator;
uint16_t acceleration_actuator;
__IO uint8_t actuatorsMsgStatus = (uint8_t) 0;

/* ----- Private method definitions ---------------------------------------- */

/* ----- Task definitions -------------------------------------------------- */
static void prvReceiveActuatorsDataTask(void *pvParameters);
static void prvTransmitSensorsDataTask(void *pvParameters);
static void prvControlMotors(void *pvParameters);
int prvIsNumber(char c);
int prvCharToInt(char c);
uint8_t prvConstructActuatorsSensorsMessage(void);

/* ----- Main -------------------------------------------------------------- */
int main()
{
	/* Configure the hardware peripherals */
	setupPeripherals();

	/* Create the tasks */
#ifdef ROBOTICS
	xTaskCreate(prvReceiveActuatorsDataTask,	/* Pointer to the task entry function */
				"Actuators_receive",			/* Name for the task */
				configMINIMAL_STACK_SIZE,		/* The size of the stack */
				NULL,							/* Pointer to parameters for the task */
				receiveActuators_TASK_PRIORITY,	/* The priority for the task */
				NULL);							/* Handle for the created task */
#endif

	xTaskCreate(prvTransmitSensorsDataTask,		/* Pointer to the task entry function */
				"Sensors_Transmit",				/* Name for the task */
				configMINIMAL_STACK_SIZE,		/* The size of the stack */
				NULL,							/* Pointer to parameters for the task */
				transmitSensors_TASK_PRIORITY,	/* The priority for the task */
				NULL);

	xTaskCreate(prvControlMotors,				/* Pointer to the task entry function */
				"Counter",				/* Name for the task */
				configMINIMAL_STACK_SIZE,		/* The size of the stack */
				NULL,							/* Pointer to parameters for the task */
				transmitPWM_TASK_PRIORITY,		/* The priority for the task */
				NULL);

	/* Start the scheduler */
	vTaskStartScheduler();

	/* If all is well, the scheduler will now be running, and the following line
	will never be reached.  If the following line does execute, then there was
	insufficient FreeRTOS heap memory available for the idle and/or timer tasks
	to be created.  See the memory management section on the FreeRTOS web site
	for more details. */
	while (1);
}

static void prvControlMotors(void *pvParameters)
{
	GPIO_ResetBits(GPIOC, LED);
	double steering_right, steering_left, acceleration, steering_feedback;
	steering_right = steering_left = acceleration = steering_feedback = -1;
	initPID();

	while (1)
	{
		// Still alive pin
		GPIO_SetBits(GPIOB, GPIO_Pin_14);

		if (steering_right != -1) {
			steering_feedback = ADC_values[0];

			// Calc PID controller output
			double output = calcPIDOutput(steering_feedback, steering_left, steering_right);

			if (output != 0) {
			// Actuate with output from PID controller
				if (output < -100) {
					TIM_SetCompare1(TIM3, TIM3PERIOD * (-output)/3500.0);
					TIM_SetCompare2(TIM3, 0);
				}
				else if (output > 100) {
					TIM_SetCompare2(TIM3, TIM3PERIOD * output/3500.0);
					TIM_SetCompare1(TIM3, 0);
				}
			}
			// TODO: remove else section (after tests)
			else {
				// Checks whether the steering is above the threshold

				if (steering_right > STEERING_THRESHOLD_VALUE) {
					TIM_SetCompare1(TIM3, (TIM3PERIOD * steering_right) / MAX_STEERING);
					TIM_SetCompare2(TIM3, 0);
				} else if (steering_left > STEERING_THRESHOLD_VALUE) {
					TIM_SetCompare1(TIM3, 0);
					TIM_SetCompare2(TIM3, (TIM3PERIOD * steering_left) / MAX_STEERING);
				} else {
					TIM_SetCompare1(TIM3, 0);
					TIM_SetCompare2(TIM3, 0);
				}

			}

			GPIO_SetBits(GPIOB, GPIO_Pin_12);
			GPIO_SetBits(GPIOB, GPIO_Pin_13);
		}

		steering_feedback = ADC_values[0];
		steering_right    = ADC_values[1];
		steering_left     = ADC_values[2];
		acceleration      = ADC_values[3];

		if (TIM_GetCapture3(TIM3) != acceleration) {
			if (acceleration > ACCELERATION_THRESHOLD_VALUE) {
				TIM_SetCompare3(TIM3, TIM3PERIOD * (acceleration / MAX_ACCELERATION));
			} else {
				TIM_SetCompare3(TIM3, 0);
			}
		}

		updatePIDTickCount();

	}
}

static void prvTransmitSensorsDataTask(void *pvParameters)
{
	TickType_t wakeTime = xTaskGetTickCount();

	while (1) {
		sendActuatorsMessage(ADC_values[0], ADC_values[1], ADC_values[2], ADC_values[3]);

		vTaskDelayUntil(&wakeTime, configTICK_RATE_HZ / 20); // 50 ms
	}
}

int prvVerifyActuatorsMessage(void)
{
	int ret = 1;

	/* Check if basic structure of message is maintained */
	if (ActuatorsMessageRx[0]  != 's' || ActuatorsMessageRx[1] != ':' ||
		ActuatorsMessageRx[5]  != 'b' || ActuatorsMessageRx[6] != ':' ||
		ActuatorsMessageRx[10] != 'a' || ActuatorsMessageRx[11] != ':') {
		ret = 0;
	}

	/* Check if steering value is a valid integer */
	if (!prvIsNumber(ActuatorsMessageRx[2]) ||
		!prvIsNumber(ActuatorsMessageRx[3]) ||
		!prvIsNumber(ActuatorsMessageRx[4])) {
		ret = 0;
	}

	/* Check if brake value is a valid integer */
	if (!prvIsNumber(ActuatorsMessageRx[7]) ||
		!prvIsNumber(ActuatorsMessageRx[8]) ||
		!prvIsNumber(ActuatorsMessageRx[9])) {
		ret = 0;
	}

	/* Check if acceleration value is a valid integer */
	if (!prvIsNumber(ActuatorsMessageRx[12]) ||
		!prvIsNumber(ActuatorsMessageRx[13]) ||
		!prvIsNumber(ActuatorsMessageRx[14])) {
		ret = 0;
	}

	/* TODO: Verify if each value is in the limit range value */

	return ret;
}

/*
 * Check if the input char is a number
 */
int prvIsNumber(char c)
{
	int ret = 1;

	if (c < '0' || c > '9') {
		ret = 0;
	}

	return ret;
}


int prvCharToInt(char c)
{
	return c - '0';
}

/*
 * The expected values varies from 0 to 100
 *  */
uint8_t prvConstructActuatorsSensorsMessage(void)
{
	uint8_t ret = 1;
	uint8_t steering_buffer[3], brake_buffer[3], acceleration_buffer[3];
	steering_actuator = brake_actuator = acceleration_actuator = 0;

	steering_buffer[0] = ActuatorsMessageRx[2];
	steering_buffer[1] = ActuatorsMessageRx[3];
	steering_buffer[2] = ActuatorsMessageRx[4];

	brake_buffer[0] = ActuatorsMessageRx[7];
	brake_buffer[1] = ActuatorsMessageRx[8];
	brake_buffer[2] = ActuatorsMessageRx[9];

	acceleration_buffer[0] = ActuatorsMessageRx[12];
	acceleration_buffer[1] = ActuatorsMessageRx[13];
	acceleration_buffer[2] = ActuatorsMessageRx[14];

	steering_actuator 	  = atoi(steering_buffer);
	brake_actuator 		  = atoi(brake_buffer);
	acceleration_actuator = atoi(acceleration_buffer);

	if (steering_actuator > 100 ||
		brake_actuator > 100 ||
		acceleration_actuator > 100) {
		ret = 0;
		steering_actuator = brake_actuator = acceleration_actuator = 0;
	}

	return ret;
}


void vApplicationMallocFailedHook(void)
{
	/* Called if a call to pvPortMalloc() fails because there is insufficient
	free memory available in the FreeRTOS heap.  pvPortMalloc() is called
	internally by FreeRTOS API functions that create tasks, queues, software
	timers, and semaphores.  The size of the FreeRTOS heap is set by the
	configTOTAL_HEAP_SIZE configuration constant in FreeRTOSConfig.h. */
	while (1);
}
/*-----------------------------------------------------------*/

void vApplicationStackOverflowHook(xTaskHandle pxTask, signed char *pcTaskName)
{
	(void) pcTaskName;
	(void) pxTask;

	/* Run time stack overflow checking is performed if
	configconfigCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2.  This hook
	function is called if a stack overflow is detected. */
	while (1);
}
/*-----------------------------------------------------------*/

void vApplicationIdleHook(void)
{
	volatile size_t xFreeStackSpace;

	/* This function is called on each cycle of the idle task.  In this case it
	does nothing useful, other than report the amount of FreeRTOS heap that
	remains unallocated. */
	xFreeStackSpace = xPortGetFreeHeapSize();

	if (xFreeStackSpace > 100)
	{
		/* By now, the kernel has allocated everything it is going to, so
		if there is a lot of heap remaining unallocated then
		the value of configTOTAL_HEAP_SIZE in FreeRTOSConfig.h can be
		reduced accordingly. */
	}
}

#ifdef DEBUG
/* The fault handler implementation calls a function called
prvGetRegistersFromStack(). */
void HardFault_Handler(void)
{
    __asm volatile
    (
        " tst lr, #4                                                \n"
        " ite eq                                                    \n"
        " mrseq r0, msp                                             \n"
        " mrsne r0, psp                                             \n"
        " ldr r1, [r0, #24]                                         \n"
        " ldr r2, handler2_address_const                            \n"
        " bx r2                                                     \n"
        " handler2_address_const: .word prvGetRegistersFromStack    \n"
    );
}

void prvGetRegistersFromStack(uint32_t *pulFaultStackAddress)
{
	/* These are volatile to try and prevent the compiler/linker optimising them
	away as the variables never actually get used.  If the debugger won't show the
	values of the variables, make them global my moving their declaration outside
	of this function. */
	volatile uint32_t r0;
	volatile uint32_t r1;
	volatile uint32_t r2;
	volatile uint32_t r3;
	volatile uint32_t r12;
	volatile uint32_t lr; /* Link register. */
	volatile uint32_t pc; /* Program counter. */
	volatile uint32_t psr;/* Program status register. */

    r0 = pulFaultStackAddress[0];
    r1 = pulFaultStackAddress[1];
    r2 = pulFaultStackAddress[2];
    r3 = pulFaultStackAddress[3];

    r12 = pulFaultStackAddress[4];
    lr = pulFaultStackAddress[5];
    pc = pulFaultStackAddress[6];
    psr = pulFaultStackAddress[7];

    /* When the following line is hit, the variables contain the register values. */
    while (1);

    /* These lines help prevent getting warnings from compiler about unused variables */
    r0 = r1 = r2 = r3 = r12 = lr = pc = psr = 0;
    r0++;
}

#endif // #ifdef DEBUG
