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

/* Donkeycar or mobile robotics approach defines */
#define DONKEY
// #define ROBOTICS


/* Priorities at which the tasks are created. */
#define receiveActuators_TASK_PRIORITY				(tskIDLE_PRIORITY + 2)
#define transmitSensors_TASK_PRIORITY				(tskIDLE_PRIORITY + 2)
#define transmitPWM_TASK_PRIORITY					(tskIDLE_PRIORITY + 1)

/* ----- LED definitions --------------------------------------------------- */
#define LED	GPIO_Pin_13
#define RxBufferSize   5

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define ADC1_DR    ((uint32_t)0x4001244C)
#ifdef DONKEY
#define ARRAYSIZE 4
#else
#define ARRAYSIZE 3
#endif

/* Private macro -------------------------------------------------------------*/
#define countof(a)   (sizeof(a) / sizeof(*(a)))

/* Private variables ---------------------------------------------------------*/
ADC_InitTypeDef ADC_InitStructure;
DMA_InitTypeDef DMA_InitStructure;
uint32_t status;
__IO uint16_t ADC_values[ARRAYSIZE];
uint8_t RxBuffer[RxBufferSize];
uint8_t SensorsMessageTx[] = "s:0000b:0000a:0000\n";
uint8_t ActuatorsMessageRx[] = "s:000b:000a:000";
uint16_t steering_actuator;
uint16_t brake_actuator;
uint16_t acceleration_actuator;
uint16_t Timer3Period = (uint16_t) 665;
__IO uint8_t actuatorsMsgStatus = (uint8_t) 0;
uint64_t timer = 0;
double Kp, Kd, Ki;
uint8_t radio_status;

/* ----- Private method definitions ---------------------------------------- */
static void prvSetupHardware(void);

/* ----- Task definitions -------------------------------------------------- */
static void prvReceiveActuatorsDataTask(void *pvParameters);
static void prvTransmitSensorsDataTask(void *pvParameters);
static void prvTransmitPWMTask(void *pvParameters);
static void prvCounterTask(void *pvParameters);
void RCC_Configuration(void);
void GPIO_Configuration(void);
void NVIC_Configuration(void);
void DMA_Configuration(void);
void ADC_Configuration(void);
void USART_Configuration(void);
void TIM_Configuration(void);
void EXTI_Configuration(void);
void prvConstructSensorsMessage(void);
int prvIsNumber(char c);
int prvCharToInt(char c);
uint8_t prvConstructActuatorsSensorsMessage(void);

/* ----- Main -------------------------------------------------------------- */
int main()
{
	/* Configure the hardware */
	prvSetupHardware();

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

//	xTaskCreate(prvTransmitPWMTask,				/* Pointer to the task entry function */
//				"Actuators_PWM",				/* Name for the task */
//				configMINIMAL_STACK_SIZE,		/* The size of the stack */
//				NULL,							/* Pointer to parameters for the task */
//				transmitPWM_TASK_PRIORITY,		/* The priority for the task */
//				NULL);

	xTaskCreate(prvCounterTask,				/* Pointer to the task entry function */
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

double diffSetPoint(double measure, double left, double right)
{
	if (left < 400 && right < 400) {

//		if (measure > 1894) {
//			return 1700 - measure;
//		}
//		else {
//			return 2065 - measure; // 2048 is the middle
//		}
//		return 2065 - measure;
		return 1894 - measure;
	}
	return 0;
//	else if (right > 400) {
//		double res = (right - ((1 - (measure - 1190)/643) * 3200));
//		return (res < 0) ? 3200 : res;
//	}
//	else if (left > 400) {
//		return -(left - ((measure - 1833)/667* 3200));
//	}
}

static void prvCounterTask(void *pvParameters)
{
	GPIO_ResetBits(GPIOC, LED);
	int last_time = -1, elapsed_time = -1;
	double steering_right, steering_left, acceleration, steering_feedback, cumulative_error = 0, last_error, rate_error, output;
	steering_right = steering_left = acceleration = steering_feedback = last_error = -1;
	Kp = 6;
	Kd = 5;
	Kd = 1;

	while (1)
	{
		radio_status = GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_15);
		// Still alive pin
		GPIO_SetBits(GPIOB, GPIO_Pin_14);

		if (radio_status) {

			if (steering_right != -1) {
	//			timer++;
				steering_feedback = ADC_values[0];

				// Measure the diff and estimate the output of steering.
				double error = diffSetPoint(steering_feedback, steering_left, steering_right);
	//			elapsed_time = xTaskGetTickCount() - last_time;
				elapsed_time = 10;

				cumulative_error += error * elapsed_time;
				rate_error = (error - last_error) / elapsed_time; // / elapsed_time

				output = Kp * error + Ki * cumulative_error + Kd * rate_error;
	//			output = error;

				last_error = error;

				if (error != 0) {
				// Actuate with output from PID controller
					if (output < -100) {
			//				TIM_SetCompare1(TIM3, Timer3Period * (-output)/4000.0);
						TIM_SetCompare1(TIM3, Timer3Period * (-output)/3500.0);
						TIM_SetCompare2(TIM3, 0);
					}
					else if (output > 100) {
						TIM_SetCompare2(TIM3, Timer3Period * (output)/3500.0);
						TIM_SetCompare1(TIM3, 0);
					}
				}
				else {
					if (steering_right > 400) {
						TIM_SetCompare1(TIM3, Timer3Period * (steering_right)/4095.0);
						TIM_SetCompare2(TIM3, 0);
					}
					else if (steering_left > 400) {
						TIM_SetCompare2(TIM3, Timer3Period * (steering_left)/4095.0);
						TIM_SetCompare1(TIM3, 0);
					}
				}

				GPIO_SetBits(GPIOB, GPIO_Pin_12);
				GPIO_SetBits(GPIOB, GPIO_Pin_13);
			}

			steering_feedback = ADC_values[0];
			steering_right    = ADC_values[1];
			steering_left     = ADC_values[2];
			acceleration      = ADC_values[3];

			// The following order was assured in the last test.


			// Old code to stop motor.
	//		else {
	//			GPIO_ResetBits(GPIOB, GPIO_Pin_12);
	//			GPIO_ResetBits(GPIOB, GPIO_Pin_13);
	//		}

			if (TIM_GetCapture3(TIM3) != acceleration) {
				if (acceleration <= 400) acceleration = 0;
				TIM_SetCompare3(TIM3, Timer3Period * (acceleration / 4095.0));
			}

			last_time = timer;

		}
		else {
			GPIO_ResetBits(GPIOB, GPIO_Pin_12);
			GPIO_ResetBits(GPIOB, GPIO_Pin_13);
			TIM_SetCompare3(TIM3, 0);
		}

		vTaskDelay(portTICK_PERIOD_MS / 100);
	}
}
//
//static void prvTransmitPWMTask(void *pvParameters)
//{
//	GPIO_SetBits(GPIOC, LED);
//	TickType_t xNextWakeTime;
//	xNextWakeTime = xTaskGetTickCount();
//
//	int test1 = 0, test2 = 0;
//	int last;
//	double total = 0;
//
//	while(1)
//	{
////		if (ADC_values[0] > 2000) {
////			test1++;
////
////			if (last == 0) {
////				total =  (test1) / (double)(test1 + test2);
////				test1 = 1;
////				test2 = 0;
////			}
////
////			last = 1;
////		}
////		else {
////			test2++;
////
////			last = 0;
////		}
//
//		uint16_t steering_period_l = 0;
//		uint16_t steering_period_r = 0;
//		uint16_t brake_period = 0;
//		uint16_t acceleration_period = 0;
//
//#ifdef DONKEY
//		steering_period_l      = (ADC_values[0] * Timer3Period) / 4095.0;
//		steering_period_r      = (ADC_values[1] * Timer3Period) / 4095.0;
//		acceleration_period    = (ADC_values[3] * Timer3Period) / 4095.0;
//
//#endif
//#ifdef ROBOTICS
//		if (actuatorsMsgStatus) {
//			steering_period_l;     = (steering_actuator * Timer3Period)/100;
//			brake_period;        = (brake_actuator * Timer3Period)/100;
//			acceleration_period; = (acceleration_actuator * Timer3Period)/100;
//		}
//#endif
//
//		/* Set steering value */
//		if (TIM_GetCapture1(TIM3) != steering_period_l) {
//			TIM_SetCompare1(TIM3, steering_period_l);
//		}
//		if (TIM_GetCapture2(TIM3) != steering_period_r) {
//			TIM_SetCompare2(TIM3, steering_period_r);
//		}
//		/* Set brake value */
//		if (TIM_GetCapture3(TIM3) != brake_period) {
//			TIM_SetCompare3(TIM3, brake_period);
//		}
//		/* Set acceleration value */
//		if (TIM_GetCapture4(TIM3) != acceleration_period) {
//			TIM_SetCompare4(TIM3, acceleration_period);
//		}
//		//	GPIO_InitTypeDef GPIO_InitStructure;n
//		GPIO_ResetBits(GPIOC, LED);
//		vTaskDelayUntil(&xNextWakeTime, 50 / portTICK_PERIOD_MS);
//		GPIO_SetBits(GPIOC, LED);
//		vTaskDelayUntil(&xNextWakeTime, 200 / portTICK_PERIOD_MS);
//
//		vTaskDelay(portTICK_PERIOD_MS);
//	}
//}
//
///*-----------------------------------------------------------*/
//static void prvReceiveActuatorsDataTask(void *pvParameters)
//{
//	int RxBufferCount;
//	while (1)
//	{
//		/* I don't know why, but I need to do this twice.
//		 * This is not totally stable, need a quick fix
//		 * */
//		while(USART_GetFlagStatus(USART1, USART_FLAG_RXNE) == RESET);
//		while (USART_ReceiveData(USART1) != 's') ;
//		while(USART_GetFlagStatus(USART1, USART_FLAG_RXNE) == RESET);
//		while (USART_ReceiveData(USART1) != 's') ;
//
//		for (RxBufferCount = 1; RxBufferCount < countof(ActuatorsMessageRx); ++RxBufferCount) {
//			while(USART_GetFlagStatus(USART1, USART_FLAG_RXNE) == RESET);
//			ActuatorsMessageRx[RxBufferCount] = USART_ReceiveData(USART1);
//
//		}
//
//		if (prvVerifyActuatorsMessage()) {
//			actuatorsMsgStatus = prvConstructActuatorsSensorsMessage();
//		}
//
//		vTaskDelay(50 / portTICK_PERIOD_MS);
//	}
//}

static void prvTransmitSensorsDataTask(void *pvParameters)
{
	int i = 0;

	while (1)
	{
		timer++;
		prvConstructSensorsMessage();

		for (i = 0; i < countof(SensorsMessageTx); ++i) {
			USART_SendData(USART1, SensorsMessageTx[i]);
			while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
		}

		vTaskDelay(1000 * portTICK_PERIOD_MS);
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

//	steering_actuator     = steering_actuator     == 100 ? 99 : steering_actuator;
//	brake_actuator 		  = brake_actuator        == 100 ? 99 : brake_actuator;
//	acceleration_actuator = acceleration_actuator == 100 ? 99 : acceleration_actuator;

	return ret;
}

/*
 * Mount message to send
 * Format: s:0000b:0000a:0000
 * Where s -> steering direction
 *       b -> brake
 *       a -> acceleration
 * */
void prvConstructSensorsMessage(void)
{
	/* TODO: be sure that this is the actual order */
	uint16_t steering_sensor     = 4095; // Channel 0
	uint16_t brake_sensor        = ADC_values[0]; // Channel 2
	uint16_t acceleration_sensor = ADC_values[2]; // Channel 3

	if (ADC_values[3] > 300) {
		steering_sensor = 4095 - ADC_values[3];
	}
	else if (ADC_values[1] > 300) {
		steering_sensor = 4095 + ADC_values[1];
	}

	steering_sensor /= 2;

	SensorsMessageTx[2] = '0' + (steering_sensor / 1000);
	SensorsMessageTx[3] = '0' + (steering_sensor / 100) % 10;
	SensorsMessageTx[4] = '0' + (steering_sensor / 10) % 10;
	SensorsMessageTx[5] = '0' + steering_sensor % 10;

	SensorsMessageTx[8] = '0' + brake_sensor / 1000;
	SensorsMessageTx[9] = '0' + (brake_sensor / 100) % 10;
	SensorsMessageTx[10] = '0' + (brake_sensor / 10) % 10;
	SensorsMessageTx[11] = '0' + brake_sensor % 10;

	SensorsMessageTx[14] = '0' + acceleration_sensor / 1000;
	SensorsMessageTx[15] = '0' + (acceleration_sensor / 100) % 10;
	SensorsMessageTx[16] = '0' + (acceleration_sensor / 10) % 10;
	SensorsMessageTx[17] = '0' + acceleration_sensor % 10;
}

void TIM_Configuration(void)
{
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;

	/* Compute the prescaler value */
//	int PrescalerValue = (uint16_t) (SystemCoreClock / 24000000) - 1;
	int PrescalerValue = (uint16_t) (SystemCoreClock / (50000)) - 1;
	/* Time base configuration */
	TIM_TimeBaseStructure.TIM_Period = Timer3Period ;
	TIM_TimeBaseStructure.TIM_Prescaler = PrescalerValue;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);

	/* PWM1 Mode configuration: Channel1 */
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = 0;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;

	TIM_OC1Init(TIM3, &TIM_OCInitStructure);

	TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable);

	/* PWM1 Mode configuration: Channel2 */
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = 0;

	TIM_OC2Init(TIM3, &TIM_OCInitStructure);
	TIM_OC2PreloadConfig(TIM3, TIM_OCPreload_Enable);

	/* PWM1 Mode configuration: Channel3 */
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = 0;

	TIM_OC3Init(TIM3, &TIM_OCInitStructure);
	TIM_OC3PreloadConfig(TIM3, TIM_OCPreload_Enable);

	/* PWM1 Mode configuration: Channel4 */
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = 0;

	TIM_OC4Init(TIM3, &TIM_OCInitStructure);
	TIM_OC4PreloadConfig(TIM3, TIM_OCPreload_Enable);

	TIM_ARRPreloadConfig(TIM3, ENABLE);

	/* TIM3 enable counter */
	TIM_Cmd(TIM3, ENABLE);
}


void USART_Configuration(void)
{
	USART_InitTypeDef USART_InitStructure;
	USART_InitStructure.USART_BaudRate = 9600;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl =
			USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

	USART_Init(USART1, &USART_InitStructure);
	/* Enable the USART2 */
	USART_Cmd(USART1, ENABLE);
}

/**
 * @brief  Configures the ADC.
 * @param  None
 * @retval None
 */
void ADC_Configuration(void)
{
	/* ADC1 configuration ------------------------------------------------------*/
	ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
	ADC_InitStructure.ADC_ScanConvMode = ENABLE;
	ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
	ADC_InitStructure.ADC_NbrOfChannel = ARRAYSIZE;

	ADC_Init(ADC1, &ADC_InitStructure);
	/* ADC1 regular channels configuration */
	ADC_RegularChannelConfig(ADC1, ADC_Channel_0, 1, ADC_SampleTime_28Cycles5);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_1, 2, ADC_SampleTime_28Cycles5);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_2, 3, ADC_SampleTime_28Cycles5);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_3, 4, ADC_SampleTime_28Cycles5);

	/* Enable ADC1 DMA */
	ADC_DMACmd(ADC1, ENABLE);

	/* Enable ADC1 */
	ADC_Cmd(ADC1, ENABLE);

	/* Enable ADC1 reset calibration register */
	ADC_ResetCalibration(ADC1);
	/* Check the end of ADC1 reset calibration register */
	while (ADC_GetResetCalibrationStatus(ADC1));

	/* Start ADC1 calibration */
	ADC_StartCalibration(ADC1);
	/* Check the end of ADC1 calibration */
	while (ADC_GetCalibrationStatus(ADC1))
		;

	//Start ADC1 Software Conversion
	ADC_SoftwareStartConvCmd(ADC1, ENABLE);
	//wait for DMA complete
	while (!status) {};
	ADC_SoftwareStartConvCmd(ADC1, DISABLE);
}

/**
 * @brief  Configures the DMA for reading the ADC values.
 * @param  None
 * @retval None
 */
void DMA_Configuration(void)
{
	/* DMA1 channel1 configuration ----------------------------------------------*/
	DMA_DeInit(DMA1_Channel1);
	DMA_InitStructure.DMA_BufferSize = ARRAYSIZE;
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t) ADC1_DR;
	DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t) ADC_values;
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
	DMA_Init(DMA1_Channel1, &DMA_InitStructure);
	/* Enable DMA1 channel1 */
	DMA_ITConfig(DMA1_Channel1, DMA_IT_TC, ENABLE);
	DMA_Cmd(DMA1_Channel1, ENABLE);

	//Enable DMA1 Channel transfer
	DMA_Cmd(DMA1_Channel1, ENABLE);
}

/**
 * @brief  Configures the different system clocks.
 * @param  None
 * @retval None
 */
void RCC_Configuration(void)
{
	/* TIM3 clock enable */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

	/* GPIOA and GPIOB clock enable */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB |
						 RCC_APB2Periph_GPIOC | RCC_APB2Periph_AFIO, ENABLE);

	/* Enable DMA1 clocks */
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

	/* Enable ADC1, ADC2, ADC3 and GPIOC clocks */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);

}

/**
 * @brief  Configures the different GPIO ports.
 * @param  None
 * @retval None
 */
void GPIO_Configuration(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	/* Configure PC.02, PC.03 and PC.04 (ADC Channel12, ADC Channel13 and
	 ADC Channel14) as analog inputs */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	/* Set up the LED outputs */
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Pin = LED;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	/* Configure USART2 RTS and USART2 Tx as alternate function push-pull */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	/* Configure USART2 CTS and USART2 Rx as input floating */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	/* Configure ADC inputs */
//	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2;
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
//	GPIO_Init(GPIOA, &GPIO_InitStructure);

	/* Configure PWM outputs */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	/* Configure PWM outputs TIM3_CH3 */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
}

/**
 * @brief  Configures Vector Table base location.
 * @param  None
 * @retval None
 */
void NVIC_Configuration(void)
{
	NVIC_InitTypeDef NVIC_InitStructure;

	//Enable DMA1 channel IRQ Channel */
	NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 7;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 7;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

//	/* Enable and set EXTI0 Interrupt to the lowest priority */
//	NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn;
//	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 7;
//	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 7;
//	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
//	NVIC_Init(&NVIC_InitStructure);
//
//	/* Enable and set EXTI0 Interrupt to the lowest priority */
//	NVIC_InitStructure.NVIC_IRQChannel = EXTI1_IRQn;
//	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 7;
//	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 7;
//	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
//	NVIC_Init(&NVIC_InitStructure);

//	/* Enable and set EXTI0 Interrupt to the lowest priority */
//	NVIC_InitStructure.NVIC_IRQChannel = EXTI2_IRQn;
//	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 7;
//	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 7;
//	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
//	NVIC_Init(&NVIC_InitStructure);
//
//	/* Enable and set EXTI0 Interrupt to the lowest priority */
//	NVIC_InitStructure.NVIC_IRQChannel = EXTI3_IRQn;
//	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 7;
//	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 7;
//	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
//	NVIC_Init(&NVIC_InitStructure);
//
//	/* Enable and set EXTI0 Interrupt to the lowest priority */
//	NVIC_InitStructure.NVIC_IRQChannel = EXTI4_IRQn;
//	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 7;
//	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 7;
//	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
//	NVIC_Init(&NVIC_InitStructure);
//
//	/* Enable and set EXTI0 Interrupt to the lowest priority */
//	NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;
//	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 7;
//	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 7;
//	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
//	NVIC_Init(&NVIC_InitStructure);
}

/*-----------------------------------------------------------*/

static void prvSetupHardware(void)
{
	/* Ensure that all 4 interrupt priority bits are used as the pre-emption priority. */
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);

	/* System clocks configuration ---------------------------------------------*/
	RCC_Configuration();

	/* USART configuration -----------------------------------------------------*/
	USART_Configuration();

	/* GPIO configuration ------------------------------------------------------*/
	GPIO_Configuration();

	/* DMA configuration -------------------------------------------------------*/
	DMA_Configuration();

	/* EXTI configuration ----------------------------------------------------- */
//	EXTI_Configuration();

	/* NVIC configuration ------------------------------------------------------*/
	NVIC_Configuration();

	/* ADC configuration -------------------------------------------------------*/
	ADC_Configuration();

	/* TIM configuration -------------------------------------------------------*/
	TIM_Configuration();
}
/*-----------------------------------------------------------*/

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

void EXTI_Configuration(void)
{
	EXTI_InitTypeDef EXTI_InitStructure;

	/* Connect EXTI0 Line to PA.00 pin */
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinSource0);

	/* Configure EXTI0 line */
	EXTI_InitStructure.EXTI_Line = EXTI_Line0;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);

	/* Configure EXTI1 line */
	EXTI_InitStructure.EXTI_Line = EXTI_Line1;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);

//	/* Configure EXTI0 line */
//	EXTI_InitStructure.EXTI_Line = EXTI_Line2;
//	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
//	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
//	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
//	EXTI_Init(&EXTI_InitStructure);
//
//	/* Configure EXTI1 line */
//	EXTI_InitStructure.EXTI_Line = EXTI_Line3;
//	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
//	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
//	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
//	EXTI_Init(&EXTI_InitStructure);
//
//	/* Configure EXTI1 line */
//	EXTI_InitStructure.EXTI_Line = EXTI_Line4;
//	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
//	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
//	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
//	EXTI_Init(&EXTI_InitStructure);
//
//	/* Configure EXTI1 line */
//	EXTI_InitStructure.EXTI_Line = EXTI_Line5;
//	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
//	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
//	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
//	EXTI_Init(&EXTI_InitStructure);
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
