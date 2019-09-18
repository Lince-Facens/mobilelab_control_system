/*
 * FreeRTOS Coding Standard and Style Guide:
 * http://www.freertos.org/FreeRTOS-Coding-Standard-and-Style-Guide.html
 */

/* Kernel includes. */
#include "FreeRTOS.h"
#include "task.h"

/* STM32 Library includes. */
#include "stm32f10x.h"
#include "stm32f10x_usart.h"

#include <stdio.h>
#include <string.h>

/* Priorities at which the tasks are created. */
#define mainBLINK_TASK_PRIORITY				tskIDLE_PRIORITY + 1

//	/* Private macro -------------------------------------------------------------*/

/* ----- LED definitions --------------------------------------------------- */
#define mainLED_1	GPIO_Pin_13

/* Private define ------------------------------------------------------------*/
#define TxBufferSize   (countof(TxBuffer) - 1)
#define RxBufferSize   5

/* Private macro -------------------------------------------------------------*/
#define countof(a)   (sizeof(a) / sizeof(*(a)))

/* Private variables ---------------------------------------------------------*/
USART_InitTypeDef USART_InitStructure;
uint8_t TxBuffer[] =
		"\n\rUSART Hyperterminal Hardware Flow Control Example: USART - \
Hyperterminal communication using hardware flow control\n\r";
uint8_t RxBuffer[RxBufferSize];
uint8_t NbrOfDataToTransfer = TxBufferSize;
uint8_t TxCounter = 0;
uint8_t RxCounter = 0;


/* ----- Private method definitions ---------------------------------------- */
static void prvSetupHardware(void);

/* ----- Task definitions -------------------------------------------------- */
static void prvTransmitTask(void *pvParameters);
static void prvReceiveTask(void *pvParameters);

/* ----- Main -------------------------------------------------------------- */
int main()
{
	/* Configure the hardware */
	prvSetupHardware();

	/* Create the tasks */

	xTaskCreate(prvTransmitTask, /* Pointer to the task entry function */
				"Blink2", /* Name for the task */
				configMINIMAL_STACK_SIZE, /* The size of the stack */
				NULL, /* Pointer to parameters for the task */
				mainBLINK_TASK_PRIORITY, /* The priority for the task */
				NULL); /* Handle for the created task */

	xTaskCreate(prvReceiveTask, /* Pointer to the task entry function */
				"Blink3", /* Name for the task */
				configMINIMAL_STACK_SIZE, /* The size of the stack */
				NULL, /* Pointer to parameters for the task */
				mainBLINK_TASK_PRIORITY + 1, /* The priority for the task */
				NULL); /* Handle for the created task */

	/* Start the scheduler */
	vTaskStartScheduler();

	/* If all is well, the scheduler will now be running, and the following line
	 will never be reached.  If the following line does execute, then there was
	 insufficient FreeRTOS heap memory available for the idle and/or timer tasks
	 to be created.  See the memory management section on the FreeRTOS web site
	 for more details. */
	while (1)
		;
}

/*-----------------------------------------------------------*/
static void prvTransmitTask(void *pvParameters)
{
	while (1)
	{
		int aux = NbrOfDataToTransfer;
		while (aux--)
		{
			USART_SendData(USART1, TxBuffer[TxCounter++]);
			while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET)
				;
		}
		TxCounter = 0;
	}
}

static void prvReceiveTask(void *pvParameters)
{
	TickType_t xNextWakeTime;
	xNextWakeTime = xTaskGetTickCount();
	GPIO_SetBits(GPIOC, mainLED_1);
	while (1)
	{
		RxBuffer[RxCounter++] = USART_ReceiveData(USART1);

		if (RxBuffer[RxCounter-1] != 0) {
			GPIO_ResetBits(GPIOC, mainLED_1);
			vTaskDelayUntil(&xNextWakeTime, 50 / portTICK_PERIOD_MS);
			GPIO_SetBits(GPIOC, mainLED_1);
				vTaskDelayUntil(&xNextWakeTime, 50 / portTICK_PERIOD_MS);
		}

		RxCounter = (RxCounter + 1) % RxBufferSize;
		if (RxCounter == 100) {
			int a = RxCounter + 1;
		}
	}
}

/*-----------------------------------------------------------*/

static void prvSetupHardware(void)
{
	/* Ensure that all 4 interrupt priority bits are used as the pre-emption priority. */
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA |
						   RCC_APB2Periph_GPIOC |
						   RCC_APB2Periph_USART1 |
						   RCC_APB2Periph_AFIO, ENABLE);

	/* Set up the LED outputs */
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Pin = mainLED_1;
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
/*-----------------------------------------------------------*/

void vApplicationMallocFailedHook(void)
{
	/* Called if a call to pvPortMalloc() fails because there is insufficient
	 free memory available in the FreeRTOS heap.  pvPortMalloc() is called
	 internally by FreeRTOS API functions that create tasks, queues, software
	 timers, and semaphores.  The size of the FreeRTOS heap is set by the
	 configTOTAL_HEAP_SIZE configuration constant in FreeRTOSConfig.h. */
	while (1)
		;
}
/*-----------------------------------------------------------*/

void vApplicationStackOverflowHook(xTaskHandle pxTask, signed char *pcTaskName)
{
	(void) pcTaskName;
	(void) pxTask;

	/* Run time stack overflow checking is performed if
	 configconfigCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2.  This hook
	 function is called if a stack overflow is detected. */
	while (1)
		;
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
	while (1)
		;

	/* These lines help prevent getting warnings from compiler about unused variables */
	r0 = r1 = r2 = r3 = r12 = lr = pc = psr = 0;
	r0++;
}

#endif // #ifdef DEBUG
