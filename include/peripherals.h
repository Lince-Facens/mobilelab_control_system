/*
 * Peripherals configurations definitions
 */

#include "stm32f10x.h"

/* Private defines */
#define LED	GPIO_Pin_13
#define ADC1_DR ((uint32_t)0x4001244C)
#define ARRAYSIZE 4

/* Public defines */
#define TIM3PERIOD 665

/* Public variables */
__IO uint16_t ADC_values[ARRAYSIZE];

/* Public functions */
void setupPeripherals(void);

/* Private functions */
void RCC_Configuration(void);
void GPIO_Configuration(void);
void DMA_Configuration(void);
void ADC_Configuration(void);
void USART_Configuration(void);
void TIM_Configuration(void);
void EXTI_Configuration(void);
