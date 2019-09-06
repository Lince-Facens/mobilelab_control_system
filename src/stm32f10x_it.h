/**
 ******************************************************************************
 * @file    mobilelab_data_gathering/stm32f10x_it.h
 * @author  Emanuel Huber da Silva
 * @version V1.0
 * @date    06-August-2019
 * @brief   Main Interrupt Service Routines.
 *          This file contains the headers of the interrupt handlers.
 ******************************************************************************

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __STM32F10x_IT_H
#define __STM32F10x_IT_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x.h"

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */

void NMI_Handler(void);
void HardFault_Handler(void);
void MemManage_Handler(void);
void BusFault_Handler(void);
void UsageFault_Handler(void);
void SVC_Handler(void);
void DebugMon_Handler(void);
void PendSV_Handler(void);
void SysTick_Handler(void);
void WWDG_IRQHandler(void);
void PVD_IRQHandler(void);
void TAMPER_IRQHandler(void);
void RTC_IRQHandler(void);
void FLASH_IRQHandler(void);
void RCC_IRQHandler(void);
void EXTI0_IRQHandler(void);
void USART1_IRQHandler(void);
void DMA1_Channel1_IRQHandler(void);
#endif /* __STM32F10x_IT_H */
