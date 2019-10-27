/**
  ******************************************************************************
  * @file    USART/HyperTerminal_HwFlowControl/stm32f10x_it.c 
  * @author  MCD Application Team
  * @version V3.5.0
  * @date    08-April-2011
  * @brief   Main Interrupt Service Routines.
  *          This file provides template for all exceptions handler and 
  *          peripherals interrupt service routine.
  ******************************************************************************
  * @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2011 STMicroelectronics</center></h2>
  ******************************************************************************
  */ 

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "stm32f10x_it.h"

extern int status;

// left steering
uint64_t pwm0Start = 0, pwm0Stop = 0, pwm0 = 0;
// right steering
uint64_t pwm1Start = 0, pwm1Stop = 0, pwm1 = 0;
// acceleration
uint64_t pwm2Start = 0, pwm2Stop = 0, pwm2 = 0;

extern uint16_t Timer3Period;
extern uint64_t timer;

extern __IO uint16_t ADC_values[5];

/** @addtogroup STM32F10x_StdPeriph_Examples
  * @{
  */

/** @addtogroup USART_HyperTerminal_HwFlowControl
  * @{
  */ 

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/******************************************************************************/
/*            Cortex-M3 Processor Exceptions Handlers                         */
/******************************************************************************/

/**
  * @brief  This function handles NMI exception.
  * @param  None
  * @retval None
  */
void NMI_Handler(void)
{
}

/**
  * @brief  This function handles Debug Monitor exception.
  * @param  None
  * @retval None
  */
void DebugMon_Handler(void)
{
}


void DMA1_Channel1_IRQHandler(void)
{
	//Test on DMA1 Channel1 Transfer Complete interrupt
	if (DMA_GetITStatus(DMA1_IT_TC1))
	{
		status = 1;
		//Clear DMA1 interrupt pending bits
		DMA_ClearITPendingBit(DMA1_IT_GL1);
	}
}


/**
 * @brief  This function handles External line 0 interrupt request.
 * @param  None
 * @retval None
 */
void EXTI0_IRQHandler(void)
{
	if (EXTI_GetITStatus(EXTI_Line0) != RESET)
	{
		pwm0Start = timer;

		/* Clear the  EXTI line 0 pending bit */
		EXTI_ClearITPendingBit(EXTI_Line0);
	}
}

/*
* @brief  This function handles External line 0 interrupt request.
* @param  None
* @retval None
*/
void EXTI1_IRQHandler(void)
{
	if (EXTI_GetITStatus(EXTI_Line1) != RESET)
	{
		pwm0Stop = timer;

		if (pwm0Start != 0) {
			volatile double diff = (pwm0Stop - pwm0Start) / 1820.0;
			if (diff >= 1) {
				diff = 0.99;
			}
			else if (diff < 0) {
				diff = 0;
			}
			TIM_SetCompare1(TIM3, Timer3Period * diff);

			pwm0Start = pwm0Stop = 0;
		}

		/* Clear the  EXTI line 1 pending bit */
		EXTI_ClearITPendingBit(EXTI_Line1);
	}
}

/**
 * @brief  This function handles External line 0 interrupt request.
 * @param  None
 * @retval None
 */
void EXTI2_IRQHandler(void)
{
	if (EXTI_GetITStatus(EXTI_Line2) != RESET)
	{
		pwm1Start = timer;

		/* Clear the  EXTI line 0 pending bit */
		EXTI_ClearITPendingBit(EXTI_Line2);
	}
}

/*
* @brief  This function handles External line 0 interrupt request.
* @param  None
* @retval None
*/
void EXTI3_IRQHandler(void)
{
	if (EXTI_GetITStatus(EXTI_Line3) != RESET)
	{
		pwm1Stop = timer;

		if (pwm1Start != 0) {
			volatile double diff = (pwm1Stop - pwm1Start) / 1820.0;
			if (diff >= 1) {
				diff = 0.99;
			}
			else if (diff < 0) {
				diff = 0;
			}
			TIM_SetCompare2(TIM3, Timer3Period * diff);

			pwm1Start = pwm1Stop = 0;
		}

		/* Clear the  EXTI line 1 pending bit */
		EXTI_ClearITPendingBit(EXTI_Line3);
	}
}

/**
 * @brief  This function handles External line 0 interrupt request.
 * @param  None
 * @retval None
 */
void EXTI4_IRQHandler(void)
{
	if (EXTI_GetITStatus(EXTI_Line4) != RESET)
	{
		pwm2Start = timer;

		/* Clear the  EXTI line 0 pending bit */
		EXTI_ClearITPendingBit(EXTI_Line4);
	}
}

/*
* @brief  This function handles External line 0 interrupt request.
* @param  None
* @retval None
*/
void EXTI9_5_IRQHandler(void)
{
	if (EXTI_GetITStatus(EXTI_Line5) != RESET)
	{
		pwm2Stop = timer;

		if (pwm2Start != 0) {
			volatile double diff = (pwm2Stop - pwm2Start) / 1820.0;
			if (diff >= 1) {
				diff = 0.99;
			}
			else if (diff < 0) {
				diff = 0;
			}
			TIM_SetCompare3(TIM3, Timer3Period * diff);

			pwm2Start = pwm2Stop = 0;
		}

		/* Clear the  EXTI line 1 pending bit */
		EXTI_ClearITPendingBit(EXTI_Line5);
	}
}

/******************************************************************************/
/*                 STM32F10x Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f10x_xx.s).                                            */
/******************************************************************************/

/**
  * @brief  This function handles PPP interrupt request.
  * @param  None
  * @retval None
  */
/*void PPP_IRQHandler(void)
{
}*/

/**prvSetupHardware
  * @}
  */ 

/**
  * @}
  */ 

/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
