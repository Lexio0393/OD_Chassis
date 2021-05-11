/**
  ******************************************************************************
  * @file    Project/STM32F4xx_StdPeriph_Templates/stm32f4xx_it.c 
  * @author  MCD Application Team
  * @version V1.8.0
  * @date    04-November-2016
  * @brief   Main Interrupt Service Routines.
  *          This file provides template for all exceptions handler and 
  *          peripherals interrupt service routine.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2016 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_it.h"

/* User Includes, DO NOT add to .h file*/
#include "main.h"
#include "task.h"
#include "bsp_09s.h"
#include "pps.h"
#include "C620.h"

extern uint8_t g_Print_FinishFlag;
extern uint8_t g_Display_usart[200];
/** @addtogroup Template_Project
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/******************************************************************************/
/*            Cortex-M4 Processor Exceptions Handlers                         */
/******************************************************************************/

/**
  * @brief  This function handles USART1 exception.
  * @param  None
  * @retval None
  */
void USART1_IRQHandler(void)
{
	Locator_SerialIsr();
}

/**
  * @brief  This function handles USART2 exception.
  * @param  None
  * @retval None
  */
void USART2_IRQHandler(void)
{
	Remote_SerialIsr();
}

/**
  * @brief  This function handles USART3 exception.
  * @param  None
  * @retval None
  */
void USART3_IRQHandler(void)
{
	Locator_SerialIsr();
}


/**
  * @brief  This function handles TIM2 exception.
  * @param  None
  * @retval None
  */
void TIM2_IRQHandler(void)	//5ms Tasks
{
	if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET) 
	{
		static uint16_t time_print_tick = 0;
		
		TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
		TIM_ClearFlag(TIM2, TIM_FLAG_Update);
		
		time_print_tick++;
		
		if (time_print_tick >= 500 && g_Print_FinishFlag) 
		{
			time_print_tick = 0;
			Serial_SendString(Serial6, (uint8_t*)g_Display_usart);
		}
		
		TIM2_5ms_Task();
	}
}

/**
  * @brief  This function handles TIM5 exception.
  * @param  None
  * @retval None
  */
void TIM5_IRQHandler(void)	//10ms Tasks
{
	if (TIM_GetITStatus(TIM5, TIM_IT_Update) != RESET) 
	{
		TIM_ClearITPendingBit(TIM5, TIM_IT_Update);
		TIM_ClearFlag(TIM5, TIM_FLAG_Update);
		TIM5_10ms_Task();
	}
}

void CAN1_TX_IRQHandler(void)
{
	if (CAN_GetITStatus(CAN1,CAN_IT_TME) != RESET)
	{
		CAN_ClearITPendingBit(CAN1,CAN_IT_TME);
	}
}

void CAN2_TX_IRQHandler(void)
{

}

void CAN1_RX0_IRQHandler(void)
{
	CanRxMsg RxMsg;
	if(CAN_GetITStatus(CAN1,CAN_IT_FMP0) != RESET)
	{
		CAN_ClearITPendingBit(CAN1,CAN_IT_FF0);
		CAN_ClearFlag (CAN1, CAN_IT_FF0);
 		CAN_Receive (CAN1, CAN_FIFO0 ,&RxMsg);	
		C620_GetFeedbackInfo(&RxMsg);
	}
}

void CAN2_RX0_IRQHandler(void)
{

}



/**
  * @brief  This function handles NMI exception.
  * @param  None
  * @retval None
  */
void NMI_Handler(void)
{
}

/**
  * @brief  This function handles Hard Fault exception.
  * @param  None
  * @retval None
  */
void HardFault_Handler(void)
{
  /* Go to infinite loop when Hard Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Memory Manage exception.
  * @param  None
  * @retval None
  */
void MemManage_Handler(void)
{
  /* Go to infinite loop when Memory Manage exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Bus Fault exception.
  * @param  None
  * @retval None
  */
void BusFault_Handler(void)
{
  /* Go to infinite loop when Bus Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Usage Fault exception.
  * @param  None
  * @retval None
  */
void UsageFault_Handler(void)
{
  /* Go to infinite loop when Usage Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles SVCall exception.
  * @param  None
  * @retval None
  */
void SVC_Handler(void)
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

/**
  * @brief  This function handles PendSVC exception.
  * @param  None
  * @retval None
  */
void PendSV_Handler(void)
{
}

/**
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval None
  */
void SysTick_Handler(void)
{
}

/******************************************************************************/
/*                 STM32F4xx Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f4xx.s).                                               */
/******************************************************************************/

/**
  * @brief  This function handles PPP interrupt request.
  * @param  None
  * @retval None
  */
/*void PPP_IRQHandler(void)
{
}*/

/**
  * @}
  */ 


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
