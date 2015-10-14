/**
  ******************************************************************************
  * @file    TIM/PWM_Input/stm32f0xx_it.c 
  * @author  MCD Application Team
  * @version V1.0.0
  * @date    18-May-2012
  * @brief   Main Interrupt Service Routines.
  *          This file provides template for all exceptions handler and 
  *          peripherals interrupt service routine.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2012 STMicroelectronics</center></h2>
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
#include "bsp.h"

#include "stm32f0xx_it.h"

/** @addtogroup STM32F0xx_StdPeriph_Examples
  * @{
  */

/** @addtogroup TIM_PWM_Input
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
__IO uint16_t IC2Value = 10;
__IO uint16_t DutyCycle = 10;
__IO uint32_t Frequency = 10;

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/******************************************************************************/
/*            Cortex-M0 Processor Exceptions Handlers                         */
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
  * @brief  This function handles SVCall exception.
  * @param  None
  * @retval None
  */
void SVC_Handler(void)
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
void ADC1_COMP_IRQHandler(void)  
{   
//	if(ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC) != RESET){   
//		ADC_ClearFlag(ADC1, ADC_FLAG_EOC);
//   }
}
/**
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval None
  */
void SysTick_Handler(void)
{
	if(TimeMs>0) {
		TimeMs--;
	}
}

/******************************************************************************/
/*                 STM32F0xx Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f0xx.s).                                               */
/******************************************************************************/
/**
  * @brief  This function handles TIM2 global interrupt request.
  * @param  None
  * @retval None
  */

void USART1_IRQHandler(void)
{
	u8 Rev;
  	if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET) { 	
		Rev = USART_ReceiveData(USART1);
		//while(!((USART1->ISR)&(1<<7)));//等待发送完
		//USART1->TDR= Rev;
	} 
}
void EXTI4_15_IRQHandler(void)
{
	#if 1
	u16 period; 
	Device.PWMOffLineCnt = 0;
	Device.PWMOfflineFlag = PWM_ONLINE;
	if(EXTI_GetITStatus(EXTI_Line6) != RESET) {
		EXTI_ClearITPendingBit(EXTI_Line6);
		if(GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_6)) {	//上升沿
			LIQUID_OUT_OF_1();
			Device.PWMRaisingTime = TIM_GetCounter(TIM1);
		} else {
			LIQUID_OUT_OF_0();
			Device.PWMFallingTime = TIM_GetCounter(TIM1);
			if( Device.PWMFallingTime>Device.PWMRaisingTime ) {
				period = Device.PWMFallingTime - Device.PWMRaisingTime;
				if(period>TIMER_CRITICAL_POINT) {
					period = TIMER1_AUTO_LOAD - Device.PWMFallingTime + Device.PWMRaisingTime;
				}
			} else {
				period = Device.PWMRaisingTime - Device.PWMFallingTime;
				if(period>TIMER_CRITICAL_POINT) {
					period = TIMER1_AUTO_LOAD - Device.PWMRaisingTime + Device.PWMFallingTime;
				}
			}
			if( period > PUMP_OPEN_CRITICAL_POINT ) {
				if(Device.PumpStatusFlag != PUMP_STATUS_OPEN) {
					Device.PumpStatusFlag = PUMP_STATUS_OPEN;
					Open_Pump();
				}
			} else {
				if(Device.PumpStatusFlag != PUMP_STATUS_CLOSED) {
					Device.PumpStatusFlag = PUMP_STATUS_CLOSED;
					Close_Pump();
				}
			}
		}
		
	} else if(EXTI_GetITStatus(EXTI_Line7) != RESET) {
		EXTI_ClearITPendingBit(EXTI_Line7);
		if(GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_7)) {	//上升沿
			LIQUID_OUT_OF_1();
			Device.PWMRaisingTime = TIM_GetCounter(TIM1);
		} else {
			LIQUID_OUT_OF_0();
			Device.PWMFallingTime = TIM_GetCounter(TIM1);
			if( Device.PWMFallingTime>Device.PWMRaisingTime ) {
				period = Device.PWMFallingTime - Device.PWMRaisingTime;
				if(period>TIMER_CRITICAL_POINT) {
					period = TIMER1_AUTO_LOAD - Device.PWMFallingTime + Device.PWMRaisingTime;
				}
			} else {
				period = Device.PWMRaisingTime - Device.PWMFallingTime;
				if(period>TIMER_CRITICAL_POINT) {
					period = TIMER1_AUTO_LOAD - Device.PWMRaisingTime + Device.PWMFallingTime;
				}
			}
			if( period > PUMP_OPEN_CRITICAL_POINT ) {
				if(Device.PumpStatusFlag != PUMP_STATUS_OPEN) {
					Device.PumpStatusFlag = PUMP_STATUS_OPEN;
					Open_Pump();
				}
			} else {
				if(Device.PumpStatusFlag != PUMP_STATUS_CLOSED) {
					Device.PumpStatusFlag = PUMP_STATUS_CLOSED;
					Close_Pump();
				}
			}
		}
	}
	#endif

}
void TIM1_BRK_UP_TRG_COM_IRQHandler(void)
{
	static u8 c;
	if(TIM_GetITStatus(TIM1, TIM_IT_Update) != RESET) {
		TIM_ClearITPendingBit(TIM1, TIM_IT_Update);
		c++;
		if(c%2==0) {
			LIQUID_OUT_OF_0();
		} else {
			LIQUID_OUT_OF_1();
		}
	}
}
void TIM3_IRQHandler(void)
{

}

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

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
