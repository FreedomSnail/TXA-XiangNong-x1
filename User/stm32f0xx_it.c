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

uint8_t count = 0;
uint8_t is_initiated = 0;
//$PUMP,100,**

void USART1_IRQHandler(void)
{
	uint8_t Rev;
	u8 checksumRecv,checksumCal;
	u8 parity1;
	u8 parity2;
	u8 i;
	if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)	   //判断读寄存器是否非空
  	{	
    	Rev = USART_ReceiveData(USART1);
		//while(!((USART1->ISR)&(1<<7)));//等待发送完
		//USART1->TDR= Rev;
		switch(Uart1.PackageStatus) {
			case RECV_IDLE:
				if(Rev == '$') {
					Uart1.RxIndex = 0;
					Uart1.PackageStatus = RECV_START;
				}
				break;
			case RECV_START:
				switch(Rev) {
					case '\r':
					case '\n':
						parity1 = Uart1.RxDataBuf[Uart1.RxIndex-2];
						parity2 = Uart1.RxDataBuf[Uart1.RxIndex-1];
						if((parity1>='0')&&(parity1<='9')) {
							parity1-='0';
						} else if((parity1>='A')&&(parity1<='F')) {
							parity1-=55;
						}
						if((parity2>='0')&&(parity2<='9')) {
							parity2-='0';
						} else if((parity2>='A')&&(parity2<='F')) {
							parity2-=55;
						}
						checksumRecv = parity1*16+parity2;
						checksumCal = 0;
						for(i=0;i<Uart1.RxIndex-3;i++) {
							checksumCal ^= Uart1.RxDataBuf[i];
						}
						if(checksumCal == checksumRecv) {	//校验ok
							if((Uart1.RxDataBuf[0]=='P')&&(Uart1.RxDataBuf[1]=='U')&&(Uart1.RxDataBuf[2]=='M')&&(Uart1.RxDataBuf[3]=='P')){
								Device.LiquidSpeed = (Uart1.RxDataBuf[5]-'0')*100+(Uart1.RxDataBuf[6]-'0')*10+(Uart1.RxDataBuf[7]-'0');
								if(Device.LiquidSpeed<100) {
									TIM_SetCompare1(TIM3,Device.LiquidSpeed*10);
								} else {
									TIM_SetCompare1(TIM3,1030);
								}
							} else if((Uart1.RxDataBuf[0]=='A')&&(Uart1.RxDataBuf[1]=='T')&&(Uart1.RxDataBuf[2]=='O')&&(Uart1.RxDataBuf[3]=='M')){
								Device.Atomizer = (Uart1.RxDataBuf[5]-'0')*100+(Uart1.RxDataBuf[6]-'0')*10+(Uart1.RxDataBuf[7]-'0');
								if(Device.Atomizer<=120) {
									Device.Atomizer += 90;	// x10us
									TIM_SetCompare2(TIM1,Device.Atomizer);
									TIM_SetCompare3(TIM1,Device.Atomizer);
								}
							}
						} else {
							if((Uart1.RxDataBuf[0]=='A')&&(Uart1.RxDataBuf[1]=='T')&&(Uart1.RxDataBuf[2]=='O')&&(Uart1.RxDataBuf[3]=='M')){
								Device.Atomizer = (Uart1.RxDataBuf[5]-'0')*100+(Uart1.RxDataBuf[6]-'0')*10+(Uart1.RxDataBuf[7]-'0');
								if(Device.Atomizer<=120) {
									Device.Atomizer += 90;	// x10us
									TIM_SetCompare2(TIM1,Device.Atomizer);
									TIM_SetCompare3(TIM1,Device.Atomizer);
								}
							}
						}
						Uart1.PackageStatus = RECV_IDLE;
						Uart1.RxIndex = 0;
						break;
					default:
						Uart1.RxDataBuf[Uart1.RxIndex] = Rev;
						Uart1.RxIndex++;
						if(Uart1.RxIndex==RX_MAX_NUM) {//防止数组溢出	
							Uart1.RxIndex = 0;
						}
						break;
				}
				break;
			default:
				
				break;
	} 
	
		
	}
	
}
void TIM1_BRK_UP_TRG_COM_IRQHandler(void)
{
	
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
