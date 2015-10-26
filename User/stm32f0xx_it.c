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
void SysTick_Handler(void)	// 10ms执行一次这个中断
{
	if(TimeMs>0) {
		TimeMs--;
	}
	if( Device.LoseRemoteSignalCnt > 0) {
		Device.LoseRemoteSignalCnt--;
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
#if 1
void USART1_IRQHandler(void)
{
	u8 Rev;
	static u16 dataLen;
  	if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET) { 	
		Rev = USART_ReceiveData(USART1);
		//while(!((USART1->ISR)&(1<<7)));//等待发送完
		//USART1->TDR= Rev;
		#if 1
		switch(Uart1.PackageStatus) {
			case RECV_IDLE:
				if(Rev == _SDK_SOF) {
					Uart1.RxIndex = 1;
					Uart1.PackageStatus = RECV_COMMAND_SET;
					memset(Uart1.RxDataBuf,0,RX_MAX_NUM);
					Uart1.RxDataBuf[0] = _SDK_SOF;
				}
				break;
			case RECV_COMMAND_SET:
				Uart1.RxDataBuf[Uart1.RxIndex] = Rev;
				if(Uart1.RxIndex == sizeof(SDKHeader)) {
					Uart1.PackageStatus = RECV_COMMAND_ID;
					#if 0
					if(Rev == 0x00) {	//命令集-激活验证类 -机载设备==>飞控
						Uart1.PackageStatus = RECV_COMMAND_ID;
					} else if(Rev == 0x02) { //命令集-推送数据类 - 飞控==>机载设备
						Uart1.PackageStatus = RECV_COMMAND_ID;
					} else {
						Uart1.PackageStatus = RECV_IDLE;
					}
					#endif
				}
				Uart1.RxIndex++;
				break;
			case RECV_COMMAND_ID:
				Uart1.RxDataBuf[Uart1.RxIndex++] = Rev;
				dataLen = ((unsigned int)(0x03&&Uart1.RxDataBuf[2])<<8)+(unsigned int)Uart1.RxDataBuf[1];
				Uart1.PackageStatus = RECV_WAIT_DONE;

				#if 0
				Uart1.RxDataBuf[Uart1.RxIndex++] = Rev;
				if(Rev == 0x02) {
					dataLen = ((unsigned int)(0x03&&Uart1.RxDataBuf[2])<<8)+(unsigned int)Uart1.RxDataBuf[1];
					Uart1.PackageStatus = RECV_WAIT_DONE;
				} else {
					//if(Uart1.RxDataBuf[Uart1.RxIndex-1] == 0x00) {

					//} else {
						Uart1.PackageStatus = RECV_IDLE;
					//}
				}
				#endif
				break;
			case RECV_WAIT_DONE:
				if(Uart1.RxIndex<RX_MAX_NUM) {
					Uart1.RxDataBuf[Uart1.RxIndex++] = Rev;
					if(Uart1.RxIndex == dataLen) {
						Uart1.RxFlag = 1;
						//Pro_Receive_Interface();//一帧数据接收完成
						//USART_Send_Buf(USART1,Uart1.RxDataBuf,dataLen);
						//USART_Out(USART1,"!");
						Uart1.PackageStatus = RECV_IDLE;
					}
				} else {//接收到的数据致使数组越界
					Uart1.PackageStatus = RECV_IDLE;
				}
				break;
			default:
				break;
		}
		#endif
	} 
}
#else
void USART1_IRQHandler(void)
{
	uint8_t Rev;
	u8 checksumRecv,checksumCal;
	u8 parity1;
	u8 parity2;
	u8 i;
	u16 temp;
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
								temp = (Uart1.RxDataBuf[5]-'0')*100+(Uart1.RxDataBuf[6]-'0')*10+(Uart1.RxDataBuf[7]-'0');
								if(temp<50) {	
									Device.LiquidSpeed = 0;
									TIM_SetCompare1(TIM3,Device.LiquidSpeed*10);
									TIM_SetCompare2(TIM1,90);//关闭
									TIM_SetCompare3(TIM1,90);
								} else if(temp < 100) {
									Device.LiquidSpeed = temp;
									TIM_SetCompare1(TIM3,Device.LiquidSpeed*10);
									TIM_SetCompare2(TIM1,210);//关闭
									TIM_SetCompare3(TIM1,210);
								} else {
									TIM_SetCompare1(TIM3,1030);
									TIM_SetCompare2(TIM1,210);//关闭
									TIM_SetCompare3(TIM1,210);
								}
							} else if((Uart1.RxDataBuf[0]=='A')&&(Uart1.RxDataBuf[1]=='T')&&(Uart1.RxDataBuf[2]=='O')&&(Uart1.RxDataBuf[3]=='M')){
								temp = (Uart1.RxDataBuf[5]-'0')*100+(Uart1.RxDataBuf[6]-'0')*10+(Uart1.RxDataBuf[7]-'0');
								if(temp<=100) {
									Device.Atomizer = temp;
									temp = temp*6/5 + 90; // x10us
									TIM_SetCompare2(TIM1,temp);
									TIM_SetCompare3(TIM1,temp);
								}
							}
						} 
						#if 1
						else {
							if((Uart1.RxDataBuf[0]=='P')&&(Uart1.RxDataBuf[1]=='U')&&(Uart1.RxDataBuf[2]=='M')&&(Uart1.RxDataBuf[3]=='P')){
								temp = (Uart1.RxDataBuf[5]-'0')*100+(Uart1.RxDataBuf[6]-'0')*10+(Uart1.RxDataBuf[7]-'0');
								while(!((USART1->ISR)&(1<<7)));//等待发送完
								USART1->TDR= '*';
								if(temp<50) {
									Device.LiquidSpeed = 0;
									Device.AtomizerTargetPWM = 90;
									TIM_SetCompare1(TIM3,Device.LiquidSpeed*10);
									//TIM_SetCompare2(TIM1,90);//关闭
									//TIM_SetCompare3(TIM1,90);
								} else if(temp < 100) {
									Device.LiquidSpeed = temp;
									Device.AtomizerTargetPWM = 210;
									TIM_SetCompare1(TIM3,Device.LiquidSpeed*10);
									//TIM_SetCompare2(TIM1,210);//关闭
									//TIM_SetCompare3(TIM1,210);
								} else {
									Device.AtomizerTargetPWM = 210;
									TIM_SetCompare1(TIM3,1030);
									//TIM_SetCompare2(TIM1,210);//关闭
									//TIM_SetCompare3(TIM1,210);
								}
							}
						}
						#endif
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
#endif
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
