/****************************************Copyright(c)********************************************
**
**                            2015-2023, Guangzhou TX-Aviation Technology Co.Ltd.
**
**----------------------------------------File Info----------------------------------------------
** File name : main.c
** Latest modified date :
** Latest version :
** Description :
**-----------------------------------------------------------------------------------------------
** Created by : Luzhi
** Created date :2015-8-5 10:51:20
** Version :V1.0
** Description :
**-----------------------------------------------------------------------------------------------
** Modified by :
** Modified date :
** Version :
** Description :
************************************************************************************************/
#include "bsp.h"
/************************************************************************************************
** Function name :			
** Description :
** 
** Input :
** Output :
** Return :
** Others :
** ʹ�ö�ʱ����ʱ10ms�ı���
************************************************************************************************/
void Delay_10ms(u8 time) 
{
	TimeMs = time;
	while(TimeMs>0) {
		TimeMs = TimeMs;
	}
}
/************************************************************************************************
** Function name :			
** Description :
** 
** Input :
** Output :
** Return :
** Others :
** 
APP ID ��            1008902
ͨѶ��Կ      818e4fccacb5d597aa4c006a15b7b031185a49ec3f86aa50a023b00d04146a9c
����   2

************************************************************************************************/
int main(void)
{	
	
	u8 cnt=0;
	u16 AmpSum = 0;
	u16 V12sSum = 0;
	u16 V6sSum = 0;
	
	SysTickInit();
	USART_Config(USART1,9600);
	TIM1_Config();
	TIM3_Config();
	ADC1_DMA_Init();
	USART_Out(USART1,"Power On!\r\n");
	Device.LiquidSpeed = 0;
	Delay_10ms(100);
	Device.AmpRef = Get_Amp_Ref();
	AmpSum = 0;
	USART_Out(USART1,"v=%d\r\n",Device.AmpRef);
	while(1){
		if(TimeMs == 0) {
			TimeMs = 10;
			cnt++;
			if(cnt<10) {
				AmpSum  += Get_Amp_Val();
				V12sSum += Get_12S_Val();
				V6sSum  += Get_6S_Val();
			} else {
				Device.Amp = AmpSum/10;
				Device.V12s = V12sSum/10;
				Device.V6s = V6sSum/10;
				AmpSum  = Get_Amp_Val();
				V12sSum = Get_12S_Val();
				V6sSum  = Get_6S_Val();
				cnt=0;
			}
			Send_Msg_2_M100();
		}
	}
}

