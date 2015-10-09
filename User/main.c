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
	USART_Config(USART1,115200);
	TIM1_Config();
	TIM3_Config();
	ADC1_DMA_Init();
	//USART_Out(USART1,"Power On!\r\n");
	Device.LiquidSpeed = 0;
	Device.AtomizerCurPWM = Device.AtomizerTargetPWM = 90;
	Delay_10ms(10);
	Device.AmpRef = Get_Amp_Ref();
	AmpSum = 0;
	//USART_Out(USART1,"v=%d\r\n",Device.AmpRef);
	DJI_Pro_Test_Setup();
	//USART_Out(USART1,"*\r\n");
	DJI_Onboard_API_Activation();
	Delay_10ms(200);
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
			Atomizer_Soft_Start();
		}
		if(Uart1.RxFlag >0 ) {
			//USART_Out(USART1,"\r\n");
			Pro_Receive_Interface();//һ֡���ݽ������
			if((DataFromMobile.CommandSet == 0x02)&&(DataFromMobile.CommandId == 0x02)) {
				switch(DataFromMobile.data[0]) {
					case '1':	//��ˮ��
						Open_Pump();
						break;
					case '0':
						Close_Pump();
						break;
					default:
						break;
				}
			}
			Uart1.RxFlag = 0;
		}
	}
}

