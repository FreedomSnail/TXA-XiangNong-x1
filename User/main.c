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
** 使用定时器延时10ms的倍数
************************************************************************************************/
void Delay_10ms(u16 time) 
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
APP ID ：            1008902
通讯密钥      818e4fccacb5d597aa4c006a15b7b031185a49ec3f86aa50a023b00d04146a9c
级别   2

************************************************************************************************/
int main(void)
{	
	
	u8 cnt=0;
	u16 AmpSum = 0;
	u16 V12sSum = 0;
	u16 V6sSum = 0;
	
	
	SysTickInit();
	USART_Config(USART1,115200);
	//USART_Config(USART1,230400);
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
	//while(1) {
		DJI_Onboard_API_Activation();
	//	Delay_10ms(300);
	//}
	#if 1
	/*
		返回码：
		0x0000：成功
		0x0001：参数非法
		0x0002：数据包加密，未能正确识别
		0x0003：新的APP，正在激活
		0x0004：DJI GO 没有响应
		0x0005：DJI GO 没有联网
		0x0006：服务器拒绝
		0x0007：权限级别不够
		0x0008：SDK版本错误
	*/
	cnt = 3;
	while(cnt>0) {
		TimeMs = 200;
		while(TimeMs>0) {
			TimeMs = TimeMs;
			if(Uart1.RxFlag >0 ) {
				Pro_Receive_Interface();//一帧数据接收完成
				if((DataFromMobile.CommandSet == 0x00)&&(DataFromMobile.CommandId == 0x00)) {
					USART_Out(USART1,"ok\r\n");
					//激活成功
					TimeMs = 0;
					cnt = 0;
				} else {
					//激活失败
					
					//Delay_10ms(30);
					USART_Out(USART1,"error\r\n");
				}
				Uart1.RxFlag = 0;
			}
		}
		
		DJI_Onboard_API_Activation();
		
	}
	#endif
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
			//USART_Out(USART1,"123\r\n");
			Send_Msg_2_M100();
			Atomizer_Soft_Start();
		}
		if(Uart1.RxFlag >0 ) {
			//USART_Out(USART1,"\r\n");
			Pro_Receive_Interface();//一帧数据接收完成
			if((DataFromMobile.CommandSet == 0x02)&&(DataFromMobile.CommandId == 0x02)) {
				switch(DataFromMobile.data[0]) {
					case '1':	//打开水泵
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

