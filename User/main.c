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
		�����룺
		0x0000���ɹ�
		0x0001�������Ƿ�
		0x0002�����ݰ����ܣ�δ����ȷʶ��
		0x0003���µ�APP�����ڼ���
		0x0004��DJI GO û����Ӧ
		0x0005��DJI GO û������
		0x0006���������ܾ�
		0x0007��Ȩ�޼��𲻹�
		0x0008��SDK�汾����
	*/
	cnt = 3;
	while(cnt>0) {
		TimeMs = 200;
		while(TimeMs>0) {
			TimeMs = TimeMs;
			if(Uart1.RxFlag >0 ) {
				Pro_Receive_Interface();//һ֡���ݽ������
				if((DataFromMobile.CommandSet == 0x00)&&(DataFromMobile.CommandId == 0x00)) {
					USART_Out(USART1,"ok\r\n");
					//����ɹ�
					TimeMs = 0;
					cnt = 0;
				} else {
					//����ʧ��
					
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
			Pro_Receive_Interface();//һ֡���ݽ������
			if((DataFromMobile.CommandSet == 0x02)&&(DataFromMobile.CommandId == 0x02)) {	//�ƶ��豸ÿ��500ms��һ����
				Device.LoseRemoteSignalCnt = 70;
				switch(DataFromMobile.data[0]) {
					case '1':	//��ˮ��
						if(Device.LiquidSpeed == 0) {
							Open_Pump();
						}
						break;
					case '0':
						if(Device.LiquidSpeed > 0) {
							Close_Pump();
						}
						break;
					default:
						break;
				}
			}
			Uart1.RxFlag = 0;
		}
		if( Device.LoseRemoteSignalCnt == 0) {
			if(Device.LiquidSpeed > 0) {
				Close_Pump();
			}
		}
	}
}

