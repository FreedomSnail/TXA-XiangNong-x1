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
** Created date :2015-10-9 13:57:02
** Version :V1.0
** Description :
**-----------------------------------------------------------------------------------------------
** Modified by :
** Modified date :
** Version :
** Description :
	�˰汾��Ӧ�ɷɿ�Ӳ���Ŀ���:�������pwm����������ˮ�õĿ��ء�
	
	��·��Ӳ���汾:
	stm32f03x-VB0.SchDoc
	��ص�ѹ�������-����M0-VB0.PcbDoc
	ũҩ����ص�����ӿڰ�-VC0.PcbDoc
	
	pwm�����ʹ��PB6-TX ,PB7-RX�е�����һ��������
	�����pwm�ź�����:����20ms,�ߵ�ƽʱ�䷶Χ0.9ms~2.1ms
	��pwm���ź�С��1.5msʱ���ر�ˮ�ã���֮����ˮ��

	ϵͳʱ��32M
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
************************************************************************************************/
int main(void)
{	
	SysTickInit();
	GPIO_Config();
	TIM1_Config();
	TIM3_Config();
	//ADC1_DMA_Init();
	while(1){
		if(TimeMs == 0) {
			TimeMs = 10;	
			Device.PWMOffLineCnt++;
			if(Device.PWMOffLineCnt>2) {
				Device.PWMOfflineFlag = PWM_OFFLINE;
				if(Device.PumpStatusFlag != PUMP_STATUS_CLOSED) {
					Device.PumpStatusFlag = PUMP_STATUS_CLOSED;
					Close_Pump();
					
				}
			}
		} 
	}
}

