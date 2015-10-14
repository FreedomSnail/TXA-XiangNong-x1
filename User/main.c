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
	此版本适应旧飞控硬件的控制:检测输入pwm脉宽，来控制水泵的开关。
	
	电路板硬件版本:
	stm32f03x-VB0.SchDoc
	电池电压电流检测-基于M0-VB0.PcbDoc
	农药机电池到电调接口板-VC0.PcbDoc
	
	pwm输入脚使用PB6-TX ,PB7-RX中的任意一个都可以
	输入的pwm信号特征:周期20ms,高电平时间范围0.9ms~2.1ms
	当pwm的信号小于1.5ms时，关闭水泵，反之开启水泵

	系统时钟32M
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

