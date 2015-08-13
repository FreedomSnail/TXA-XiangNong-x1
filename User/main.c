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
** 
************************************************************************************************/

int main(void)
{	
	u8 i;
	u8 checksum=0;
	// 局部变量，用于存从flash读到的电压值			 
	__IO uint32_t ADC_ConvertedValueLocal[2]; 
	// 1234567890
	u8 cnt=0;
	u16 Amp;
	u16 AmpSum = 0;
	u16 Vsum = 0;
	u8 Str[]="$LIQUID,48.0V,100A,000*xx\r\n";
	SysTickInit();
	USART_Config(USART1,9600);
	TIM3_Config();
	ADC1_DMA_Init();
	USART_Out(USART1,"Power On!\r\n");
	Device.liquidSpeed = 0;
	TimeMs = 100;
	while(TimeMs>0) {
		i++;
	}
	cnt = 0;
	while(cnt<10) {
		if(TimeMs == 0) {
			TimeMs = 10;
			cnt++;
			ADC_ConvertedValueLocal[0]= RegularConvData_Tab[0];//计算电流
			ADC_ConvertedValueLocal[0] = ADC_ConvertedValueLocal[0]*3300/4096;
			AmpSum += ADC_ConvertedValueLocal[0];
		}
		
	}
	Device.AmpRef = AmpSum/10;
	AmpSum = 0;
	USART_Out(USART1,"v=%d\r\n",Device.AmpRef);
	while(1){
		if(TimeMs == 0) {
			TimeMs = 10;
			
			ADC_ConvertedValueLocal[0]= RegularConvData_Tab[0];//计算电流
			ADC_ConvertedValueLocal[0] = ADC_ConvertedValueLocal[0]*3300/4096;
			//USART_Out(USART1,"v=%d\r\n",ADC_ConvertedValueLocal[0]);
			#if 1 	// 5v供电
			if(ADC_ConvertedValueLocal[0]<Device.AmpRef) {
				Amp = 0;
			} else if(ADC_ConvertedValueLocal[0]<(Device.AmpRef+997)) {	// (2500+150*13.3)/2=2247
				ADC_ConvertedValueLocal[0] = (ADC_ConvertedValueLocal[0]-Device.AmpRef)*2;
				Amp = ADC_ConvertedValueLocal[0]*10/133;
			} else {
				Amp = 150;
			}
			#else
			if(ADC_ConvertedValueLocal[0]<1650) {
				Amp = 0;
			} else if(ADC_ConvertedValueLocal[0]<2967) {	// 1650+150*3.3/5*13.3
				ADC_ConvertedValueLocal[0] -= 1650;
				Amp = ADC_ConvertedValueLocal[0]*500/(33*133);
			} else {
				Amp = 150;
			}
			#endif
			ADC_ConvertedValueLocal[1]= RegularConvData_Tab[1];//计算电压
			ADC_ConvertedValueLocal[1] = ADC_ConvertedValueLocal[1]*33*18/4096;

			cnt++;
			if(cnt<11) {
				AmpSum += Amp;
				Vsum += ADC_ConvertedValueLocal[1];
			} else {
				Amp = AmpSum/10;
				AmpSum = 0;
				ADC_ConvertedValueLocal[1] = Vsum/10;
				Vsum = 0;
				cnt=0;
	
				Str[8] = ADC_ConvertedValueLocal[1]/100 + '0';
				Str[9] = ADC_ConvertedValueLocal[1]/10%10 + '0';
				Str[11] = ADC_ConvertedValueLocal[1]%10 + '0';
			
				Str[14] = Amp/100 + '0';
				Str[15] = Amp/10%10 + '0';
				Str[16] = Amp%10 + '0';
			}
			
			
			

			Str[19] = Device.liquidSpeed/100 + '0';
			Str[20] = Device.liquidSpeed/10%10 + '0';
			Str[21] = Device.liquidSpeed%10 + '0';
			for(i=1;i<=21;i++) {// $和*号不参加校验
				checksum^=Str[i];
			}
			if(((checksum&0xF0)>>4)>9) {	// A~F		
				Str[23]=((checksum&0xF0)>>4)+'A'-10;
			} else {						// 0~9
				Str[23]=((checksum&0xF0)>>4)+'0';
			}		
			if((checksum&0x0F)>9) {
				Str[24]=(checksum&0x0F)+'A'-10;
			} else {
				Str[24]=(checksum&0x0F)+'0';
			}
			USART_Out(USART1,Str);
			//USART_Out(USART1,"This is a test!\r\n");
		}
	}
}
