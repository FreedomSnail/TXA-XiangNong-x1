/****************************************Copyright(c)********************************************
**
**                            2015-2023, Guangzhou TX-Aviation Technology Co.Ltd.
**
**----------------------------------------File Info----------------------------------------------
** File name : bsp.c
** Latest modified date :
** Latest version :
** Description :
**-----------------------------------------------------------------------------------------------
** Created by : Luzhi
** Created date :2015-8-5 13:57:14
** Version :V1.0
** Description :
**-----------------------------------------------------------------------------------------------
** Modified by :
** Modified date :
** Version :
** Description :
************************************************************************************************/
#include "bsp.h"

u32	TimeMs;

Device_TYPEDEF Device;

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
void bsp_Init(void)
{
	
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
////SYSTICK 配置， 10ms一次systick中断
void  SysTickInit(void)
{
    RCC_ClocksTypeDef  rcc_clocks;
    uint32_t         cnts;

    RCC_GetClocksFreq(&rcc_clocks);

    cnts = (uint32_t)rcc_clocks.HCLK_Frequency/SYS_TICKS_PER_SEC;

	if(SysTick_Config(cnts))
	{
		while(1);
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
void TIM1_Config(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	TIM_OCInitTypeDef TIM_OCInitStructure;

	/* TIM3 clock enable */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);

	/* GPIOB clock enable */
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);

	/* TIM1 chennel2 configuration : PB0 */
	GPIO_InitStructure.GPIO_Pin	= GPIO_Pin_0;
	GPIO_InitStructure.GPIO_Mode	= GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd	= GPIO_PuPd_UP; 
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	/* Connect TIM pin to AF1 */
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource0, GPIO_AF_2);
#if 1
	/* Enable the TIM2 global Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = TIM1_BRK_UP_TRG_COM_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
#endif

	//TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
 	//TIM_OCInitStructure.TIM_
	TIM_OCInitStructure.TIM_Pulse = 1000;
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	//TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCPolarity_High;
	TIM_OCInitStructure.TIM_OutputNState = TIM_OutputState_Enable;
	TIM_OC3Init(TIM1,&TIM_OCInitStructure);
	
	TIM_SetAutoreload( TIM1,1000);	//PWM总周期时间设置
	
	TIM_SetCompare3(TIM1,500);		//PWM高电平时间设置
  /* Enable the CC2 Interrupt Request */
	TIM_ITConfig(TIM1, TIM_IT_Update, ENABLE);
	TIM_Cmd(TIM1, ENABLE);
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
void TIM3_Config(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	TIM_OCInitTypeDef TIM_OCInitStructure;

	/* TIM3 clock enable */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

	/* GPIOB clock enable */
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);

	/* TIM3 chennel1 configuration : PB4 */
	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_4;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP; 
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	/* Connect TIM pin to AF1 */
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource4, GPIO_AF_1);

	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;
 	//TIM_OCInitStructure.TIM_
  	TIM_OCInitStructure.TIM_Pulse = 30000;
  	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_PrescalerConfig(TIM3,320-1,TIM_PSCReloadMode_Immediate);
	TIM_OC1Init(TIM3,&TIM_OCInitStructure);
	TIM_SetAutoreload(TIM3,1000);	//PWM总周期时间设置
	
	TIM_SetCompare1(TIM3,0);		//PWM高电平时间设置
  	/* Enable the CC2 Interrupt Request */
  	//TIM_ITConfig(TIM3, TIM_IT_CC1, ENABLE);
  	TIM_Cmd(TIM3, ENABLE);	
	#if 0
	/* Enable the TIM3 global Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	#endif
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
void ADC_Config(void)
{
		ADC_InitTypeDef 		 ADC_InitStructure;
		GPIO_InitTypeDef		 GPIO_InitStructure;
	
		//GPIOC Periph clock enable
		RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);
	
		//ADC1 Periph clock enable
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
	
		//Configure ADC Channel1 as analog input
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1 ;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
		GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
		GPIO_Init(GPIOA, &GPIO_InitStructure);
	
		//ADC1 Configuration
		//ADCs DeInit
		ADC_DeInit(ADC1);
	
		//Configure the ADC1 in continous mode withe a resolutuion equal to 12 bits
		ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
		ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
		ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;    
	//	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T3_TRGO;
		ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
		ADC_InitStructure.ADC_ScanDirection = ADC_ScanDirection_Upward;
		ADC_Init(ADC1, &ADC_InitStructure); 
	
		//转换ADC1通道,周期采样时间
		ADC_ChannelConfig(ADC1, ADC_Channel_1 , ADC_SampleTime_28_5Cycles);   
		
		//ADC Calibration-ADC校准
		ADC_GetCalibrationFactor(ADC1);
	
		//Enable the auto delay feature-启用自动延时功能	
		ADC_WaitModeCmd(ADC1, ENABLE); 
	
		//使能ADC1
		ADC_AutoPowerOffCmd(ADC1, ENABLE); 
	
		//Enable ADCperipheral[PerIdx]-启用ADC外设[PerIdx]
		ADC_Cmd(ADC1, ENABLE);	   
	
		//Wait the ADCEN falg-等待ADC1校准完成
		while(!ADC_GetFlagStatus(ADC1, ADC_FLAG_ADEN)); 
	
		//ADC1 regular Software Start Conv
		ADC_StartOfConversion(ADC1);

	
}
#define ADC1_DR_Address                0x40012440
__IO uint16_t RegularConvData_Tab[2];
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
void ADC1_DMA_Init(void)
{
	ADC_InitTypeDef	  ADC_InitStruct;
  	DMA_InitTypeDef	  DMA_InitStruct;
	GPIO_InitTypeDef	GPIO_InitStruct;
	 /* ADC1 DeInit */  
  	ADC_DeInit(ADC1);

	/* Enable  GPIOA clock */
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);
  	/* ADC1 Periph clock enable */
  	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
  	/* DMA1 clock enable */
  	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1 , ENABLE);

  	/* Configure PA.01  as analog input */
  	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1;
  	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AN;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL ;
  	GPIO_Init(GPIOA, &GPIO_InitStruct);				// PC1,输入时不用设置速率

 
  	/* DMA1 Channel1 Config */
  	DMA_DeInit(DMA1_Channel1);
  	//DMA_InitStruct.DMA_PeripheralBaseAddr = (uint32_t)ADC1_DR_Address;
	DMA_InitStruct.DMA_PeripheralBaseAddr = (uint32_t)&ADC1->DR;
  	DMA_InitStruct.DMA_MemoryBaseAddr = (uint32_t)RegularConvData_Tab;
  	DMA_InitStruct.DMA_DIR = DMA_DIR_PeripheralSRC;
  	DMA_InitStruct.DMA_BufferSize =4;
  	DMA_InitStruct.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  	DMA_InitStruct.DMA_MemoryInc = DMA_MemoryInc_Enable;
  	DMA_InitStruct.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
  	DMA_InitStruct.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
  	DMA_InitStruct.DMA_Mode = DMA_Mode_Circular;
  	DMA_InitStruct.DMA_Priority = DMA_Priority_High;
  	DMA_InitStruct.DMA_M2M = DMA_M2M_Disable;
  	DMA_Init(DMA1_Channel1, &DMA_InitStruct);
  
  	/* DMA1 Channel1 enable */
  	DMA_Cmd(DMA1_Channel1, ENABLE);
  
	//	 /* ADC DMA request in circular mode */
  	ADC_DMARequestModeConfig(ADC1, ADC_DMAMode_Circular);
  
  	/* Enable ADC_DMA */
  	ADC_DMACmd(ADC1, ENABLE);  
  
  	/* Initialize ADC structure */
  	ADC_StructInit(&ADC_InitStruct);
  
  	/* Configure the ADC1 in continous mode withe a resolutuion equal to 12 bits	*/
  	ADC_InitStruct.ADC_Resolution = ADC_Resolution_12b;
	ADC_InitStruct.ADC_ContinuousConvMode = ENABLE; 
  	ADC_InitStruct.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
  	ADC_InitStruct.ADC_DataAlign = ADC_DataAlign_Right;
  	ADC_InitStruct.ADC_ScanDirection = ADC_ScanDirection_Backward;
  	ADC_Init(ADC1, &ADC_InitStruct); 
 
//	 /* Convert the ADC1 temperature sensor  with 55.5 Cycles as sampling time */ 
//	 ADC_ChannelConfig(ADC1, ADC_Channel_TempSensor , ADC_SampleTime_55_5Cycles);  
//	 ADC_TempSensorCmd(ENABLE);
  
  	/* Convert the ADC1 Vref	with 55.5 Cycles as sampling time */ 
  	ADC_ChannelConfig(ADC1, ADC_Channel_0  , ADC_SampleTime_55_5Cycles); 
	ADC_ChannelConfig(ADC1, ADC_Channel_1  , ADC_SampleTime_55_5Cycles); 

	//	 ADC_VrefintCmd(ENABLE);
  
  	/* ADC Calibration */
	
  	ADC_GetCalibrationFactor(ADC1);
   	ADC_DMACmd(ADC1, ENABLE);
  	/* Enable ADC1 */
  	ADC_Cmd(ADC1, ENABLE);	 
  
  	/* Wait the ADCEN falg */
  	while(!ADC_GetFlagStatus(ADC1, ADC_FLAG_ADEN)); 
  
  	/* ADC1 regular Software Start Conv */ 
  	ADC_StartOfConversion(ADC1);
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

void PUMP_Output(void)
{
	u8 i;
	u8 checksum=0;
	u8 Str[] = "$PUMP,050,*00xx";
	Str[13] = '\r';	//回车
	Str[14] = '\n'; //换行
	for(i=1;i<=9;i++) {// $和*号不参加校验
		checksum^=Str[i];
	}
	if(((checksum&0xF0)>>4)>9) {	// A~F		
		Str[11]=((checksum&0xF0)>>4)+'A'-10;
	} else {						// 0~9
		Str[11]=((checksum&0xF0)>>4)+'0';
	}		
	if((checksum&0x0F)>9) {
		Str[12]=(checksum&0x0F)+'A'-10;
	} else {
		Str[12]=(checksum&0x0F)+'0';
	}
	USART_Out(USART1,Str);
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
void Liquid_Output(void)
{
	u8 i;
	u8 checksum=0;
	u8 Str[] = "$LIQUID,V050,*00xx";
	Str[13] = '\r';	//回车
	Str[14] = '\n'; //换行
	for(i=1;i<=9;i++) {// $和*号不参加校验
		checksum^=Str[i];
	}
	if(((checksum&0xF0)>>4)>9) {	// A~F		
		Str[11]=((checksum&0xF0)>>4)+'A'-10;
	} else {						// 0~9
		Str[11]=((checksum&0xF0)>>4)+'0';
	}		
	if((checksum&0x0F)>9) {
		Str[12]=(checksum&0x0F)+'A'-10;
	} else {
		Str[12]=(checksum&0x0F)+'0';
	}
	USART_Out(USART1,Str);
}

