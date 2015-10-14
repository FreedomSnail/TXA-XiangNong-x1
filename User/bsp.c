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
** 配置液位感应器输入信号脚
************************************************************************************************/
void GPIO_Config(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_AHBPeriphClockCmd(RCC_LIQUID_OUT_OF_PORT, ENABLE);
	/* PA5 */
	GPIO_InitStructure.GPIO_Pin   = LIQUID_OUT_OF_PIN;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL; 
	GPIO_Init(LIQUID_OUT_OF_PORT, &GPIO_InitStructure);	
#if 0
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);
	/* Pb6,7 */
	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_6|GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL; 
	GPIO_Init(GPIOB, &GPIO_InitStructure);	
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
** 此定时器的定时计数功能提供给检测pwm脉宽用，这里同时初始化了输入pwm信号用到的管脚PB6、7,配置管脚
为输入上升下降沿均触发外部中断
************************************************************************************************/
void TIM1_Config(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	EXTI_InitTypeDef EXTI_InitStructure;

	/* TIM1 clock enable */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);

#if 0
	/* Enable the TIM1 global Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = TIM1_BRK_UP_TRG_COM_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
#endif

	TIM_PrescalerConfig(TIM1,32-1,TIM_PSCReloadMode_Immediate);
	TIM_SetAutoreload(TIM1,TIMER1_AUTO_LOAD);	//定时器溢出周期50ms
  	TIM_ITConfig(TIM1, TIM_IT_Update, ENABLE);
	TIM_Cmd(TIM1, ENABLE);

	
#if 1
		/* GPIOB clock enable */
		RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);//使能时钟  中断用到了 SYSCFG 也要使能SYSCFG
	
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6|GPIO_Pin_7;
		GPIO_InitStructure.GPIO_Mode	= GPIO_Mode_IN;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
		GPIO_InitStructure.GPIO_PuPd	= GPIO_PuPd_UP; 
		GPIO_Init(GPIOB, &GPIO_InitStructure);
	
		/* 连接IO口到中断线 */
		SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOB,EXTI_PinSource6);//|EXTI_PinSource7
		SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOB,EXTI_PinSource7);
		/* 配置中断线6为边降沿触发*/
		EXTI_InitStructure.EXTI_Line = EXTI_Line6|EXTI_Line7;
		EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
		EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
		EXTI_InitStructure.EXTI_LineCmd = ENABLE;
		EXTI_Init(&EXTI_InitStructure);
		
		/* Enable the ext6 global Interrupt */
		NVIC_InitStructure.NVIC_IRQChannel = EXTI4_15_IRQn;
		NVIC_InitStructure.NVIC_IRQChannelPriority = 1;
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
** 此定时器比较输出功能提供给水泵控制用，这里同时初始化了水泵控制相应的管脚
************************************************************************************************/
void TIM3_Config(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	TIM_OCInitTypeDef TIM_OCInitStructure;

	/* TIM3 clock enable */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

	/* GPIOB clock enable */
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);

	/* TIM3 chennel1 configuration : PA6 */
	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_6;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP; 
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	/* Connect TIM pin to AF1 */
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource6, GPIO_AF_1);

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
** 此定时器的输入捕获功能提供给PB6关键检测pwm脉宽用
************************************************************************************************/
void TIM16_Config(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	TIM_OCInitTypeDef TIM_OCInitStructure;

	/* TIM16 clock enable */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM16, ENABLE);

	/* GPIOA clock enable */
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);

	/* TIM16 chennel1N  configuration : PB6 */
	GPIO_InitStructure.GPIO_Pin	= GPIO_Pin_6;
	GPIO_InitStructure.GPIO_Mode	= GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd	= GPIO_PuPd_UP; 
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	/* Connect TIM pin to AF5 */
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource6, GPIO_AF_5);
#if 0
	/* Enable the TIM2 global Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = TIM1_BRK_UP_TRG_COM_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
#endif

	TIM_Cmd(TIM16, ENABLE);
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
__IO uint16_t RegularConvData_Tab[6];
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

  	/* Configure PA.0,1,2  as analog input */
  	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2;
  	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AN;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL ;
  	GPIO_Init(GPIOA, &GPIO_InitStruct);				// PC1,输入时不用设置速率
 
  	/* DMA1 Channel1 Config */
  	DMA_DeInit(DMA1_Channel1);
  	//DMA_InitStruct.DMA_PeripheralBaseAddr = (uint32_t)ADC1_DR_Address;
	DMA_InitStruct.DMA_PeripheralBaseAddr = (uint32_t)&ADC1->DR;
  	DMA_InitStruct.DMA_MemoryBaseAddr = (uint32_t)RegularConvData_Tab;
  	DMA_InitStruct.DMA_DIR = DMA_DIR_PeripheralSRC;
  	DMA_InitStruct.DMA_BufferSize =3;
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
	ADC_ChannelConfig(ADC1, ADC_Channel_2  , ADC_SampleTime_55_5Cycles); 
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
void Open_Pump(void)
{
	TIM_SetCompare1(TIM3,800);
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
void Close_Pump(void)
{
	TIM_SetCompare1(TIM3,0);
}

