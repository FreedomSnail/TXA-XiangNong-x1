#ifndef _BSP_H_
#define _BSP_H_

#include "stm32f0xx.h"
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include  <stdlib.h>
#include  <stdarg.h>




#ifndef TRUE
	#define TRUE  1
#endif

#ifndef FALSE
	#define FALSE 0
#endif

/* ͨ��ȡ��ע�ͻ������ע�͵ķ�ʽ�����Ƿ�����ײ�����ģ�� */


//#include "bsp_spi_flash.h"
//#include "bsp_cpu_flash.h"
//#include "bsp_sdio_sd.h"
//#include "bsp_i2c_gpio.h"
//#include "bsp_eeprom_24xx.h"
//#include "bsp_si4730.h"
//#include "bsp_hmc5883l.h"
//#include "bsp_mpu6050.h"
//#include "bsp_bh1750.h"
//#include "bsp_bmp085.h"
//#include "bsp_wm8978.h"
//#include "bsp_fsmc_sram.h"
//#include "bsp_nand_flash.h"
//#include "bsp_nor_flash.h"
//#include "LCD_RA8875.h"
//#include "LCD_SPFD5420.h"
//#include "bsp_touch.h"
//#include "bsp_camera.h"
//#include "bsp_ad7606.h"
//#include "bsp_gps.h"
//#include "bsp_oled.h"
//#include "bsp_mg323.h"
//#include "bsp_ks103.h"
//#include "bsp_oled12864.h"
#include "bsp_usart.h"
#include "m100_sdk.h"

#define 	SYS_TICKS_PER_SEC       100 
#define countof(a) (sizeof(a) / sizeof(*(a)))//���������ڵĳ�Ա����



typedef struct {
	u16 AmpRef;			//��׼����ֵ
	//u16 AmpInstant;	//����˲��ֵ
	//u16 AmpEverage;	//����ƽ��ֵ
	u16 Amp;
	u16 V12s;
	u16 V6s;
	u16	LiquidSpeed;
	u16 Atomizer;
	u16 AtomizerCurPWM;
	u16 AtomizerTargetPWM;
}
Device_TYPEDEF;

extern Device_TYPEDEF Device;


extern u32	TimeMs;
extern __IO uint16_t RegularConvData_Tab[];

void  SysTickInit(void);
void TIM1_Config(void);
void TIM3_Config(void);
void ADC1_DMA_Init(void);
void PUMP_Output(void);
u16 Get_Amp_Ref(void);
u16 Get_Amp_Val(void);
u16 Get_12S_Val(void);
u16 Get_6S_Val(void);
void Send_Msg_2_M100(void);
void Atomizer_Soft_Start(void);

#endif

