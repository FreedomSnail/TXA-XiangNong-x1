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

/* 通过取消注释或者添加注释的方式控制是否包含底层驱动模块 */


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
#define countof(a) (sizeof(a) / sizeof(*(a)))//计算数组内的成员个数

#define ATOMIZER_PWM_MIN	90
#define ATOMIZER_PWM_MAX	210
#define ADC1_DR_Address                0x40012440

#define RCC_LIQUID_OUT_OF_PORT 	RCC_AHBPeriph_GPIOA 		/* GPIO端口时钟 */
#define LIQUID_OUT_OF_PORT		GPIOA
#define LIQUID_OUT_OF_PIN		GPIO_Pin_5

#define LIQUID_OUT_OF_1()	LIQUID_OUT_OF_PORT->BSRR = LIQUID_OUT_OF_PIN
#define LIQUID_OUT_OF_0()	LIQUID_OUT_OF_PORT->BRR  = LIQUID_OUT_OF_PIN


#define	TIMER1_AUTO_LOAD		50000  //定时器溢出时间
#define	TIMER_CRITICAL_POINT	30000  //定时器溢出发生在上升沿与下降沿采样时间之间

#define	PUMP_OPEN_CRITICAL_POINT	1500

typedef enum {
	PUMP_STATUS_CLOSED,
	PUMP_STATUS_OPEN
} PumpStatusTypeEnum;
typedef enum {
	PWM_OFFLINE,
	PWM_ONLINE
} PWMStatusTypeEnum;

typedef struct {
	PumpStatusTypeEnum PumpStatusFlag;	
	PWMStatusTypeEnum PWMOfflineFlag;
	u16 PWMOffLineCnt;
	u16 PWMRaisingTime;
	u16 PWMFallingTime;
}Device_TYPEDEF;

extern Device_TYPEDEF Device;


extern u32	TimeMs;
extern __IO uint16_t RegularConvData_Tab[];

void  SysTickInit(void);
void GPIO_Config(void);
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
void Open_Pump(void);
void Close_Pump(void);


#endif

