/****************************************Copyright(c)********************************************
**
**                            2015-2023, Guangzhou TX-Aviation Technology Co.Ltd.
**
**----------------------------------------File Info----------------------------------------------
** File name : bsp_usart.c
** Latest modified date :
** Latest version :
** Description :
**-----------------------------------------------------------------------------------------------
** Created by : Luzhi
** Created date :2015-8-5 13:19:08
** Version :V1.0
** Description :
**-----------------------------------------------------------------------------------------------
** Modified by :
** Modified date :
** Version :
** Description :
************************************************************************************************/
#include "bsp.h"

UartTypedef		Uart1;

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
void USART_Config(USART_TypeDef* USARTx,uint32_t baud)
{
	GPIO_InitTypeDef  GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef  NVIC_InitStructure;
        
        
	#if 1
	RCC_AHBPeriphClockCmd( RCC_AHBPeriph_GPIOB, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE );
					
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource6,GPIO_AF_0);
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource7,GPIO_AF_0);		  
	/*
	*  USART1_TX -> PA9 , USART1_RX ->		  PA10
	*/								  
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6|GPIO_Pin_7;				 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF; 
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 
	GPIO_Init(GPIOB, &GPIO_InitStructure);	
    #else
	RCC_AHBPeriphClockCmd( RCC_AHBPeriph_GPIOA, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE );
			
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource9,GPIO_AF_1);
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource10,GPIO_AF_1); 	   
	/*
	*  USART1_TX -> PA9 , USART1_RX ->		  PA10
	*/								  
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9|GPIO_Pin_10;				  
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF; 
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 
	GPIO_Init(GPIOA, &GPIO_InitStructure);	 
	#endif
			
	USART_InitStructure.USART_BaudRate = baud;//设置串口波特率
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//设置数据位
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//设置停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;//设置效验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//设置流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;//设置工作模式
	USART_Init(USARTx, &USART_InitStructure); //配置入结构体
	USART_ITConfig(USARTx,USART_IT_RXNE,ENABLE);
	USART_Cmd(USARTx, ENABLE);//使能串口1
			
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPriority = 0x01;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);	
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
		格式化串口输出函数
        "\r"	回车符	   USART_Out(USART1, "abcdefg\r")   
		"\n"	换行符	   USART_Out(USART1, "abcdefg\r\n")
		"%s"	字符串	   USART_Out(USART1, "字符串是：%s","abcdefg")
		"%d"	十进制	   USART_Out(USART1, "a=%d",10)

************************************************************************************************/
void USART_Out(USART_TypeDef* USARTx, uint8_t *Data,...)
{ 

	const char *s;
    int d;
   
    char buf[16];
    va_list ap;
    va_start(ap, Data);

	while(*Data!=0){				                          //判断是否到达字符串结束符
		if(*Data==0x5c){									  //'\'
			switch (*++Data){
				case 'r':							          //回车符
					USART_SendData(USARTx, 0x0d);	   

					Data++;
					break;
				case 'n':							          //换行符
					USART_SendData(USARTx, 0x0a);	
					Data++;
					break;
				default:
					Data++;
				    break;
			}
		}
		else if(*Data=='%'){									  //
			switch (*++Data){				
				case 's':										  //字符串
                	s = va_arg(ap, const char *);
                	for ( ; *s; s++) {
                    	USART_SendData(USARTx,*s);
						while(USART_GetFlagStatus(USARTx, USART_FLAG_TC)==RESET);
                	}
					Data++;
                	break;
            	case 'd':										  //十进制
                	d = va_arg(ap, int);
                	itoa(d, buf, 10);
                	for (s = buf; *s; s++) {
                    	USART_SendData(USARTx,*s);
						while(USART_GetFlagStatus(USARTx, USART_FLAG_TC)==RESET);
                	}
					Data++;
                	break;
				default:
					Data++;
				    break;
			}		 
		}
		else USART_SendData(USARTx, *Data++);
		while(USART_GetFlagStatus(USARTx, USART_FLAG_TC)==RESET);
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
		整形数据转字符串函数
        char *itoa(int value, char *string, int radix)
		radix=10 标示是10进制	非十进制，转换结果为0;  

	    例：d=-379;
		执行	itoa(d, buf, 10); 后
		
		buf="-379"	

************************************************************************************************/
char *itoa(long value, char *string, int radix)
{
    long     i, d;
    int     flag = 0;
    char    *ptr = string;

    /* This implementation only works for decimal numbers. */
    if (radix != 10)
    {
        *ptr = 0;
        return string;
    }

    if (!value)
    {
        *ptr++ = 0x30;
        *ptr = 0;
        return string;
    }

    /* if this is a negative value insert the minus sign. */
    if (value < 0)
    {
        *ptr++ = '-';

        /* Make the value positive. */
        value *= -1;
    }

    for (i = 1000000000; i > 0; i /= 10)
    {
        d = value / i;

        if (d || flag)
        {
            *ptr++ = (char)(d + 0x30);
            value -= (d * i);
            flag = 1;
        }
    }

    /* Null terminate the string. */
    *ptr = 0;

    return string;

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
void USART_Send_Buf(USART_TypeDef* USARTx, u8* buf, u16 len)
{ 
	if(len<1) {
		return;
	}
	while(len--){
		USART_SendData(USARTx, *buf++);
		while(USART_GetFlagStatus(USARTx, USART_FLAG_TC)==RESET);
	}
}

