#ifndef _BSP_USART_H
#define _BSP_USART_H

#define		RX_MAX_NUM				40


typedef enum
{
	RECV_IDLE,
	RECV_START
}PackageStatusTypeEnum;

typedef struct 
{
	PackageStatusTypeEnum	PackageStatus;					//���ݰ�����״̬
	u8	RxIndex;											//������ո���
	u8	RxDataBuf[RX_MAX_NUM];
}UartTypedef;


extern	UartTypedef		Uart1;


void USART_Config(USART_TypeDef* USARTx,uint32_t baud);
void USART_Out(USART_TypeDef* USARTx, uint8_t *Data,...);
char *itoa(long value, char *string, int radix);


#endif
