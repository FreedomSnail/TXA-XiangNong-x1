#ifndef _BSP_USART_H
#define _BSP_USART_H

#define		RX_MAX_NUM				100


typedef enum
{
	RECV_IDLE,
	RECV_START,
	RECV_COMMAND_SET,
	RECV_COMMAND_ID,
	RECV_WAIT_DONE
	
}PackageStatusTypeEnum;

typedef struct 
{
	PackageStatusTypeEnum	PackageStatus;					//���ݰ�����״̬
	u8  RxFlag;
	u8	RxIndex;											//������ո���
	u8	RxDataBuf[RX_MAX_NUM];
	
	//u8  RecBuff[100] = {0};	
	//������� 0x02 �ɿ��ⷢ�����ݣ�����0x00 ��׼���ݰ�(ʱ�������̬��gps�ȵ�����); 0x01 ����Ȩ�����л�;0x02 ͸�����ݣ��ƶ��豸�������豸��
	//ֻ����͸�������ݣ������մ���ɿط��ص���������
	//��ֻ���� ������Ϊ0x02 ͸�����ݣ��ƶ��豸�������豸��
	//�������������ֵֻ�趨Ϊ100�㹻
	//u8  RecvIndex;
}UartTypedef;


extern	UartTypedef		Uart1;


void USART_Config(USART_TypeDef* USARTx,uint32_t baud);
void USART_Out(USART_TypeDef* USARTx, uint8_t *Data,...);
void USART_Send_Buf(USART_TypeDef* USARTx, u8* buf, u16 len);
char *itoa(long value, char *string, int radix);


#endif
