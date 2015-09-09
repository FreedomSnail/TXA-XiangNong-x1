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
	PackageStatusTypeEnum	PackageStatus;					//数据包接收状态
	u8  RxFlag;
	u8	RxIndex;											//计算接收个数
	u8	RxDataBuf[RX_MAX_NUM];
	
	//u8  RecBuff[100] = {0};	
	//接收命令集 0x02 飞控外发的数据，包含0x00 标准数据包(时间戳、姿态、gps等等数据); 0x01 控制权归属切换;0x02 透传数据（移动设备至机载设备）
	//只接收透传的数据，不接收处理飞控发回的其他数据
	//即只接收 命令码为0x02 透传数据（移动设备至机载设备）
	//因此这个数组最大值只设定为100足够
	//u8  RecvIndex;
}UartTypedef;


extern	UartTypedef		Uart1;


void USART_Config(USART_TypeDef* USARTx,uint32_t baud);
void USART_Out(USART_TypeDef* USARTx, uint8_t *Data,...);
void USART_Send_Buf(USART_TypeDef* USARTx, u8* buf, u16 len);
char *itoa(long value, char *string, int radix);


#endif
