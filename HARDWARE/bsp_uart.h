/*
*********************************************************************************************************
*	                                  
*	模块名称 : 串口驱动模块    
*	文件名称 : bsp_uart.h
*	版    本 : V1.0
*	说    明 : 头文件
*
*	Copyright (C), 2012-2013, 安富莱电子 www.armfly.com
*
*********************************************************************************************************
*/

#ifndef __BSP_UART_H
#define __BSP_UART_H

#define	RX_MAX_NUM				200

#define	SERIAL_PORT_PUMP_BOARD		UART4
#define	SERIAL_PORT_DJI_SDK		USART3
#define	SERIAL_PORT_DEBUG		USART2
#define DEBUG_ENABLE	
#ifdef 	DEBUG_ENABLE
#define LOG_DJI_STR(s) 				printf(s)
#define LOG_DJI_VALUE(s,b) 			printf(s,b)
#else 				
#define LOG_DJI_STR(s) 			
#define LOG_DJI_VALUE(s,b) 		
#endif

typedef enum
{
	DJI_PACKAGE_RECV_IDLE,
	DJI_PACKAGE_RECV_START,
	DJI_PACKAGE_RECV_WAIT_DONE
	
}DjiSDKPackageStatus_TypeEnum;

typedef struct 
{
	DjiSDKPackageStatus_TypeEnum	DjiPackageStatus;					//数据包接收状态
	u8	RxIndex;														//计算接收个数
	u16 DataLen;
	u8	RxDataBuf[RX_MAX_NUM];
	u8	RxDataSecondBuf[RX_MAX_NUM];
	
	//u8  RecBuff[100] = {0};	
	//接收命令集 0x02 飞控外发的数据，包含0x00 标准数据包(时间戳、姿态、gps等等数据); 0x01 控制权归属切换;0x02 透传数据（移动设备至机载设备）
	//只接收透传的数据，不接收处理飞控发回的其他数据
	//即只接收 命令码为0x02 透传数据（移动设备至机载设备）
	//因此这个数组最大值只设定为100足够
	//u8  RecvIndex;
}UartTypedef;


extern	UartTypedef		UartDJI;


void bsp_InitUart(void);

void USART1_Config(USART_TypeDef* USARTx,u32 baud);

void USART2_Config(USART_TypeDef* USARTx,u32 baud);

void USART3_Config(USART_TypeDef* USARTx,u32 baud);

void UART4_Config(USART_TypeDef* USARTx,u32 baud);


void USART6_Config(USART_TypeDef* USARTx,u32 baud);


void USART_OUT(USART_TypeDef* USARTx, uint8_t *Data,...);

void USART_Send_Buf(USART_TypeDef* USARTx, u8* buf, u16 len);

void NVIC_Configuration(void);


#endif


