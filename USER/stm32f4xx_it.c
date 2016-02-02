/**
  ******************************************************************************
  * @file    Project/STM32F4xx_StdPeriph_Template/stm32f4xx_it.c 
  * @author  MCD Application Team
  * @version V1.0.1
  * @date    13-April-2012
  * @brief   Main Interrupt Service Routines.
  *          This file provides template for all exceptions handler and 
  *          peripherals interrupt service routine.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2012 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_it.h"
#include "bsp_uart.h"
#include "includes.h"

#include "led.h"


#include "DJI_Pro_Codec.h"
#include "DJI_Pro_Link.h"
#include "DJI_Pro_App.h"



//#include "DJI_Pro_Codec.h"



/** @addtogroup Template_Project
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/******************************************************************************/
/*            Cortex-M4 Processor Exceptions Handlers                         */
/******************************************************************************/

/**
  * @brief   This function handles NMI exception.
  * @param  None
  * @retval None
  */
void NMI_Handler(void)
{
}

/**
  * @brief  This function handles Hard Fault exception.
  * @param  None
  * @retval None
  */
void HardFault_Handler(void)
{
  /* Go to infinite loop when Hard Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Memory Manage exception.
  * @param  None
  * @retval None
  */
void MemManage_Handler(void)
{
  /* Go to infinite loop when Memory Manage exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Bus Fault exception.
  * @param  None
  * @retval None
  */
void BusFault_Handler(void)
{
  /* Go to infinite loop when Bus Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Usage Fault exception.
  * @param  None
  * @retval None
  */
void UsageFault_Handler(void)
{
  /* Go to infinite loop when Usage Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles SVCall exception.
  * @param  None
  * @retval None
  */
void SVC_Handler(void)
{
}

/**
  * @brief  This function handles Debug Monitor exception.
  * @param  None
  * @retval None
  */
void DebugMon_Handler(void)
{
}

/**
  * @brief  This function handles PendSVC exception.
  * @param  None
  * @retval None
  */
//void PendSV_Handler(void)
//{
//}

/*
*********************************************************************************************************
*	函 数 名: SDIO_IRQHandler
*	功能说明: This function handles WWDG interrupt request.
*	形    参：无
*	返 回 值: 无
*********************************************************************************************************
*/
void SDIO_IRQHandler(void)
{
  	#if SYSTEM_SUPPORT_UCOS  //使用UCOS操作系统
	OSIntEnter();    
	#endif


	/* 在os_core.c文件里定义,如果有更高优先级的任务就绪了,则执行一次任务切换 */
	#if SYSTEM_SUPPORT_UCOS  
	OSIntExit();    	//退出中断
	#endif
}

/*
*********************************************************************************************************
*	函 数 名: SD_SDIO_DMA_IRQHANDLER
*	功能说明: This function handles WWDG interrupt request.
*	形    参：无
*	返 回 值: 无
*********************************************************************************************************
*/
void SD_SDIO_DMA_IRQHANDLER(void)
{
  	#if SYSTEM_SUPPORT_UCOS  //使用UCOS操作系统
	OSIntEnter();    
	#endif

	/* 在os_core.c文件里定义,如果有更高优先级的任务就绪了,则执行一次任务切换 */
	#if SYSTEM_SUPPORT_UCOS  
	OSIntExit();    	//退出中断
	#endif
}

/*******************************************************************************
* Function Name  : USART2_IRQHandler
* Description    : This function handles USART2 global interrupt request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void USART2_IRQHandler(void)
{
	u8 Rev;
	#if SYSTEM_SUPPORT_UCOS  //使用UCOS操作系统
	OSIntEnter();    
	#endif
	//用户程序..
	if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET) {	//判断读寄存器是否非空	
    	// Read one byte from the receive data register
    	Rev= USART_ReceiveData(USART2);   //将读寄存器的数据缓存到接收缓冲区里
    	Rev = Rev;
		//USART_SendData(USART2,Rev);
		//while(USART_GetFlagStatus(USART2, USART_FLAG_TC)==RESET);
  	}
	//用户程序..
	#if SYSTEM_SUPPORT_UCOS  
	OSIntExit();    	//退出中断
	#endif
}

/*******************************************************************************
* Function Name  : USART2_IRQHandler
* Description    : This function handles USART2 global interrupt request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void USART3_IRQHandler(void)
{
	OS_ERR err;
	u8 Rev;
	static u8 DataLen;
	static DjiSDKPackageStatus_TypeEnum DjiPackageStatus = DJI_PACKAGE_RECV_IDLE;
	#if SYSTEM_SUPPORT_UCOS  //使用UCOS操作系统
	OSIntEnter();    
	#endif
	//用户程序..
	if(USART_GetITStatus(USART3, USART_IT_RXNE) != RESET) {	//判断读寄存器是否非空	
    	// Read one byte from the receive data register
    	Rev= USART_ReceiveData(USART3);   //将读寄存器的数据缓存到接收缓冲区里
		//USART_SendData(USART2,Rev);
		//while(USART_GetFlagStatus(USART2, USART_FLAG_TC)==RESET);
		//BSP_OS_SemPost(&SEM_SYNCH);
		switch(DjiPackageStatus) {
			case DJI_PACKAGE_RECV_IDLE:
				if(Rev == _SDK_SOF) {
			 		DjiPackageStatus = DJI_PACKAGE_RECV_START;
			 		serial_sdk.recv_index = 0;
			 		sdk_stream_store_data(&serial_sdk, _SDK_SOF);
		 		}
		 		break;
			case DJI_PACKAGE_RECV_START:
		 		sdk_stream_store_data(&serial_sdk, Rev);
		 		if(serial_sdk.recv_index == 2) {	//收到0xAA紧跟的两个字符
			 		DataLen = ((unsigned int)(0x03&&serial_sdk.comm_recv_buf[2])<<8)+(unsigned int)serial_sdk.comm_recv_buf[1];
					DjiPackageStatus = DJI_PACKAGE_RECV_WAIT_DONE;
		 		}
	 			break;
			case DJI_PACKAGE_RECV_WAIT_DONE:
	 			if(serial_sdk.recv_index < _SDK_MAX_RECV_SIZE) {
		 			sdk_stream_store_data(&serial_sdk, Rev);
		 			if(serial_sdk.recv_index == DataLen) {
			 				OSSemPost(&SemDjiCodec,OS_OPT_POST_1,&err);
			 				LED_G= ~LED_G;
		 				DjiPackageStatus = DJI_PACKAGE_RECV_IDLE;
		 			}
	 			} else {//接收到的数据致使数组越界
		 			DjiPackageStatus = DJI_PACKAGE_RECV_IDLE;
	 			}
	 			break;
			default:
		 		break;
			
		}
  	}
	//用户程序..
	#if SYSTEM_SUPPORT_UCOS  
	OSIntExit();    	//退出中断
	#endif
}

/*******************************************************************************
* Function Name  : USART2_IRQHandler
* Description    : This function handles USART2 global interrupt request.
* Input          : None
* Output         : None
* Return         : None
	与水泵控制板的串口数据通讯协议说明(借鉴djisdk协议)

协议帧
	|<帧头段>|<-帧数据段->|<--帧尾段-->|
	|SOF |LEN|    DATA    |    CRC32   |
帧结构
字段	索引（byte）	大小（bit）		说明
SOF			0				8			帧起始标识，固定为0xAA
LEN			1				8			帧长度标识
DATA		2			长度不定		帧数据段
CRC32	大小不定			32			整个帧的 CRC32 校验值

数据帧
|<-------帧数据段------->|
|CMD SET|CMD ID|CMD VALUE|
命令集 0x00 命令码 0xFE 透传数据（飞控板至水泵控制板）
CMD VALUE由[水泵开关状态8bit]+[水泵电压32bit]组成


命令集 0x02 命令码 0x02 透传数据（水泵控制板至飞控板）
CMD VALUE由[水泵开关状态8bit]+[水泵电压32bit]+[供电电压32bit]+[农药量状态8bit]+[机身编码64bit]+[授权状态8bit]组成


*******************************************************************************/
void UART4_IRQHandler(void)
{
	OS_ERR err;
	u8 Rev;
	static DjiSDKPackageStatus_TypeEnum DjiPackageStatus = DJI_PACKAGE_RECV_IDLE;

	#if SYSTEM_SUPPORT_UCOS  //使用UCOS操作系统
	OSIntEnter();    
	#endif
	//用户程序..
	if(USART_GetITStatus(UART4, USART_IT_RXNE) != RESET) {	//判断读寄存器是否非空	
    	// Read one byte from the receive data register
    	Rev= USART_ReceiveData(UART4);   //将读寄存器的数据缓存到接收缓冲区里
		//USART_SendData(UART4,Rev);while(USART_GetFlagStatus(UART4, USART_FLAG_TC)==RESET);
		switch(DjiPackageStatus) {
			case DJI_PACKAGE_RECV_IDLE:
				if(Rev == _SDK_SOF) {
			 		DjiPackageStatus = DJI_PACKAGE_RECV_START;
			 		UartPumpCtrl.RxIndex = 0;
			 		UartPumpCtrl.RxDataBuf[UartPumpCtrl.RxIndex] = _SDK_SOF;
		 		}
		 		break;
			case DJI_PACKAGE_RECV_START://收到0xAA紧跟的一个字符
				UartPumpCtrl.RxIndex++;
		 		UartPumpCtrl.RxDataBuf[UartPumpCtrl.RxIndex] = Rev;
		 		UartPumpCtrl.DataLen = Rev;
		 		//固定只接收水泵控制板上传的电压，开关状态等信息
		 		if(DATA_LENGTH_RECEIVE_PUMP_CONTROL_BOARD == UartPumpCtrl.DataLen) {
					DjiPackageStatus = DJI_PACKAGE_RECV_WAIT_DONE;
		 		} else {
					DjiPackageStatus = DJI_PACKAGE_RECV_IDLE;
		 		}
		 		
		 		break;
			case DJI_PACKAGE_RECV_WAIT_DONE:
				UartPumpCtrl.RxIndex++;
	 			if(UartPumpCtrl.RxIndex < RX_MAX_NUM) {
		 			UartPumpCtrl.RxDataBuf[UartPumpCtrl.RxIndex] = Rev;
		 			if(UartPumpCtrl.RxIndex == UartPumpCtrl.DataLen) {
			 			OSSemPost(&SemCtrlPump,OS_OPT_POST_1,&err);
		 				DjiPackageStatus = DJI_PACKAGE_RECV_IDLE;
		 			}
	 			} else {//接收到的数据致使数组越界
		 			DjiPackageStatus = DJI_PACKAGE_RECV_IDLE;
	 			}
	 			break;
			default:
		 		break;
			
		}
		
  	}
	//用户程序..
	#if SYSTEM_SUPPORT_UCOS  
	OSIntExit();    	//退出中断
	#endif
}

/**
  * @brief  This function handles PPP interrupt request.
  * @param  None
  * @retval None
  */
/*void PPP_IRQHandler(void)
{
}*/

/**
  * @}
  */ 


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
