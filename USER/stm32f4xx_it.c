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



extern u8 flag_frame ;

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
* Function Name  : USART1_IRQHandler
* Description    : This function handles USART2 global interrupt request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void USART1_IRQHandler(void)
{
	u8 Rev;
	#if SYSTEM_SUPPORT_UCOS  //使用UCOS操作系统
	OSIntEnter();    
	#endif
	
	//用户程序..
	if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET) {	//判断读寄存器是否非空	
    	// Read one byte from the receive data register
    	Rev= USART_ReceiveData(USART1);   //将读寄存器的数据缓存到接收缓冲区里
		//USART_SendData(USART1,Rev);
		//while(USART_GetFlagStatus(USART1, USART_FLAG_TC)==RESET);
		//BSP_OS_SemPost(&SEM_SYNCH);
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
void USART2_IRQHandler(void)
{
	OS_ERR err;
	u8 Rev;
	static u8 DataLen;
	static DjiSDKPackageStatus_TypeEnum DjiPackageStatus = DJI_PACKAGE_RECV_IDLE;
	#if SYSTEM_SUPPORT_UCOS  //使用UCOS操作系统
	OSIntEnter();    
	#endif
	//用户程序..
	if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET) {	//判断读寄存器是否非空	
    	// Read one byte from the receive data register
    	Rev= USART_ReceiveData(USART2);   //将读寄存器的数据缓存到接收缓冲区里
		//USART_SendData(USART2,Rev);
		//while(USART_GetFlagStatus(USART2, USART_FLAG_TC)==RESET);
		//BSP_OS_SemPost(&SEM_SYNCH);
		#if 0
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
			 			DjiPackageStatus = DJI_PACKAGE_RECV_IDLE;
		 			}
	 			} else {//接收到的数据致使数组越界
		 			DjiPackageStatus = DJI_PACKAGE_RECV_IDLE;
	 			}
	 			break;
			default:
		 		break;
			
		}
		#endif
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
		 				//if(flag_frame != 1) {
						//	flag_frame = 1;
			 				OSSemPost(&SemDjiCodec,OS_OPT_POST_1,&err);
			 				LED_G= ~LED_G;
		 				//} 
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
