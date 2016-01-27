
#include "stm32f4xx.h"
#include <stdarg.h>
#include "bsp_uart.h"

UartTypedef		UartDJI;
/*

*********************************************************************************************************
*	函 数 名: bsp_InitUart
*	功能说明: 初始化CPU的USART1串口硬件设备。未启用中断。
*	形    参：无
*	返 回 值: 无
*********************************************************************************************************
*/
void bsp_InitUart(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	
	/* 串口1 TX = PA9   RX = PA10 */

	/* 第1步： 配置GPIO */

	/* 打开 GPIO 时钟 */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

	/* 打开 UART 时钟 */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);

	/* 将 PA9 映射为 USART1_TX */
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_USART1);

	/* 将 PA10 映射为 USART1_RX */
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_USART1);

	/* 配置 USART Tx 为复用功能 */
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;	/* 输出类型为推挽 */
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;	/* 内部上拉电阻使能 */
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;	/* 复用模式 */

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	/* 配置 USART Rx 为复用功能 */
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	/* 第2步： 配置串口硬件参数 */
	USART_InitStructure.USART_BaudRate = 115200;	/* 波特率 */
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No ;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(USART1, &USART_InitStructure);

	/* Enable USART2 Receive and Transmit interrupts */
  	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);                    //使能接收中断

	USART_Cmd(USART1, ENABLE);		/* 使能串口 */

	/* CPU的小缺陷：串口配置好，如果直接Send，则第1个字节发送不出去
		如下语句解决第1个字节无法正确发送出去的问题 */
	USART_ClearFlag(USART1, USART_FLAG_TC);     /* 清发送完成标志，Transmission Complete flag */   
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
void USART1_Config(USART_TypeDef* USARTx,u32 baud)
{
	USART_InitTypeDef USART_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;

  	/* 串口1 TX = PA9   RX = PA10 */

	/* 第1步： 配置GPIO */

	/* 打开 GPIO 时钟 */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

	/* 打开 UART 时钟 */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);

	/* 将 PA9 映射为 USART1_TX */
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_USART1);

	/* 将 PA10 映射为 USART1_RX */
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_USART1);
  
  	/* 配置 USART Tx 为复用功能 */
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;	/* 输出类型为推挽 */
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;	/* 内部上拉电阻使能 */
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;	/* 复用模式 */

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);				 
  
  	/* 配置 USART Rx 为复用功能 */
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
  
  	USART_InitStructure.USART_BaudRate =baud;						//速率115200bps
  	USART_InitStructure.USART_WordLength = USART_WordLength_8b;		//数据位8位
  	USART_InitStructure.USART_StopBits = USART_StopBits_1;			//停止位1位
  	USART_InitStructure.USART_Parity = USART_Parity_No;				//无校验位
  	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;   //无硬件流控
  	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;					//收发模式

  	/* Configure USART1 */
  	USART_Init(USARTx, &USART_InitStructure);							//配置串口参数函数
 
  
  	/* Enable USART1 Receive and Transmit interrupts */
  	USART_ITConfig(USARTx, USART_IT_RXNE, ENABLE);                    //使能接收中断
  	//USART_ITConfig(USARTx, USART_IT_TXE, ENABLE);				    //使能发送缓冲空中断

  
  	/* Enable the USART1 */
  	USART_Cmd(USARTx, ENABLE);	

  	/* CPU的小缺陷：串口配置好，如果直接Send，则第1个字节发送不出去
		如下语句解决第1个字节无法正确发送出去的问题 */
	USART_ClearFlag(USARTx, USART_FLAG_TC);     /* 清发送完成标志，Transmission Complete flag */  
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
void USART2_Config(USART_TypeDef* USARTx,u32 baud)
{
	USART_InitTypeDef USART_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;

  	/* 串口1 TX = PA2   RX = PA3 */

	/* 第1步： 配置GPIO */

	/* 打开 GPIO 时钟 */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

	/* 打开 UART 时钟 */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);

	/* 将 PA2 映射为 USART1_TX */
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_USART2);

	/* 将 PA3 映射为 USART1_RX */
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_USART2);
  
  	/* 配置 USART Tx 为复用功能 */
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;	/* 输出类型为推挽 */
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;	/* 内部上拉电阻使能 */
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;	/* 复用模式 */

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);				 
  
  	/* 配置 USART Rx 为复用功能 */
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
  
  	USART_InitStructure.USART_BaudRate =baud;						//速率115200bps
  	USART_InitStructure.USART_WordLength = USART_WordLength_8b;		//数据位8位
  	USART_InitStructure.USART_StopBits = USART_StopBits_1;			//停止位1位
  	USART_InitStructure.USART_Parity = USART_Parity_No;				//无校验位
  	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;   //无硬件流控
  	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;					//收发模式

  	/* Configure USART1 */
  	USART_Init(USARTx, &USART_InitStructure);							//配置串口参数函数
 
  
  	/* Enable USART1 Receive and Transmit interrupts */
  	USART_ITConfig(USARTx, USART_IT_RXNE, ENABLE);                    //使能接收中断
  	//USART_ITConfig(USARTx, USART_IT_TXE, ENABLE);				    //使能发送缓冲空中断

  
  	/* Enable the USART1 */
  	USART_Cmd(USARTx, ENABLE);	

  	/* CPU的小缺陷：串口配置好，如果直接Send，则第1个字节发送不出去
		如下语句解决第1个字节无法正确发送出去的问题 */
	USART_ClearFlag(USARTx, USART_FLAG_TC);     /* 清发送完成标志，Transmission Complete flag */  
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
  void USART3_Config(USART_TypeDef* USARTx,u32 baud)
  {
	  USART_InitTypeDef USART_InitStructure;
	  GPIO_InitTypeDef GPIO_InitStructure;
  
	  /* 串口3 TX = PD8   RX = PD9 */
  
	  /* 第1步： 配置GPIO */
  
	  /* 打开 GPIO 时钟 */
	  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
  
	  /* 打开 USART 时钟 */
	  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);
  
	  /* 将 PD8 映射为 USART3_TX */
	  GPIO_PinAFConfig(GPIOD, GPIO_PinSource8, GPIO_AF_USART3);
  
	  /* 将 PD9 映射为 USART3_RX */
	  GPIO_PinAFConfig(GPIOD, GPIO_PinSource9, GPIO_AF_USART3);
	
	  /* 配置 USART Tx 为复用功能 */
	  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;  /* 输出类型为推挽 */
	  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;	  /* 内部上拉电阻使能 */
	  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;	  /* 复用模式 */
  
	  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
	  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	  GPIO_Init(GPIOD, &GPIO_InitStructure);			   
	
	  /* 配置 USART Rx 为复用功能 */
	  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
	  GPIO_Init(GPIOD, &GPIO_InitStructure);
	
	  USART_InitStructure.USART_BaudRate =baud; 					  //速率115200bps
	  USART_InitStructure.USART_WordLength = USART_WordLength_8b;	  //数据位8位
	  USART_InitStructure.USART_StopBits = USART_StopBits_1;		  //停止位1位
	  USART_InitStructure.USART_Parity = USART_Parity_No;			  //无校验位
	  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;	//无硬件流控
	  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;				  //收发模式
  
	  /* Configure USART1 */
	  USART_Init(USARTx, &USART_InitStructure); 						  //配置串口参数函数
   
	
	  /* Enable USART1 Receive and Transmit interrupts */
	  USART_ITConfig(USARTx, USART_IT_RXNE, ENABLE);					//使能接收中断
	  //USART_ITConfig(USARTx, USART_IT_TXE, ENABLE);				  //使能发送缓冲空中断
  
	
	  /* Enable the USART1 */
	  USART_Cmd(USARTx, ENABLE);  
  
	  /* CPU的小缺陷：串口配置好，如果直接Send，则第1个字节发送不出去
		  如下语句解决第1个字节无法正确发送出去的问题 */
	  USART_ClearFlag(USARTx, USART_FLAG_TC);	  /* 清发送完成标志，Transmission Complete flag */	
  }

  /******************************************************
		整形数据转字符串函数
        char *itoa(int value, char *string, int radix)
		radix=10 标示是10进制	非十进制，转换结果为0;  

	    例：d=-379;
		执行	itoa(d, buf, 10); 后
		
		buf="-379"							   			  
**********************************************************/
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

} /* NCL_Itoa */

/******************************************************
		格式化串口输出函数
        "\r"	回车符	   USART_OUT(USART1, "abcdefg\r")   
		"\n"	换行符	   USART_OUT(USART1, "abcdefg\r\n")
		"%s"	字符串	   USART_OUT(USART1, "字符串是：%s","abcdefg")
		"%d"	十进制	   USART_OUT(USART1, "a=%d",10)
**********************************************************/
void USART_OUT(USART_TypeDef* USARTx, uint8_t *Data,...)
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

/*
*********************************************************************************************************
*	函 数 名: NVIC_Configuration
*	功能说明: 中断优先级配置，这里先把优先级分组分配好，然后在
*			  具体的模块里面实现优先级配置 
*	形    参：无
*	返 回 值: 无
*********************************************************************************************************
*/
void NVIC_Configuration(void)
{	
	#if 1
	NVIC_InitTypeDef NVIC_InitStructure;
  	
	/* Configure one bit for preemption priority */
  	#if defined (VECT_TAB_RAM)
	/* Set the Vector Table base location at 0x20000000 */ 
	NVIC_SetVectorTable(NVIC_VectTab_RAM, 0x0); 
	#elif defined(VECT_TAB_FLASH_IAP)
	NVIC_SetVectorTable(NVIC_VectTab_FLASH, 0x2000);
	#else  /* VECT_TAB_FLASH  */
	/* Set the Vector Table base location at 0x08000000 */ 
	NVIC_SetVectorTable(NVIC_VectTab_FLASH, 0x00);   
	#endif 
	#endif
	/* 设置NVIC优先级分组为Group2：0-3抢占式优先级，0-3的响应式优先级 */
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);

	#if 0	//如果需要开串口1中断
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;			     	//设置串口1中断
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 4;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	#endif
	
	#if 2	//如果需要开串口2中断
	NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;			     	//设置串口1中断
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 4;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	#endif
	
	#if 3	//如果需要开串口3中断
	NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;			     	//设置串口1中断
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 5;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	#endif
	
	#if 0	//如果要开1脚中断,第二个Modem的RXD脚
	/* EXTI7*/
	NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 6;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	#endif

	#if 0	//如果要开2脚中断,数令接收板读忙检测脚
	/* EXTI1*/
	NVIC_InitStructure.NVIC_IRQChannel = EXTI2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	#endif

	#if 0	//如果要开3脚中断,第一个Modem的RXD脚
	/* EXTI3*/
	NVIC_InitStructure.NVIC_IRQChannel = EXTI3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	#endif
	
	#if 0	//如果要开6脚中断，锁相环锁定检测
	/* EXTI5,6*/
	NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	#endif
	#if 0	//如果要开15脚中断，射频板信号接收读忙检测脚
	/* EXTI15*/
	NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	#endif
	#if 0		//如果要开Timer1中断
	/* Timer1溢出中断*/
	NVIC_InitStructure.NVIC_IRQChannel = TIM1_UP_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	#endif
	
	#if 0		//如果要开Timer2中断
	/* Timer2中断*/
	NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	#endif
	#if 0		//如果要开Timer3中断
	/* Timer3中断*/
	NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	#endif
	#if 0		//如果要开Timer6中断
	/* Timer6中断*/
	NVIC_InitStructure.NVIC_IRQChannel = TIM6_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 4;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	#endif
	#if 0		//如果要开Timer7中断
	/* Timer7中断*/
	NVIC_InitStructure.NVIC_IRQChannel = TIM7_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 5;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	#endif
}

