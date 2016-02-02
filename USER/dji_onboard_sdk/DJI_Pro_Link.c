/*
 * DJI_Pro_Link.c
 *
 *  Created on: Mar 12, 2015
 *      Author: wuyuwei
 */

#include <stdio.h>
#include <string.h>
#include "stm32f4xx.h"
#include "bsp_uart.h"
#include "includes.h"

#include "DJI_Pro_Link.h"
#include "DJI_Pro_Codec.h"
#include "DJI_Pro_App.h"
static Session_Queue Send_Session_Common_Tab;
static unsigned char Send_Global_Common_Memory[PKG_MAX_SIZE];
static Session_Queue Send_Session_Tab[SESSION_AND_MEM_COUNT];
static Memory_Manage_Unit Send_MMU_Tab[SESSION_AND_MEM_COUNT];
static unsigned char Send_Global_Memory[SESSION_AND_MEM_COUNT * PKG_MAX_SIZE];

static void Send_Pro_Data(unsigned char *buf)
{
	ProHeader *pHeader = (ProHeader *)buf;
	USART_Send_Buf(SERIAL_PORT_DJI_SDK,buf,pHeader->length);
	//USART_Send_Buf(SERIAL_PORT_DJI_SDK,"+\r\n",3);
}


Memory_Manage_Unit *Request_Send_MMU(unsigned short size)
{
//	unsigned char i;
	Memory_Manage_Unit *p2mmu ;
	#if 0
	for(i = 0 ; i < SESSION_AND_MEM_COUNT; i ++)
	{
		if(Send_MMU_Tab[i].usage_flag == 0)
		{
			p2mmu = &Send_MMU_Tab[i];
			p2mmu->usage_flag = 1;
			p2mmu->end_addr = p2mmu->start_addr + size;
			break;
		}
	}
	#else
	p2mmu = &Send_MMU_Tab[0];
	p2mmu->usage_flag = 1;
	p2mmu->end_addr = p2mmu->start_addr + size;
	#endif
	return p2mmu;
}


void Free_Send_MMU(Memory_Manage_Unit *mmu)
{

	if(mmu->usage_flag == 1)
	{
		mmu->usage_flag = 0;
	}
}

Session_Queue * Request_Send_Session(unsigned short size)
{
//	int i;
	Session_Queue *p2session=NULL;
	Memory_Manage_Unit *p2mmu=NULL;
	#if 0
	for(i = 0 ; i < SESSION_AND_MEM_COUNT ; i ++)
	{
		if(Send_Session_Tab[i].usage_flag == 0)
		{
			Send_Session_Tab[i].usage_flag = 1;
			p2session = &Send_Session_Tab[i];
			break;
		}
	}
	#else
	p2session = &Send_Session_Tab[0];
	#endif
	p2mmu = Request_Send_MMU(size);
	if(p2mmu == NULL)
	{
		p2session = NULL;
	}
	else
	{
		p2session->mmu = p2mmu;
	}
	return p2session;
}

void Free_All_Session(void)
{
	int i;
	for(i = 0 ; i < SESSION_AND_MEM_COUNT ; i ++)
	{
		Send_Session_Tab[i].usage_flag = 0;
		Send_MMU_Tab[i].usage_flag = 0;
		
	}
}



void Free_Send_Session(Session_Queue * session)
{
	if(session->usage_flag == 1)
	{
		Free_Send_MMU(session->mmu);
		session->usage_flag = 0;
	}
}


void Pro_Link_Setup(void)
{
	unsigned char i;

	for(i = 0; i < SESSION_AND_MEM_COUNT; i ++)
	{
		Send_Session_Tab[i].session_id = i + 2;
		Send_Session_Tab[i].usage_flag = 0;
		Send_Session_Tab[i].pre_seq_num = 0x10000;

		Send_MMU_Tab[i].mmu_index = i;
		Send_MMU_Tab[i].usage_flag = 0;
		Send_MMU_Tab[i].start_addr = (unsigned long)&Send_Global_Memory[i * PKG_MAX_SIZE];
	}

	Send_Session_Common_Tab.usage_flag = 0;
	Send_Session_Common_Tab.session_id = 1;
	

}

void Pro_Config_Comm_Encrypt_Key(const char *key)
{
	sdk_set_encrypt_key_interface(key);
}


int Pro_Send_Interface(ProSendParameter *parameter)
{
	unsigned short ret = 0;
	static unsigned short global_seq_num = 0;
  	Session_Queue *p2session = NULL;
	if(parameter->length > PRO_DATA_MAX_SIZE)
	{
		return -1;
	}


	switch(parameter->session_mode)//°üÀàÐÍ
	{
	
	case 0:
		ret = sdk_encrypt_interface(Send_Global_Common_Memory,parameter->buf,parameter->length,
				0,parameter->need_encrypt,0,global_seq_num ++);
		if(ret == 0)
		{
			return -1;
		}
		Send_Pro_Data(Send_Global_Common_Memory);
		break;
	case 1:
		#if 1
		if(global_seq_num == Send_Session_Common_Tab.pre_seq_num)
		{
			global_seq_num ++;
		}
		//printf("ZZ=%d\r\n",global_seq_num);
		Send_Session_Common_Tab.pre_seq_num = global_seq_num;

		ret = sdk_encrypt_interface(Send_Global_Common_Memory,parameter->buf,parameter->length,
				0,parameter->need_encrypt,1,global_seq_num ++);

		if(ret == 0)
		{
			return -1;
		}
		
		Send_Pro_Data(Send_Global_Common_Memory);
		break;
		#endif
	case 2:
		p2session = Request_Send_Session(parameter->length + sizeof(ProHeader) + 4);
		if(p2session != NULL)
		{
			
			if(global_seq_num == p2session->pre_seq_num)
			{
				global_seq_num ++;
			}
			p2session->pre_seq_num = global_seq_num;

			ret = sdk_encrypt_interface((unsigned char*)p2session->mmu->start_addr,
					parameter->buf,parameter->length,0,parameter->need_encrypt,
					p2session->session_id,global_seq_num ++);
			
			if(ret == 0)
			{
				Free_Send_Session(p2session);
				return -1;
			}
			Send_Pro_Data((unsigned char*)p2session->mmu->start_addr);
			ret = 0;
		}else {
			Free_All_Session();
		}
		break;
		
	}


	return 0;
}


