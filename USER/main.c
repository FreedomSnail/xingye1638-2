#include "sys.h"
#include "delay.h"
#include "usart.h"
#include "bsp_uart.h"
#include "includes.h"

#include "led.h"


#include "ap_core.h"

#include "DJI_Pro_Codec.h"
#include "DJI_Pro_Link.h"
#include "DJI_Pro_App.h"
#include "DJI_Pro_Flight_Ctrl.h"
#include "ap_core.h"
#include "navigation.h"
#include "geodetic.h"


//ALIENTEK 探索者STM32F407开发板 UCOSIII实验
//例4-1 UCOSIII UCOSIII移植

//UCOSIII中以下优先级用户程序不能使用，ALIENTEK
//将这些优先级分配给了UCOSIII的5个系统内部任务
//优先级0：中断服务服务管理任务 OS_IntQTask()
//优先级1：时钟节拍任务 OS_TickTask()
//优先级2：定时任务 OS_TmrTask()
//优先级OS_CFG_PRIO_MAX-2：统计任务 OS_StatTask()
//优先级OS_CFG_PRIO_MAX-1：空闲任务 OS_IdleTask()
//技术支持：www.openedv.com
//淘宝店铺：http://eboard.taobao.com  
//广州市星翼电子科技有限公司  
//作者：正点原子 @ALIENTEK

//任务优先级
#define START_TASK_PRIO		3
//任务堆栈大小	
#define START_STK_SIZE 		512
//任务控制块
OS_TCB StartTaskTCB;
//任务堆栈	
CPU_STK START_TASK_STK[START_STK_SIZE];
//任务函数
void start_task(void *p_arg);

//任务优先级
#define APP_TASK_DJI_CODEC_PRIO		10
//任务堆栈大小	
#define APP_TASK_DJI_CODEC_STK_SIZE 		1024
//任务控制块
OS_TCB Led0TaskTCB;
//任务堆栈	
CPU_STK AppTaskDjiSDKCodecStk[APP_TASK_DJI_CODEC_STK_SIZE];
void AppTaskDjiSDKCodec(void *p_arg);

//任务优先级
#define TASK_DJI_ACTIVATION_PRIO		5
//任务堆栈大小	
#define TASK_DJI_ACTIVATION_STK_SIZE 		1024
//任务控制块
OS_TCB TaskDjiActivationTCB;
//任务堆栈	
CPU_STK TASK_DJI_ACTIVATION_STK[TASK_DJI_ACTIVATION_STK_SIZE];
//任务函数
void AppTaskDjiActivation(void *p_arg);

//任务优先级
#define TASK_DJI_OBTAIN_CTRL_PRIO		6
//任务堆栈大小	
#define TASK_DJI_OBTAIN_CTRL_STK_SIZE 		1024
//任务控制块
OS_TCB TaskDjiObtainCtrlTCB;
//任务堆栈	
CPU_STK TASK_DJI_OBTAIN_CTRL_STK[TASK_DJI_OBTAIN_CTRL_STK_SIZE];
//任务函数
void AppTaskDjiObtainCtrl(void *p_arg);


//任务优先级
#define TASK_DJI_RELEASE_CTRL_PRIO		7
//任务堆栈大小	
#define TASK_DJI_RELEASE_CTRL_STK_SIZE 		1024
//任务控制块
OS_TCB TaskDjiReleaseCtrlTCB;
//任务堆栈	
CPU_STK TASK_DJI_RELEASE_CTRL_STK[TASK_DJI_RELEASE_CTRL_STK_SIZE];
//任务函数
void AppTaskDjiReleaseCtrl(void *p_arg);




//任务优先级
#define TASK_AUTO_NAV_PRIO		8
//任务堆栈大小
#define TASK_AUTO_NAV_STK_SIZE		1024
//任务控制块
OS_TCB	TaskAutoNavTCB;
//任务堆栈
CPU_STK	TASK_AUTO_NAV_STK[TASK_AUTO_NAV_STK_SIZE];
//__align(8) CPU_STK	TASK_AUTO_NAV_STK[TASK_AUTO_NAV_STK_SIZE];

//任务函数
void AppTaskAutoNav(void *p_arg);

int main(void)
{
	OS_ERR err;
	CPU_SR_ALLOC();
	
	delay_init(168);  	//时钟初始化
	
	
	//uart_init(115200);  //串口初始化
	
	
	INTX_DISABLE();		//关中断,防止滴答定时器对外设初始化的打扰
	LED_Init();         //LED初始化
	//USART1_Config(USART1,115200);
	USART2_Config(USART2,115200);
	USART3_Config(USART3,115200);
	//NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//中断分组配置
	NVIC_Configuration();
	INTX_ENABLE();		//开中断
	
	OSInit(&err);		//初始化UCOSIII
	OS_CRITICAL_ENTER();//进入临界区
	//创建开始任务
	OSTaskCreate((OS_TCB 	* )&StartTaskTCB,		//任务控制块
				 (CPU_CHAR	* )"start task", 		//任务名字
                 (OS_TASK_PTR )start_task, 			//任务函数
                 (void		* )0,					//传递给任务函数的参数
                 (OS_PRIO	  )START_TASK_PRIO,     //任务优先级
                 (CPU_STK   * )&START_TASK_STK[0],	//任务堆栈基地址
                 (CPU_STK_SIZE)START_STK_SIZE/10,	//任务堆栈深度限位
                 (CPU_STK_SIZE)START_STK_SIZE,		//任务堆栈大小
                 (OS_MSG_QTY  )0,					//任务内部消息队列能够接收的最大消息数目,为0时禁止接收消息
                 (OS_TICK	  )0,					//当使能时间片轮转时的时间片长度，为0时为默认长度，
                 (void   	* )0,					//用户补充的存储区
                 (OS_OPT      )OS_OPT_TASK_STK_CHK|OS_OPT_TASK_STK_CLR, //任务选项
                 (OS_ERR 	* )&err);				//存放该函数错误时的返回值
	OS_CRITICAL_EXIT();	//退出临界区	 
	OSStart(&err);  //开启UCOSIII
	while(1);
}

//开始任务函数
OS_SEM SemDjiCodec;
OS_SEM SemDjiActivation;
OS_SEM SemDjiFlightCtrlObtain;
OS_SEM SemDjiFlightCtrlRelease;

u8 flag_frame=0;

OS_Q   QAutoNav;
void start_task(void *p_arg)
{
	OS_ERR err;
	CPU_SR_ALLOC();
	p_arg = p_arg;

	CPU_Init();
#if OS_CFG_STAT_TASK_EN > 0u
   OSStatTaskCPUUsageInit(&err);  	//统计任务                
#endif
	
#ifdef CPU_CFG_INT_DIS_MEAS_EN		//如果使能了测量中断关闭时间
    CPU_IntDisMeasMaxCurReset();	
#endif
	
#if	OS_CFG_SCHED_ROUND_ROBIN_EN  //当使用时间片轮转的时候
	 //使能时间片轮转调度功能,时间片长度为1个系统时钟节拍，既1*5=5ms
	OSSchedRoundRobinCfg(DEF_ENABLED,1,&err);  
#endif		
	
	OS_CRITICAL_ENTER();	//进入临界区
	
	OSQCreate(&QAutoNav, "qmsg", 10, &err);

	OSSemCreate ((OS_SEM*	)&SemDjiActivation,
                 (CPU_CHAR*	)"1",
                 (OS_SEM_CTR)0,		
                 (OS_ERR*	)&err);
    OSSemCreate ((OS_SEM*	)&SemDjiFlightCtrlObtain,
                 (CPU_CHAR*	)"2",
                 (OS_SEM_CTR)0,		
                 (OS_ERR*	)&err);
    OSSemCreate ((OS_SEM*	)&SemDjiFlightCtrlRelease,
                 (CPU_CHAR*	)"22",
                 (OS_SEM_CTR)0,		
                 (OS_ERR*	)&err);
	OSSemCreate ((OS_SEM*	)&SemDjiCodec,
                 (CPU_CHAR*	)"3",
                 (OS_SEM_CTR)0,		
                 (OS_ERR*	)&err);
    
	//创建LED_B任务
	OSTaskCreate((OS_TCB 	* )&Led0TaskTCB,		
				 (CPU_CHAR	* )"led0 task", 		
                 (OS_TASK_PTR )AppTaskDjiSDKCodec, 			
                 (void		* )0,					
                 (OS_PRIO	  )APP_TASK_DJI_CODEC_PRIO,     
                 (CPU_STK   * )&AppTaskDjiSDKCodecStk[0],	
                 (CPU_STK_SIZE)APP_TASK_DJI_CODEC_STK_SIZE/10,	
                 (CPU_STK_SIZE)APP_TASK_DJI_CODEC_STK_SIZE,		
                 (OS_MSG_QTY  )0,					
                 (OS_TICK	  )0,					
                 (void   	* )0,					
                 (OS_OPT      )OS_OPT_TASK_STK_CHK|OS_OPT_TASK_STK_CLR,
                 (OS_ERR 	* )&err);				
				 
	//创建LED_G任务
	OSTaskCreate((OS_TCB 	* )&TaskDjiActivationTCB,		
				 (CPU_CHAR	* )"1 task", 		
                 (OS_TASK_PTR )AppTaskDjiActivation, 			
                 (void		* )0,					
                 (OS_PRIO	  )TASK_DJI_ACTIVATION_PRIO,     	
                 (CPU_STK   * )&TASK_DJI_ACTIVATION_STK[0],	
                 (CPU_STK_SIZE)TASK_DJI_ACTIVATION_STK_SIZE/10,	
                 (CPU_STK_SIZE)TASK_DJI_ACTIVATION_STK_SIZE,		
                 (OS_MSG_QTY  )0,					
                 (OS_TICK	  )0,					
                 (void   	* )0,				
                 (OS_OPT      )OS_OPT_TASK_STK_CHK|OS_OPT_TASK_STK_CLR, 
                 (OS_ERR 	* )&err);
    OSTaskCreate((OS_TCB 	* )&TaskDjiObtainCtrlTCB,		
				 (CPU_CHAR	* )"2 task", 		
                 (OS_TASK_PTR )AppTaskDjiObtainCtrl, 			
                 (void		* )0,					
                 (OS_PRIO	  )TASK_DJI_OBTAIN_CTRL_PRIO,     	
                 (CPU_STK   * )&TASK_DJI_OBTAIN_CTRL_STK[0],	
                 (CPU_STK_SIZE)TASK_DJI_OBTAIN_CTRL_STK_SIZE/10,	
                 (CPU_STK_SIZE)TASK_DJI_OBTAIN_CTRL_STK_SIZE,		
                 (OS_MSG_QTY  )0,					
                 (OS_TICK	  )0,					
                 (void   	* )0,				
                 (OS_OPT      )OS_OPT_TASK_STK_CHK|OS_OPT_TASK_STK_CLR, 
                 (OS_ERR 	* )&err);
	OSTaskCreate((OS_TCB 	* )&TaskDjiReleaseCtrlTCB,		
				 (CPU_CHAR	* )"3 task", 		
                 (OS_TASK_PTR )AppTaskDjiReleaseCtrl, 			
                 (void		* )0,					
                 (OS_PRIO	  )TASK_DJI_RELEASE_CTRL_PRIO,     	
                 (CPU_STK   * )&TASK_DJI_RELEASE_CTRL_STK[0],	
                 (CPU_STK_SIZE)TASK_DJI_RELEASE_CTRL_STK_SIZE/10,	
                 (CPU_STK_SIZE)TASK_DJI_RELEASE_CTRL_STK_SIZE,		
                 (OS_MSG_QTY  )0,					
                 (OS_TICK	  )0,					
                 (void   	* )0,				
                 (OS_OPT      )OS_OPT_TASK_STK_CHK|OS_OPT_TASK_STK_CLR, 
                 (OS_ERR 	* )&err);
	OSTaskCreate((OS_TCB 	* )&TaskAutoNavTCB,		
				 (CPU_CHAR	* )"4 task", 		
                 (OS_TASK_PTR )AppTaskAutoNav, 			
                 (void		* )0,					
                 (OS_PRIO	  )TASK_AUTO_NAV_PRIO,     	
                 (CPU_STK   * )&TASK_AUTO_NAV_STK[0],	
                 (CPU_STK_SIZE)TASK_AUTO_NAV_STK_SIZE/10,	
                 (CPU_STK_SIZE)TASK_AUTO_NAV_STK_SIZE,		
                 (OS_MSG_QTY  )0,					
                 (OS_TICK	  )0,					
                 (void   	* )0,				
                 (OS_OPT      )OS_OPT_TASK_STK_CHK|OS_OPT_TASK_STK_CLR, 
                 (OS_ERR 	* )&err);	
	OS_TaskSuspend((OS_TCB*)&StartTaskTCB,&err);		//挂起开始任务			 
	OS_CRITICAL_EXIT();	//进入临界区
	OSTaskDel((OS_TCB*)0,&err);	//删除start_task任务自身
}

//led0任务函数


void AppTaskDjiSDKCodec(void *p_arg)
{
	OS_ERR err;
	OS_SEM_CTR SemFlag;
	u8 msg;
	p_arg = p_arg;
	while(1) {
		//LOG_DJI_STR(" ");
		OSSemPend(&SemDjiCodec, 100, OS_OPT_PEND_BLOCKING,0,&err); 
		if(OS_ERR_NONE == err) {
			//LED_B = ~LED_B;
			//LOG_DJI_STR(".");
			Pro_Receive_Interface();
		}
	}
}

//led1任务函数
void AppTaskDjiActivation(void *p_arg)
{
	OS_ERR err;
	u8 msg = MSG_TYPE_RC_CTRL;
	u8 nav_flag=0;
	
	CPU_SR_ALLOC();
	p_arg = p_arg;
	DJI_Onboard_API_Activation_Init();
	while(1) {
		LOG_DJI_STR("\r\ntry Activate!\r\n");		
		DJI_Onboard_API_Activation();
		OSSemPend(&SemDjiActivation, 200, OS_OPT_PEND_BLOCKING,0,&err); 
		if(OS_ERR_NONE == err) {
			LOG_DJI_STR("\r\nActivate ok!\r\n");
			break;
		}
	}
	while(1) {
		OSTimeDlyHMSM(0, 0, 1, 0, OS_OPT_TIME_HMSM_STRICT,&err);
		send_flight_data((float)std_broadcast_data.pos.lati,(float)std_broadcast_data.pos.longti,(float)std_broadcast_data.pos.alti, (float)std_broadcast_data.pos.height,0, core_state.target_waypoint, 0,1,1,1,0);
		#if 1
		//LOG_DJI_VALUE("\r\sss=%d\r\n",std_broadcast_data.ctrl_info.cur_ctrl_dev_in_navi_mode);
		if((std_broadcast_data.ctrl_info.cur_ctrl_dev_in_navi_mode == 1)) {//app control
			if(nav_flag<2) {
				nav_flag++;
			}
			LED_B= 1;
		} else {	//rc control
			if(nav_flag>1) {
				nav_flag = 0;
				#if 1
				OSQPost((OS_Q*		)&QAutoNav,		
						(void*		)&msg,
						(OS_MSG_SIZE)1,
						(OS_OPT 	)OS_OPT_POST_FIFO,
						(OS_ERR*	)&err);
				#endif
			}
			LED_B= 0;
		}
		#endif
		//LED_R;
		//LOG_DJI_STR("\r\nnormal\r\n");
	}
}

void AppTaskDjiObtainCtrl(void *p_arg)
{
	OS_ERR err;
	u8 msg = MSG_TYPE_NAV_OBTAIN_CTL;
	u8 cnt;
	CPU_SR_ALLOC();
	p_arg = p_arg;
	while(1) {
		OSSemPend(&SemDjiFlightCtrlObtain, 0, OS_OPT_PEND_BLOCKING,0,&err); 
		cnt = 9;
		while(1) {
			LOG_DJI_VALUE("\r\nFlight ctrl obtain countdown = %d\r\n",cnt);
			DJI_Onboard_API_Control(1);//获取控制权	
			OSSemPend(&SemDjiFlightCtrlObtain, 200, OS_OPT_PEND_BLOCKING,0,&err); 
			if(OS_ERR_NONE == err) {
				LOG_DJI_STR("\r\nObtain Success!\r\n");
				#if 1
				OSQPost((OS_Q*		)&QAutoNav,		
						(void*		)&msg,
						(OS_MSG_SIZE)1,
						(OS_OPT 	)OS_OPT_POST_FIFO,
						(OS_ERR*	)&err);
				break;
				#endif
			}
			cnt--;
			if(cnt==0) {
				LOG_DJI_STR("\r\nObtain timeout!\r\n");
				break;
			}
		}
	}
}
void AppTaskDjiReleaseCtrl(void *p_arg)
{
	OS_ERR err;
	u8 *msg;
	CPU_SR_ALLOC();
	p_arg = p_arg;	
	while(1) {
		OSSemPend(&SemDjiFlightCtrlRelease, 0, OS_OPT_PEND_BLOCKING,0,&err); 
		while(1) {
			LOG_DJI_STR("\r\ntry release flight control!\r\n");
			DJI_Onboard_API_Control(0);//释放控制权	
			OSSemPend(&SemDjiFlightCtrlRelease, 200, OS_OPT_PEND_BLOCKING,0,&err); 
			if(OS_ERR_NONE == err) {
				LOG_DJI_STR("\r\nrelease Success!\r\n");
				break;
			}
		}
	}
}
#define	AUTO_NAV_STATUS_IDLE 0
#define	AUTO_NAV_STATUS_CHECK_GPS 1
#define	AUTO_NAV_STATUS_CHECK_HEIGHT 2
#define AUTO_NAV_STATUS_RAISE_TARTGET_HEIGHT 3
#define	AUTO_NAV_STATUS_RUN 4

void AppTaskAutoNav(void *p_arg)
{
	OS_ERR err;
	u8 *msg;
	OS_MSG_SIZE size;
	u8 status=AUTO_NAV_STATUS_IDLE;
	u8 cnt = 0;
	CPU_SR_ALLOC();
	p_arg = p_arg;	
	
	while(1) {
		msg=OSQPend((OS_Q*			)&QAutoNav,   
					(OS_TICK		)10,
                    (OS_OPT			)OS_OPT_PEND_BLOCKING,
                    (OS_MSG_SIZE*	)&size,		
                    (CPU_TS*		)0,
                    (OS_ERR*		)&err);
        if(OS_ERR_NONE == err) {
        	switch(*msg) {
				case MSG_TYPE_RC_CTRL:
					status = AUTO_NAV_STATUS_IDLE;
					LOG_DJI_STR("\r\nrc control,lose serial ctrl!\r\n");
					break;
				case MSG_TYPE_NAV_START:
					if(AUTO_NAV_STATUS_RUN != status) {
						status = AUTO_NAV_STATUS_IDLE;
						LOG_DJI_STR("\r\nNav start!\r\n");
						OSSemPost(&SemDjiFlightCtrlObtain,OS_OPT_POST_1,&err);
					} else {
						cnt++;
						if(cnt<3) {
							LOG_DJI_STR("\r\nNav already start!\r\n");
						} else {
							cnt = 0;
							status = AUTO_NAV_STATUS_IDLE;
							LOG_DJI_STR("\r\nNav restart!\r\n");
							OSSemPost(&SemDjiFlightCtrlObtain,OS_OPT_POST_1,&err);
						}
						
					}
					break;
				case MSG_TYPE_NAV_OBTAIN_CTL:
					status = AUTO_NAV_STATUS_CHECK_GPS;
					LOG_DJI_STR("\r\nNav start!\r\n");
					break;
				case MSG_TYPE_NAV_DONE:
					status = AUTO_NAV_STATUS_IDLE;
					LOG_DJI_STR("\r\ntarget complete,release ctrl!\r\n");
					OSSemPost(&SemDjiFlightCtrlRelease,OS_OPT_POST_1,&err);
					break;
				default:
					break;
        	}
		}
		
		#if 1
		switch(status) {
			case AUTO_NAV_STATUS_CHECK_GPS:
				if(auto_nav_check_gps()) {
					status = AUTO_NAV_STATUS_CHECK_HEIGHT;
					LOG_DJI_STR("\r\ngps ok!\r\n");
					auto_nav_math_init();
					LOG_DJI_STR("\r\nnav init!\r\n");
				} else {
					LOG_DJI_STR("\r\npchecking gps!\r\n");
				}
				break;
			case AUTO_NAV_STATUS_CHECK_HEIGHT:
				if(auto_nav_check_height()) {
					status = AUTO_NAV_STATUS_RAISE_TARTGET_HEIGHT;
				} else {
					status = AUTO_NAV_STATUS_RUN;
					LOG_DJI_STR("\r\nheight already >=2m\r\n");
					LOG_DJI_STR("\r\nstart target waypoint!\r\n");
				}
				break;
			case AUTO_NAV_STATUS_RAISE_TARTGET_HEIGHT:
				if(auto_nav_raise_to_tartget_height()) {
					status = AUTO_NAV_STATUS_RUN;
					LOG_DJI_STR("\r\nflight raise to tartget height\r\n");
					LOG_DJI_STR("\r\nstart target waypoint!\r\n");
				}
				break;
			case AUTO_NAV_STATUS_RUN:
				LOG_DJI_STR(".");
				if(auto_nav()) {
					status = AUTO_NAV_STATUS_IDLE;
					LOG_DJI_STR("\r\ntarget complete,release ctrl!\r\n");
					OSSemPost(&SemDjiFlightCtrlRelease,OS_OPT_POST_1,&err);
				}
				break;
			default:
				break;
		}
		#endif
	}
}


//浮点测试任务
void AppTaskuotANav(void *p_arg)
{
	static float float_num=0.01;
	u8 len=0;
	CPU_SR_ALLOC();
	while(1)
	{
		float_num+=0.01f;
		OS_CRITICAL_ENTER();	//进入临界区
		//LOG_DJI_VALUE("%.4f\r\n",float_num);
		#if 0
		len++;
		len %= 9;
		if(len < 1) {
			DJI_Onboard_send("  ", 2);
		} else {
			//DJI_Onboard_send("12345678", len);
			send_data_to_mobile("12345678",len);
		}
		#endif
		//printf("float_num的值为: %.4f\r\n",y);
		OS_CRITICAL_EXIT();		//退出临界区
		delay_ms(900);			//延时500ms
		//LED_R= ~LED_R;
	}
}

