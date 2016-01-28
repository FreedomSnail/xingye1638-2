/*
 * DJI_Pro_App.cpp
 *
 *  Created on: Mar 17, 2015
 *      Author: Ying Jiahang
 */
#include "stm32f4xx.h"
#include "bsp_uart.h"
#include "includes.h"

#include "DJI_Pro_Codec.h"
#include "DJI_Pro_Link.h"
#include "DJI_Pro_App.h"
#include "DJI_Pro_Flight_Ctrl.h"
#include <string.h>
#include <stdio.h>
#include <math.h>
#include "std.h"
#include "ap_core.h"





//#define MAKE_VERSION(a,b,c,d) (((a << 24)&0xff000000) | ((b << 16)&0x00ff0000) | ((c << 8)&0x0000ff00) | (d&0x000000ff))
//#define SDK_VERSION           (MAKE_VERSION(3,1,100,0))

static Command_Result_Notify p_control_management_interface = 0;
static Command_Result_Notify p_transparent_data_interface = 0;



static unsigned char Pro_Encode_Data[1024];
static dji_sdk_cmd_unit cmd_unit = {0};
static activation_data_t activation_msg = {14,2,0x02030A00,""};
static const char *key;



ProFrameData_Unit  DataFromDji;
unsigned char Activation_Ack = 0;//激活成功为1
//sdk_std_msg_t  FlightMsg;


bool_t wp_download_finished = TRUE;
uint8_t wp_packet_idx = 0;      ///<航点数据包索引
float flight_plan_offset = 0;   ///<线间距
float task_area = 0;            ///<航线面积
float task_distance = 0;        ///<航线总距离


uint32_t device_id = 150920097;///<出厂编号
uint8_t is_usable = TRUE;///<是否可用

u16 array_to_short(u8 *array)
{
    toShort cc;
    cc.bytePtr[0] = *array;
    cc.bytePtr[1] = *(array+1);
    return cc.value;
}

/**
 * @brief 字节数组转换为float
 * @param array
 * @return
 */
float array_to_float(u8 *array)
{
    toFloat cc;
    cc.bytePtr[0] = *array;
    cc.bytePtr[1] = *(array+1);
    cc.bytePtr[2] = *(array+2);
    cc.bytePtr[3] = *(array+3);
    return cc.value;
}
double array_to_double(u8 *array)
{
    toDouble cc;
    cc.bytePtr[0] = *array;
    cc.bytePtr[1] = *(array+1);
    cc.bytePtr[2] = *(array+2);
    cc.bytePtr[3] = *(array+3);
    cc.bytePtr[4] = *(array+4);
    cc.bytePtr[5] = *(array+5);
    cc.bytePtr[6] = *(array+6);
	cc.bytePtr[7] = *(array+7);
	
    return cc.value;
}

void App_Send_Data(unsigned char flag, unsigned char is_enc, unsigned char  cmd_set, unsigned char cmd_id,unsigned char *pdata,int len)
{
	ProSendParameter param;
	unsigned char *ptemp = (unsigned char *)Pro_Encode_Data;
	*ptemp++ = cmd_set;
	*ptemp++ = cmd_id;

	memcpy(Pro_Encode_Data + SET_CMD_SIZE,pdata,len);

	param.session_mode = flag;
	param.length = len + SET_CMD_SIZE;
	param.buf = Pro_Encode_Data;
	param.need_encrypt = is_enc;
	
	Pro_Send_Interface(&param);
}

//----------------------------------------------------------------------
// cmd agency
//----------------------------------------------------------------------


void App_Complex_Send_Cmd(unsigned char cmd)
{
	cmd_unit.cmd.cmd_data = cmd;
	cmd_unit.is_send_cmd = 1;	
	cmd_unit.cmd.cmd_sequence ++;
	App_Send_Data(2,1,MY_CTRL_CMD_SET, API_CMD_REQUEST,(unsigned char*)&cmd_unit.cmd,sizeof(cmd_unit.cmd));
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
void DJI_Onboard_API_Activation_Init(void)
{

	activation_msg.app_id =1008902;
	activation_msg.app_api_level = 2;
	//activation_msg.app_ver = 50357248;
	activation_msg.app_ver = SDK_VERSION;
	
	memcpy(activation_msg.app_bundle_id,"1234567890123456789012", 32);
	key = "818e4fccacb5d597aa4c006a15b7b031185a49ec3f86aa50a023b00d04146a9c";   
	       
	Pro_Config_Comm_Encrypt_Key(key);
	Pro_Link_Setup();
	
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
void DJI_Onboard_API_Activation(void)
{
	
	App_Send_Data( 2, 0, MY_ACTIVATION_SET, API_USER_ACTIVATION,(unsigned char*)&activation_msg,sizeof(activation_msg));
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

void DJI_Onboard_API_Control(unsigned char arg)
{
		unsigned char send_data = arg;
		App_Send_Data(1,0,MY_CTRL_CMD_SET,API_OPEN_SERIAL,(unsigned char*)&send_data,sizeof(send_data));
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
void DJI_Onboard_send(u8* str, u8 len)
{
	if(len < 1) {
		return;
	}
	App_Send_Data( 2, 0, MY_ACTIVATION_SET,0xFE,str,len);
}

void DJI_Pro_App_Send_Data(unsigned char session_mode, unsigned char is_enc, unsigned char  cmd_set, unsigned char cmd_id,
                   unsigned char *pdata,int len,ACK_Callback_Func ack_callback, int timeout ,int retry_time)
{
	ProSendParameter param;
	unsigned char *ptemp = (unsigned char *)Pro_Encode_Data;
	*ptemp++ = cmd_set;
	*ptemp++ = cmd_id;

	memcpy(Pro_Encode_Data + SET_CMD_SIZE,pdata,len);

	param.ack_callback = ack_callback;
    param.session_mode = session_mode;
	param.length = len + SET_CMD_SIZE;
	param.buf = Pro_Encode_Data;
    param.retry_time = retry_time;

	param.ack_timeout = timeout; 
    param.need_encrypt = is_enc;
	Pro_Send_Interface(&param);
}
static void DJI_Pro_Control_Management_CallBack(ProHeader *header)
{
    unsigned short ack_data = 0xFFFF;
    if(header->length - EXC_DATA_SIZE <= 2)
    {
        memcpy((unsigned char *)&ack_data,(unsigned char *)&header->magic, (header->length - EXC_DATA_SIZE));
        if(p_control_management_interface)
            p_control_management_interface(ack_data);

    }
    else
    {
        printf("%s,line %d:ERROR,ACK is exception,seesion id %d,sequence %d\n",
               __func__,__LINE__,header->session_id,header->sequence_number);
    }

    switch(ack_data)
    {
    case 0x0001:
        printf("%s,line %d, release control successfully\n",__func__,__LINE__);
        break;
    case 0x0002:
        printf("%s,line %d, obtain control successfully\n",__func__,__LINE__);
        break;
    case 0x0003:
        printf("%s,line %d, obtain control failed\n",__func__,__LINE__);
        break;
    default:
        printf("%s,line %d, there is unkown error,ack=0x%X\n",__func__,__LINE__,ack_data);
        break;
    }
}

int DJI_Pro_Control_Management(unsigned char cmd,Command_Result_Notify user_notice_entrance)
{
    unsigned char data = cmd & 0x1;
    DJI_Pro_App_Send_Data(2,0, MY_CTRL_CMD_SET, API_CTRL_MANAGEMENT,&data,1,NULL,500,1);
    //usleep(50000);
    p_control_management_interface = user_notice_entrance ? user_notice_entrance : 0;
    DJI_Pro_App_Send_Data(2,0, MY_CTRL_CMD_SET, API_CTRL_MANAGEMENT,&data,1,DJI_Pro_Control_Management_CallBack,500,1);
    return 0;
}
/*
 *  interface: attitude control interface
 */

int DJI_Pro_Attitude_Control(attitude_data_t *p_user_data)
{
    DJI_Pro_App_Send_Data(0,0, MY_CTRL_CMD_SET, API_CTRL_REQUEST,
               (unsigned char *)p_user_data,sizeof(attitude_data_t),
                  0,0,1);
    return 0;
}

static void DJI_Pro_Send_To_Mobile_Device_CallBack(ProHeader *header)
{
    unsigned short ack_data = 0xFFFF;
    if(header->length - EXC_DATA_SIZE <= 2)
    {
        memcpy((unsigned char *)&ack_data,(unsigned char *)&header->magic, (header->length - EXC_DATA_SIZE));
        if(p_transparent_data_interface)
            p_transparent_data_interface(ack_data);
    }
    else
    {
        printf("%s,line %d:ERROR,ACK is exception,seesion id %d,sequence %d\n",
               __func__,__LINE__,header->session_id,header->sequence_number);
    }
}

int DJI_Pro_Send_To_Mobile_Device(unsigned char *data,unsigned char len,
                                  Command_Result_Notify user_notice_entrance)
{
    if(len > 100)
    {
        return -1;
    }

    p_transparent_data_interface = user_notice_entrance ? user_notice_entrance : 0;
    DJI_Pro_App_Send_Data(2, 0, MY_ACTIVATION_SET, API_TRANSPARENT_DATA_TO_MOBILE,
               data,len,DJI_Pro_Send_To_Mobile_Device_CallBack,500,1);

    return 0;
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
void Updata_Flight_Data(void)
{
	u16 packageStatus;
	u16 offSet=2;
	packageStatus = array_to_short(&DataFromDji.data[0]);
	//printf("p=%d\r\n",packageStatus);
	#if 1
	if(packageStatus & (1<<0)) {		//时间戳
		memcpy((unsigned char*)&std_broadcast_data.time_stamp,(unsigned char*)(DataFromDji.data+offSet),sizeof(unsigned int));
		offSet += FLIGHT_DATA_TIME_STAMP_SIZE;
	}
	if(packageStatus & (1<<1)) {		//姿态四元素
		//std_broadcast_data.q.q0 = array_to_float(DataFromDji.data+offSet);
		//std_broadcast_data.q.q1 = array_to_float(&DataFromDji.data[offSet+4]);
		//std_broadcast_data.q.q2 = array_to_float(&DataFromDji.data[offSet+8]);
		//std_broadcast_data.q.q3 = array_to_float(&DataFromDji.data[offSet+12]);
		//memcpy((unsigned char*)&DataFromDji,(unsigned char*)&RecData,sizeof(RecData));
		memcpy((unsigned char*)&std_broadcast_data.q.q0,(unsigned char*)(DataFromDji.data+offSet),sizeof(api_quaternion_data_t));
		//printf("Q=%f %f %f %f\r\n",std_broadcast_data.q.q0,std_broadcast_data.q.q1,std_broadcast_data.q.q2,std_broadcast_data.q.q3);
		offSet += FLIGHT_DATA_ATTITUDE_SIZE;
	}
	
	if(packageStatus & (1<<2)) {		//加速度
		memcpy((unsigned char*)&std_broadcast_data.a.x,(unsigned char*)(DataFromDji.data+offSet),sizeof(api_common_data_t));
		offSet += FLIGHT_DATA_ACCELERATE_SIZE;
	}
	if(packageStatus & (1<<3)) {		//速度
		//std_broadcast_data.v.x = array_to_float(&DataFromDji.data[offSet]);
		//std_broadcast_data.v.y = array_to_float(&DataFromDji.data[offSet+4]);
		//std_broadcast_data.v.z= array_to_float(&DataFromDji.data[offSet+8]);
		memcpy((unsigned char*)&std_broadcast_data.v.x,(unsigned char*)(DataFromDji.data+offSet),sizeof(api_common_data_t));
		//printf("SPD=%f %f %f\r\n",std_broadcast_data.v.x,std_broadcast_data.v.y,std_broadcast_data.v.z);
		offSet += FLIGHT_DATA_SPEED_SIZE;
	}
	if(packageStatus & (1<<4)) {		//角速度
		memcpy((unsigned char*)&std_broadcast_data.w.x,(unsigned char*)(DataFromDji.data+offSet),sizeof(api_common_data_t));
		offSet += FLIGHT_DATA_ANGLE_SPEED_SIZE;
	}
	if(packageStatus & (1<<5)) {		//gps
		//std_broadcast_data.pos.lati = array_to_double(&DataFromDji.data[offSet]);
		//std_broadcast_data.pos.longti = array_to_double(&DataFromDji.data[offSet+8]);
		//std_broadcast_data.pos.alti = array_to_float(&DataFromDji.data[offSet+12]);
		//std_broadcast_data.pos.height = array_to_float(&DataFromDji.data[offSet+16]);
		//std_broadcast_data.pos.health_flag = DataFromDji.data[offSet+20];
		memcpy((unsigned char*)&std_broadcast_data.pos.lati,(unsigned char*)(DataFromDji.data+offSet),sizeof(api_pos_data_t));
		//printf("GPS=%lf %lf %f %f %d\r\n",std_broadcast_data.pos.lati,std_broadcast_data.pos.longti,std_broadcast_data.pos.alti,std_broadcast_data.pos.height,std_broadcast_data.pos.health_flag);
		//printf("GPS health=%d\r\n",std_broadcast_data.pos.health_flag);
		offSet += FLIGHT_DATA_GPS_SIZE;
	}
	if(packageStatus & (1<<6)) {		//磁感器
		//std_broadcast_data.mag.x = array_to_short(&DataFromDji.data[offSet]);
		//std_broadcast_data.mag.y = array_to_short(&DataFromDji.data[offSet+2]);
		//std_broadcast_data.mag.z = array_to_short(&DataFromDji.data[offSet+4]);
		memcpy((unsigned char*)&std_broadcast_data.mag.x,(unsigned char*)(DataFromDji.data+offSet),sizeof(api_mag_data_t));
		//printf("MAGNETIC=%d %d %d\r\n",std_broadcast_data.mag.x,std_broadcast_data.mag.y,std_broadcast_data.mag.z);
		offSet += FLIGHT_DATA_MAGNETIC_SIZE;
	}
	if(packageStatus & (1<<7)) {		//遥控器通道值
		//std_broadcast_data.rc.roll = array_to_short(&DataFromDji.data[offSet]);
		//std_broadcast_data.rc.pitch= array_to_short(&DataFromDji.data[offSet+2]);
		//std_broadcast_data.rc.yaw= array_to_short(&DataFromDji.data[offSet+4]);
		//std_broadcast_data.rc.throttle= array_to_short(&DataFromDji.data[offSet+6]);
		//std_broadcast_data.rc.mode= array_to_short(&DataFromDji.data[offSet+8]);
		//std_broadcast_data.rc.gear= array_to_short(&DataFromDji.data[offSet+10]);
		memcpy((unsigned char*)&std_broadcast_data.rc.roll,(unsigned char*)(DataFromDji.data+offSet),sizeof(api_rc_data_t));
		//printf("rc=%d %d %d %d %d %d\r\n",std_broadcast_data.rc.roll,std_broadcast_data.rc.pitch,std_broadcast_data.rc.yaw,std_broadcast_data.rc.throttle,std_broadcast_data.rc.mode,std_broadcast_data.rc.gear);
		//printf("rc=%d\r\n",std_broadcast_data.rc.throttle);
		offSet += FLIGHT_DATA_REMOTE_CH_SIZE;
	}
	if(packageStatus & (1<<8)) {		//云台姿态
		memcpy((unsigned char*)&std_broadcast_data.gimbal.x,(unsigned char*)(DataFromDji.data+offSet),sizeof(api_common_data_t));
		offSet += FLIGHT_DATA_CRADLE_HEAD_SIZE;
	}
	if(packageStatus & (1<<9)) {		//飞行状态
		std_broadcast_data.status = *(DataFromDji.data+offSet);
		offSet += FLIGHT_DATA_FLIGHT_STATUS_SIZE;
	}
	if(packageStatus & (1<<10)) {		//剩余电池百分比
		std_broadcast_data.battery_remaining_capacity= *(DataFromDji.data+offSet);
		offSet += FLIGHT_DATA_BATTERY_LEVEL_SIZE;
	}
	if(packageStatus & (1<<11)) {		//控制设备
		memcpy((unsigned char*)&std_broadcast_data.ctrl_info,(unsigned char*)(DataFromDji.data+offSet),sizeof(api_ctrl_info_data_t));
		offSet += FLIGHT_DATA_DEVICE_CTRL_SIZE;
	}
	#endif
}
#if 0
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
void Updata_Flight_Data(void)
{
	u16 packageStatus;
	u8 offSet=2;
	packageStatus = array_to_short(&DataFromDji.data[0]);
	//printf("p=%d\r\n",packageStatus);
	#if 1
	if(packageStatus & (1<<0)) {		//时间戳
		offSet += FLIGHT_DATA_TIME_STAMP_SIZE;
	}
	if(packageStatus & (1<<1)) {		//姿态四元素
		FlightMsg.q.q0 = array_to_float(DataFromDji.data+offSet);
		FlightMsg.q.q1 = array_to_float(&DataFromDji.data[offSet+4]);
		FlightMsg.q.q2 = array_to_float(&DataFromDji.data[offSet+8]);
		FlightMsg.q.q3 = array_to_float(&DataFromDji.data[offSet+12]);
		printf("Q=%f %f %f %f\r\n",FlightMsg.q.q0,FlightMsg.q.q1,FlightMsg.q.q2,FlightMsg.q.q3);
		offSet += FLIGHT_DATA_ATTITUDE_SIZE;
	}
	
	if(packageStatus & (1<<2)) {		//加速度
		offSet += FLIGHT_DATA_ACCELERATE_SIZE;
	}
	if(packageStatus & (1<<3)) {		//速度
		FlightMsg.v.x = array_to_float(&DataFromDji.data[offSet]);
		FlightMsg.v.y = array_to_float(&DataFromDji.data[offSet+4]);
		FlightMsg.v.z= array_to_float(&DataFromDji.data[offSet+8]);
		printf("SPD=%f %f %f\r\n",FlightMsg.v.x,FlightMsg.v.y,FlightMsg.v.z);
		offSet += FLIGHT_DATA_SPEED_SIZE;
	}
	if(packageStatus & (1<<4)) {		//角速度
		offSet += FLIGHT_DATA_ANGLE_SPEED_SIZE;
	}
	if(packageStatus & (1<<5)) {		//gps
		FlightMsg.pos.lati = array_to_double(&DataFromDji.data[offSet]);
		FlightMsg.pos.longti = array_to_double(&DataFromDji.data[offSet+8]);
		FlightMsg.pos.alti = array_to_float(&DataFromDji.data[offSet+12]);
		FlightMsg.pos.height = array_to_float(&DataFromDji.data[offSet+16]);
		printf("GPS=%lf %lf %f %f\r\n",FlightMsg.pos.lati,FlightMsg.pos.longti,FlightMsg.pos.alti,FlightMsg.pos.height);
		offSet += FLIGHT_DATA_GPS_SIZE;
	}
	if(packageStatus & (1<<6)) {		//磁感器
		FlightMsg.mag.x = array_to_short(&DataFromDji.data[offSet]);
		FlightMsg.mag.y = array_to_short(&DataFromDji.data[offSet+2]);
		FlightMsg.mag.z = array_to_short(&DataFromDji.data[offSet+4]);
		printf("MAGNETIC=%d %d %d\r\n",FlightMsg.mag.x,FlightMsg.mag.y,FlightMsg.mag.z);
		offSet += FLIGHT_DATA_MAGNETIC_SIZE;
	}
	#endif
}
#endif
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
void Pro_Receive_Interface(void)
{
  	int Heard_CRC32=0;
	ProFrameData_Unit  RecData;
	SDKHeader RecHeader ;
//	ProAckParameter Ack;
	OS_ERR err;
	memcpy(&RecHeader, serial_sdk.comm_recv_buf,sizeof(SDKHeader));
	
	if (RecHeader.sof != _SDK_SOF) return;
	if (RecHeader.version != 0) return;
	if (RecHeader.length > _SDK_MAX_RECV_SIZE) return;
	if (RecHeader.length > sizeof(SDKHeader) && RecHeader.length < _SDK_FULL_DATA_SIZE_MIN) return;
	
	if(sdk_stream_crc16_calc((unsigned char*)&RecHeader, _SDK_HEAD_DATA_LEN)!=RecHeader.head_crc) return;//校验帧头
	
	memcpy(&Heard_CRC32,&serial_sdk.comm_recv_buf[RecHeader.length - _SDK_CRC_DATA_SIZE] ,_SDK_CRC_DATA_SIZE);
	if(sdk_stream_crc32_calc((unsigned char*)serial_sdk.comm_recv_buf, RecHeader.length - _SDK_CRC_DATA_SIZE)!=Heard_CRC32) return;//整体校验

	RecData.CommandSet = serial_sdk.comm_recv_buf[12];
	RecData.CommandId = serial_sdk.comm_recv_buf[13];
	RecData.dataLen = RecHeader.length - _SDK_FULL_DATA_SIZE_MIN-2;//透传数据的长度
	if(RecData.dataLen>0) {
		memcpy(&RecData.data , &serial_sdk.comm_recv_buf[14],RecData.dataLen);
	}
	memset((unsigned char*)&DataFromDji,0,sizeof(DataFromDji));
	memcpy((unsigned char*)&DataFromDji,(unsigned char*)&RecData,sizeof(RecData));
	#if 0
	if(RecData.dataLen>0) {
		switch(RecData.CommandSet) {
			#if 0
			case 0x01://命令集 控制命令类
				LOG_DJI_STR("Control data!\r\n");
				switch(RecData.CommandId) {
					case 0x00:	//请求获取/释放控制权(飞控至机载设备)
													//返回码
													// 0x0000：拒绝获取控制权(未满足获取控制权条件)
													// 0x0001：成功释放控制权
													// 0x0002：成功获得控制权
													// 0x0003：正在获取控制权		
						switch(RecData.data[0]) {
							case 0x00:
								break;
							case 0x01:
								break;
							case 0x02:
								break;
							case 0x03:
								break;
							default:
								break;
						}
						//USART_Send_Buf(SERIAL_PORT_DEBUG,DataFromDji.data,DataFromDji.dataLen);
						
						break;
					case 0x01:	//切换飞行状态(飞控至机载设备)
						
						break;
					case 0x02:	//查询飞行状态切换结果(飞控至机载设备)
						
						break;
					
					default:
						break;
				}
				break;
			#endif
			case 0x02://命令集 推送数据类,飞控==>机载设备
				switch(RecData.CommandId) {
					case 0x00:	//飞行数据
						Updata_Flight_Data();
						//printf("Flight data!\r\n");
						break;
					case 0x02:	//移动设备至机载设备
						//printf("Mobile data!\r\n");
						//printf("len=%d\r\n",RecData.dataLen);
						USART_Send_Buf(SERIAL_PORT_DEBUG,DataFromDji.data,DataFromDji.dataLen);
						//handle_transparent_transmission(DataFromDji.data);
						break;
					default:
						break;
				}
				break;
			default:
				break;
		}
	} else {
		if((RecData.CommandSet == 0)&&(RecData.CommandId == 0)) {//激活应答
			OSSemPost(&SemDjiActivation,OS_OPT_POST_1,&err);
			printf("ACK Activation!\r\n");
		} else if((RecData.CommandSet == 0x01)&&(RecData.CommandId == 0)) {
			OSSemPost(&SemDjiFlightCtrlRelease,OS_OPT_POST_1,&err);
			printf("ACK release!\r\n");
		} else if((RecData.CommandSet == 0x02)&&(RecData.CommandId == 0)) {
			OSSemPost(&SemDjiFlightCtrlObtain,OS_OPT_POST_1,&err);
			printf("ACK obtain!\r\n");
		}
		// 0x0000：拒绝获取控制权(未满足获取控制权条件)
		// 0x0001：成功释放控制权
		// 0x0002：成功获得控制权
		// 0x0003：正在获取控制权
	}
	#endif
	#if 1
	if(RecHeader.is_ack) {
		if((RecData.CommandSet == 0)&&(RecData.CommandId == 0)) {//激活应答
			OSSemPost(&SemDjiActivation,OS_OPT_POST_1,&err);
			//printf("ACK Activation!\r\n");
		} else if((RecData.CommandSet == 0x01)&&(RecData.CommandId == 0)) {
			OSSemPost(&SemDjiFlightCtrlRelease,OS_OPT_POST_1,&err);
			//printf("ACK release!\r\n");
		} else if((RecData.CommandSet == 0x02)&&(RecData.CommandId == 0)) {
			OSSemPost(&SemDjiFlightCtrlObtain,OS_OPT_POST_1,&err);
			//printf("ACK obtain!\r\n");
		}
		// 0x0000：拒绝获取控制权(未满足获取控制权条件)
		// 0x0001：成功释放控制权
		// 0x0002：成功获得控制权
		// 0x0003：正在获取控制权	
	}else {
		switch(RecData.CommandSet) {
			#if 0
			case 0x01://命令集 控制命令类
				LOG_DJI_STR("Control data!\r\n");
				switch(RecData.CommandId) {
					case 0x00:	//请求获取/释放控制权(飞控至机载设备)
													//返回码
													// 0x0000：拒绝获取控制权(未满足获取控制权条件)
													// 0x0001：成功释放控制权
													// 0x0002：成功获得控制权
													// 0x0003：正在获取控制权		
						switch(RecData.data[0]) {
							case 0x00:
								break;
							case 0x01:
								break;
							case 0x02:
								break;
							case 0x03:
								break;
							default:
								break;
						}
						//USART_Send_Buf(SERIAL_PORT_DEBUG,DataFromDji.data,DataFromDji.dataLen);
						
						break;
					case 0x01:	//切换飞行状态(飞控至机载设备)
						
						break;
					case 0x02:	//查询飞行状态切换结果(飞控至机载设备)
						
						break;
					
					default:
						break;
				}
				break;
			#endif
			case 0x02://命令集 推送数据类,飞控==>机载设备
				switch(RecData.CommandId) {
					case 0x00:	//飞行数据
						Updata_Flight_Data();
						//printf("Flight data!\r\n");
						break;
					case 0x02:	//移动设备至机载设备
						//printf("Mobile data!\r\n");
						//USART_Send_Buf(SERIAL_PORT_DEBUG,DataFromDji.data,DataFromDji.dataLen);
						handle_transparent_transmission(DataFromDji.data);
						break;
					default:
						break;
				}
				break;
			default:
				break;
		}
	}
	#endif
}
/**
 * @brief 透传数据到移动设备
 * @param data 数组头指针
 * @param len 数据长度
 */
void send_data_to_mobile(u8 *data,unsigned char len)
{
    //具体实现
    DJI_Pro_Send_To_Mobile_Device(data,len,0);
}

/**
 * @brief 发送设备信息 0x08 0x00
 * @param device_id 出厂编号
 * @param is_usable 是否可用
 */
void send_device_info(uint32_t device_id, uint8_t is_usable)
{
    uint8_t cmd[7];
    cmd[0] = 0x08;
    cmd[1] = 0x00;
    cmd[2] = (device_id >> 24)&0xff;
    cmd[3] = (device_id >> 16)&0xff;
    cmd[4] = (device_id >> 8)&0xff;
    cmd[5] = device_id&0xff;
    cmd[6] = is_usable;

    send_data_to_mobile(cmd,7);
}

/**
 * @brief 发送飞行数据到移动端 0x08 0x01
 * @param latitude 纬度 deg
 * @param longitude 经度 deg
 * @param altitude 海拔高度 m
 * @param height 对地高度 m
 * @param speed 速度 m/s
 * @param target_waypoint 目标航点 0-100
 * @param yaw 偏航角 deg
 * @param pump_current 水泵电流 A
 * @param is_onboard_controlling 机载设备是否正在控制
 * @param uint8_t is_pump_running 水泵是否在运转
 * @param uint8_t is_dose_run_out 是否药尽
 */
void send_flight_data(float latitude, float longitude, float altitude,
                      float height, float speed, uint8_t target_waypoint,
                      float yaw, float pump_current, uint8_t is_onboard_controlling,
                      uint8_t is_pump_running, uint8_t is_dose_run_out)
{
    uint8_t cmd[37];
    toFloat tt;
    
    cmd[0] = 0x08;
    cmd[1] = 0x01;


    tt.value = latitude;
    cmd[2] = tt.bytePtr[0];
    cmd[3] = tt.bytePtr[1];
    cmd[4] = tt.bytePtr[2];
    cmd[5] = tt.bytePtr[3];

    tt.value = longitude;
    cmd[6] = tt.bytePtr[0];
    cmd[7] = tt.bytePtr[1];
    cmd[8] = tt.bytePtr[2];
    cmd[9] = tt.bytePtr[3];

    tt.value = altitude;
    cmd[10] = tt.bytePtr[0];
    cmd[11] = tt.bytePtr[1];
    cmd[12] = tt.bytePtr[2];
    cmd[13] = tt.bytePtr[3];

    tt.value = height;
    cmd[14] = tt.bytePtr[0];
    cmd[15] = tt.bytePtr[1];
    cmd[16] = tt.bytePtr[2];
    cmd[17] = tt.bytePtr[3];

    tt.value = speed;
    cmd[18] = tt.bytePtr[0];
    cmd[19] = tt.bytePtr[1];
    cmd[20] = tt.bytePtr[2];
    cmd[21] = tt.bytePtr[3];

    cmd[22]  = target_waypoint;

    tt.value = yaw;
    cmd[23] = tt.bytePtr[0];
    cmd[24] = tt.bytePtr[1];
    cmd[25] = tt.bytePtr[2];
    cmd[26] = tt.bytePtr[3];

    tt.value = pump_current;
    cmd[27] = tt.bytePtr[0];
    cmd[28] = tt.bytePtr[1];
    cmd[29] = tt.bytePtr[2];
    cmd[30] = tt.bytePtr[3];

    cmd[31] = is_onboard_controlling;
    cmd[32] = is_pump_running;
    cmd[33] = is_dose_run_out;

    send_data_to_mobile(cmd,34);
}

/**
 * @brief 发送获取航点数据包的请求
 * @param packet_idx
 */
void send_waypoint_request(uint8_t packet_idx)
{
    uint8_t cmd[3];
    cmd[0] = 0x07;
    cmd[1] = 0x02;
    cmd[2] = packet_idx;

    send_data_to_mobile(cmd,3);
}

/**
 * @brief 发送航点数据包 共93字节
 * @param packet_idx
 */
void send_waypoint_packet(uint8_t packet_idx)
{
    uint8_t data[93];
    uint8_t t = 3;
    int i;
    uint8_t m;
    toFloat tt;
    data[0] = 0x07;
    data[1] = 0x01;
    data[2] = packet_idx;

    

    for(i = packet_idx*10; i < (packet_idx*10 + 10); i++)
    {
        if(i > 99)
        {
            break;
        }

        data[t] = i;
        t++;

        
        tt.value = ap_get_waypoint_lla(i).lat/M_PI*180;
        for( m = 0; m < 4; m++)
        {
            data[t] = tt.bytePtr[m];
            t++;
        }

        tt.value = ap_get_waypoint_lla(i).lon/M_PI*180;
        for( m = 0; m < 4; m++)
        {
            data[t] = tt.bytePtr[m];
            t++;
        }

        //printf("waypoint to mobile:%d,lat %f lon %f\n",i,ap_get_waypoint_lla(i).lat,ap_get_waypoint_lla(i).lon);
    }

    send_data_to_mobile(data,93);
}

/**
 * @brief 停止航点发送
 */
void end_waypoint_transfer()
{
    uint8_t cmd[2];
    cmd[0] = 0x07;
    cmd[1] = 0x03;

    send_data_to_mobile(cmd,2);
}

//处理收到的透传数据
//buf[0] 0x06 地面站控制指令
//       buf[1] 0x00 开始航线飞行
//       buf[1] 0x01 停止航线飞行
//       buf[1] 0x02 获取设备信息
//buf[0] 0x07 航线上传下载相关指令
//       buf[1] 0x00 航线头部信息（高度、速度、间距、面积）
//              0x01 航点数据包
//              0x02 请求获取航点数据包
//              0x03 停止获取航点数据包
//              0x04 地面站发送的航线任务下载请求
void handle_transparent_transmission(u8 *buf)
{
	uint8_t i;
	uint8_t index;
	uint8_t cmd[22];
	float lat;
	float lon;
	toFloat tt;
	OS_ERR err;
	u8 msg = MSG_TYPE_NAV_START;
    if(buf[0] == 0x06)//地面站控制命令,执行前先获取控制权
    {
        if(buf[1] == 0x00)//开始航线飞行
        {
            //DJI_Sample_Funny_Ctrl(DRAW_SQUARE_SAMPLE);
            //OSSemPost(&SemDjiFlightCtrlObtain,OS_OPT_POST_1,&err);
            
			#if 1
			OSQPost((OS_Q*		)&QAutoNav, 	
					(void*		)&msg,
					(OS_MSG_SIZE)1,
					(OS_OPT 	)OS_OPT_POST_FIFO,
					(OS_ERR*	)&err);
			//printf("\r\npost q!\r\n",index,lat,lon);
			#endif
        }
        else if(buf[1] == 0x01)//停止航线飞行
        {
        	//printf("\r\nstopppppppppp!\r\n",index,lat,lon);
			//OSSemPost(&SemDjiFlightCtrlRelease,OS_OPT_POST_1,&err);
        }
        else if(buf[1] == 0x02)//获取设备信息
        {
            send_device_info(device_id,is_usable);
        }
        else if(buf[1] == 0x03)//增加航线高度
        {
            d_alt += 0.1*buf[2];
        }
        else if(buf[1] == 0x04)//减小航线高度
        {
            d_alt -= 0.1*buf[2];
        }
    }
    else if(buf[0] == 0x07)//航点上传下载相关命令
    {
        if(buf[1] == 0x00)//收到地面站的航线上传请求
        {
            for( i = 0; i < 100; i++)
            {
                ap_set_waypoint(i,0,0,0);
            }
            wp_download_finished = FALSE;
            wp_packet_idx = 0;

            task_altitude = array_to_float(&buf[2]);
            task_speed = array_to_float(&buf[6]);
            flight_plan_offset = array_to_float(&buf[10]);
            task_area = array_to_float(&buf[14]);
            task_distance = array_to_float(&buf[18]);

            //printf("mission altitude:%f,speed:%f,offset:%f,area:%f\n",task_altitude,task_speed,flight_plan_offset,task_area);

            send_waypoint_request(wp_packet_idx++);
        }
        else if(buf[1] == 0x01)//地面站上传的数据包
        {
            for( i = 0; i < 90; i+=9)
            {
                index = buf[i+2];
                lat = array_to_float(&buf[i+3]);
                lon = array_to_float(&buf[i+7]);

                if(index == 0)
                {
                    core_state.lla_origin.lat = (double)lat/180.0*M_PI;
                    core_state.lla_origin.lon = (double)lon/180.0*M_PI;
                    core_state.lla_origin.alt = task_altitude;
                }

                ap_set_waypoint(index,(double)lat/180.0*M_PI,(double)lon/180.0*M_PI,task_altitude);

                //printf("waypoint from mobile:%d,%f,%f\n",index,lat,lon);
                //printf("waypoint from mobile enu:%f,%f,%f\n",ap_get_waypoint(index).x,ap_get_waypoint(index).y,ap_get_waypoint(index).z);

                if(fabs(lat) <= 0.1)
                {
                    wp_download_finished = TRUE;
                    
                }
            }

            if((wp_packet_idx <= 9)&&(!wp_download_finished))
            {
                send_waypoint_request(wp_packet_idx++);
            }

            if(wp_download_finished)
            {
                end_waypoint_transfer();
                end_waypoint_transfer();
                end_waypoint_transfer();
                printf("\r\nupload done!\r\n");
            }
        }
        else if(buf[1] == 0x02)//地面站请求获取航点数据包
        {
            index = buf[2];//数据包索引
            send_waypoint_packet(index);
        }
        else if(buf[1] == 0x04)//地面站发送的航线任务下传请求
        {
            cmd[0] = 0x07;
            cmd[1] = 0x00;

            

            tt.value = task_altitude;
            cmd[2] = tt.bytePtr[0];
            cmd[3] = tt.bytePtr[1];
            cmd[4] = tt.bytePtr[2];
            cmd[5] = tt.bytePtr[3];

            tt.value = task_speed;
            cmd[6] = tt.bytePtr[0];
            cmd[7] = tt.bytePtr[1];
            cmd[8] = tt.bytePtr[2];
            cmd[9] = tt.bytePtr[3];

            tt.value = flight_plan_offset;
            cmd[10] = tt.bytePtr[0];
            cmd[11] = tt.bytePtr[1];
            cmd[12] = tt.bytePtr[2];
            cmd[13] = tt.bytePtr[3];

            tt.value = task_area;
            cmd[14] = tt.bytePtr[0];
            cmd[15] = tt.bytePtr[1];
            cmd[16] = tt.bytePtr[2];
            cmd[17] = tt.bytePtr[3];

            tt.value = task_distance;
            cmd[18] = tt.bytePtr[0];
            cmd[19] = tt.bytePtr[1];
            cmd[20] = tt.bytePtr[2];
            cmd[21] = tt.bytePtr[3];

            send_data_to_mobile(cmd,22);
            printf("\r\ndownload done!\r\n");
        }
    }
}

