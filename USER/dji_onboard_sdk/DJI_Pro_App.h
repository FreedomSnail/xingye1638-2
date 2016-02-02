/*
 * DJI_Pro_App.h
 *
 *  Created on: Mar 17, 2015
 *      Author: Ying Jiahang
 */

#ifndef __DJI_PRO_APP_H__
#define __DJI_PRO_APP_H__
/*
 * cmd_set    cmd_id     data
 * (1byte)   (1byte)    (* byte)
 */
#include <stdint.h>
#include "DJI_Pro_Codec.h"
#include "DJI_Pro_Link.h"
#include "DJI_Pro_App.h"
#define MY_DEV_ID               0x00
#define MY_ACTIVATION_SET       0x00
#define MY_CTRL_CMD_SET         0x01
#define MY_BROADCAST_CMD_SET    0x02
// cmd_id
#define API_VER_QUERY	      	0x00
#define API_CTRL_MANAGEMENT     0x00
#define API_OPEN_SERIAL         0x00
#define API_STD_DATA            0x00
#define API_CMD_REQUEST       	0x01
#define API_CMD_STATUS_REQUEST 	0x02
#define API_CTRL_REQUEST      	0x03
#define API_TRANSPARENT_DATA_TO_MOBILE  0xFE
#define API_TRANSPARENT_DATA_TO_OBOARD  0x02
#define API_GIMBAL_CTRL_SPEED_REQUEST   0x1A
#define API_GIMBAL_CTRL_ANGLE_REQUEST   0x1B

#define API_MISSION_WP_INFO   	0x10
#define API_MISSION_WP_DATA   	0x11
#define API_MISSION_WP_CMD    	0x12

#define API_MISSION_HP_START  	0x20
#define API_MISSION_HP_CMD    	0x21

#define API_CTRL_GIMBAL_SPEED 	0x1A

#define API_VERSION_QUERY       0x00
#define API_USER_ACTIVATION     0x01
#define API_INFO_QUERY          0x02
#define	API_SIM_ECHO            0xFF

#define API_CAMERA_SHOT         0x20
#define API_CAMERA_VIDEO_START  0x21
#define API_CAMERA_VIDEO_STOP   0X22

#define HORIZ_ATT               0x00
#define HORIZ_VEL               0x40
#define HORIZ_POS               0x80

#define VERT_VEL                0x00
#define VERT_POS                0x10
#define VERT_TRU                0x20

#define YAW_ANG                 0x00
#define YAW_RATE                0x08

#define HORIZ_GND               0x00
#define HORIZ_BODY              0x02

#define YAW_GND                 0x00
#define YAW_BODY                0x01

#define MAKE_VERSION(a,b,c,d) (((a << 24)&0xff000000) | ((b << 16)&0x00ff0000) | ((c << 8)&0x0000ff00) | (d&0x000000ff))
#define SDK_VERSION           (MAKE_VERSION(3,1,100,0))

#define DATA_LENGTH_RECEIVE_PUMP_CONTROL_BOARD	27
#define DATA_LENGTH_SEND_PUMP_CONTROL_BOARD		13


// data_type
typedef float 	fp32;
typedef double	fp64;

//----------------------------------------------------------------------
// uav std_msgs reciever
//----------------------------------------------------------------------
#define MSG_ENABLE_FLAG_LEN		2

#define ENABLE_MSG_TIME			0x0001
#define ENABLE_MSG_Q			0x0002
#define ENABLE_MSG_A			0x0004
#define ENABLE_MSG_V			0x0008
#define ENABLE_MSG_W			0x0010
#define ENABLE_MSG_POS			0x0020
#define ENABLE_MSG_MAG			0x0040
#define ENABLE_MSG_RC			0x0080
#define ENABLE_MSG_GIMBAL		0x0100
#define ENABLE_MSG_STATUS		0x0200
#define ENABLE_MSG_BATTERY		0x0400
#define ENABLE_MSG_DEVICE		0x0800

#pragma  pack(1)

typedef union 
{
    unsigned char bytePtr[2];
    uint16_t value;
}toShort;

typedef union 
{
    unsigned char bytePtr[4];
    uint32_t value;
}toInt;

typedef union 
{
    unsigned char bytePtr[4];
    float value;
}toFloat;

typedef union 
{
    unsigned char bytePtr[8];
    double value;
}toDouble;


typedef struct
{
	unsigned char ctrl_flag;
	fp32 	roll_or_x;
	fp32	pitch_or_y;
	fp32	thr_z;
	fp32	yaw;
}attitude_data_t;

typedef struct
{
	fp32	q0;
	fp32	q1;
	fp32	q2;
	fp32	q3;
}api_quaternion_data_t;

typedef struct
{
	fp32	x;
	fp32	y;
	fp32	z;
}api_common_data_t;
/*
 *struct of vellocity data
 */

typedef struct
{
    fp32 x;
    fp32 y;
    fp32 z;
    unsigned char health_flag         :1;
    unsigned char feedback_sensor_id  :4;
    unsigned char reserve             :3;
}api_vel_data_t;

typedef struct
{
	fp64	lati;
	fp64	longti;
	fp32	alti;
	fp32	height;
	unsigned char health_flag;
}api_pos_data_t;

typedef struct
{
	int16_t	roll;
	int16_t	pitch;
	int16_t	yaw;
	int16_t	throttle;
	int16_t	mode;
	int16_t gear;
}api_rc_data_t;

typedef struct
{
	int16_t	x;
	int16_t	y;
	int16_t	z;
}api_mag_data_t;

typedef struct
{
    unsigned char cur_ctrl_dev_in_navi_mode   :3;/*0->rc  1->app  2->serial*/
    unsigned char serial_req_status           :1;/*1->opensd  0->close*/
    unsigned char reserved                    :4;
}api_ctrl_info_data_t;


typedef struct
{
	unsigned int			time_stamp;
	api_quaternion_data_t	q;
	api_common_data_t		a;
	api_common_data_t		v;
	api_common_data_t		w;
	api_pos_data_t			pos;
	api_mag_data_t			mag;
	api_rc_data_t			rc;
	api_common_data_t		gimbal;
	unsigned char			status;
	unsigned char			battery_remaining_capacity;
	api_ctrl_info_data_t	ctrl_info;
}sdk_std_msg_t;

#pragma  pack()

//----------------------------------------------------------------------
// App layer function
//----------------------------------------------------------------------
typedef struct 
{
    unsigned short	sequence_number;
    unsigned char	session_id 	: 5;
    unsigned char	need_encrypt	: 1;
    unsigned char	reserve	   	: 2;
}req_id_t;

typedef int16_t (*func_cmd_handler)(unsigned char cmd_id,unsigned char* pbuf,unsigned short len, req_id_t req_id);
typedef struct _cmd_tab
{
    unsigned char             cmd_id;
    func_cmd_handler    pf_cmd_handler;
}cmd_handler_table_t;

typedef struct _set_tab
{
    unsigned char                cmd_set;
    cmd_handler_table_t*   p_cmd_handler_table;
}set_handler_table_t;

#define DATA_MAX_SIZE 	(1000u)
#define ERR_INDEX       (0xff)
#define EXC_DATA_SIZE	(16u)
#define SET_CMD_SIZE	(2u)
typedef struct
{
	unsigned char     len;
    unsigned char     cmd_set;
    unsigned char     cmd_id;
    unsigned char     data_buf[DATA_MAX_SIZE];
}dji_sdk_data_msg_t;

// app_send_func.
void App_Send_Data(unsigned char flag, unsigned char is_enc, unsigned char  cmd_set, unsigned char cmd_id,unsigned char *pdata,int len);


//----------------------------------------------------------------------
// for cmd agency
//----------------------------------------------------------------------

#define	REQ_TIME_OUT			0x0000
#define REQ_REFUSE			0x0001
#define CMD_RECIEVE			0x0002
#define STATUS_CMD_EXECUTING		0x0003
#define STATUS_CMD_EXE_FAIL		0x0004
#define STATUS_CMD_EXE_SUCCESS		0x0005

typedef void (*cmd_ack_callback)(unsigned short *ack);

typedef struct
{
	unsigned char			cmd_sequence;
	unsigned char			cmd_data;
}cmd_agency_data_t;

typedef struct
{
	unsigned char 		is_send_cmd;
	cmd_agency_data_t	cmd;
	unsigned short 		ack_result;
	cmd_ack_callback 	ack_callback;
}dji_sdk_cmd_unit;

void App_Complex_Send_Cmd(unsigned char cmd);

//----------------------------------------------------------------------
// for activation 
//----------------------------------------------------------------------

#define SDK_ERR_SUCCESS			0x0000
#define SDK_ERR_COMMAND_NOT_SUPPORTED	0xFF00
#define SDK_ERR_NO_AUTHORIZED		0xFF01
#define SDK_ERR_NO_RIGHTS		0xFF02

typedef struct
{
	unsigned int	app_id;
	unsigned int	app_api_level;
	unsigned int	app_ver;
	unsigned char	app_bundle_id[32];
}activation_data_t;

typedef struct
{
	unsigned short	version_ack;
	unsigned int	version_crc;
	char     	version_name[32];
}version_query_data_t;




typedef struct ProFrameData_Unit
{
	unsigned char CommandSet;
	unsigned char CommandId;
	unsigned char data[128];
	unsigned short dataLen;
}ProFrameData_Unit;

typedef struct
{
	unsigned char is_pump_running;	//水泵是否工作
	float pump_voltage;				//水泵工作电压
	float supply_voltage;			//供电电压，外部提供给水泵控制板的电压
	unsigned char is_dose_run_out;	//农药量是否耗尽
	unsigned char is_usable;		//是否可用
	uint64_t device_id;				//出厂编号
}pump_board_data_t;



typedef void (*Command_Result_Notify)(unsigned short result);



#define MSG_TYPE_RC_CTRL		1
#define MSG_TYPE_NAV_START		2
#define MSG_TYPE_NAV_OBTAIN_CTL	3
#define MSG_TYPE_NAV_DONE		4


extern OS_SEM SemDjiCodec;
extern OS_SEM SemDjiActivation;
extern OS_SEM SemDjiFlightCtrlObtain;
extern OS_SEM SemDjiFlightCtrlRelease;
extern OS_SEM SemCtrlPump;
extern OS_Q   QAutoNav;



extern ProFrameData_Unit  DataFromDji;
extern pump_board_data_t pumpBoardInfo;

void DJI_Onboard_API_Activation_Init(void);

void DJI_Onboard_API_Activation(void);

void DJI_Onboard_API_Control(unsigned char arg);

void DJI_Onboard_send(u8* str, u8 len);
void send_data_to_mobile(u8 *data,unsigned char len);

int DJI_Pro_Attitude_Control(attitude_data_t *p_user_data);


void Pro_Receive_Interface(void);
void Updata_Flight_Data(void);

void send_flight_data(float latitude, float longitude, float altitude,
                      float height, float speed, uint8_t target_waypoint,
                      float yaw, float pump_current, uint8_t is_onboard_controlling,
                      uint8_t is_pump_running, uint8_t is_dose_run_out,uint8_t pump_permission,uint64_t device_id);


void handle_transparent_transmission(u8 *buf);

void send_cmd_to_pump_board(u8* str,u8 len);

void send_cmd_to_flight_ctrl_board(u8* str,u8 len);

void Pro_Receive_Pump_Ctrl_Board(void);

#endif
