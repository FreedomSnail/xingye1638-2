/****************************************Copyright(c)********************************************
**
**                         2013-2023, Freedom ...
**
**----------------------------------------File Info----------------------------------------------
** File name : DJI_Pro_Flight_Ctrl.c
** Latest modified date :
** Latest version :
** Description :
**-----------------------------------------------------------------------------------------------
** Created by : 陆志
** Created date :2016年1月21日10:37:52
** Version :
** Description :
**-----------------------------------------------------------------------------------------------
** Modified by :
** Modified date :
** Version :
** Description :
************************************************************************************************/
#include "stm32f4xx.h"
#include "bsp_uart.h"
#include "includes.h"

#include "DJI_Pro_Codec.h"
#include "DJI_Pro_Link.h"
#include "DJI_Pro_App.h"
#include "DJI_Pro_Flight_Ctrl.h"
#include "ap_core.h"
#include "navigation.h"
#include "geodetic.h"


float d_alt = 0;        ///<遥控干预航线高度偏差
float d_alt_p_gain = 10;///<高度偏差累加 增益P

int block = 0;

float task_altitude = 0;        ///<航线高度
float target_altitude = 0;
float task_speed = 0;           ///<航线速度

sdk_std_msg_t std_broadcast_data;

api_common_data_t GetVelInfo(void)
{
	return std_broadcast_data.v;
}
api_pos_data_t GetPosInfo(void)
{
	return std_broadcast_data.pos;
}
int GetRcThrottleInfo(void)
{
    return std_broadcast_data.rc.throttle;
}

int GetRcPitchInfo(void)
{
    return std_broadcast_data.rc.pitch;
}

int16_t GetRcGearInfo(void)
{
    return std_broadcast_data.rc.gear;
}

void ctrl_attitude_alt(float pitch, float roll, float yaw, float altitude)
{
    //获取遥控器油门-10000 ~ 10000
    int throttle;
    float target_alt;
    attitude_data_t user_ctrl_data;
   
    throttle= GetRcThrottleInfo();
	//printf("%d\r\n",throttle);
	//printf("%f\r\n",altitude);
	
    d_alt += d_alt_p_gain*(throttle/10000000.0);

    if(d_alt > 10)
    {
        d_alt = 10;
    }
    else if(d_alt < -10)
    {
        d_alt = -10;
    }

    //目标高度不能低于2m
    if((altitude + d_alt) < 2)
    {
        d_alt = 2 - altitude;
    }

    target_alt = altitude + d_alt;
	
    user_ctrl_data.ctrl_flag = 0x50;
    user_ctrl_data.roll_or_x = roll;
    user_ctrl_data.pitch_or_y = pitch;
    user_ctrl_data.thr_z = target_alt;
    user_ctrl_data.yaw = yaw;
    DJI_Pro_Attitude_Control(&user_ctrl_data);
}

void ctrl_enu_speed(float speed_e, float speed_n, float yaw, float speed_u)
{
    attitude_data_t user_ctrl_data;
	
    user_ctrl_data.ctrl_flag = 0x40;
    user_ctrl_data.roll_or_x = speed_n;
    user_ctrl_data.pitch_or_y = speed_e;
    user_ctrl_data.thr_z = speed_u;
    user_ctrl_data.yaw = yaw;
    DJI_Pro_Attitude_Control(&user_ctrl_data);
}

u8 auto_nav_check_gps(void)
{
    if (GetPosInfo().health_flag <= 2) {
       return 0;
    } else {
		return 1;
    }
}

void auto_nav_math_init(void)
{
	api_pos_data_t pos;
    //初始化ap_core
    ap_init_core();
    //设置飞行器初始位置
    pos = GetPosInfo();
    ap_set_position(pos.lati,pos.longti,pos.height);
    //初始化导航
    ap_nav_init();
    block = 0;
    core_state.target_waypoint = 0;
}

u8 auto_nav_check_height(void)
{
	api_pos_data_t pos;
    pos = GetPosInfo();
    target_altitude = task_altitude;
    //若高度不足2m，先上升到航线任务高度
    if (pos.height < 2) {
    	return 1;
    } else {
		return 0;
    }
}


float kp = 1.5;
float kd = 0.0;
float last_vert_pos_error = 0;
float vert_speed_max = 1;
float vert_speed_min = -1;

float run_alitutde_pid(float alt_current, float alt_setpoint)
{
    float vert_speed_setpoint = 0;
    float vert_pos_error = alt_setpoint - alt_current;
    float d_vert_pos_error = vert_pos_error - last_vert_pos_error;
    vert_speed_setpoint = kp * vert_pos_error + kd * d_vert_pos_error;
    last_vert_pos_error = d_vert_pos_error;
    
    if (vert_speed_setpoint > vert_speed_max) {
        vert_speed_setpoint = vert_speed_max;
    }
    
    if (vert_speed_setpoint < vert_speed_min) {
        vert_speed_setpoint = vert_speed_min;
    }
    
    return vert_speed_setpoint;
}

/************************************************************************************************
** Function name :		  
** Description :
** 
** Input :
** Output :
** Return :
** Others :
** 取10个样本进行平均值计算，采集10个样本的时间大约需要1s时间。
这个10个样本里，数据大于11.0米的会被抛弃
小于11米的数据要与气压传感器的高度数据比较，相对误差范围小于(0.6f+airHeight*0.2f)的数据
被认为是超声波的有效值，记录下10个样本里的所有效值数据最后求平均值
************************************************************************************************/
void Ultra_Sonic_Wave_Software_Filter(u16 sampleHeight)
{
	static float sampleHeightSum = 0;
	static float airHeightSum = 0;
	static u8 sampleCnt = 0;
	static u8 validDataCnt = 0;
    //static float last_sample_height = 0;
	float airHeight;
	float airHeightAverage;
	float sampleHeightAverage;
	float sample;
	airHeightAverage = airHeightAverage;//防止编译警告
	airHeight = GetPosInfo().height;
	sample = sampleHeight*0.001f;
	airHeightSum += airHeight;
	if(sampleCnt<10) {
		sampleCnt++;
        if((sample<8.0f) && (sample>0.4f)) {//超量程或者收不到回波信号
            sampleHeightSum += sample;
            validDataCnt++;  
        }
	} else {
		sampleCnt = 0;
		airHeightAverage = airHeightSum/10;
		airHeightSum = 0;
		if(validDataCnt > 0) {
			sampleHeightAverage = sampleHeightSum/validDataCnt;
			sampleHeightSum = 0;
			validDataCnt = 0;
			ultraSonicHeight = sampleHeightAverage*1000;
		}
	}
}

u8 is_use_ultra_sonic = 1;

void ctrl_attitude_alt_by_speed(float speed_e, float speed_n, float yaw, float altitude)
{
    float verts_speed_setpoint;
    
    if (is_use_ultra_sonic) {
        if ((ultraSonicHeight > 8)||(ultraSonicHeight < 0.4)) {
            verts_speed_setpoint = run_alitutde_pid(GetPosInfo().height,altitude); 
        } else {
            verts_speed_setpoint = run_alitutde_pid(ultraSonicHeight,altitude); 
        } 
    } else {
        verts_speed_setpoint = run_alitutde_pid(GetPosInfo().height,altitude); 
    }
    
    
    ctrl_enu_speed(speed_e, speed_n, yaw*180/M_PI,verts_speed_setpoint);   
}

u8 auto_nav_raise_to_tartget_height(void)
{
	float height;
	float speed;
	u8 result = 0;
    height = GetPosInfo().height;
    speed = GetVelInfo().z;
    //ctrl_attitude_alt(0, 0, 0, task_altitude);
    ctrl_attitude_alt_by_speed(0, 0, 0, task_altitude);
    //printf("height=%f,v=%f\r\n",height,speed);
	if((task_altitude - height) < 0.1f) {
		result = 1;
	} else if(fabsf(task_altitude - height) < 0.5f){
		if( speed < 0.03f ) {
			result = 1;
		}
	}  
	return result;
}

u8 auto_nav(void)
{
	api_pos_data_t pos;
	struct EnuCoor enu_speed;
	api_common_data_t vel;
	int target_wp;
	bool_t result;
	float speed;
	float yaw;
	float speed_n;
	float speed_e;
	float speed_u;
	float speed_temp;
    bool_t flight_back = FALSE;
    //float next_line_bearing = 0;
    float yaw_kp = 1;
	u8 rlt=0;
	//u8 msg;
	//OS_ERR err;
	int throttle;
	//设置ENU速度
	yaw_kp = yaw_kp;//防止编译警告

	vel = GetVelInfo();
	enu_speed.x = vel.y;
	enu_speed.y = vel.x;
	enu_speed.z = vel.z;
	ap_set_speed_enu(&enu_speed);

	//设置飞行器位置
	pos = GetPosInfo();
	ap_set_position(pos.lati,pos.longti,pos.height);
	
	target_wp = core_state.target_waypoint;
	switch (block) {
	    case 0:
	        ap_nav_go_to_waypoint(core_state.target_waypoint);
	        result = ap_nav_is_approaching_waypoint(core_state.target_waypoint,1);
	        if(result) {
	            core_state.target_waypoint++;
							block ++;
				//printf("waypoint %d arrived \n",core_state.target_waypoint);block++;
	            //printf("pos e:%f n:%f\n",ap_get_position_enu()->x,ap_get_position_enu()->y);
	        }
	    	break;
	    case 1:
            ap_nav_route_line(ap_get_position_enu(),&ap_enu_waypoints[target_wp-1],&ap_enu_waypoints[target_wp]);
        
	        if(ap_nav_is_approaching_waypoint(core_state.target_waypoint,1)) {
	            core_state.target_waypoint++;
	        }
	        break;
	    default:
	        break;
	}

	speed = task_speed;
    
    if(GetRcPitchInfo() < -10) {      
        speed += 1.5f*task_speed*GetRcPitchInfo()/10000.0f;      
        if(speed < 0.0f) {
            if (target_wp >= 1) {
                speed = -speed;
                flight_back = TRUE;
                ap_nav_route_line(ap_get_position_enu(),&ap_enu_waypoints[target_wp],&ap_enu_waypoints[target_wp-1]);                    
            } else {
                speed = 0.0f;
            }
        }
        if(speed > 4) {
            speed = 4;
        }
    }

    //判断与目标航点距离，提前减速
	if(ap_get_dist_to_target_wp() < 10) {
	    if(speed > 5) {
			speed = 5;
	    }
	}
	if(ap_get_dist_to_target_wp() < 8) {
	    if(speed > 3) {
	        speed = 3;
	    }
	}
	if(ap_get_dist_to_target_wp() < 4) {
	    if(speed > 2) {
	        speed = 2;
	   	}
	}
    
    //判断与当前航点距离，限制加速
    /*
    core_state.target_waypoint--;
    if(ap_get_dist_to_target_wp() < 8) {
	    if(speed > 5) {
			speed = 5;
	    }
	}
    if(ap_get_dist_to_target_wp() < 6) {
	    if(speed > 4) {
			speed = 4;
	    }
	}
	if(ap_get_dist_to_target_wp() < 4) {
	    if(speed > 3) {
	        speed = 3;
	    }
	}
	if(ap_get_dist_to_target_wp() < 2) {
	    if(speed > 1) {
	        speed = 1;
	   	}
	}
    core_state.target_waypoint++;
    */
    
    yaw_kp = 1;
    
    if(ap_get_dist_to_target_wp() < 3) {
        yaw_kp = 0.4;
    }
    if(ap_get_dist_to_target_wp() < 2) {
        yaw_kp = 0.2;
    }
    if(ap_get_dist_to_target_wp() < 1) {
        yaw_kp = 1;
    }
    
	if(ap_lla_waypoints[target_wp].lat != 0) {
        
        /*
        next_line_bearing = 0;
        
        if ((ap_get_dist_to_target_wp() < 3)&&(ap_lla_waypoints[target_wp + 1].lat != 0)) {
            next_line_bearing = ap_get_bearing(ap_lla_waypoints[target_wp],ap_lla_waypoints[target_wp+1]);  
            printf("h_ctl_course_setpoint:%f\r\nnext_line_bearing:%f\r\n",h_ctl_course_setpoint,next_line_bearing);
        }
        if (target_wp > 0) {
            h_ctl_course_setpoint = yaw_kp * h_ctl_course_setpoint + (1 - yaw_kp) * next_line_bearing;
            printf("h_ctl_course_setpoint result:%f\r\n\r\n",h_ctl_course_setpoint);
        }
        */
        yaw = h_ctl_course_setpoint;
             
	    if((ap_get_dist_to_target_wp() < 3)&& (ap_lla_waypoints[target_wp+1].lat != 0)) {
	        yaw = ap_get_bearing(*ap_get_position_lla(),ap_lla_waypoints[target_wp+1]);
	    }
        
        /*
        next_line_bearing = 0;
        if (ap_lla_waypoints[target_wp + 1].lat != 0) {
            next_line_bearing = ap_get_bearing(ap_lla_waypoints[target_wp],ap_lla_waypoints[target_wp+1]);  
        }
        */
        
	    speed_temp = sqrt(enu_speed.x*enu_speed.x+enu_speed.y*enu_speed.y) + 0.1;
		if(speed_temp < speed) {
			speed = speed_temp;
		}
        
	    if(yaw > M_PI) {
	        yaw = yaw - 2*M_PI;
	    }
    
        if(flight_back) {
            if (target_wp >= 1) {
                if (yaw > 0) {
                    yaw -= 3.141592653f;
                } else {
                    yaw += 3.141592653f;
                }
            }
        }
        
	    speed_n = cosf(h_ctl_course_setpoint)*speed;
	    speed_e = sinf(h_ctl_course_setpoint)*speed;
	    
	    //获取遥控器油门-10000 ~ 10000
        throttle = GetRcThrottleInfo();
        
        //printf("ctrl_attitude_alt() alt:%f\n",target_altitude);
        
        //发送控制指令
        if (fabsf(throttle) < 10) {
            //ctrl_attitude_alt(speed_e, speed_n, yaw*180/M_PI, target_altitude); 
            speed_u = run_alitutde_pid(pos.height,target_altitude);        
        } else {
            speed_u = 2*(throttle/10000.0); 
            //记录干预垂直速度后的高度
            target_altitude = pos.height;
        }
        ctrl_enu_speed(speed_e, speed_n, yaw*180/M_PI,speed_u);   
	} else {
	    //释放控制权
	    #if 0
	    msg = MSG_TYPE_NAV_DONE;
	    OSQPost((OS_Q*		)&QAutoNav,		
						(void*		)&msg,
						(OS_MSG_SIZE)1,
						(OS_OPT 	)OS_OPT_POST_FIFO,
						(OS_ERR*	)&err);
						#endif
		rlt = 1;
	}
    return rlt;
}
