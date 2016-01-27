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
float task_speed = 0;           ///<航线速度

sdk_std_msg_t std_broadcast_data;


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
    //usleep(20000);
}

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
    //若高度不足2m，先上升到航线任务高度
    if (pos.height < 2) {
    	return 1;
    } else {
		return 0;
    }
}

u8 auto_nav_height_init(void)
{
	api_pos_data_t pos;
    pos = GetPosInfo();
    ctrl_attitude_alt(0, 0, 0, task_altitude);
	if(fabsf(task_altitude - pos.height) < 0.2){
	//printf("pos.height break\n";
 		return 1;
	} else {
		return 0;
	}  
}

void auto_nav(void)
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
	float speed_temp;
	OS_ERR err;
	
	//设置ENU速度

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

	if(ap_get_dist_to_target_wp() < 8) {
	    if(speed > 5) {
			speed = 5;
	    }
	}
	if(ap_get_dist_to_target_wp() < 6) {
	    if(speed > 3) {
	        speed = 3;
	    }
	}
	if(ap_get_dist_to_target_wp() < 4) {
	    if(speed > 2) {
	        speed = 2;
	   	}
	}
	if(ap_lla_waypoints[target_wp].lat != 0) {
	    yaw = h_ctl_course_setpoint;
	    if(h_ctl_course_setpoint > M_PI) {
	        yaw = h_ctl_course_setpoint - 2*M_PI;
	    }

	    if((ap_get_dist_to_target_wp() < 3)&& (ap_lla_waypoints[target_wp+1].lat != 0)) {
	        yaw = ap_get_bearing(*ap_get_position_lla(),ap_lla_waypoints[target_wp+1]);
	    }
	    speed_temp = sqrt(enu_speed.x*enu_speed.x+enu_speed.y*enu_speed.y) + 1;
		if( speed_temp < speed) {
			speed = speed_temp ;
		}
	    speed_n = cosf(h_ctl_course_setpoint)*speed;
	    speed_e = sinf(h_ctl_course_setpoint)*speed;
	    //发送控制指令
	    ctrl_attitude_alt(speed_e, speed_n, yaw*180/M_PI, task_altitude);
	} else {
	    //释放控制权
	    //DJI_Pro_Control_Management(0,NULL);
	    /////////////break;
	    OSSemPost(&SemAutoNav,OS_OPT_POST_1,&err);
	}
    
}
#if 0
void auto_nav(void)
{
	int i;
	api_pos_data_t pos;
	struct EnuCoor enu_speed;
	api_common_data_t vel;
	int target_wp;
	bool_t result;
	float speed;
	float yaw;
	float speed_n;
	float speed_e;
	
    block = 0;

    if (GetPosInfo().health_flag <= 2) {
        //printf("gps health_flag < 2\n");
        return;
    }

    DJI_Pro_Control_Management(1,NULL);//获取飞行控制权 

    //初始化ap_core
    ap_init_core();
    //设置飞行器初始位置
    
    pos = GetPosInfo();
    ap_set_position(pos.lati,pos.longti,pos.height);
    //初始化导航
    ap_nav_init();
    //ap_nav_set_carrot(5);

    //若高度不足2m，先上升到航线任务高度
    if (pos.height < 2)
    {
        for(i=0;i<1000;i++)
        {
            //printf("pos.height %f\n", pos.height);
            pos = GetPosInfo();

            ctrl_attitude_alt(0, 0, 0, task_altitude);

            if(fabsf(task_altitude - pos.height) < 0.2)
            {
                //printf("pos.height break\n");
                break;
            }
        }
    }







    for(i=0;i<1000000;i++)
    {
        //查询机载设备是否有控制权
        //api_ctrl_info_data_t ctrl_dev;
        //DJI_Pro_Get_CtrlInfo(&ctrl_dev);
        //printf("ctrl_dev:%d\n",ctrl_dev.cur_ctrl_dev_in_navi_mode);
        //if(ctrl_dev.cur_ctrl_dev_in_navi_mode != 2)
        //{
            //break;
        //}

        //设置ENU速度
        
        vel = GetVelInfo();
        enu_speed.x = vel.y;
        enu_speed.y = vel.x;
        enu_speed.z = vel.z;
        ap_set_speed_enu(&enu_speed);

        //设置飞行器位置
        pos = GetPosInfo();
        ap_set_position(pos.lati,pos.longti,pos.height);

        if(ap_get_waypoint_lla(core_state.target_waypoint).lat == 0)
        {
            printf("lat is zero \n");
            break;
        }

        target_wp = core_state.target_waypoint;

        switch (block) {
        case 0:
        {
            ap_nav_go_to_waypoint(core_state.target_waypoint);
            result = ap_nav_is_approaching_waypoint(core_state.target_waypoint,0.3);
            if(result)
            {
                core_state.target_waypoint++;

                printf("waypoint %d arrived \n",core_state.target_waypoint);
                block++;
                printf("pos e:%f n:%f\n",ap_get_position_enu()->x,
                       ap_get_position_enu()->y);
            }
        }
            break;
        case 1:
        {
            ap_nav_route_line(ap_get_position_enu(),
                              &ap_enu_waypoints[target_wp-1],
                              &ap_enu_waypoints[target_wp]);
            if(ap_nav_is_approaching_waypoint(core_state.target_waypoint,1))
            {
                core_state.target_waypoint++;
            }
        }
            break;
        default:
            break;
        }

        speed = task_speed;

        if(ap_get_dist_to_target_wp() < 8)
        {
            if(speed > 5)
                speed = 5;
        }

        if(ap_get_dist_to_target_wp() < 6)
        {
            if(speed > 3)
                speed = 3;
        }

        if(ap_get_dist_to_target_wp() < 4)
        {
            if(speed > 2)
                speed = 2;
        }
        if(ap_lla_waypoints[target_wp].lat != 0)
        {
            yaw = h_ctl_course_setpoint;
            if(h_ctl_course_setpoint > M_PI)
            {
                yaw = h_ctl_course_setpoint - 2*M_PI;
            }

            if((ap_get_dist_to_target_wp() < 3)
                    && (ap_lla_waypoints[target_wp+1].lat != 0))
            {
                yaw = ap_get_bearing(*ap_get_position_lla(),ap_lla_waypoints[target_wp+1]);
            }

            speed_n = cosf(h_ctl_course_setpoint)*speed;
            speed_e = sinf(h_ctl_course_setpoint)*speed;
            //发送控制指令
            ctrl_attitude_alt(speed_e, speed_n, yaw*180/M_PI, task_altitude);
        }
        else
        {
            //释放控制权
            DJI_Pro_Control_Management(0,NULL);
            break;
        }
    }
}
#endif
