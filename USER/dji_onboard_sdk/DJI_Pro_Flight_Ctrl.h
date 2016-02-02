#ifndef DJI_PRO_FLIGHT_CTRL_H_
#define DJI_PRO_FLIGHT_CTRL_H_





extern float d_alt;        ///<遥控干预航线高度偏差
extern float d_alt_p_gain;///<高度偏差累加 增益P

extern int block;

extern float task_altitude;        ///<航线高度
extern float task_speed;           ///<航线速度

extern sdk_std_msg_t std_broadcast_data;
void ctrl_attitude_alt(float pitch, float roll, float yaw, float altitude);

int16_t GetRcGearInfo(void);


u8 auto_nav_check_gps(void);

void auto_nav_math_init(void);

u8 auto_nav_check_height(void);

u8 auto_nav_raise_to_tartget_height(void);

u8 auto_nav(void);


#endif
