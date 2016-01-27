#ifndef AP_CORE_H_INCLUDED
#define AP_CORE_H_INCLUDED

#ifdef __cplusplus
extern "C" {
#endif

#include "math/geodetic.h"
#include <inttypes.h>

#ifndef NULL
#ifdef _cplusplus
#define NULL 0
#else
#define NULL ((void *)0)
#endif
#endif

/*圆周率*/
#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

#ifndef M_PI_4
#define M_PI_4 (M_PI/4)
#endif

#ifndef M_PI_2
#define M_PI_2 (M_PI/2)
#endif

/* 布尔型 */
typedef uint8_t bool_t;

/**
 *@brief GPS定位状态
 */
enum GPSStatus
{
    GPS_FIX_NONE,   ///<未定位 0
    GPS_FIX_2D,     ///<2D定位 1
    GPS_FIX_3D      ///<3D定位 2
};

/**
 *@brief IMU状态
 */
struct IMU {
    struct FloatRates gyro;      ///<角速度 rad/s
    struct Vector3 accel;        ///<加速度 m/(s^2)
    struct Vector3 mag;          ///<磁场
    volatile bool_t gyro_valid;  ///<陀螺仪数据是否有效
    volatile bool_t accel_valid; ///<加速度数据是否有效
    volatile bool_t mag_valid;   ///<磁罗盘数据是否有效
};
extern struct IMU ap_imu;///<IMU状态

/**
 *@brief 陀螺仪参数
 */
struct GyroParameter
{
    float bias_p;              ///<陀螺仪漂移偏差p，rad/s， 4bytes
    float bias_q;              ///<陀螺仪漂移偏差q，rad/s， 4bytes
    float bias_r;              ///<陀螺仪漂移偏差r，rad/s， 4bytes
};

/**
 *@brief 姿态表示
 */
struct Orientation {
  struct Quaternion quat;     ///<四元数
  struct FloatEulers eulers;  ///<欧拉角
  struct FloatRMat rmat;      ///<旋转矩阵
};

/**
 *@brief 飞控当前状态
 */
struct APState
{
    uint32_t time_sec;              ///<系统开机后的时间 S
    uint32_t time_usec;             ///<系统开机后的时间 uS

    struct LlaCoor lla_pos;         ///<lla坐标，rad，rad，m

    struct EcefCoor ecef_pos;       ///<ECEF坐标，m

    struct LlaCoor lla_origin;      ///<ENU的LLA原点

    struct EnuCoor enu_pos;         ///<ENU坐标，相对于NED原点，m

    struct EnuCoor enu_speed;       ///<ENU坐标系下的速度，m/s

    struct Orientation orientation; ///<姿态

    struct FloatRates body_rates_f; ///<角速度 rad/s

    float baro_pressure;            ///<气压  pa
    float baro_altitude;            ///<气压高度 m

    float altitude;                 ///<融合后的高度 m 由哪几个参数融合由实际需要确定

    float gs_speed;                 ///<地速 m/s
    float air_speed;                ///<空速 m/s

    float voltage;                  ///<电池电压 V
    float current;                  ///<电流 A
    float temperature;              ///<温度 单位：摄氏度

    int16_t pwm_in[6];              ///<PWM输入 ms
    int16_t pwm_out[8];             ///<PWM输出 ms

    uint16_t number_waypoint;       ///<航点总数目
    uint16_t current_waypoint_index;///<当前要上传或下载的航点索引
    uint8_t target_waypoint;        ///<目标航点

    enum GPSStatus gps_status;      ///<GPS状态
    uint16_t week;                  ///<GPS week
    uint32_t tow;                   ///<GPS time of week in ms
    int32_t gps_course;             ///<GPS航向，rad, [0, 2*Pi] (CW/north),北向为零，顺时针
    uint8_t num_satellite;          ///<卫星数目

    bool_t is_home_set;             ///<返航点是否已设置
    bool_t is_in_the_air;           ///<飞行器是否在空中  条件根据实际应用设定
    bool_t is_simulating;           ///<是否正在进行仿真
    bool_t is_paramter_changed;     ///<参数是否已改变，若为TRUE，请保存飞控参数，并设为FALSE
};

extern struct APState core_state;             ///<ap_core状态（不需要保存到flash中）
extern struct GyroParameter gyro_parameter;   ///<陀螺仪参数

/**
 *@brief 获取ap_core版本号
 */
float ap_get_core_version(void);

/**
 *@brief 获取LLA坐标
 *@return
 */
struct LlaCoor *ap_get_position_lla(void);

/**
 *@brief 设置LLA坐标
 *@param lla_pos
 */
void ap_set_position_lla(struct LlaCoor *lla_pos);

/**
 *@brief 设置ENU坐标
 *@param enu_pos
 */
void ap_set_position_enu(struct EnuCoor *enu_pos);

/**
 * @brief 获取ENU坐标
 * @return
 */
struct EnuCoor *ap_get_position_enu(void);

/**
 * @brief 获取水平速度大小 m/s
 * @return
 */
float ap_get_horizontal_speed_norm(void);

/**
 *@brief 获取水平速度指向
 *北为零，顺时针为正，rad
 *@return
 */
float ap_get_horizontal_speed_dir(void);

/**
 * @brief 设置ENU坐标系的速度
 * @return
 */
void ap_set_speed_enu(struct EnuCoor *enu_speed);

/**
 * @brief 获取ENU坐标系的速度
 * @return
 */
struct EnuCoor *ap_get_speed_enu(void);

/**
 * @brief 设置角速度
 * @param rate
 */
void ap_set_rotation_rates(struct FloatRates *rate);

/**
 * @brief 获取角速度
 * @return
 */
struct FloatRates *ap_get_rotation_rates(void);

/**
 * @brief 设置姿态四元数
 * @param quat
 */
void ap_set_quaternion(struct Quaternion quat);

/**
 * @brief 获取姿态四元数
 * @return
 */
struct Quaternion ap_get_quaternion(void);

/**
 * @brief 获取姿态欧拉角
 * @return
 */
struct FloatEulers *ap_get_euler_angle(void);

/**
 *@brief 初始化全局变量
 */
extern void ap_init_core(void);

/**
 *@brief 角度转弧度
 */
extern float ap_util_radian(float d);

/**
 *@brief 设置飞行器位置
 *latitude 纬度 rad
 *longitude 经度 rad
 *altitude 高度 m
 */
extern void ap_set_position(float latitude, float longitude, float altitude);

/**
 *@brief 控制器更新
 */
extern void ap_update_controller(void);

/**
 *@brief 开始校准IMU
 */
extern void ap_begin_calibrate_imu(void);

/**
 *@brief IMU校准是否完成
 *若返回真，获取陀螺仪偏差，并保存飞控参数
 */
extern bool_t ap_is_imu_calibration_finished(void);

/**
 * @brief 获取回家点距离
 */
extern float ap_get_dist_to_home(void);

/**
 * @brief 获取目标航点距离
 * @return
 */
extern float ap_get_dist_to_target_wp(void);

/**
 * @brief 设置航点数据
 */
extern void ap_set_waypoint(uint8_t index, float lat, float lng, float alt);

/**
 * @brief 获取航点数据 ENU
 */
extern struct EnuCoor ap_get_waypoint(uint8_t index);

/**
 * @brief 获取航点数据 LLA
 */
extern struct LlaCoor ap_get_waypoint_lla(uint8_t index);

/**
 * @brief 是否正在运行模拟器
 */
extern bool_t ap_is_simulating(void);

/**
 *@brief 设置IMU的角速度 rad/s
 */
extern void ap_set_imu_gyro(float gyro_p, float gyro_q, float gyro_r);

/**
 *@brief 设置IMU的加速度 m/(s^2)
 */
extern void ap_set_imu_accel(float accel_x, float accel_y, float accel_z);

/**
 *@brief 设置IMU的加速度 m/(s^2)
 */
extern void ap_set_imu_mag(float mag_x, float mag_y, float mag_z);

/**
 *@brief 设置气压传感器数据 pa
 */
extern void ap_set_baro_pressure(float pressure);

#ifdef __cplusplus
}
#endif // __cplusplus

#endif // AP_CORE_H_INCLUDED
