#ifndef AP_CORE_NAVIGATION_H
#define AP_CORE_NAVIGATION_H

#ifdef __cplusplus
extern "C" {
#endif

#include "ap_core.h"
#include "geodetic.h"
#include "std.h"

extern float h_ctl_course_setpoint; ///<目标航向 rad, CW/nort

extern bool_t nav_in_circle;///<是否处于盘旋状态

extern float circle_bank;   ///<盘旋倾斜角

extern struct EnuCoor ap_enu_waypoints[240];///<航点， ENU坐标
extern struct LlaCoor ap_lla_waypoints[240];///<航点， LLA坐标

/**
 * @brief 导航初始化
 */
void ap_nav_init(void);

/**
 * @brief 设置carrot距离
 * @param carrot
 */
void ap_nav_set_carrot(float carrot_dist);

/**
 * @brief 获取返航点距离
 */
float ap_get_dist_to_home(void);

/**
 * @brief 获取目标航点距离
 * @return
 */
float ap_get_dist_to_target_wp(void);

/**
 *@brief 导航至目标位置xy
 *@param current_pos 当前位置 ENU
 *@param target_pos 目标位置 ENU
 */
void ap_nav_to_xy(struct EnuCoor *current_pos, struct EnuCoor *target_pos);

/**
 *@brief 导航至目标航点
 *@param wp_index 目标航点索引
 */
void ap_nav_go_to_waypoint(uint8_t wp_index);

/**
 *@brief 绕点xy盘旋
 *@param current_pos 当前位置 ENU
 *@param center_pos 盘旋中心位置 ENU
 *@param radius 盘旋半径 m 若半径大于零则为顺时针旋转
 *@param speed 盘旋速度 m/s
 */
void ap_nav_circle_xy(struct EnuCoor *current_pos, struct EnuCoor *center_pos, float radius, float speed);

/**
 *@brief 绕航点盘旋
 */
void ap_nav_circle_waypoint(uint8_t wp, float radius);

/**
 *@brief 沿直线导航
 *@param current_pos 当前位置 ENU
 *@param start_pos 起始位置 ENU
 *@param target_pos 目标位置 ENU
 */
void ap_nav_route_line(struct EnuCoor *current_pos,struct EnuCoor *wp_start, struct EnuCoor *target_pos);

/**
 *@brief 判断是否已经到达位置点xy
 *@param current_pos 当前位置 ENU
 *@param start_pos 起始位置 ENU
 *@param target_pos 目标位置 ENU
 *@param approaching_time 接近距离，小于该距离视为到达 m
 *@return
 */
bool_t ap_nav_is_approaching_xy(struct EnuCoor* current_pos,
                                struct EnuCoor* start_pos,
                                struct EnuCoor* target_pos,
                                float approaching_time);

/**
 * @brief 判断是否已经到达航点
 * @param waypoint_idx 航点索引
 * @param approaching_dist 接近距离，小于该距离视为到达 m
 * @return
 */
bool_t ap_nav_is_approaching_waypoint(uint8_t waypoint_idx, uint8_t approaching_dist);

#ifdef __cplusplus
}
#endif // __cplusplus

#endif
