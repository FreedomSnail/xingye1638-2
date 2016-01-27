#ifndef GEODETIC_H
#define GEODETIC_H

#ifdef __cplusplus
extern "C" {
#endif

#include "algebra.h"

/**
 * @brief 地心地固坐标ECEF
 * @details 地心为坐标原点
 * 单位: m
 */
struct EcefCoor {
  float x; ///< 和零度纬度、零度经度相交
  float y; ///< 右手定则确定
  float z; ///< 指向北
};

/**
 * @brief WGS84经纬度坐标
 */
struct LlaCoor {
  float lat; ///< 纬度 rad
  float lon; ///< 经度 rad
  float alt; ///< WGS84椭球以上的高度 m
};

/**
 * @brief ENU东北天坐标
 */
struct EnuCoor {
  float x; ///< E 东 m
  float y; ///< N 北 m
  float z; ///< U 天 m
};

/**
 *@brief 计算两个经纬度坐标之间的距离
 *@return float distance 距离 单位:m
 */
extern float ap_get_distance(struct LlaCoor lla1, struct LlaCoor lla2);

/**
 * @brief 计算两个坐标连线的方位角
 * lla1为原点，lla1-lla2连线和正北的夹角 -PI ~ PI
 * @param lla1
 * @param lla2
 * @return
 */
extern float ap_get_bearing(struct LlaCoor lla1,struct LlaCoor lla2);

/**
 *@brief LLA转ECEF
 */
extern struct EcefCoor lla2ecef(struct LlaCoor lla);

/**
 *@brief LLA转ENU
 */
extern struct EnuCoor lla2enu(struct LlaCoor* lla);

/**
 *@brief ENU转LLA
 *@param enu ENU坐标
 *@param lla_origin ENU坐标原点LLA
 *@return LLA坐标
 */
extern struct LlaCoor enu2lla(struct EnuCoor enu,struct LlaCoor lla_origin);

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif
