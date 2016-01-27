/**
 * @file subsystems/ahrs/ahrs_float_cmpl.h
 *
 * Complementary filter in float to estimate the attitude, heading and gyro bias.
 *
 * Propagation can be done in rotation matrix or quaternion representation.
 */

#ifndef AHRS_FLOAT_CMPL
#define AHRS_FLOAT_CMPL

#ifdef __cplusplus
extern "C" {
#endif

#include "std.h"
#include "ap_core.h"
#include "math/algebra.h"

/*AHRS状态标记*/
#define AHRS_ALIGN 0
#define AHRS_UNINIT 1
#define AHRS_RUNNING 2
struct Ahrs
{
    uint8_t status;//三种状态：AHRS、AHRS_UNINIT、AHRS_RUNNING
};

/*全局AHRS状态*/
extern struct Ahrs ahrs;

/*
 *AHRS初始化，自驾启动时调用
 *需要在AHRS算法中添加实现
 */
extern void ahrs_init(void);

/*
 *AHRS设置初始姿态
 *必须将AHRS状态设置为AHRS_RUNNING
 */
extern void ahrs_align(void);

/*
 *Propagation
 *陀螺仪角速率积分为角度
 *读取全局#imu中的数据
 */
extern void ahrs_propagate(float dt);

/*
 *使用加速度测量值更新AHRS状态
 *读取全局#imu中的数据
 */
extern void ahrs_update_accel(float dt);

/*
 *使用磁场计测量值更新AHRS状态
 *读取全局#imu中的数据
 */
extern void ahrs_update_mag(float dt);

/*
 *使用GPS测量值更新AHRS状态
 *读取全局#gps中的数据
 */
extern void ahrs_update_gps(void);

struct AhrsFloatCmpl {
  struct FloatRates gyro_bias;
  struct FloatRates rate_correction;
  struct FloatRates imu_rate;

  float accel_omega;  ///< filter cut-off frequency for correcting the attitude from accels (pseudo-gravity measurement)
  float accel_zeta;   ///< filter damping for correcting the gyro-bias from accels (pseudo-gravity measurement)
  float mag_omega;    ///< filter cut-off frequency for correcting the attitude (heading) from magnetometer
  float mag_zeta;     ///< filter damping for correcting the gyro bias from magnetometer

  /** sets how strongly the gravity heuristic reduces accel correction.
   * Set to zero in order to disable gravity heuristic.
   */
  uint8_t gravity_heuristic_factor;
  float weight;

  bool_t heading_aligned;
  struct Vector3 mag_h;

  /* internal counters for the gains */
  uint16_t accel_cnt; ///< number of propagations since last accel update
  uint16_t mag_cnt;   ///< number of propagations since last mag update
};

extern struct AhrsFloatCmpl ahrs_impl;


/** Update yaw based on a heading measurement.
 * e.g. from GPS course
 * @param heading Heading in body frame, radians (CW/north)
 */
void ahrs_update_heading(float heading);

/** Hard reset yaw to a heading.
 * Doesn't affect the bias.
 * Sets ahrs_impl.heading_aligned to TRUE.
 * @param heading Heading in body frame, radians (CW/north)
 */
void ahrs_realign_heading(float heading);

void ahrs_float_get_euler_from_accel_mag(struct FloatEulers* e, struct Vector3* accelf, struct Vector3* magf);

/** Compute a quaternion representing roll and pitch from an accelerometer measurement. */
static inline void ahrs_float_get_quat_from_accel(struct Quaternion* q, struct Vector3 *accel) {
  /* normalized accel measurement in floating point */
  float_vect3_normalize(accel);

  /* check for 180deg case */
  if ( ABS(accel->z - 1.0f) < 5*FLT_MIN ) {
    float_quat_assign(q, 0.0, 1.0, 0.0, 0.0);
  }
  else {
    /*
     * axis we want to rotate around is cross product of accel and reference [0,0,-g]
     * normalized: cross(acc_normalized, [0,0,-1])
     * vector part of quaternion is the axis
     * scalar part (angle): 1.0 + dot(acc_normalized, [0,0,-1])
     */
    q->qx = -accel->y;
    q->qy = accel->x;
    q->qz = 0.0;
    q->qi = 1.0f - accel->z;
    float_quat_normalize(q);
  }
}

void ahrs_float_get_quat_from_accel_mag(struct Quaternion* q, struct Vector3* accel, struct Vector3* mag);

#if !USE_MAGNETOMETER && !defined(AHRS_H_X) && !defined(AHRS_H_Y)
#define AHRS_H_X 1
#define AHRS_H_Y 0
#define AHRS_H_Z 0
#endif

#ifdef __cplusplus
}
#endif // __cplusplus

#endif /* AHRS_FLOAT_CMPL_RMAT */
