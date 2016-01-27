#ifndef ALGEBRA_H
#define ALGEBRA_H

#ifdef __cplusplus
extern "C" {
#endif

#include <math.h>
#include <float.h> // for FLT_MIN

/* 2的开根号 */
#ifndef M_SQRT2
#define M_SQRT2         1.41421356237309504880
#endif

/**
 *@brief 二维向量
 */
struct Vector2
{
    float x;
    float y;
};

/**
 *@brief 三维向量
 */
struct Vector3
{
    float x;
    float y;
    float z;
};

/**
 *@brief 四元数
 */
struct Quaternion
{
    float qi;
    float qx;
    float qy;
    float qz;
};

/**
 *@brief 3X3矩阵
 */
struct Matrix33
{
    float m[3 * 3];
};

/**
 *@brief 旋转矩阵
 */
struct FloatRMat
{
    float m[3 * 3];
};

/**
 *@brief 欧拉角
 */
struct FloatEulers
{
    float phi;      ///< 滚转角 rad
    float theta;    ///< 俯仰角 rad
    float psi;      ///< 偏航角 rad
};

/**
 *@brief 角速度
 */
struct FloatRates
{
    float p;        ///< 滚转角速度 rad/s
    float q;        ///< 俯仰角速度 rad/s
    float r;        ///< 偏航角速度 rad/s
};

//向量代数

//二维向量

/**
 *@brief 二维向量的模的平方
 */
float float_vect2_norm2(struct Vector2 *v);

/**
 *@brief 二维向量的模
 */
float float_vect2_norm(struct Vector2 *v);

/**
 *@brief 二维向量单位化
 */
void float_vect2_normalize(struct Vector2 *v);

/**
 *@brief 二维向量赋值 a = b
 */
void float_vect2_copy(struct Vector2 *a,struct Vector2 *b);

/*
 * 三维向量
 */
#define SQUARE(_a) ((_a)*(_a))

/*************************此处的宏定义对于具有xyz变量的结构体都适用，不需要一定是Vector3*************************/
/* a += b */
#define VECT3_ADD(_a, _b) {     \
    (_a).x += (_b).x;       \
    (_a).y += (_b).y;       \
    (_a).z += (_b).z;       \
  }

/* a -= b */
#define VECT3_SUB(_a, _b) {     \
    (_a).x -= (_b).x;       \
    (_a).y -= (_b).y;       \
    (_a).z -= (_b).z;       \
  }

/* c = a + b */
#define VECT3_SUM(_c, _a, _b) {                 \
    (_c).x = (_a).x + (_b).x;     \
    (_c).y = (_a).y + (_b).y;     \
    (_c).z = (_a).z + (_b).z;     \
  }

/* a += b*s */
#define VECT3_ADD_SCALED(_a, _b, _s) {      \
    (_a).x += ((_b).x * (_s));        \
    (_a).y += ((_b).y * (_s));        \
    (_a).z += ((_b).z * (_s));        \
  }

/* c = a - b */
#define VECT3_DIFF(_c, _a, _b) {                \
    (_c).x = (_a).x - (_b).x;     \
    (_c).y = (_a).y - (_b).y;     \
    (_c).z = (_a).z - (_b).z;     \
  }

/* _vo = _vi * _s */
#define VECT3_SMUL(_vo, _vi, _s) {      \
    (_vo).x =  (_vi).x * (_s);        \
    (_vo).y =  (_vi).y * (_s);        \
    (_vo).z =  (_vi).z * (_s);        \
  }

/* _vo =  _vi / _s */
#define VECT3_SDIV(_vo, _vi, _s) {      \
    (_vo).x =  (_vi).x / (_s);        \
    (_vo).y =  (_vi).y / (_s);        \
    (_vo).z =  (_vi).z / (_s);        \
  }

#define VECT3_CROSS_PRODUCT(_vo, _v1, _v2) {        \
    (_vo).x = (_v1).y*(_v2).z - (_v1).z*(_v2).y;    \
    (_vo).y = (_v1).z*(_v2).x - (_v1).x*(_v2).z;    \
    (_vo).z = (_v1).x*(_v2).y - (_v1).y*(_v2).x;    \
  }

/**
 *@brief 三维向量赋值
 */
void float_vect3_assign(struct Vector3 *v, float x, float y, float z);

/**
 *@brief 三维向量赋值 a = b
 */
void float_vect3_copy(struct Vector3 *a,struct Vector3 *b);

/**
 *@brief 三维向量的模的平方
 */
float float_vect3_norm2(struct Vector3 *v);

/**
 *@brief 三维向量的模
 */
float float_vect3_norm(struct Vector3 *v);

/**
 *@brief 三维向量单位化
 */
void float_vect3_normalize(struct Vector3 *v);

/**
 *@brief 角速度赋值 a = b
 */
void float_rates_copy(struct FloatRates *a,struct FloatRates *b);

/**
 *@brief 角速度累加 a += b
 */
void float_rates_add(struct FloatRates *a,struct FloatRates *b);

/**
 *@brief 角速度相减 a -= b
 */
void float_rates_sub(struct FloatRates *a,struct FloatRates *b);

/**
 *@brief 角速度求和 a = b + c
 */
void float_rates_sum(struct FloatRates *a,struct FloatRates *b,struct FloatRates *c);

/**
 *@brief 角速度_ro += _s*(向量_v)
 */
void float_rates_add_scaled_vect(struct FloatRates *_ro, struct Vector3 *_v, float _s);

/**
 *@brief 角速度重置为零
 */
void float_rates_zero(struct FloatRates *rates);

/**
 *@brief 求角速度的模
 */
float float_rates_norm(struct FloatRates _v);

// 旋转矩阵

/**
 *@brief 获取3X3矩阵第row行第col列的元素 0-2
 */
#define MAT33_ELMT(_m, _row, _col) ((_m).m[(_row)*3+(_col)])

/**
 *@brief 获取旋转矩阵第row行第col列的元素 0-2
 */
#define RMAT_ELMT(_rm, _row, _col) MAT33_ELMT(_rm, _row, _col)

/**
 *@brief 初始化一个矩阵为单位矩阵
 */
void float_rmat_identity(struct FloatRMat *rm);

/**
 *@brief 矩阵转置
 *m_b2a = transp(_m_a2b)
 */
void float_rmat_inv(struct FloatRMat *m_b2a, struct FloatRMat *m_a2b);

/**
 *@brief 矩阵相乘
 *m_a2c = m_b2c * m_a2b
 */
void float_rmat_comp(struct FloatRMat *m_a2c, struct FloatRMat *m_a2b,
                            struct FloatRMat *m_b2c);

/**
 *@brief 求旋转矩阵的模
 */
float float_rmat_norm(struct FloatRMat *rm);

/**
 *@brief 三维向量旋转一个矩阵
 *vb = m_a2b * va
 */
void float_rmat_vmult(struct Vector3 *vb, struct FloatRMat *m_a2b,
                             struct Vector3 *va);

/**
 *@brief 三维向量旋转一个转置矩阵
 *vb = m_b2a^T * va
 */
void float_rmat_transp_vmult(struct Vector3 *vb, struct FloatRMat *m_b2a,
                                    struct Vector3 *va);

/**
 *@brief 角速度旋转一个矩阵
 *rb = m_a2b * ra
 */
void float_rmat_ratemult(struct FloatRates *rb, struct FloatRMat *m_a2b,
                                struct FloatRates *ra);

/**
 *@brief 轴角对转换为旋转矩阵
 */
void float_rmat_of_axis_angle(struct FloatRMat *rm, struct Vector3 *uv, float angle);

/**
 *@brief 四元数转换为旋转矩阵
 */
void float_rmat_of_quat(struct FloatRMat *rm, struct Quaternion *q);


// 四元数
/**
 *@brief 四元数赋值
 */
void float_quat_assign(struct Quaternion *q,float qi,float qx,float qy,float qz);

/**
 *@brief 初始化一个四元数为单位四元数
 */
void float_quat_identity(struct Quaternion *q);

/**
 *@brief 四元数的模
 */
float float_quat_norm(struct Quaternion *q);

/**
 *@brief 四元数单位化
 */
void float_quat_normalize(struct Quaternion *q);

/**
 *@brief 四元数相乘
 *a2c = a2b comp b2c , aka  a2c = a2b * b2c
 */
void float_quat_comp(struct Quaternion *a2c, struct Quaternion *a2b, struct Quaternion *b2c);

/**
 *@brief 四元数相乘并单位化
 * a2c = a2b comp b2c , aka  a2c = a2b * b2c
 */
void float_quat_comp_norm_shortest(struct Quaternion *a2c, struct Quaternion *a2b, struct Quaternion *b2c);

/**
 *@brief 四元数积分
 */
void float_quat_integrate(struct Quaternion *q, struct FloatRates *omega, float dt);

/**
 *@brief 三维向量旋转一个四元数的角度
 *vb = q_a2b * va * q_a2b^-1
 */
void float_quat_vmult(struct Vector3 *v_out, struct Quaternion *q, struct Vector3 *v_in);

/**
 *@brief 欧拉角转换为四元数
 */
void float_quat_of_eulers(struct Quaternion *q, struct FloatEulers *e);

/**
 *@brief 单位向量和角度转换为四元数
 */
void float_quat_of_axis_angle(struct Quaternion *q, const struct Vector3 *uv, float angle);

/**
 *@brief 姿态向量转换为四元数
 *向量的长度（模）为旋转角度
 */
void float_quat_of_orientation_vect(struct Quaternion *q, const struct Vector3 *ov);

/**
 *@brief 旋转矩阵转换为四元数
 */
void float_quat_of_rmat(struct Quaternion *q, struct FloatRMat *rm);

/**
 *@brief 旋转矩阵转换为欧拉角
 */
void float_eulers_of_rmat(struct FloatEulers *e, struct FloatRMat *rm);

/**
 *@brief 四元数转换为欧拉角
 */
void float_eulers_of_quat(struct FloatEulers *e, struct Quaternion *q);

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* PPRZ_ALGEBRA_FLOAT_H */
