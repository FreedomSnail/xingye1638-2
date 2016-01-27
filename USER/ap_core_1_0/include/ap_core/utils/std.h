#ifndef STD_H
#define STD_H

#include <inttypes.h>
#include <math.h>

/*真假*/
#ifndef FALSE
#define FALSE 0
#endif
#ifndef TRUE
#define TRUE 1
#endif

/*__cplusplus是cpp中的自定义宏，那么定义了这个宏的话表示这是一段cpp的代码*/
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

/*判断第b位是否已被置位*/
#ifndef bit_is_set
    #define bit_is_set(x,b) ((x>>b)&0x1)
#endif

/*置位*/
#define SetBit(alt,n) alt|=(1<<n)
/*复位*/
#define ClearBit(alt,n) alt&=~(1<<n)

/*角度和弧度转换*/
/*将弧度限制在-PI到PI之间*/
#define NormRadAngle(x) { \
    while(x > M_PI) x -= 2*M_PI; \
    while (x < -M_PI) x += 2*M_PI; \
    }
/*弧度转为角度*/
#define DegOfRad(x) ((x) * (180.0f/M_PI))
/*弧度转角度x10*/
#define DeciDegOfRad(x) ((x) * (1800.0f/M_PI))
/*角度转为弧度*/
#define RadOfDeg(x) ((x) * (M_PI/180.0f))

/*最小值和最大值*/
#define Min(x,y) (x < y ? x : y)
#define Max(x,y) (x > y ? x : y)

/*绝对值*/
#ifndef ABS
#define ABS(val) ((val)<0? -(val):(val))
#endif

/*边界限制*/
#define Bound(_x,_min,_max) {if(_x>_max) _x=_max; else if(_x<_min) _x=_min;}
#define BoundAbs(_x,_max) Bound(_x,-(_max),_max)
#define Chop(_x, _min, _max) ( (_x) < (_min) ? (_min) : (_x) > (_max) ? (_max) : (_x) )
#define ChopAbs(x, max) Chop(x, -(max), (max))

#endif // STD_H
