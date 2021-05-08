#ifndef __MOTION_H
#define __MOTION_H

/* 速度矢量 */
typedef struct
{
	float module;
	float direction;
}vector_t;

typedef struct
{
	float x;
	float y;
}Speed_t;

/* 位置点的结构体 单位mm */
typedef struct
{
	float x;
	float y;
}Point_t;

typedef struct
{
	float tangentDir;
	float trackLength;
}PathInfo_t;

/* 姿态结构体，以点斜式描述机器人所处位置与在该位置上姿态 ，斜率用角度制的角度代替 */
typedef struct
{
	Point_t point;
	/* 角度制 */
	float   direction;
	/* 速度 */
	float vel;
}Pose_t;

typedef struct
{
	Point_t point;
	float vel;
	float direction;
	unsigned short startPtr;
	unsigned short endPtr;
}PointU_t;

/* Exported constants --------------------------------------------------------*/

/* 角度制弧度制互换系数 */
#define CHANGE_TO_RADIAN    0.01745329251994f   	//角度制转换为弧度制系数
#define CHANGE_TO_ANGLE     57.29577951308232f		//弧度制转换为角度制系数

#define SQRT_2	1.414213562373f	

#ifndef PI
#define PI                  3.1415926f				//圆周率
#endif

#endif
