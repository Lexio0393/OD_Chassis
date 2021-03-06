#ifndef _CHASSIS_H
#define _CHASSIS_H

#include "stm32f4xx.h"
#include "arm_math.h"

#include "motion.h"
#include "algorithm.h"

//The Speed of chassis (use for controling)
typedef struct
{
	float vel;
	float direction;
	float omega;
}ChassisVel_t;

//typedef struct
//{
//	float velTarget;
//	float velAct;
//}wheelVel_t;

////The velocity of each wheel (save the inverse kinematics solutions)
//typedef struct
//{
//	wheelVel_t rightFront;
//	wheelVel_t leftFront;
//	wheelVel_t leftRear;
//	wheelVel_t rightRear;
//}wheelState_t;

typedef struct
{
	float rf;
	float lf;
	float lr;
	float rr;
}wheel_t;

/* 底盘基本参数 */
#define WHEEL_DIAMETER  (152.4f)		//轮子直径（单位：mm）
#define DISX_OPS2CENTER (0.0f)			//定位系统X轴方向到中心距离
#define DISY_OPS2CENTER (110.0f)		//定位系统Y轴方向到中心距离
#define MOVEBASE_RADIUS (302.91f)	  //底盘旋转半径

/* 按照平面坐标系象限数排序 */
#define RIGHT_FRONT_NUM 			(1)				//右前轮ID号		ID_C620_01	0x201
#define LEFT_FRONT_NUM 				(2)				//左前轮ID号		ID_C620_02	0x202
#define LEFT_REAR_NUM				  (3)				//左后轮ID号		ID_C620_03	0x203
#define RIGHT_REAR_NUM 				(4)				//右后轮ID号		ID_C620_04	0x204

/* 车轮位置 
 * 定位器所在Y轴逆向为起始方向，逆时针为正
 */
#define RIGHT_FRONT_VERTICAL_ANG (135.0f)
#define LEFT_FRONT_VERTICAL_ANG  (-135.0f)
#define LEFT_REAR_VERTICAL_ANG   (-45.0f)
#define RIGHT_REAR_VERTICAL_ANG  (45.0f)

/* 
 * | v1 |    | cos( 135) sin( 135) R | 							v1 = -Vx + Vy + Omega * R
 * | v2 |    | cos(-135) sin(-135) R |   | Vx |     v2 = -Vx - Vy + Omega * R
 * |    | =  |                       | = | Vy |     
 * | v3 |    | cos( -45) sin( -45) R |   | Om |     v3 =  Vx - Vy + Omega * R
 * | v4 |    | cos(  45) sin(  45) R |              v4 =  Vx + Vy + Omega * R
 */

//两种控制模式
void OutputVel2Wheel_RigidC(float vel, float direction, float omega);    //Rigid Coordinate(Chassis Coordinate) 
void OutputVel2Wheel_FixedC(float vel, float direction, float omega);    //Fixes Coordinate(World Coordinate)

//将世界坐标系下速度分解到车体坐标系
Speed_t Transform2ChassisCoordinate(float vel, float direction, float postureAngle);

//根据车体坐标系中的速度计算单个轮速
float CalcWheelSpeed(float velX , float velY , float omega , float angleN);

//将解算出的轮速发送到电机驱动
void SendCmd2Driver(float rfVel, float lfVel, float lrVel, float rrVel);

//车轮线速度与电机转速转换函数
float RotateVel2Vel(int rpm);
int Vel2RotateVel(float vel);

//全局变量
typedef struct
{
	ChassisVel_t chassisVel;
	Speed_t Speed_C;
	wheel_t wheelState;
}gChassis_t;

extern gChassis_t gChassis;
#endif


//？？
//void YawLock(float ExpectedYaw)			//偏航角锁定                          
//{
//	Underpan.Speed_a = UnderpanConfig.OmegaRatio * (ExpectedYaw - LocatorInfo.Yaw);
//}
