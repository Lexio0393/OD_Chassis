#ifndef __PATH_FOLLOW_H
#define __PATH_FOLLOW_H

#include "stm32f4xx.h"

#include "pid.h"
#include "motion.h"
#include "chassis.h"

#define TEST_PATH_NUM   (6)				//测试路径点数
#define VELL_TESTPATH		(0.5f)		//测试插值位置
#define MAX_PLAN_VEL    (0.0f)		//底盘最大速度

PID_Increment_t PID_Chassis_Speed;
/*************
 * Pose_t Path[NUM]				: 通过B样条曲线计算出的均匀路径点，{Point.x, Point.y, Posture, Speed}
 * float  PathLength[NUM] : 通过B样条曲线计算出的每一均匀路径点上对应的路程长度，用于寻找离当前位置最近两点，返回数组下标
 *
 * function：
 * PathFollowing()					 : 根据当前位置点更新输出速度
 * FindSpan()				   			 : 根据已行驶的路径长度，在PathLength中搜索范围，返回数组下标
 * VectorLinerInterpolation(): 用最近两点插值计算出Vtarget(包含：Speed 速度大小, direction 速度方向)
 * CalcSpeedFromAct2Vir()		 : 用当前位置点和插值所得虚拟点计算出Vadjust
 * VectorSynthesis()         : 矢量合成，将Varget和Vadjust合成为输出速度
 * JudegeSpeedLessEqual()    : 判断速度是否小于某值
 * JudgeStop()               : 判断在某点固定范围内停留时间是否大于某一值
 ************/
 

void PathFollowing(float percent, float Kp, Pose_t *Path, PathInfo_t *PathInfo);
uint8_t FindSpan(uint8_t totalNum, float PassedLen, PathInfo_t *PathInfo);
PointU_t VectorLinerInterpolation(Pose_t startPos, Pose_t endPos, float percent);
vector_t CalcSpeedFromAct2Vir(Pose_t actPos, PointU_t virPos, float Kp);
vector_t VectorSynthesis(float targetVel, float targetDirection, vector_t adjustVel);

uint8_t JudgeSpeedLessEqual(float speedCompared);
uint8_t JudgeStop(float disChange,uint8_t countTime);

//void VelControl(robotVel_t robotVel);
//void Chassis_SetVelocityPIDRatio(PID_Increment_t Chassis_Struct, float Kp, float Ki, float Kd);
//void Chassis_SetVelocity(PID_Increment_t Chassis_Struct, int16_t Velocity);
//void VelCrl(PID_Increment_t Chassis_Struct, int32_t vel);


extern Pose_t testPath[TEST_PATH_NUM];
extern PathInfo_t testPathInfo[TEST_PATH_NUM];
#endif
