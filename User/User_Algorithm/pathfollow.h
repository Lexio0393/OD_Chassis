#ifndef __PATH_FOLLOW_H
#define __PATH_FOLLOW_H

#include "stm32f4xx.h"

#include "motion.h"

/*************
 * Pose_t Path[NUM]				: 通过B样条曲线计算出的均匀路径点，{Point.x, Point.y, Posture, Speed}
 * float  PathLength[NUM] : 通过B样条曲线计算出的每一均匀路径点上对应的路程长度，用于寻找离当前位置最近两点，返回数组下标
 *
 * 函数：
 * CalcLengthPassed()  			 : 计算在理想路径上已经走过的路程(来自pps.h)
 * FindSpan()				   			 : 根据当前路径，在PathLength中搜索范围，返回数组下标
 * VectorLinerInterpolation(): 用最近两点插值计算出Vtarget(包含：Speed 速度大小, direction 速度方向)
 * CalcSpeedFromAct2Vir()			 : 用当前位置点和插值所得虚拟点计算出Vadjust
 * VectorSynthesis()         : 矢量合成，将Varget和Vadjust合成为
 ************/
 
//机器人速度结构体
typedef struct
{
	//速度大小
	float vel;
	//速度方向
	float direction;
	//角速度大小
	float omega;
}robotVel_t;


typedef struct gRobot_t
{
	//虚拟位置点坐标
	PointU_t virtualPos;
	
	//已经走过的路程
	float passedLength;
	//轨迹的总路程
	float totalLength;
	//当前轨迹总路径点数
	uint8_t totalNum;
	
	//轨迹跟随速度调节量
	robotVel_t adjustVel;
	//速度环输出的速度大小
	float outputVel;
	//速度环输出的速度方向
	float outputDirection;

//  速度记录量，速度环进行新输出时，上一次的速度大小
	//原始速度大小
	float originVel;
	//原始速度方向
	float originVelDir;
	//角速度大小（度/秒）
//	float omega;	
//	//和速度大小
//	float sumVel;
//	//和速度方向
//	float sumVelDir;

	
}gRobot_t;
	


void PathFollowing(float percent, float Kp);

uint8_t FindSpan(uint8_t totalNum, float PassedLength, float *PathLength);

PointU_t VectorLinerInterpolation(Pose_t startPos, Pose_t endPos, float percent);

vector_t CalcSpeedFromAct2Vir(Pose_t actPos, PointU_t virPos, float Kp);

vector_t VectorSynthesis(vector_t targetVel, vector_t adjustVel);


gRobot_t gRobot;
#endif
