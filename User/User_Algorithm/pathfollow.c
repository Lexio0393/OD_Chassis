#include "pathfollow.h"

#include "algorithm.h"
#include "motion.h"

#include "pps.h"

#define MAX_PLAN_VEL  (0.0f)
#define TEST_PATH_NUM (10)
#define VELL_TESTPATH				(0.5f)

float testPathLen[TEST_PATH_NUM];
Pose_t testPath[TEST_PATH_NUM];

void PathFollowing(float percent, float Kp)
{
	static uint8_t actNum = 0;
	
	static float passedLength = 0.0f;
	
	static Pose_t actPoint = {0.0f};
	
	Pose_t startPoint, endPoint = {0.0f};
	
	vector_t outputVel = {0.0f};
	
	robotVel_t finalVel = {0.0f};
	
	while(actNum < gRobot.totalNum)
	{
		actPoint.point.x = GetX(); 
		actPoint.point.y = GetY();
		
		passedLength = GetLengthPassed();
		
		actNum = FindSpan(gRobot.totalNum, passedLength, testPathLen);
		
		if(actNum >= gRobot.totalNum)
		{
			actNum = 0;
			break;
		}
		else
		{
			gRobot.virtualPos.u = percent;
			gRobot.virtualPos.startPtr = actNum;
			gRobot.virtualPos.endPtr   = actNum + 1;
			startPoint = testPath[gRobot.virtualPos.startPtr];
			endPoint   = testPath[gRobot.virtualPos.endPtr];
		}
		
		gRobot.virtualPos = VectorLinerInterpolation(startPoint, endPoint, percent);
		
		outputVel = CalcSpeedFromAct2Vir(actPoint, gRobot.virtualPos, Kp);
		
		if(outputVel.module > 0.01f)
		{
			gRobot.outputVel = outputVel.module;
			gRobot.outputDirection = outputVel.direction;
		}
		else
		{
			gRobot.outputVel = outputVel.module;
			gRobot.outputDirection = gRobot.outputDirection;
		}
	}
	
}


uint8_t FindSpan(uint8_t totalNum, float PassedLen, float *PathLength)
{
	uint8_t low, high, mid = 0;
	
	float totalLength = PathLength[totalNum];
	
	if(PassedLen >= totalLength)
		return (uint8_t) totalNum;
	
	low = 0;
	high = totalNum + 1;
	mid = (low + high) / 2;
	
	while(PassedLen < PathLength[mid] || PassedLen > PathLength[mid + 1])
	{
		if(PassedLen < PathLength[mid])
			high = mid;
		else
			low = mid;
		
		mid = (low + high) / 2;
	}
	
	return (uint8_t) mid; 
}
	
//点斜式代替矢量大小及方向

PointU_t VectorLinerInterpolation(Pose_t startPos, Pose_t endPos, float percent)
{
	float xErr, yErr = 0.0f;
	PointU_t virutalPos = {0.0f};
	
	xErr = endPos.point.x - startPos.point.x;
	yErr = endPos.point.y - startPos.point.y;
	
	virutalPos.point.x = startPos.point.x + percent * xErr;
	virutalPos.point.y = startPos.point.y + percent * yErr;
	virutalPos.u = percent;
	virutalPos.direction = RAD2ANGLE(atan2f(yErr, xErr));		//注意actan参数为0时，是否需要考虑特殊情况？

	return virutalPos;
}

vector_t CalcSpeedFromAct2Vir(Pose_t actPos, PointU_t virPos, float Kp)
{
	vector_t adjustVel = {0.0f};
	
	float xErr, yErr, posErr= 0.0f;
	
	xErr = virPos.point.x - actPos.point.x;
	yErr = virPos.point.y - actPos.point.y;
	arm_sqrt_f32(xErr * xErr + yErr * yErr, &posErr);
	
	adjustVel.module = Kp * posErr;
	adjustVel.direction = RAD2ANGLE(atan2f(yErr, xErr));
	
//	adjustVel.module = MAX_PLAN_VEL;
	
	return adjustVel;
}

vector_t VectorSynthesis(vector_t targetVel, vector_t adjustVel)
{
	Speed_t Decompose = {0.0f};
	vector_t outputVel = {0.0f};
	
	float targetDir, adjustDir = 0.0f;	//初值待修改
	
	Decompose.x = targetVel.module * arm_cos_f32(ANGLE2RAD(targetVel.direction)) + \
						   adjustVel.module * arm_cos_f32(ANGLE2RAD(adjustVel.direction));
	
	Decompose.y = targetVel.module * arm_sin_f32(ANGLE2RAD(targetVel.direction)) + \
						   adjustVel.module * arm_sin_f32(ANGLE2RAD(adjustVel.direction));
	
	arm_sqrt_f32(Decompose.x * Decompose.x + Decompose.y * Decompose.y, &outputVel.module);
	outputVel.direction = RAD2ANGLE(atan2f(Decompose.y, Decompose.x));
	
	return outputVel;
}









