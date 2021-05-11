#include "pathfollow.h"

#include "algorithm.h"
#include "motion.h"
#include "chassis.h"

#include "pps.h"

void PathFollowing(float percent, float Kp, Pose_t *Path, PathInfo_t *PathInfo)
{
	uint8_t indexNum = 0;
	Pose_t startPoint, endPoint = {0.0f};
	
	vector_t adjustVel = {0.0f};
	vector_t finalVel = {0.0f};	
	
	static uint8_t searchFlag = 0;
	static float passedLength = 0.0f;
	static Pose_t actPoint = {0.0f};
	
	//初始化设置，保证循环能顺利进行
	if(indexNum == 0)
	{
		searchFlag = 0;
		passedLength = PathInfo[indexNum].trackLength;
	}
	
	while(indexNum <= gChassis.debugInfomation.totalNum || passedLength < gChassis.debugInfomation.totalLength)
	{

		actPoint.point.x = GetX(); 
		actPoint.point.y = GetY();
		passedLength = GetActLengthPaseed(indexNum, PathInfo);
		
		if(passedLength < gChassis.debugInfomation.totalLength && indexNum <= gChassis.debugInfomation.totalNum)
		{
			indexNum = FindSpan(gChassis.debugInfomation.totalNum, passedLength, PathInfo);
			
			gChassis.debugInfomation.virtualPos.startPtr = indexNum;
			gChassis.debugInfomation.virtualPos.endPtr   = indexNum + 1;
			startPoint = Path[gChassis.debugInfomation.virtualPos.startPtr];
			endPoint   = Path[gChassis.debugInfomation.virtualPos.endPtr];
			
			searchFlag = 1;
		}
		else if(passedLength >= gChassis.debugInfomation.totalLength - 10.0f && passedLength <= gChassis.debugInfomation.totalLength + 10.0f)
		{
			passedLength = gChassis.debugInfomation.totalLength;
			actPoint = Path[gChassis.debugInfomation.totalNum];
			
			gChassis.debugInfomation.outputVel = 0.0f;
			gChassis.debugInfomation.outputDirection = gChassis.debugInfomation.outputDirection;
			
			searchFlag = 0;
			break;
		}
		else if(passedLength >= gChassis.debugInfomation.totalLength && searchFlag == 1)  //跑过死区太多就调整，如果到达附近就跳出
		{
			passedLength = gChassis.debugInfomation.totalLength;
			indexNum = gChassis.debugInfomation.totalNum;
			
			gChassis.debugInfomation.virtualPos.startPtr = indexNum;
			gChassis.debugInfomation.virtualPos.endPtr   = indexNum;
			startPoint = Path[gChassis.debugInfomation.virtualPos.startPtr];
			endPoint   = Path[gChassis.debugInfomation.virtualPos.endPtr];
		}

		if(searchFlag)
		{
			gChassis.debugInfomation.virtualPos = VectorLinerInterpolation(startPoint, endPoint, percent);
			adjustVel = CalcSpeedFromAct2Vir(actPoint, gChassis.debugInfomation.virtualPos, Kp);
					
			finalVel = VectorSynthesis(gChassis.debugInfomation.virtualPos.vel, gChassis.debugInfomation.virtualPos.direction, adjustVel);
	
			if(finalVel.module > 0.01f)
			{
				gChassis.debugInfomation.outputVel = finalVel.module;
				gChassis.debugInfomation.outputDirection = finalVel.direction;
			}
			else
			{
				gChassis.debugInfomation.outputVel = finalVel.module;
				gChassis.debugInfomation.outputDirection = gChassis.debugInfomation.outputDirection;
			}
//			OutputVel2Wheel_FixedC(gChassis.debugInfomation.outputVel, gChassis.debugInfomation.outputDirection, GetWZ());
		}
	}	
}


uint8_t FindSpan(uint8_t totalNum, float PassedLen, PathInfo_t *PathInfo)
{
	uint8_t low, high, mid = 0;
	
	float totalLength = PathInfo[totalNum].trackLength;
	
	if(PassedLen >= totalLength)
		return (uint8_t) totalNum;
	
	low = 0;
	high = totalNum + 1;
	mid = (low + high) / 2;
	
	while(PassedLen < PathInfo[mid].trackLength || PassedLen > PathInfo[mid + 1].trackLength)
	{
		if(PassedLen < PathInfo[mid].trackLength)
			high = mid;
		else
			low = mid;
		
		mid = (low + high) / 2;
	}
	
	return (uint8_t) mid; 
}

PointU_t VectorLinerInterpolation(Pose_t startPos, Pose_t endPos, float percent)
{
	float xErr, yErr = 0.0f;
	PointU_t virutalPos = {0.0f};
	
	if(startPos.point.x == endPos.point.x && startPos.point.y == endPos.point.y)
	{
		xErr = endPos.point.x - startPos.point.x;
		yErr = endPos.point.y - startPos.point.y;
		
		virutalPos.point.x = startPos.point.x + percent * xErr;
		virutalPos.point.y = startPos.point.y + percent * yErr;
		virutalPos.vel = startPos.vel * (1 - percent) + endPos.vel * percent;
		virutalPos.direction = startPos.direction;
	}
	else
	{
		xErr = endPos.point.x - startPos.point.x;
		yErr = endPos.point.y - startPos.point.y;
		
		virutalPos.point.x = startPos.point.x + percent * xErr;
		virutalPos.point.y = startPos.point.y + percent * yErr;
		virutalPos.vel = startPos.vel * (1 - percent) + endPos.vel * percent;
		virutalPos.direction = RAD2ANGLE(atan2f(yErr, xErr));		//注意actan参数为0时，是否需要考虑特殊情况？
	}
	
	return virutalPos;
}

vector_t CalcSpeedFromAct2Vir(Pose_t actPos, PointU_t virPos, float Kp)
{
	vector_t adjustVel = {0.0f};
	
	float xErr, yErr, disErr= 0.0f;
	
	xErr = virPos.point.x - actPos.point.x;
	yErr = virPos.point.y - actPos.point.y;
	arm_sqrt_f32(xErr * xErr + yErr * yErr, &disErr);
	
	adjustVel.module = Kp * disErr;
	adjustVel.direction = RAD2ANGLE(atan2f(yErr, xErr));
	
	return adjustVel;
}

vector_t VectorSynthesis(float targetVel, float targetDirection, vector_t adjustVel)
{
	Speed_t Decompose = {0.0f};
	vector_t outputVel = {0.0f};
	
	Decompose.x = targetVel * arm_cos_f32(ANGLE2RAD(targetDirection)) + \
						   adjustVel.module * arm_cos_f32(ANGLE2RAD(adjustVel.direction));
	
	Decompose.y = targetVel * arm_sin_f32(ANGLE2RAD(targetDirection)) + \
						   adjustVel.module * arm_sin_f32(ANGLE2RAD(adjustVel.direction));
	
	arm_sqrt_f32(Decompose.x * Decompose.x + Decompose.y * Decompose.y, &outputVel.module);
	outputVel.direction = RAD2ANGLE(atan2f(Decompose.y, Decompose.x));
	
	return outputVel;
}

uint8_t JudgeSpeedLessEqual(float speedCompared)
{
	PosVel_t actSpeed = GetSpeedWithoutOmega();
	float speedX = actSpeed.x;
	float speedY = actSpeed.y;
	
	float speed = sqrtf(speedX * speedX + speedY * speedY);
	
	if(speed > speedCompared)
	{
		return 0;
	}
	else
	{
		return 1;
	}
}

uint8_t JudgeStop(float disChange,uint8_t countTime)
{
	static uint8_t counter = 0;
	float disX , disY = 0.0f;
	static float posXRecord , posYRecord = 0.0f;
	
	disX = GetX() - posXRecord;
	disY = GetY() - posYRecord;
	
	if(sqrtf(disX * disX + disY * disY)<=disChange)
	{
		counter++;
	}
	else
	{
		counter = 0;
	}
	
	posXRecord = GetX();
	posYRecord = GetY();
	
	if(counter<=countTime)
	{
		return 0;
	}
	else
	{
		counter = 0;
		return 1;
	}
}
//uint8_t JudgeStop2(uint8_t shagaiNum,float disChange,uint8_t countTime)
//{
//	static uint8_t counter = 0;
//	float disX , disY = 0.0f;
//	static float posXRecord , posYRecord = 0.0f;
//	
//	disX = GetX() - posXRecord;
//	disY = GetY() - posYRecord;
//	
//	switch(shagaiNum)
//	{
//		case FIRSTSHAGAI:
//		{
//			if(fabs(disY) < disChange)
//			{
//				counter++;
//			}
//			else
//			{
//				counter = 0;
//			}
//			break;
//		}
//		case SECONDSHAGAI:
//		{
//			if(fabs(disX) < disChange)
//			{
//				counter++;
//			}
//			else
//			{
//				counter = 0;
//			}
//			break;
//		}
//		case THIRDSHAGAI:
//		{
//			if(fabs(disX) < disChange)
//			{
//				counter++;
//			}
//			else
//			{
//				counter = 0;
//			}
//			break;
//		}
//		default:
//			break;
//	}
//	
//	posXRecord = GetX();
//	posYRecord = GetY();
//	
//	if(counter<=countTime)
//	{
//		return 0;
//	}
//	else
//	{
//		counter = 0;
//		return 1;
//	}
//}

//void VelControl(robotVel_t robotVel)
//{
//	
//}

//void Chassis_SetVelocityPIDRatio(PID_Increment_t Chassis_Struct, float Kp, float Ki, float Kd)
//{
//	Chassis_Struct.Kp = Kp;
//	Chassis_Struct.Ki = Ki;
//	Chassis_Struct.Kd = Kd;
//	Chassis_Struct.MaxOutput = 10000;
//	
//	Chassis_Struct.Deadband = 50.0f;
//	Chassis_Struct.MaxIncrease = 600.0f;
//	Chassis_Struct.MaxInt = 3000.0f;
//}



//void Chassis_SetVelocity(PID_Increment_t Chassis_Struct, int16_t Velocity)
//{
//	
//}

//void VelCrl(PID_Increment_t Chassis_Struct, int32_t vel);


