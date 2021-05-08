#include "pathfollow.h"

#include "algorithm.h"
#include "motion.h"
#include "chassis.h"

#include "pps.h"

#define MAX_PLAN_VEL    (0.0f)
#define TEST_PATH_NUM   (10)
#define VELL_TESTPATH		(0.5f)

float testPathLen[TEST_PATH_NUM];
Pose_t testPath[TEST_PATH_NUM];

void PathFollowing(float percent, float Kp)
{
	//�������жϱ�־λ
	static uint8_t indexNum = 0;
	static uint8_t searchFlag = 0;
	
	
	static float passedLength = 0.0f;
	static Pose_t actPoint = {0.0f};
	
	Pose_t startPoint, endPoint = {0.0f};
	
	
	vector_t adjustVel = {0.0f};
	vector_t finalVel = {0.0f};
	
	//��ʼ�����ã���֤ѭ����˳������
	if(indexNum == 0)
	{
		searchFlag = 0;
	}
	
	/**********************************
	 * ѭ�������Ƿ���Ҫ�޸ģ�
	 * ��������ѭ�� ���߲���ʹ��while
	 * �ⲿ��Ҫif����ж��Ƿ�ﵽ�յ㣬��ǰ����pathfollowing������Ӧ
   ***********************************
	 */
	
	while(indexNum <= gRobot.totalNum || passedLength < gRobot.totalLength)
	{

		actPoint.point.x = GetX(); 
		actPoint.point.y = GetY();
//			passedLength = GetLengthPassed(indexNum);   //����д����
		
		if(passedLength < gRobot.totalLength && indexNum <= gRobot.totalNum)
		{
			indexNum = FindSpan(gRobot.totalNum, passedLength, testPathLen);
			
			gRobot.virtualPos.startPtr = indexNum;
			gRobot.virtualPos.endPtr   = indexNum + 1;
			startPoint = testPath[gRobot.virtualPos.startPtr];
			endPoint   = testPath[gRobot.virtualPos.endPtr];
			
			searchFlag = 1;
		}
		else if(passedLength >= gRobot.totalLength - 1000.0f || passedLength <= gRobot.totalLength + 1000.0f)
		{
			//data 
			searchFlag = 0;
			break;
		}
		else if(passedLength >= gRobot.totalLength && searchFlag == 1)  //�ܹ�������������︽��������
		{
			passedLength = gRobot.totalLength;
			indexNum = gRobot.totalNum;
			
			gRobot.virtualPos.startPtr = indexNum;
			gRobot.virtualPos.endPtr   = indexNum;
			startPoint = testPath[gRobot.virtualPos.startPtr];
			endPoint   = testPath[gRobot.virtualPos.endPtr];
		}

		if(searchFlag)
		{
			gRobot.virtualPos = VectorLinerInterpolation(startPoint, endPoint, percent);
			adjustVel = CalcSpeedFromAct2Vir(actPoint, gRobot.virtualPos, Kp);
					
			finalVel = VectorSynthesis(gRobot.virtualPos.vel, gRobot.virtualPos.direction, adjustVel);
	
			if(finalVel.module > 0.01f)
			{
				gRobot.outputVel = finalVel.module;
				gRobot.outputDirection = finalVel.direction;
			}
			else
			{
				gRobot.outputVel = finalVel.module;
				gRobot.outputDirection = gRobot.outputDirection;
			}
	
			OutputVel2Wheel_FixedC(gRobot.outputVel, gRobot.outputDirection, GetWZ());
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
	
//��бʽ����ʸ����С������

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
		virutalPos.direction = RAD2ANGLE(atan2f(yErr, xErr));		//ע��actan����Ϊ0ʱ���Ƿ���Ҫ�������������
	}
	
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

vector_t VectorSynthesis(float targetVel, float targetDirection, vector_t adjustVel)
{
	Speed_t Decompose = {0.0f};
	vector_t outputVel = {0.0f};
	
	float targetDir, adjustDir = 0.0f;	//��ֵ���޸�
	
	Decompose.x = targetVel * arm_cos_f32(ANGLE2RAD(targetDirection)) + \
						   adjustVel.module * arm_cos_f32(ANGLE2RAD(adjustVel.direction));
	
	Decompose.y = targetVel * arm_sin_f32(ANGLE2RAD(targetDirection)) + \
						   adjustVel.module * arm_sin_f32(ANGLE2RAD(adjustVel.direction));
	
	arm_sqrt_f32(Decompose.x * Decompose.x + Decompose.y * Decompose.y, &outputVel.module);
	outputVel.direction = RAD2ANGLE(atan2f(Decompose.y, Decompose.x));
	
	return outputVel;
}









