#ifndef __PATH_FOLLOW_H
#define __PATH_FOLLOW_H

#include "stm32f4xx.h"

#include "pid.h"
#include "motion.h"
#include "chassis.h"

#define TEST_PATH_NUM   (6)				//����·������
#define VELL_TESTPATH		(0.5f)		//���Բ�ֵλ��
#define MAX_PLAN_VEL    (0.0f)		//��������ٶ�

PID_Increment_t PID_Chassis_Speed;
/*************
 * Pose_t Path[NUM]				: ͨ��B�������߼�����ľ���·���㣬{Point.x, Point.y, Posture, Speed}
 * float  PathLength[NUM] : ͨ��B�������߼������ÿһ����·�����϶�Ӧ��·�̳��ȣ�����Ѱ���뵱ǰλ��������㣬���������±�
 *
 * function��
 * PathFollowing()					 : ���ݵ�ǰλ�õ��������ٶ�
 * FindSpan()				   			 : ��������ʻ��·�����ȣ���PathLength��������Χ�����������±�
 * VectorLinerInterpolation(): ����������ֵ�����Vtarget(������Speed �ٶȴ�С, direction �ٶȷ���)
 * CalcSpeedFromAct2Vir()		 : �õ�ǰλ�õ�Ͳ�ֵ�������������Vadjust
 * VectorSynthesis()         : ʸ���ϳɣ���Varget��Vadjust�ϳ�Ϊ����ٶ�
 * JudegeSpeedLessEqual()    : �ж��ٶ��Ƿ�С��ĳֵ
 * JudgeStop()               : �ж���ĳ��̶���Χ��ͣ��ʱ���Ƿ����ĳһֵ
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
