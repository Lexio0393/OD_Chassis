#ifndef __PATH_FOLLOW_H
#define __PATH_FOLLOW_H

#include "stm32f4xx.h"

#include "motion.h"

/*************
 * Pose_t Path[NUM]				: ͨ��B�������߼�����ľ���·���㣬{Point.x, Point.y, Posture, Speed}
 * float  PathLength[NUM] : ͨ��B�������߼������ÿһ����·�����϶�Ӧ��·�̳��ȣ�����Ѱ���뵱ǰλ��������㣬���������±�
 *
 * ������
 * CalcLengthPassed()  			 : ����������·�����Ѿ��߹���·��(����pps.h)
 * FindSpan()				   			 : ���ݵ�ǰ·������PathLength��������Χ�����������±�
 * VectorLinerInterpolation(): ����������ֵ�����Vtarget(������Speed �ٶȴ�С, direction �ٶȷ���)
 * CalcSpeedFromAct2Vir()			 : �õ�ǰλ�õ�Ͳ�ֵ�������������Vadjust
 * VectorSynthesis()         : ʸ���ϳɣ���Varget��Vadjust�ϳ�Ϊ
 ************/
 
//�������ٶȽṹ��
typedef struct
{
	//�ٶȴ�С
	float vel;
	//�ٶȷ���
	float direction;
	//���ٶȴ�С
	float omega;
}robotVel_t;


typedef struct gRobot_t
{
	//����λ�õ�����
	PointU_t virtualPos;
	
	//�Ѿ��߹���·��
	float passedLength;
	//�켣����·��
	float totalLength;
	//��ǰ�켣��·������
	uint8_t totalNum;
	
	//�켣�����ٶȵ�����
	robotVel_t adjustVel;
	//�ٶȻ�������ٶȴ�С
	float outputVel;
	//�ٶȻ�������ٶȷ���
	float outputDirection;

//  �ٶȼ�¼�����ٶȻ����������ʱ����һ�ε��ٶȴ�С
	//ԭʼ�ٶȴ�С
	float originVel;
	//ԭʼ�ٶȷ���
	float originVelDir;
	//���ٶȴ�С����/�룩
//	float omega;	
//	//���ٶȴ�С
//	float sumVel;
//	//���ٶȷ���
//	float sumVelDir;

	
}gRobot_t;
	


void PathFollowing(float percent, float Kp, Pose_t *Path, PathInfo_t *PathInfo);

uint8_t FindSpan(uint8_t totalNum, float PassedLen, PathInfo_t *PathInfo);

PointU_t VectorLinerInterpolation(Pose_t startPos, Pose_t endPos, float percent);

vector_t CalcSpeedFromAct2Vir(Pose_t actPos, PointU_t virPos, float Kp);

vector_t VectorSynthesis(float targetVel, float targetDirection, vector_t adjustVel);

gRobot_t gRobot;
#endif
