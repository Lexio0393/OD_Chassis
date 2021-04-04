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

typedef struct
{
	PointU_t virtualPos;
	PointU_t virtualTarget;
	
	float passedLength;
	
	float totalLength;
	float pathLength;					//�ﵽĳһԤ���ʱӦ�߹��ĳ���
	float lengthIncrement;		//·���ۼӣ�����һ����

	
	int16_t projectionIndex;
	int16_t projextionIndexNew;
	
	float adjustVel;
	float adjustVelDir;
	float targetVel;
	float targetVelDir;
	
	int16_t lookaheadIndex;
	
	ChassisVel_t outputVel;
	
	float originVel;
	float originVelDir;
	
	float lookaheadDis;
	
}PathInfo_t;

/* ���̻������� */
#define WHEEL_DIAMETER  (152.4f)		//����ֱ������λ��mm��
#define DISX_OPS2CENTER (0.0f)			//��λϵͳX�᷽�����ľ���
#define DISY_OPS2CENTER (110.0f)		//��λϵͳY�᷽�����ľ���
#define MOVEBASE_RADIUS (302.91f)	  //������ת�뾶

/* ����ƽ������ϵ���������� */
#define RIGHT_FRONT_NUM 			(1)				//��ǰ��ID��		ID_C620_01	0x201
#define LEFT_FRONT_NUM 				(2)				//��ǰ��ID��		ID_C620_02	0x202
#define LEFT_REAR_NUM				  (3)				//�����ID��		ID_C620_03	0x203
#define RIGHT_REAR_NUM 				(4)				//�Һ���ID��		ID_C620_04	0x204

/* ����λ�� 
 * ��λ������Y������Ϊ��ʼ������ʱ��Ϊ��
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

//���ֿ���ģʽ
void OutputVel2Wheel_RigidC(float vel, float direction, float omega);    //Rigid Coordinate(Chassis Coordinate) 
void OutputVel2Wheel_FixedC(float vel, float direction, float omega);    //Fixes Coordinate(World Coordinate)

//����������ϵ���ٶȷֽ⵽��������ϵ
Speed_t Transform2ChassisCoordinate(float vel, float direction, float postureAngle);

//���ݳ�������ϵ�е��ٶȼ��㵥������
float CalcWheelSpeed(float velX , float velY , float omega , float angleN);

//������������ٷ��͵��������
void SendCmd2Driver(float rfVel, float lfVel, float lrVel, float rrVel);

//�������ٶ�����ת��ת������
float RotateVel2Vel(int rpm);
int Vel2RotateVel(float vel);

//ȫ�ֱ���
typedef struct
{
	ChassisVel_t chassisVel;
	Speed_t Speed_C;
	wheel_t wheelState;
	PathInfo_t PathInfo;
}gChassis_t;

extern gChassis_t gChassis;
#endif



//void YawLock(float ExpectedYaw)			//ƫ��������                          
//{
//	Underpan.Speed_a = UnderpanConfig.OmegaRatio * (ExpectedYaw - LocatorInfo.Yaw);
//}
