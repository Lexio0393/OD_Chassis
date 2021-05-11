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

typedef struct
{
	float rf;
	float lf;
	float lr;
	float rr;
}wheel_t;

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
}debugInfo_t;

typedef enum
{
	waitForStart,
	goTo1stZone,
	reach1stPos,
	waitCommand,
	stop,
}autoStatus_t;

//�ֱ�����ָ��
typedef struct
{
	uint8_t nextFlag;
	uint8_t stopFlag;
}teleCmd_t;

//ȫ�ֱ���
typedef struct
{
	ChassisVel_t chassisVel;
	Speed_t Speed_C;
	wheel_t wheelState;
	
	teleCmd_t 	teleCommand;
	debugInfo_t debugInfomation;
	autoStatus_t runnigStatus;
}gChassis_t;

#define TELENOCMD				0
#define TELENEXT				1

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

extern gChassis_t gChassis;
#endif


//����
//void YawLock(float ExpectedYaw)			//ƫ��������                          
//{
//	Underpan.Speed_a = UnderpanConfig.OmegaRatio * (ExpectedYaw - LocatorInfo.Yaw);
//}
