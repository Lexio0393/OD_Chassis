#ifndef _C620_H
#define _C620_H

#include "stm32f4xx.h"

//#include "bsp_can.h"

/*********************************************************************************/
#include "pid.h"


//�����תһ�ܵĻ�еת�ӽǶ�
#define COUNTS_PER_ROUND (8191)
#define SECOUND_PER_MIN  (60)

/* �Ƕ��ƻ����ƻ��� */
#define ANGLE2RAD(x) (x / 180.0f * PI)			//�Ƕ���ת��Ϊ������
#define RAD2ANGLE(x) (x /PI * 180.0f)			  //������ת��Ϊ�Ƕ���

/* ���ٱ� */
#define M3508_REDUCTION_RATIO	(3591.f/187.0f)			//M3508���ٱ�

/* ������Ʋ��� */
typedef struct
{
	/* ʵʱ���� */
	uint16_t Angle;
	int16_t  Velocity;
	int16_t  Current;
	uint8_t  Temprature;
	int32_t  Position;
	
	/* ģʽѡ�� */
	uint8_t  Mode;
	
	/* PID������� */
	int16_t Current_Ref;
	int16_t Velocity_Ref;
	int32_t Pulse_Ref;
}Motor_t;

/* �������ģʽ */
typedef enum
{
	MotorMode_Disable  = (uint8_t)0x00,		//ʧ��ģʽ(ж��״̬) 
	MotorMode_Current  = (uint8_t)0x01,		//����ģʽ
	MotorMode_Velocity = (uint8_t)0x02,		//�ٶ�ģʽ
	MotorMode_Position = (uint8_t)0x03,		//λ��ģʽ
}MotorMode_TypeDef;

void Motor_Init(void);		//���������ز�����ʼ������

//�����޸�PID������ز���
void Motor_SetControlMode(uint16_t Motor_ID, MotorMode_TypeDef MotorMode);	
void Motor_SetVelocityPIDRatio(uint16_t Motor_ID, float Kp, float Ki, float Kd);
void Motor_SetPositionPIDRatio(uint16_t Motor_ID, float Kp, float Ki, float Kd);

//�����޸�Ref��PID����ֵ
void Motor_SetVelocity(uint16_t Motor_ID, int16_t Velocity);
void Motor_SetPosition(uint16_t Motor_ID, int32_t Position);
//�ò���

//����PID���㲢��¼���
void VelCrl(uint16_t Motor_ID, int32_t vel);
void PosCrl(uint16_t Motor_ID, int32_t pos);

void DisableCrl(uint16_t Motor_ID);

extern Motor_t C620[4];

/*********************************************************************************/
/* C620���յ�ַ */
#define ID_C620_CTRL		0x200
#define ID_C620_Base		0x201
#define RIGHT_FRONT_ID	0x201		//ID_C610_01
#define LEFT_FRONT_ID	  0x202		//ID_C610_02
#define LEFT_REAR_ID	  0x203		//ID_C610_03
#define RIGHT_REAR_ID	  0x204		//ID_C610_04
//#define CAN_C620			 CAN1

void C620_GetFeedbackInfo(CanRxMsg* RxMsg);

//��CAN������������
uint8_t SetJoggingVel(CAN_TypeDef* CANx, uint16_t Motor_CTRL, Motor_t* Motor_Array);
uint8_t SendPosCmd(CAN_TypeDef* CANx, uint16_t Motor_CTRL, Motor_t* Motor_Array);

#endif

