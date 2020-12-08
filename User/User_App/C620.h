#ifndef _C620_H
#define _C620_H

#include "stm32f4xx.h"

//#include "bsp_can.h"

/*********************************************************************************/
#include "pid.h"


//电机旋转一周的机械转子角度
#define COUNTS_PER_ROUND (8191)
#define SECOUND_PER_MIN  (60)

/* 角度制弧度制互换 */
#define ANGLE2RAD(x) (x / 180.0f * PI)			//角度制转化为弧度制
#define RAD2ANGLE(x) (x /PI * 180.0f)			  //弧度制转换为角度制

/* 减速比 */
#define M3508_REDUCTION_RATIO	(3591.f/187.0f)			//M3508减速比

/* 电机控制参数 */
typedef struct
{
	/* 实时数据 */
	uint16_t Angle;
	int16_t  Velocity;
	int16_t  Current;
	uint8_t  Temprature;
	int32_t  Position;
	
	/* 模式选择 */
	uint8_t  Mode;
	
	/* PID输入参数 */
	int16_t Current_Ref;
	int16_t Velocity_Ref;
	int32_t Pulse_Ref;
}Motor_t;

/* 电机工作模式 */
typedef enum
{
	MotorMode_Disable  = (uint8_t)0x00,		//失能模式(卸力状态) 
	MotorMode_Current  = (uint8_t)0x01,		//电流模式
	MotorMode_Velocity = (uint8_t)0x02,		//速度模式
	MotorMode_Position = (uint8_t)0x03,		//位置模式
}MotorMode_TypeDef;

void Motor_Init(void);		//电机控制相关参数初始化函数

//用于修改PID控制相关参数
void Motor_SetControlMode(uint16_t Motor_ID, MotorMode_TypeDef MotorMode);	
void Motor_SetVelocityPIDRatio(uint16_t Motor_ID, float Kp, float Ki, float Kd);
void Motor_SetPositionPIDRatio(uint16_t Motor_ID, float Kp, float Ki, float Kd);

//用于修改Ref，PID控制值
void Motor_SetVelocity(uint16_t Motor_ID, int16_t Velocity);
void Motor_SetPosition(uint16_t Motor_ID, int32_t Position);
//用不上

//用于PID计算并记录结果
void VelCrl(uint16_t Motor_ID, int32_t vel);
void PosCrl(uint16_t Motor_ID, int32_t pos);

void DisableCrl(uint16_t Motor_ID);

extern Motor_t C620[4];

/*********************************************************************************/
/* C620接收地址 */
#define ID_C620_CTRL		0x200
#define ID_C620_Base		0x201
#define RIGHT_FRONT_ID	0x201		//ID_C610_01
#define LEFT_FRONT_ID	  0x202		//ID_C610_02
#define LEFT_REAR_ID	  0x203		//ID_C610_03
#define RIGHT_REAR_ID	  0x204		//ID_C610_04
//#define CAN_C620			 CAN1

void C620_GetFeedbackInfo(CanRxMsg* RxMsg);

//用CAN将计算结果发送
uint8_t SetJoggingVel(CAN_TypeDef* CANx, uint16_t Motor_CTRL, Motor_t* Motor_Array);
uint8_t SendPosCmd(CAN_TypeDef* CANx, uint16_t Motor_CTRL, Motor_t* Motor_Array);

#endif

