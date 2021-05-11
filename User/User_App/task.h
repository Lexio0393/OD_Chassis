#ifndef _TASK_H
#define _TASK_H

#include "stm32f4xx.h"

typedef enum
{
		RemoteStatus_FixedCoordinate = (uint8_t)0x00,			
    RemoteStatus_Standby  		   = (uint8_t)0x01,	
		RemoteStatus_RigidCoordinate = (uint8_t)0x02,
}RemoteStatus_t;

typedef enum
{
    ChassisStatus_Remote    = (uint8_t)0x00,
    ChassisStatus_LostForce	= (uint8_t)0x01,
    ChassisStatus_AutoMode  = (uint8_t)0x02,
}ChassisStatus_t;

void Config_Task(void);
void TIM2_5ms_Task(void);
void TIM5_10ms_Task(void);

void Task_GetChassisStatus(void);

void Task_RemoteControl(void);

void Task_OutputVel2Wheel(void);

void RemoteStatus_Send_Task(void);
void LostForceStatus_Send_Task(void);
void AutoStatus_Send_Task(void);

void AutoStatus_Execute_Task(void);


#endif
