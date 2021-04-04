#include "task.h"

#include "bsp_main.h"

#include "chassis.h"

#include "C620.h"
#include "pps.h"
#include "bsp_09s.h"

volatile RemoteStatus_t RemoteStatus = RemoteStatus_FixedCoordinate;
volatile ChassisStatus_t ChassisStatus = ChassisStatus_LostForce;

/*
 *M3508����ת��Ϊ8192rpm,���������Ϊ8192*Pi*D/60/M3508_REDUCTION_RATIO = 3404.07981f mm/s
 *���ת�پ�Ϊ��8192ʱ, ǰ����������ͶӰΪ3404.07981 * Sqrt(2) = 4814.098583mm/s
 *ҡ��ֵ��ΧΪ-683*SQRT_2~0~683*SQRT_2  �ٶ�ϵ��ΪMaxVelocity/(683*SQRT_2)
 *
 *��ת���ٶ�Ϊ3404.07981f mm/s	MaxVelocity/MOVEBASE_RADIUS (302.91f) = 11.2379248��/s
 *ҡ��ֵ��ΧΪ-683~0~683			11.2379248/683 = 0.01645377��/s
 *
 *RockerValue * Chassis_Ratio����ֵ�ĵ�λΪmm/s��㣬��ΧΪ(-4814 ~ 4814)(-180��/s ~ 180��/s)
 *��ϵ������ҡ��ֵת��Ϊ���ٻ���ת�Ƕ�
 */
 
const float Chassis_VelRatio = 4.9840115f;
const float	Chassis_OmeRatio = 0.0164538f;

float RockerValue_LX = 0.000f, RockerValue_LY = 0.000f;
float RockerValue_RX = 0.000f, RockerValue_RY = 0.000f;

float Vel_Vector = 0.000f, Vel_ControlValue = 0.000f;	//Vel_ControlValue = Vel_Vector*Chassis_VelRatio

vector_t finalVel = {0};

void Config_Task(void)
{
	BSP_Init();
	Beep_On();
	Delay_ms(50);
	Beep_Off();
	
	LocatorInit();
	Beep_On();
	Delay_ms(50);
	Beep_Off();
	
	Motor_Init();
	Beep_On();
	Delay_ms(50);
	Beep_Off();
}

void TIM2_5ms_Task(void)
{
	static uint32_t Counter = 0;
	Counter++;

	Task_OutputVel2Wheel();

	if(Counter >= 200)
	{
		Counter = 0;
		
		LED1_Toggle();
	}
}

void TIM5_10ms_Task(void)
{
	static uint32_t Counter = 0;
	Counter++;
	
	Task_GetChassisStatus();

	if(Counter >= 50)
	{
		Counter = 0;
		
		LED2_Toggle();
	}	
}

void Task_GetChassisStatus(void)
{
	/* ���̿���ģʽѡ�� */
	switch((uint8_t)Remote_GetChanalValue(Remote_B))
	{
		case Remote_SW0:
			ChassisStatus = ChassisStatus_Remote;
			break;
		case Remote_SW1:
			ChassisStatus = ChassisStatus_LostForce;
			break;
		case Remote_SW2:
			ChassisStatus = ChassisStatus_AutoMode;
			break;
		default:
			ChassisStatus = ChassisStatus_LostForce;	 //ʧ�ܱ���
	}
	
		/* ģʽ�����¼� */
	switch((uint8_t)ChassisStatus)
	{
		/* ң��ģʽ */
		case ChassisStatus_Remote:
			Task_RemoteControl();
			break;
		
		/* ʧ��ģʽ */
		case ChassisStatus_LostForce:
			finalVel.module = 0.0f;
			finalVel.direction = GetAngle();
			break;

		/* �Զ�ģʽ */
		case ChassisStatus_AutoMode:
//			Task_AutoRunning();	��ʱ��ʧ��ģʽ����
			finalVel.module = 0;
			finalVel.direction = GetAngle();
			break;	
		
		default:
			finalVel.module = 0;
			finalVel.direction = GetAngle();
			break;
	}
}
	

void Task_RemoteControl(void)
{	
	switch((uint8_t)Remote_GetChanalValue(Remote_F))
	{
	
		case Remote_SW0:
			
			RemoteStatus = RemoteStatus_FixedCoordinate;
		
			RockerValue_RX = -Remote_GetChanalValue(Remote_RX);		
			RockerValue_RY = Remote_GetChanalValue(Remote_RY);
			RockerValue_LX = Remote_GetChanalValue(Remote_LX);
			
			gChassis.chassisVel.omega = RockerValue_LX * Chassis_OmeRatio / 3;  //3Ϊ���٣���ȥ
		
			arm_sqrt_f32(RockerValue_RX * RockerValue_RX + RockerValue_RY * RockerValue_RY, &Vel_Vector);
			Vel_ControlValue = Vel_Vector * Chassis_VelRatio	/ 3;		//3Ϊ���٣���ȥ
		
			finalVel.module = Vel_ControlValue;
		
			if(finalVel.module>0.01f)
			{
				if(RockerValue_RY == 0 && RockerValue_RX == 0)
					finalVel.direction = 90.0f;
				else
					finalVel.direction = RAD2ANGLE(atan2f(RockerValue_RY, RockerValue_RX));
			}
			else
			{
				finalVel.direction = 90.0f;
			}	
			
			break;
			
		case Remote_SW1:
			
			RemoteStatus = RemoteStatus_Standby;
			
			finalVel.module = 0;
			finalVel.direction = 90.0f;
		
			break;
		
		case Remote_SW2:
			
			RemoteStatus = RemoteStatus_RigidCoordinate;
			RockerValue_RX = -Remote_GetChanalValue(Remote_RX);		
			RockerValue_RY = Remote_GetChanalValue(Remote_RY);
			RockerValue_LX = Remote_GetChanalValue(Remote_LX);
			
			gChassis.chassisVel.omega = RockerValue_LX * Chassis_OmeRatio / 3;	//3Ϊ���٣���ȥ
		
			arm_sqrt_f32(RockerValue_RX * RockerValue_RX + RockerValue_RY * RockerValue_RY, &Vel_Vector);
			Vel_ControlValue = Vel_Vector * Chassis_VelRatio	/ 3;		//3Ϊ���٣���ȥ
		
			finalVel.module = Vel_ControlValue;
		
			if(finalVel.module>0.01f)
			{
				if(RockerValue_RY == 0 && RockerValue_RX == 0)
					finalVel.direction = 90.0f;
				else
					finalVel.direction = RAD2ANGLE(atan2f(RockerValue_RY, RockerValue_RX));
			}
			else
			{
				finalVel.direction = 90.0f;
			}	
			break;
		
		default:
			
			ChassisStatus = ChassisStatus_LostForce;
			break;
	}
}

void Task_OutputVel2Wheel(void)
{
	switch((uint8_t)ChassisStatus)
	{
		case ChassisStatus_Remote:
			
		  gChassis.chassisVel.vel = finalVel.module;
			gChassis.chassisVel.direction = finalVel.direction;
			
			RemoteStatus_Send_Task();
			break;
		
		case ChassisStatus_LostForce:
			
		  gChassis.chassisVel.vel = finalVel.module;
			gChassis.chassisVel.direction = finalVel.direction;
		
			LostForceStatus_Send_Task();
			break;

		case ChassisStatus_AutoMode:
			
			AutoStatus_Send_Task();
			break;
	
		default:
			LostForceStatus_Send_Task();
			break;
	}
}

void RemoteStatus_Send_Task(void)
{
	switch((uint8_t)RemoteStatus)
	{
		case RemoteStatus_FixedCoordinate:
			OutputVel2Wheel_FixedC(gChassis.chassisVel.vel, gChassis.chassisVel.direction, gChassis.chassisVel.omega); \
			SetJoggingVel(CAN1, ID_C620_CTRL, C620);
			break;
		
		case RemoteStatus_Standby:
			OutputVel2Wheel_FixedC(0.0f, 90.0f, 0.0f);
			SetJoggingVel(CAN1, ID_C620_CTRL, C620);
			break;		
		
		case RemoteStatus_RigidCoordinate:
			OutputVel2Wheel_RigidC(gChassis.chassisVel.vel, gChassis.chassisVel.direction, gChassis.chassisVel.omega);
			SetJoggingVel(CAN1, ID_C620_CTRL, C620);
			break;
	}
}

void LostForceStatus_Send_Task(void)
{
	DisableCrl(RIGHT_FRONT_ID);
	DisableCrl(LEFT_FRONT_ID);
	DisableCrl(RIGHT_REAR_ID);
	DisableCrl(LEFT_REAR_ID);
	
	SetJoggingVel(CAN1, ID_C620_CTRL, C620);
}
void AutoStatus_Send_Task(void)
{
	DisableCrl(RIGHT_FRONT_ID);
	DisableCrl(LEFT_FRONT_ID);
	DisableCrl(RIGHT_REAR_ID);
	DisableCrl(LEFT_REAR_ID);
	
	SetJoggingVel(CAN1, ID_C620_CTRL, C620);	
}

