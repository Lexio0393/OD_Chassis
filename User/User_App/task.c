#include "task.h"

#include "bsp_main.h"

#include "chassis.h"

#include "C620.h"
#include "pps.h"
//#include "bsp_09s.h"
#include "bsp_dt7.h"

volatile RemoteStatus_t RemoteStatus = RemoteStatus_FixedCoordinate;
volatile ChassisStatus_t ChassisStatus = ChassisStatus_LostForce;

//M3508极限转速为8192rpm,则最大轮速为8192*Pi*D/60/M3508_REDUCTION_RATIO = 3404.07981f mm/s
//电机转速均为±8192时, 前进方向轮速投影为3404.07981 * Sqrt(2) = 4814.098583mm/s
//摇杆值范围为-683*SQRT_2~0~683*SQRT_2  速度系数为MaxVelocity/(683*SQRT_2)

//旋转线速度为3404.07981f mm/s	MaxVelocity/MOVEBASE_RADIUS (302.91f) = 11.2379248°/s
//摇杆值范围为-683~0~683			11.2379248/683 = 0.01645377°/s

//RockerValue * Chassis_Ratio所得值的单位为mm/s或°，范围为(-4814 ~ 4814)(-180°/s ~ 180°/s)
//该系数负责将摇杆值转化为车速或旋转角度

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
	/* 底盘控制模式选择 */
	switch((uint8_t)Remote_GetChanalValue(Remote_S1))
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
			ChassisStatus = ChassisStatus_LostForce;	 //失能保护
	}
	
		/* 模式触发事件 */
	switch((uint8_t)ChassisStatus)
	{
		/* 遥控模式 */
		case ChassisStatus_Remote:
			Task_RemoteControl();
			break;
		
		/* 失力模式 */
		case ChassisStatus_LostForce:
			finalVel.module = 0.0f;
			finalVel.direction = GetAngle();
			break;

		/* 自动模式 */
		case ChassisStatus_AutoMode:
//			Task_AutoRunning();	暂时用失力模式代替
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
	switch((uint8_t)Remote_GetChanalValue(Remote_S2))
	{
	
		case Remote_SW0:
			
			RemoteStatus = RemoteStatus_FixedCoordinate;
		
			RockerValue_RX = -Remote_GetChanalValue(Remote_RX);		
			RockerValue_RY = Remote_GetChanalValue(Remote_RY);
			RockerValue_LX = Remote_GetChanalValue(Remote_LX);
			
			gChassis.chassisVel.omega = RockerValue_LX * Chassis_OmeRatio / 3;  //3为限速，可去
		
			arm_sqrt_f32(RockerValue_RX * RockerValue_RX + RockerValue_RY * RockerValue_RY, &Vel_Vector);
			Vel_ControlValue = Vel_Vector * Chassis_VelRatio	/ 3;		//3为限速，可去
		
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
			
			gChassis.chassisVel.omega = RockerValue_LX * Chassis_OmeRatio / 3;	//3为限速，可去
		
			arm_sqrt_f32(RockerValue_RX * RockerValue_RX + RockerValue_RY * RockerValue_RY, &Vel_Vector);
			Vel_ControlValue = Vel_Vector * Chassis_VelRatio	/ 3;		//3为限速，可去
		
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

