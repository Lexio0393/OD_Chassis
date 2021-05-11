#include "task.h"

#include "bsp_main.h"

#include "chassis.h"

#include "C620.h"
#include "pps.h"
#include "bsp_09s.h"

#include "pathfollow.h"

volatile RemoteStatus_t RemoteStatus = RemoteStatus_FixedCoordinate;
volatile ChassisStatus_t ChassisStatus = ChassisStatus_LostForce;

/**********************************************************************************************
 *		M3508极限转速为8192rpm,则最大轮速为8192*Pi*D/60/M3508_REDUCTION_RATIO = 3404.07981f mm/s
 *		电机转速均为±8192时, 前进方向轮速投影为3404.07981 * Sqrt(2) = 4814.098583mm/s
 *		摇杆值范围为-683*SQRT_2~0~683*SQRT_2  速度系数为MaxVelocity/(683*SQRT_2)
 *		
 *		旋转线速度为3404.07981f mm/s	MaxVelocity/MOVEBASE_RADIUS (302.91f) = 11.2379248°/s
 *		摇杆值范围为-683~0~683			11.2379248/683 = 0.01645377°/s
 *		
 *		RockerValue * Chassis_Ratio所得值的单位为mm/s或°，范围为(-4814 ~ 4814)(-180°/s ~ 180°/s)
 *		该系数负责将摇杆值转化为车速或旋转角度
 ***********************************************************************************************
 */
 
//遥控模式相关参数
const float Chassis_VelRatio = 4.9840115f;
const float	Chassis_OmeRatio = 0.0164538f;
float RockerValue_LX = 0.000f, RockerValue_LY = 0.000f;
float RockerValue_RX = 0.000f, RockerValue_RY = 0.000f;
float Vel_Vector = 0.000f, Vel_ControlValue = 0.000f;	//Vel_ControlValue = Vel_Vector*Chassis_VelRatio
vector_t finalVel = {0};

//用于定时器中周期返回底盘当前速度
#define FREQUNCE_TIM_TASK (1)

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

//每5ms发送一次最新速度计算值
void TIM2_5ms_Task(void)
{
	static uint32_t Counter = 0;
	Counter++;

	Task_OutputVel2Wheel();

	if(Counter >= 50)
	{
		Counter = 0;
		LED1_Toggle();
	}
}

//每10ms执行一次整车速度环PID、车身速度解算
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

//每20ms计算一次路径跟随速度计算
void TIM5_20ms_Task(void)
{
	static uint32_t Counter = 0;
	Counter++;
	
	AutoStatus_Execute_Task();
	
	if(Counter >= 50)
	{
		Counter = 0;
		LED3_Toggle();
	}	
}
	

//1ms
//加滤波
void Task_SpeedCalculate(void)
{
	static PosVel_t SpeedFromPos = {0.0f};
	
	static float Prev_x, Prev_y= 0.0f;
	static float Delta_x, Delta_y = 0.0f;
	
	Delta_x = LocatorInfo.xVal - Prev_x;
	Delta_y = LocatorInfo.yVal - Prev_y;
	
	if(sqrt(pow(Delta_x, 2) + pow(Delta_x, 2))>=5.0f)
	{		
		if(ValueInRange_f(Delta_x, -150.0, 150.0))
			Prev_x = LocatorInfo.xVal;
	
		if(ValueInRange_f(Delta_y, -150.0, 150.0))
			Prev_y = LocatorInfo.yVal;
	
		SpeedFromPos.x = Delta_x * FREQUNCE_TIM_TASK * 1000;
		SpeedFromPos.y = Delta_y * FREQUNCE_TIM_TASK * 1000;
	}		
	
	ppsReturn.ppsSpeedX = SpeedFromPos.x;
	ppsReturn.ppsSpeedY = SpeedFromPos.y;
}

//5ms
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
	//未添加PID速度环前，用outputVel直接控制
	OutputVel2Wheel_FixedC(gChassis.debugInfomation.outputVel, gChassis.debugInfomation.outputDirection, GetWZ());
	SetJoggingVel(CAN1, ID_C620_CTRL, C620);	
}


//10ms
void Task_GetChassisStatus(void)
{
	/* 底盘控制模式选择 */
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
//			Task_AutoVelCrl();
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

//void Task_AutoVelCrl(void)
//{
//	float vel = 0.0f;
//	float velDireciton = 0.0f;
//	PosVel_t actVel = {0.0f};
//	
//	actVel = GetSpeedWithoutOmega();
//	vel = sqrtf(actVel.x * actVel.x + actVel.y * actVel.y);
//	
//	if(vel>0.01f)
//	{
//		velDireciton = atan2f(actVel.y,actVel.x)*CHANGE_TO_ANGLE;
//	}
//	else
//	{
//		velDireciton = velDireciton;
//	}
//	
//		VelControl((robotVel_t){vel , velDireciton , GetWZ()});	
//}






//20ms
void AutoStatus_Execute_Task(void)
{
//	static uint8_t remoteFlag = 0;
	
	gChassis.runnigStatus = waitForStart;
	
	Delay_ms(50);
	
	for(int i=0;i<18;i++)
	{
		if(i%2 == 1)
		{
			LED0_On();
		}
		
		Delay_ms(50);
	}
	
	LED0_On();
	
	gChassis.teleCommand.nextFlag = TELENOCMD;
	gChassis.teleCommand.stopFlag = TELENOCMD;
	
	Beep_On();
	Delay_ms(500);
	Beep_Off();
	
	while(1)
	{
		if(Remote_GetChanalValue(Remote_B))
		{
			gChassis.teleCommand.nextFlag = TELENEXT;
		}
		
		Auto_Running();
		
		//printf();
	}
}

void Auto_Running(void)
{
	static uint8_t judgeStopFlag = 0, judgeStopDoneFlag = 0;
	static uint8_t judgeReachFlag = 0, judgeReachDoneFlag = 0;
	static uint8_t waitStableFlag = 0;
	
	if(gChassis.teleCommand.stopFlag == 1)
	{
		
	}
	
	switch(gChassis.runnigStatus)
	{
		//等待遥控器命令出发
		case waitForStart:
		{
			if(gChassis.teleCommand.nextFlag == TELENEXT)
			{
				//插入路径点处理
				//收到遥控器信息后开始计时，if中为遥控器信号接收函数
				gChassis.runnigStatus = goTo1stZone;
			}
			break;
		}
		//路径跟随第一条路线
		case goTo1stZone:
		{
			
			/***************
			 * 路程中动作区
			 * 补充if代码即可
			 ***************/
			
			float dis2FinalX = GetX() - testPath[TEST_PATH_NUM - 1].point.x;
			float dis2FinalY = GetY() - testPath[TEST_PATH_NUM - 1].point.y;
			
			if((sqrtf(dis2FinalX*dis2FinalX + dis2FinalY*dis2FinalY)<50.0f&&JudgeSpeedLessEqual(700.0f)))
			{

				OutputVel2Wheel_FixedC(100.0f,180.0f,0.0f);    //速度、方向待修改
				
				judgeStopFlag = 0;
				judgeStopDoneFlag = 0;
				
				gChassis.teleCommand.nextFlag = TELENOCMD;
				gChassis.runnigStatus = reach1stPos;
				return;
			}
			
			PathFollowing(VELL_TESTPATH, 2, testPath, testPathInfo);
			
			break;
		}
		//判断到达第一条路径终点
		case reach1stPos:
		{
			static uint8_t posAchieve = 0, posTimeOut = 0;
			
			if(judgeStopDoneFlag == 0)	//说明底盘未停止
			{
				OutputVel2Wheel_FixedC(100.0f,180.0f,0.0f);
			}
			
			//通过X坐标判断是否停止，距离浮动3mm，持续实际5ms以上
			if(JudgeStop(3.0f,5) && judgeStopFlag==0)
			{
				judgeStopFlag = 1;
				//STOPPK
			}
			if(judgeStopFlag && judgeStopDoneFlag == 0)
			{
				if(fabs(GetY()) > 1500.0f)					//坐标待修改
				{
					posAchieve	=	1;
					//POSOK
				}
				else
				{
					posTimeOut++;
					posTimeOut = posTimeOut > 100 ? 100 : posTimeOut;
				}
			}
			
			if((posAchieve || posTimeOut > 20) && judgeStopDoneFlag == 0)
			{		
				judgeStopDoneFlag = 1;
			}
			
			if(judgeStopDoneFlag == 1 && gChassis.teleCommand.nextFlag == TELENEXT) //收到遥控器下一指令
			{
				posAchieve = 0;
				posTimeOut = 0;
				judgeStopFlag = 0;
				judgeStopDoneFlag= 0;

				gChassis.teleCommand.nextFlag = TELENOCMD;
				gChassis.runnigStatus = waitCommand;
			}
			break;
		}
		case waitCommand:
		{	
			gChassis.debugInfomation.outputVel = 0.0f;
			gChassis.debugInfomation.outputDirection = gChassis.debugInfomation.outputDirection;
			
			Beep_On();
			Delay_ms(1000);
			Beep_Off();
			
			break;
		}
		case stop:
		{
			gChassis.debugInfomation.outputVel = 0.0f;
			gChassis.debugInfomation.outputDirection = gChassis.debugInfomation.outputDirection;
			ChassisStatus = ChassisStatus_LostForce;
			break;
		}
		default:
		{
			gChassis.debugInfomation.outputVel = 0.0f;
			gChassis.debugInfomation.outputDirection = gChassis.debugInfomation.outputDirection;
			ChassisStatus = ChassisStatus_LostForce;
			break;
		}
	}
}

	
	
	
//void Task_SpeedCalculate(void)
//{
//	static PosVel_t SpeedFromPos = {0.0f};
//	
//	static float Prev_x, Prev_y= 0.0f;
//	static float Delta_x, Delta_y = 0.0f;
//	
//	if()
//	{
//		if(LocatorInfo.xVal != Prev_x)	
//		{
//			Delta_x = LocatorInfo.xVal - Prev_x;
//				
//			if(ValueInRange_f(Delta_x, -150.0, 150.0))
//				Prev_x = LocatorInfo.xVal;
//		}
//		
//		if(LocatorInfo.yVal != Prev_y)	
//		{
//			Delta_y = LocatorInfo.yVal - Prev_y;
//				
//			if(ValueInRange_f(Delta_y, -150.0, 150.0))
//				Prev_y = LocatorInfo.yVal;
//		}
//		
//		SpeedFromPos.x = Delta_x * FREQUNCE_TIM_TASK * 1000;
//		SpeedFromPos.y = Delta_y * FREQUNCE_TIM_TASK * 1000;
//	}
//	
//	ppsReturn.ppsSpeedX = SpeedFromPos.x;
//	ppsReturn.ppsSpeedY = SpeedFromPos.y;
//}
	
