#ifndef __PPS_H
#define __PPS_H

#include "stm32f4xx.h"
#include "motion.h"

#define GET_PPS_VALUE_NUM		6	//接收几个来自定位系统的float数据
#define GET_PPS_DATA_NUM		24  //接收几个来自定位系统的uint8_t数据 /* 6 * 4byte = 24 */
#define FunId 					3	//功能指令ID个数

/* 发送给定位系统的信息 */
typedef union
{
	uint8_t data[GET_PPS_DATA_NUM];
	float	value[GET_PPS_VALUE_NUM];
}PosSend_t;

/* 功能设置变量定义 */
typedef enum
{
	FunId_Idle  = '\0',
	FunId_Reset	= '0',
	FunId_Calibration	= 'R',
	FunId_SetA		= 'J',
	FunId_SetX		= 'X',
	FunId_SetY		= 'Y',
	FunId_SetXY		= 'D',
	FunId_SetAXY	= 'A',	
}Locator_FunIdTypeDef;

/* 定位系统默认返回值 */
typedef struct
{
	float Yaw;
	float Pitch;
	float Row;
	float xVal;
	float yVal;
	float Omega;
}LocatorInfo_t;

/* 处理默认返回值 */
typedef struct{
		float ppsAngle;			//返回值处理后所得角度
		float ppsX;				//返回值处理后所得X坐标值
		float ppsY;				//返回值处理后所得Y坐标值
		float ppsSpeedX;		//返回值处理后所得X轴速度
		float ppsSpeedY;		//返回值处理后所得Y轴速度
		float ppsWZ ;			//返回值处理后所得Z轴角速度
}Pos_t;

/* 
   定位系统速度（减去绕底盘中心旋转的角速度产生的线速度）
   当定位系统不处于底盘中心时使用
   该值为车体线速度计算量，是车体实时线速度值
*/

typedef struct
{
	float x;			//定位系统所在坐标系在X轴速度
	float y;			//定位系统所在坐标系在Y轴速度
}PosVel_t;

/* 定位系统所在坐标系为绝对坐标系，是车体坐标系向Y轴平移后的坐标系 */

void LocatorInit(void);
void SetAngle(float setValue);		//设置定位系统的角度
void SetX(float setValue);			//设置定位系统的X值
void SetY(float setValue);			//设置定位系统的Y值

float GetAngle(void);				//返回定位系统的角度
float GetX(void);					//返回定位系统的X值
float GetY(void);					//返回定位系统的Y值
float GetSpeedX(void);				//返回定位系统的X轴的速度
float GetSpeedY(void);				//返回定位系统的Y轴的速度
float GetWZ(void);					//返回定位系统的Z轴角速度值


/**
  ******************************************************************************
  ******************************************************************************
  * @brief 记录角度连续函数
  * RecordContiniousAngle				记录连续角度变化
  * GetContiniousAngle					返回累积角度变化值
  
  * @brief 定位系统速度计算函数
  * PosVel_t GetSpeedWithoutOmega		返回减去绕底盘中心旋转的角速度产生的线速度后的速度
  
  * @brief 坐标检查函数
  * void CheckPos1						通过定位系统实时返回值判断是否正确
  * void CheckPos2						通过向定位系统发送的控制指令判断是否正确
  ******************************************************************************
  */
  
void RecordContiniousAngle(float value);
float GetContiniousAngle(void);

Pose_t GetCurrentPoint(void);
float GetLengthPassed(void);

void Locator_SerialIsr(void);

PosVel_t GetSpeedWithoutOmega(void);
//void CheckPos1(void);
//void CheckPos2(PosSend_t pos);
extern Pos_t ppsReturn;
extern LocatorInfo_t LocatorInfo;
#endif
