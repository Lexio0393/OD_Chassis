#include "chassis.h"

#include "C620.h"
#include "pps.h"

gChassis_t gChassis = {0};

void OutputVel2Wheel_FixedC(float vel, float direction, float omega)
{
	wheel_t wheel = {0.0f};
	Speed_t chassisVel = {0.0f};
	
	chassisVel = Transform2ChassisCoordinate(vel, direction, GetAngle());
	
	wheel.rf = CalcWheelSpeed(chassisVel.x, chassisVel.y, omega, RIGHT_FRONT_VERTICAL_ANG);
	wheel.lf = CalcWheelSpeed(chassisVel.x, chassisVel.y, omega, LEFT_FRONT_VERTICAL_ANG);
	wheel.lr = CalcWheelSpeed(chassisVel.x, chassisVel.y, omega, LEFT_REAR_VERTICAL_ANG);
	wheel.rr = CalcWheelSpeed(chassisVel.x, chassisVel.y, omega, RIGHT_REAR_VERTICAL_ANG);
	
	SendCmd2Driver(wheel.rf, wheel.lf, wheel.lr, wheel.rr);
}

void OutputVel2Wheel_RigidC(float vel, float direction, float omega)
{
	wheel_t wheel = {0.0f};
	Speed_t chassisVel = {0.0f};
	
	chassisVel = Transform2ChassisCoordinate(vel, direction, 0.0f);
	
	wheel.rf = CalcWheelSpeed(chassisVel.x, chassisVel.y, omega, RIGHT_FRONT_VERTICAL_ANG);
	wheel.lf = CalcWheelSpeed(chassisVel.x, chassisVel.y, omega, LEFT_FRONT_VERTICAL_ANG);
	wheel.lr = CalcWheelSpeed(chassisVel.x, chassisVel.y, omega, LEFT_REAR_VERTICAL_ANG);
	wheel.rr = CalcWheelSpeed(chassisVel.x, chassisVel.y, omega, RIGHT_REAR_VERTICAL_ANG);
	
	SendCmd2Driver(wheel.rf, wheel.lf, wheel.lr, wheel.rr);
}

void SendCmd2Driver(float rfVel, float lfVel, float lrVel, float rrVel)
{
	VelCrl(RIGHT_FRONT_ID, Vel2RotateVel(rfVel));
	VelCrl(LEFT_FRONT_ID, Vel2RotateVel(lfVel));
	VelCrl(LEFT_REAR_ID, Vel2RotateVel(lrVel));
	VelCrl(RIGHT_REAR_ID,Vel2RotateVel(rrVel));
	
	SetJoggingVel(CAN1, ID_C620_CTRL, C620);
}
	
Speed_t Transform2ChassisCoordinate(float vel, float direction, float postureAngle)
{
	float Dir_C = 0.0f;
	Speed_t Decompose = {0.0f};
	
	Dir_C = direction - postureAngle;
	Decompose.x = vel * arm_cos_f32(ANGLE2RAD(Dir_C));
	Decompose.y = vel * arm_sin_f32(ANGLE2RAD(Dir_C));
	
	gChassis.Speed_C.x = Decompose.x;
	gChassis.Speed_C.y = Decompose.y;
	
	return Decompose;
}

//Speed_t Transform2ChassisCoordinate(float vel, float direction, float postureAngle)
//{
//	float SinValue, CosValue;
//	Speed_t Decompose = {0.0f};
//	
//	arm_sin_cos_f32(postureAngle, &SinValue, &CosValue);		//DSP 三角函数,单位 Deg;

//	//在世界坐标系分解速度
//	Decompose.x = vel * arm_cos_f32(ANGLE2RAD(direction));
//	Decompose.y = vel * arm_sin_f32(ANGLE2RAD(direction));
//	
//	gChassis.Speed_C.x = Decompose.x * CosValue + Decompose.y * SinValue;
//	gChassis.Speed_C.y = -Decompose.x * SinValue + Decompose.y * CosValue;
//	
//	return Decompose;
//}

float CalcWheelSpeed(float velX , float velY , float omega , float angleN)
{
	return (float)(velX * arm_cos_f32(ANGLE2RAD(angleN)) + velY * arm_sin_f32(ANGLE2RAD(angleN)) + omega * MOVEBASE_RADIUS);
}

//wheel_t CalcWheelSpeed(float velX , float velY , float omega)
//{
//	wheel_t wheel = {0.0f};
//	
//	wheel.rf = -velX + velY + omega * MOVEBASE_RADIUS;
//	wheel.lf = -velX - velY + omega * MOVEBASE_RADIUS;
//	wheel.lr =  velX - velY + omega * MOVEBASE_RADIUS;
//	wheel.rr =  velX + velY + omega * MOVEBASE_RADIUS;
//	
//	return wheel;
//}

int Vel2RotateVel(float vel)
{
	return (int)(vel /(PI * WHEEL_DIAMETER) * SECOUND_PER_MIN * M3508_REDUCTION_RATIO);
}

float RotateVel2Vel(int rpm)
{
	return ((float)rpm / SECOUND_PER_MIN) / M3508_REDUCTION_RATIO * PI * WHEEL_DIAMETER;
}
