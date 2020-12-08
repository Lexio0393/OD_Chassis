#include "chassis.h"

void OutputVel2Wheel_FixedC(float vel, float direction, float omega)
{
	//gChassis has been recoreded
//	Transform2ChassisCoordinate();
}

vector_t Transform2ChassisCoordinate(float vel, float direction, float postureAngle)
{
	vector_t Vel_C = {0.0f};
	float Dir_C= 0.0f;

	Dir_C = direction - postureAngle;
	
	Vel_C = vel * arm_cos_f32(ANGLE2RAD(Dir_C));
	Vel_C = vel * arm_sin_f32(ANGLE2RAD(Dir_C));
}