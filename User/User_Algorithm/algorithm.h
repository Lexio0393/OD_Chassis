#ifndef _ALGORITHM_H
#define _ALGORITHM_H

#include "stm32f4xx.h"
#include "arm_math.h"

#include <stdio.h>
#include <math.h>

typedef struct
{
	float a;
	float b;
	float c;
}QudraticFunction;

/* 角度制弧度制互换 */
#define ANGLE2RAD(x) (x / 180.0f * PI)		//角度制转化为弧度制
#define RAD2ANGLE(x) (x /PI * 180.0f)			//弧度制转换为角度制

/*
//轨迹用
typedef struct
{
	float	x;
	float	y;
}Point_t;

typedef struct
{
	float Kp;
	
	float Deadband;
	float MaxOutput;
	
	float Value;
	float Ref;
	float Output;
	
	Signal_TypeDef Signal;
}TrackValue_TypeDef;
*/

void absLimit(float* Val, float Limit);

float Min_f(float x, float y);
float Max_f(float x, float y);
float MinAbs_f(float x, float y);
float MaxAbs_f(float x, float y);

float InvSqrt(float Val);

uint8_t ValueInRange_u(uint32_t Value, uint32_t Min, uint32_t Max);
uint8_t ValueInRange_i(int32_t Value, int32_t Min, int32_t Max);
uint8_t ValueInRange_f(float Value, float Min, float Max);
float FlexibelValue(float dstVal, float srcVal, float step);

float TurnInferiorArc(float targetAngle , float actualAngle);
void AngleLimit(float *angle);
float ReturnLimitAngle(float angle);

/*
//1ì?￡ó?
void TrackValue_Init(TrackValue_TypeDef *TrackValue, float Kp, float Deadband, float MaxOutput);
void TrackValue_Reset(TrackValue_TypeDef *TrackValue, float Kp);
void TrackValue_Calc(TrackValue_TypeDef *TrackValue);
*/
#endif
