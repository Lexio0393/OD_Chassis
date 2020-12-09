#ifndef __PPS_H
#define __PPS_H

#include "stm32f4xx.h"

#define GET_PPS_VALUE_NUM		6	//���ռ������Զ�λϵͳ��float����
#define GET_PPS_DATA_NUM		24  //���ռ������Զ�λϵͳ��uint8_t���� /* 6 * 4byte = 24 */
#define FunId 					3	//����ָ��ID����

/* ���͸���λϵͳ����Ϣ */
typedef union
{
	uint8_t data[GET_PPS_DATA_NUM];
	float	value[GET_PPS_VALUE_NUM];
}PosSend_t;

/* �������ñ������� */
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

/* ��λϵͳĬ�Ϸ���ֵ */
typedef struct
{
	float Yaw;
	float Pitch;
	float Row;
	float xVal;
	float yVal;
	float Omega;
}LocatorInfo_t;

/* ����Ĭ�Ϸ���ֵ */
typedef struct{
		float ppsAngle;			//����ֵ��������ýǶ�
		float ppsX;				//����ֵ���������X����ֵ
		float ppsY;				//����ֵ���������Y����ֵ
		float ppsSpeedX;		//����ֵ���������X���ٶ�
		float ppsSpeedY;		//����ֵ���������Y���ٶ�
		float ppsWZ ;			//����ֵ���������Z����ٶ�
}Pos_t;

/* 
   ��λϵͳ�ٶȣ���ȥ�Ƶ���������ת�Ľ��ٶȲ��������ٶȣ�
   ����λϵͳ�����ڵ�������ʱʹ��
   ��ֵΪ�������ٶȼ��������ǳ���ʵʱ���ٶ�ֵ
*/

typedef struct
{
	float x;			//��λϵͳ��������ϵ��X���ٶ�
	float y;			//��λϵͳ��������ϵ��Y���ٶ�
}PosVel_t;

/* ��λϵͳ��������ϵΪ��������ϵ���ǳ�������ϵ��Y��ƽ�ƺ������ϵ */

void LocatorInit(void);
void SetAngle(float setValue);		//���ö�λϵͳ�ĽǶ�
void SetX(float setValue);			//���ö�λϵͳ��Xֵ
void SetY(float setValue);			//���ö�λϵͳ��Yֵ

float GetAngle(void);				//���ض�λϵͳ�ĽǶ�
float GetX(void);					//���ض�λϵͳ��Xֵ
float GetY(void);					//���ض�λϵͳ��Yֵ
float GetSpeedX(void);				//���ض�λϵͳ��X����ٶ�
float GetSpeedY(void);				//���ض�λϵͳ��Y����ٶ�
float GetWZ(void);					//���ض�λϵͳ��Z����ٶ�ֵ


/**
  ******************************************************************************
  ******************************************************************************
  * @brief ��¼�Ƕ���������
  * RecordContiniousAngle				��¼�����Ƕȱ仯
  * GetContiniousAngle					�����ۻ��Ƕȱ仯ֵ
  
  * @brief ��λϵͳ�ٶȼ��㺯��
  * PosVel_t GetSpeedWithoutOmega		���ؼ�ȥ�Ƶ���������ת�Ľ��ٶȲ��������ٶȺ���ٶ�
  
  * @brief �����麯��
  * void CheckPos1						ͨ����λϵͳʵʱ����ֵ�ж��Ƿ���ȷ
  * void CheckPos2						ͨ����λϵͳ���͵Ŀ���ָ���ж��Ƿ���ȷ
  ******************************************************************************
  */
  
void RecordContiniousAngle(float value);
float GetContiniousAngle(void);


float GetLengthPassed(void);

void Locator_SerialIsr(void);

PosVel_t GetSpeedWithoutOmega(void);
//void CheckPos1(void);
//void CheckPos2(PosSend_t pos);
extern Pos_t ppsReturn;
extern LocatorInfo_t LocatorInfo;
#endif
