#include "pps.h"
#include "bsp_serial.h"
#include "bsp_systick.h"
#include "algorithm.h"

volatile uint8_t LocatorUpdated = 0;

uint8_t TxMsg[16] = 
{
	'A',	'C',	'T',	0x00,
	0x00,	0x00,	0x00,	0x00,
	0x00,	0x00,	0x00,	0x00,
	0x00,	0x00,	0x00,	0x00,	
};

volatile uint8_t TxIndex = 0, MsgLength = 0;

LocatorInfo_t LocatorInfo = 
{
	0.0f,
	0.0f,
	0.0f,
	0.0f,
	0.0f,
	0.0f,
};

Pos_t ppsReturn={0.f};	

void LocatorInit(void)
{
	Serial_SendString(Serial1, (uint8_t*)"ACT0");
	Delay_ms(10);    
			
	Serial_SendString(Serial1, (uint8_t*)"ACT0");
	Delay_ms(10);  
			
	Serial_SendString(Serial1, (uint8_t*)"ACT0");
	Delay_ms(10);
}


void Locator_SerialIsr(void)
{
	static uint8_t ch;	
	static uint8_t count = 0;
	static uint8_t i = 0;
	static PosSend_t posture = {0};
	
	
	if(USART_GetFlagStatus(USART3, USART_IT_RXNE) == SET)
	{
		USART_ClearITPendingBit(USART3, USART_IT_RXNE);
		ch = USART_ReceiveData(USART3);
		
		switch(count)
		{
			case 0:							//Wait Start Signal[1]
				if(ch == 0x0d)
					count++;
				else
					count = 0;
				break;
				
			case 1:							//Wait Start Signal[2]
				if(ch == 0x0a)
				{
					i = 0;
					count++;
				}
				
				else if(ch == 0x0d);
				
				else
					count = 0;
				break;
				
			case 2:							//Data Recieving
				posture.data[i] = ch;
				i++;
			
				if(i >= GET_PPS_DATA_NUM)
				{
					i = 0;
					count++;
				}
				break;
				
			case 3:						//Wait End Signal[1]
				if(ch == 0x0a)
					count++;
				else
					count = 0;
				break;
				
			case 4:						//Wait End Signal[2]
				if(ch == 0x0d)		//End Signal[2] Matched
				{
					
					LocatorInfo.Yaw		= posture.value[0];
					LocatorInfo.Pitch	= posture.value[1];
					LocatorInfo.Row		= posture.value[2];
					LocatorInfo.xVal	= posture.value[3];
					LocatorInfo.yVal	= posture.value[4];
					LocatorInfo.Omega	= posture.value[5];
					LocatorUpdated = 1;
				}
				count = 0;
				break;
				
			default:
				count = 0;
				break;
		}
	}
	
	/*	发 送 中 断	
	if(USART_GetITStatus(USART3, USART_IT_TC))	//Tx Int
	{
		USART_ClearITPendingBit(USART3, USART_IT_TC);
		
		if(TxIndex < MsgLength)
			USART_SendData(USART3, TxMsg[TxIndex]);
		TxIndex++;
	}
	*/
}

static float continiousAngle = 0.0f;
static int angleLoop = 0;

void RecordContiniousAngle(float value)
{
	static float angleRecord = 0.0f;
	
	if(value < -135.0f && angleRecord > 135.0f)
	{
		angleLoop++;
	}
	
	if(value > 135.0f && angleRecord < -135.0f)
	{
		angleLoop--;
	}
	
	angleRecord = value;
	continiousAngle = value + angleLoop * 360.0f;
}

float GetContiniousAngle(void)
{
	return continiousAngle;
}

void SetAngle(float setValue)
{
	RecordContiniousAngle(setValue);
	
	TxMsg[FunId] = FunId_SetA;
	*((float*)&TxMsg[4]) = continiousAngle;
	TxIndex = 1;
	MsgLength = 8;
	USART_SendData(USART1, TxMsg[0]);
	
	ppsReturn.ppsAngle = continiousAngle;
}

void SetX(float setValue)
{
//	ppsReturn.ppsX = -setValue + DISY_OPS2CENTER * sinf(ANGLE2RAD(ppsReturn.ppsAngle));
	TxMsg[FunId] = FunId_SetX;
	*((float*)&TxMsg[4]) = setValue;
	TxIndex = 1;
	MsgLength = 8;
	USART_SendData(USART1, TxMsg[0]);
	
	ppsReturn.ppsX = setValue;
}

void SetY(float setValue)
{
//	ppsReturn.ppsY = -setValue - DISY_OPS2CENTER * cosf(ANGLE2RAD(ppsReturn.ppsAngle)) + DISY_OPS2CENTER;
	TxMsg[FunId] = FunId_SetY;
	*((float*)&TxMsg[4]) = setValue;
	TxIndex = 1;
	MsgLength = 8;
	USART_SendData(USART1, TxMsg[0]);
	
	ppsReturn.ppsY = setValue;
}

/*返回定位系统的角度*/                               
float GetAngle(void)
{
	ppsReturn.ppsAngle = LocatorInfo.Yaw;
	return ReturnLimitAngle(ppsReturn.ppsAngle);
}
/*返回定位系统的X值*/
float GetX(void)
{
	ppsReturn.ppsX = LocatorInfo.xVal;
	return ppsReturn.ppsX;
}
/*返回定位系统的Y值*/
float GetY(void)
{
	ppsReturn.ppsY = LocatorInfo.yVal;
	return ppsReturn.ppsY;
}
/*返回定位系统的X轴的速度*/
//float GetSpeedX(void)
////{
//	SetStartTime();
//	static float Prev_x = 0.0f;
//	static float Delta_x;
//	
//	if(LocatorUpdated)
//	{
//		SetEndTime();
//		if(LocatorInfo.xVal != Prev_x)	//x坐标改变了(这里数据没收到的话，浮点值是完全一样的)
//		{
//			Delta_x = LocatorInfo.xVal - Prev_x;
//			
//			if(ValueInRange_f(Delta_x, -150.0, 150.0))
//				Prev_x = LocatorInfo.xVal;
//		}
//		
//		ppsReturn.ppsSpeedX = Delta_x / GetTimeInterval();
//	}
//	return ppsReturn.ppsSpeedX;
//}
/*返回定位系统的Y轴的速度*/
//float GetSpeedY(void)
//{
//	SetStartTime();
//	static float Prev_y = 0.0f;
//	static float Delta_y;
//	
//	if(LocatorUpdated)
//	{
//		SetEndTime();
//		if(LocatorInfo.yVal != Prev_y)	//y坐标改变了(这里数据没收到的话，浮点值是完全一样的)
//		{
//			Delta_y = LocatorInfo.yVal - Prev_y;
//			
//			if(ValueInRange_f(Delta_y, -150.0, 150.0))
//				Prev_y = LocatorInfo.yVal;
//		}
//		
//		ppsReturn.ppsSpeedY = Delta_y / GetTimeInterval();
//	}
//	return ppsReturn.ppsSpeedY;
//}
/*返回定位系统的Z轴角速度值*/
float GetWZ(void)
{
	ppsReturn.ppsWZ = LocatorInfo.Omega;
	return ppsReturn.ppsWZ;
}


//返回减去绕底盘中心旋转角速度产生的线速度后的速度
//PosVel_t GetSpeedWithoutOmega(void)
//{
//	PosVel_t vel = {0.0f};
//	float rotateVel;
//	float rotateVelDirection = 0.0f;
//	
//	rotateVel = ANGLE2RAD(GetWZ()) * sqrtf(DISX_OPS2CENTER * DISX_OPS2CENTER + DISY_OPS2CENTER * DISY_OPS2CENTER);
//	
//	rotateVelDirection = 90.0f + RAD2ANGLE(atan2f(DISY_OPS2CENTER,DISX_OPS2CENTER)) + GetAngle();
//	
//	AngleLimit(&rotateVelDirection);
//	
//	vel.x = GetSpeedX() - rotateVel * cosf(ANGLE2RAD(rotateVelDirection));
//	vel.y = GetSpeedY() - rotateVel * sinf(ANGLE2RAD(rotateVelDirection));
//	
//	return vel;
//}

//暂时代替正交编码器计算路程
//计算累计行走路径，后期需要添加计算在预计路径上的行走长度
float GetLengthPassed(void)
{
	static float Last_x, Last_y= 0.0f;
	static float Err_x, Err_y;
	static float PassedLen = 0.0f;
		
	if(LocatorUpdated)
	{
		if(LocatorInfo.xVal != Last_x)	//x坐标改变了(这里数据没收到的话，浮点值是完全一样的)
		{
			Err_x = LocatorInfo.xVal - Last_x;
			
			if(ValueInRange_f(Err_x, -150.0, 150.0))
				Last_x = LocatorInfo.xVal;
		}
		
		if(LocatorInfo.yVal != Last_y)	//y坐标改变了(这里数据没收到的话，浮点值是完全一样的)
		{
			Err_y = LocatorInfo.yVal - Last_y;
			
			if(ValueInRange_f(Err_y, -150.0, 150.0))
				Last_y = LocatorInfo.yVal;
		}
		
		PassedLen += sqrt(pow(Err_x, 2) + pow(Err_y, 2));
	}
	return PassedLen;
}
