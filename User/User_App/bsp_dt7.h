#ifndef _BSP_DT7_H
#define _BSP_DT7_H

#include "stm32f4xx.h"

/* ----------------------- RC Channel Definition---------------------------- */
#define RC_CH_VALUE_MIN      ((uint16_t)364 )
#define RC_CH_VALUE_OFFSET   ((uint16_t)1024)
#define RC_CH_VALUE_MAX      ((uint16_t)1684)
#define RC_CH_VALUE_DEADBAND ((uint16_t)10)
/* ----------------------- RC Switch Definition----------------------------- */
#define RC_SW_UP   ((uint16_t)1)
#define RC_SW_MID  ((uint16_t)3)
#define RC_SW_DOWN ((uint16_t)2)
/* ----------------------- PC Key Definition-------------------------------- */
#define KEY_PRESSED_OFFSET_W     ((uint16_t)0x01<<0)
#define KEY_PRESSED_OFFSET_S     ((uint16_t)0x01<<1)
#define KEY_PRESSED_OFFSET_A     ((uint16_t)0x01<<2)
#define KEY_PRESSED_OFFSET_D     ((uint16_t)0x01<<3)
#define KEY_PRESSED_OFFSET_Q     ((uint16_t)0x01<<4)
#define KEY_PRESSED_OFFSET_E     ((uint16_t)0x01<<5)
#define KEY_PRESSED_OFFSET_SHIFT ((uint16_t)0x01<<6)
#define KEY_PRESSED_OFFSET_CTRL  ((uint16_t)0x01<<7)

#define RC_FRAME_LENGTH 18u
#define NULL 0
/* ----------------------- Data Struct ------------------------------------- */
typedef struct
{
	struct
	{
		uint16_t ch0;
		uint16_t ch1;
		uint16_t ch2;
		uint16_t ch3;
		uint8_t s1;
		uint8_t s2;
	}rc;
	
	struct
	{
		int16_t x;
		int16_t y;
		int16_t z;
		uint8_t press_l;
		uint8_t press_r;
	}mouse;
	
	struct
	{
		uint16_t v;
	}key;
	
}RC_Ctl_t;

typedef enum
{
	  Remote_RX = (uint8_t)0x00,
    Remote_RY = (uint8_t)0x01,
    Remote_LX = (uint8_t)0x02,
	  Remote_LY = (uint8_t)0x03,
	
		Remote_S1 = (uint8_t)0x04,
    Remote_S2 = (uint8_t)0x05,
}RemoteChanel_TypeDef;

typedef enum
{
    Remote_SW0   = (int16_t)0,
    Remote_SW1   = (int16_t)1,
    Remote_SW2   = (int16_t)2,
    Remote_SWErr = (int16_t)3,
}Remote_SwitchStatus_TypeDef;

void RC_Init(void);
void DJI_DT7_SerialIsr(void);
void RemoteDataProcess(uint8_t *pData);

int16_t Remote_GetChanalValue(RemoteChanel_TypeDef RemoteChanel);
uint16_t Remote_GetChanalPulse(uint8_t Chanel);
#endif
