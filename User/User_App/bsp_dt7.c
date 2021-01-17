#include "bsp_dt7.h"

volatile unsigned char sbus_rx_buffer[2][RC_FRAME_LENGTH]; //double sbusrx buffer to save data

volatile static RC_Ctl_t RC_CtrlData;

uint16_t Remote_Value[6];

int16_t Remote_GetChanalValue(RemoteChanel_TypeDef RemoteChanel)
{
	static uint16_t PulseWidth;
	
	if((uint8_t)RemoteChanel < 6)
	{
		PulseWidth = Remote_Value[RemoteChanel];
	}
	
	switch ((uint8_t)RemoteChanel)
	{
		case Remote_RX:
		case Remote_RY:
		case Remote_LY:
		case Remote_LX:
			
				if(PulseWidth <= RC_CH_VALUE_DEADBAND && PulseWidth >=  RC_CH_VALUE_DEADBAND)
				{
					return (int16_t)0;
				}
				return (int16_t)PulseWidth;
  
		case Remote_S1:
		case Remote_S2:
		 
				if(PulseWidth == RC_SW_UP)
					return Remote_SW0;
				if(PulseWidth == RC_SW_MID)
					return Remote_SW1;
				if(PulseWidth == RC_SW_DOWN)
					return Remote_SW2;
				return Remote_SWErr;
	
	}
	
	return 0xFFFF >> 1;
}

void DJI_DT7_SerialIsr(void)
{
	if(USART_GetITStatus(USART2, USART_IT_IDLE) != RESET)
	{
		//Target is Memory0
		if(DMA_GetCurrentMemoryTarget(DMA1_Stream5) == 0)
		{
			DMA_Cmd(DMA1_Stream5, DISABLE);

			DMA1_Stream5->NDTR = (uint16_t)RC_FRAME_LENGTH; //relocate the dma memory pointer to the beginning position
			DMA1_Stream5->CR |= (uint32_t)(DMA_SxCR_CT); 		//enable the current selected memory is Memory 1
			
			if(DMA_GetCurrDataCounter(DMA1_Stream5) == RC_FRAME_LENGTH) //ensure received complete frame data.
			{
				RemoteDataProcess((uint8_t*)sbus_rx_buffer[0]);
			}
		
			DMA_Cmd(DMA1_Stream5, ENABLE);
		}
		//Target is Memory1
		else
		{
			DMA_Cmd(DMA1_Stream5, DISABLE);
			
			DMA1_Stream5->NDTR = (uint16_t)RC_FRAME_LENGTH; //relocate the dma memory pointer to the beginning position
			DMA1_Stream5->CR &= ~(uint32_t)(DMA_SxCR_CT); //enable the current selected memory is Memory 0			
			
			if(DMA_GetCurrDataCounter(DMA1_Stream5) == RC_FRAME_LENGTH)
			{
				RemoteDataProcess((uint8_t*)sbus_rx_buffer[1]);
			}
			
			DMA_Cmd(DMA1_Stream5, ENABLE);
		}
		
		//clear the idle pending flag
		(void)USART2->SR;
		(void)USART2->DR;
	} 
}

static void RemoteDataProcess(uint8_t *pData)
{
  if(pData == NULL)
  {
		return;
  }

  RC_CtrlData.rc.ch0 = ((int16_t)pData[0] | ((int16_t)pData[1] << 8)) & 0x07FF;
  RC_CtrlData.rc.ch1 = (((int16_t)pData[1] >> 3) | ((int16_t)pData[2] << 5))& 0x07FF;
  RC_CtrlData.rc.ch2 = (((int16_t)pData[2] >> 6) | ((int16_t)pData[3] << 2) |((int16_t)pData[4] << 10)) & 0x07FF;
  RC_CtrlData.rc.ch3 = (((int16_t)pData[4] >> 1) | ((int16_t)pData[5]<<7)) &0x07FF;
  
  RC_CtrlData.rc.s1  = ((pData[5] >> 4) & 0x000C) >> 2;
  RC_CtrlData.rc.s2  = ((pData[5] >> 4) & 0x0003);
  
  RC_CtrlData.mouse.x = ((int16_t)pData[6])  | ((int16_t)pData[7] << 8);
  RC_CtrlData.mouse.y = ((int16_t)pData[8])  | ((int16_t)pData[9] << 8);
  RC_CtrlData.mouse.z = ((int16_t)pData[10]) | ((int16_t)pData[11] << 8);
  
  RC_CtrlData.mouse.press_l = pData[12];
  RC_CtrlData.mouse.press_r = pData[13];
  RC_CtrlData.key.v = ((int16_t)pData[14]);	// | ((int16_t)pData[15] << 8);
  //your control code бн
}

