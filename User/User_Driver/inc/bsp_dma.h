#ifndef __BSP_DMA_H
#define __BSP_DMA_H

#include "stm32f4xx.h"
#include "stm32f4xx_dma.h"



/********USART2_DMA_define********/
#define USART2_RX_ADDR						(uint32_t)(&USART2->DR)	//����1���ݼĴ�����ַ
#define USART2_RX_DMA_CHANNEL           	DMA_Channel_4		//DMAͨ����
#define USART2_RX_DMA_STREAM           		DMA1_Stream5		//DMA������

void BSP_DMA_USART2RX_Init(void);

#endif

