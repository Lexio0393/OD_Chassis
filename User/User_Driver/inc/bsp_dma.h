#ifndef __BSP_DMA_H
#define __BSP_DMA_H

#include "stm32f4xx.h"
#include "stm32f4xx_dma.h"



/********USART2_DMA_define********/
#define USART2_RX_ADDR						(uint32_t)(&USART2->DR)	//串口1数据寄存器地址
#define USART2_RX_DMA_CHANNEL           	DMA_Channel_4		//DMA通道号
#define USART2_RX_DMA_STREAM           		DMA1_Stream5		//DMA数据流

void BSP_DMA_USART2RX_Init(void);

#endif

