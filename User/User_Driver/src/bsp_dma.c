#include "bsp_dma.h"
#include "bsp_dt7.h"

extern volatile unsigned char sbus_rx_buffer[2][RC_FRAME_LENGTH]; //double sbusrx buffer to save data

void BSP_DMA_USART2RX_Init(void)
{
	DMA_InitTypeDef DMA_InitStructure;
//	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);
	
//	NVIC_InitStructure.NVIC_IRQChannel = DMA1_Stream5_IRQn;
//	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x01;
//	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x01;
//	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
//	NVIC_Init(&NVIC_InitStructure);
	
	DMA_DeInit(DMA1_Stream5);
	/* 确保DMA数据流复位完成 */
	while(DMA_GetCmdStatus(DMA1_Stream5) != DISABLE)
	{

	}
	
	/*usart2 rx对应dma1，通道4，数据流5*/	
	DMA_InitStructure.DMA_Channel = USART2_RX_DMA_CHANNEL;
	/*设置DMA源：串口数据寄存器地址*/
	DMA_InitStructure.DMA_PeripheralBaseAddr = USART2_RX_ADDR;
	/*内存地址(要传输的变量的指针)*/
	DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)&sbus_rx_buffer[0][0];
	/*方向：从外设到内存*/
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
	/*传输大小DMA_BufferSize=SENDBUFF_SIZE*/
	DMA_InitStructure.DMA_BufferSize = RC_FRAME_LENGTH;
	/*外设地址不增*/
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	/*内存地址自增*/
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	/*外设数据单位*/
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	/*内存数据单位 8bit*/
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	/*DMA模式：不断循环*/
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
	/*优先级：非常高*/
	DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
	/*禁用FIFO*/
	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
	/*FIFO阈值*/
	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;
	/*存储器突发传输 1 个节拍*/
	DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
	/*外设突发传输 1 个节拍*/
	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
	/*First used memorry configuration*/
	DMA_DoubleBufferModeConfig(DMA1_Stream5,(uint32_t)&sbus_rx_buffer[1][0],DMA_Memory_0); //first used memory configuration
	/*使能DMA*/
	DMA_DoubleBufferModeCmd(DMA1_Stream5, ENABLE);
	/*配置DMA1的数据流*/
	DMA_Init(DMA1_Stream5,&DMA_InitStructure);
	
	DMA_Cmd(DMA1_Stream5,ENABLE);		
	
	/* 等待DMA数据流有效*/
//	while(DMA_GetCmdStatus(USART2_RX_DMA_STREAM) != ENABLE)
//  while(DMA_GetCmdStatus(USART2_RX_DMA_STREAM) != DISABLE)
//	{

//	}
//	DMA_SetCurrDataCounter(DMA1_Stream5, RC_FRAME_LENGTH);
	/*使能DMA接收*/ 
	USART_DMACmd(USART2,USART_DMAReq_Rx,ENABLE);
}
