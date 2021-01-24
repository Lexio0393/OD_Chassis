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
	/* ȷ��DMA��������λ��� */
	while(DMA_GetCmdStatus(DMA1_Stream5) != DISABLE)
	{

	}
	
	/*usart2 rx��Ӧdma1��ͨ��4��������5*/	
	DMA_InitStructure.DMA_Channel = USART2_RX_DMA_CHANNEL;
	/*����DMAԴ���������ݼĴ�����ַ*/
	DMA_InitStructure.DMA_PeripheralBaseAddr = USART2_RX_ADDR;
	/*�ڴ��ַ(Ҫ����ı�����ָ��)*/
	DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)&sbus_rx_buffer[0][0];
	/*���򣺴����赽�ڴ�*/
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
	/*�����СDMA_BufferSize=SENDBUFF_SIZE*/
	DMA_InitStructure.DMA_BufferSize = RC_FRAME_LENGTH;
	/*�����ַ����*/
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	/*�ڴ��ַ����*/
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	/*�������ݵ�λ*/
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	/*�ڴ����ݵ�λ 8bit*/
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	/*DMAģʽ������ѭ��*/
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
	/*���ȼ����ǳ���*/
	DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
	/*����FIFO*/
	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
	/*FIFO��ֵ*/
	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;
	/*�洢��ͻ������ 1 ������*/
	DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
	/*����ͻ������ 1 ������*/
	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
	/*First used memorry configuration*/
	DMA_DoubleBufferModeConfig(DMA1_Stream5,(uint32_t)&sbus_rx_buffer[1][0],DMA_Memory_0); //first used memory configuration
	/*ʹ��DMA*/
	DMA_DoubleBufferModeCmd(DMA1_Stream5, ENABLE);
	/*����DMA1��������*/
	DMA_Init(DMA1_Stream5,&DMA_InitStructure);
	
	DMA_Cmd(DMA1_Stream5,ENABLE);		
	
	/* �ȴ�DMA��������Ч*/
//	while(DMA_GetCmdStatus(USART2_RX_DMA_STREAM) != ENABLE)
//  while(DMA_GetCmdStatus(USART2_RX_DMA_STREAM) != DISABLE)
//	{

//	}
//	DMA_SetCurrDataCounter(DMA1_Stream5, RC_FRAME_LENGTH);
	/*ʹ��DMA����*/ 
	USART_DMACmd(USART2,USART_DMAReq_Rx,ENABLE);
}
