#ifndef _PACK_DEAL_H_
#define _PACK_DEAL_H_


#include "stm32f4xx.h"

#define FRAME_NONE     0x00

typedef struct TYPE_Pack_Info_t
{
    uint8_t data_size;         /*�����ֽڴ�С ����֡ͷ֡β    */
    uint8_t frame_head1;       /*֡ͷ1                        */
    uint8_t frame_head2;       /*֡ͷ2 ��������Ϊ FRAME_NONE  */
    uint8_t frame_end1;        /*֡β1                        */
    uint8_t frame_end2;        /*֡β2 ��������Ϊ FRAME_NONE  */
    uint8_t* pack;             /*���ݰ�����ָ��               */
    //�����ʼ��
    uint8_t i;                  /*���ݽ��� ����  ����          */
    uint8_t count;              /*���ݽ���  ��  ���ݰ��ܴ�С    */
}TYPE_Pack_Info_t, *pTYPE_Pack_Info_t;


uint8_t Pack_Deal_API_Read_Pack  (pTYPE_Pack_Info_t pPackInfo_t, uint8_t ch);
void    Pack_Deal_API_Data_Pack  (pTYPE_Pack_Info_t pPackInfo_t, uint8_t* p_t);

void    Pack_Deal_API_SetPackInfo(pTYPE_Pack_Info_t pPackInfo_t, uint8_t data_size,
                                  uint8_t frame_head1, uint8_t frame_head2,
                                  uint8_t frame_end1,  uint8_t frame_end2,
                                  uint8_t* buff);

void    Pack_Deal_API_Bit_Copy(uint8_t *pNew, uint8_t *pOld, uint8_t dataSize);

uint8_t Online_State_Deal(uint8_t oldSta, uint8_t frame_num, uint8_t min_frame);


#endif





















