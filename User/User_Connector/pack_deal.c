#include "pack_deal.h"
/********************************************************************************
* @File name:   readpack.c
* @Author:      ShengNan Wang
* @Version:     1.3.1
* @Date:        2019.12.24
* @Description: ͨ�ô��������
*   ��Ȩ���У�����Ҳûʲô��
*               
*********************************************************************************
* @�����ֲָ��:
*   �ڶ�Ӧ���ڳ�ʼ��ͷ�ļ��� ���� ��������ݰ��ṹ�� 
*       ����һ����Ӧ�Ľṹ�����Ϊȫ�ֱ���,�����ⲿ����
*       ��ֱ�ӷ��� ���ݰ��ṹ�� ȫ�ֱ���   
*
*   �ڴ����жϷ�������
*            ����һ��uint8_t�ľ�̬���ݽ��ջ�����, ���ٱ����ݰ��ֽڴ�С ��4 ��
*            ���� TYPE_Pack_Info_t ���ݰ��ṹ��Ϊ��̬����
*               ����ɶ�Ӧ���ݵĳ�ʼ��(������Ϣ), �����ͷ�ļ�
*   �ڴ����жϷ������ж��ж��Ƿ�Ϊ�����ж�
*       ����, ����жϱ�־
*             ���յ�ǰ�ֽ�����ch, ������ uint8_t Pack_Deal_API_Read_Pack(pTYPE_Pack_Info_t , ch)
*                           ���ݷ���ֵ�ж��ǻ��������ݵȺ������
*                               ����ֵΪ0ʱ��δ���һ�����ݰ�����, һ�㲻���κβ���
*                               ����ֵΪ1ʱ���һ�����ݽ���, 
*                                    ǿ��ת�� ���ջ�����ָ�� Ϊ ��������ݰ��ṹ�� ��ָ��
*                                    ����ȡֵ���� ��������ݰ��ṹ��
*
* @�����ֲָ��:
*     �ڶ�Ӧ������͵�ͷ�ļ��� �����������ݰ��ṹ�� _t
*         ����һ����Ӧ�Ľṹ�����Ϊȫ�ֱ���,�����ⲿ����
*         ��ֱ�ӷ��� ���ݰ��ṹ�� ȫ�ֱ���
*
*     �ڷ��˺�����
*          ����һ��uint8_t�ľ�̬���ݷ��ͻ�����, ���ٱ����ݰ��ֽڴ�С ��4 ��
*          ���� TYPE_Pack_Info_t ���ݰ��ṹ��Ϊ��̬����
*              ����ɶ�Ӧ���ݵĳ�ʼ��, �����ͷ�ļ�
*
                                        ��ת�����ݰ��ṹ��ָ��Ϊuint8_t*
*     ����void Pack_Deal_API_Data_Pack(pTYPE_Pack_Info_t, (uint8_t*) p_t(&_t)),   �����ݴ�������ݷ��ͻ�����
*          �����ݷ��ͻ�������ȡ�����ڷ��� pTYPE_Pack_Info_t -> count ������
*
*  @����˵��
*     1.2.0    2019,10,15
*                 1.���������ղ������ݵ�BUG
*                 2.����һ���������ݰ���Ϣ��API
*                 3.����������
*                 4.�����һ����ֲʾ��
*     1.2.1  2019.12.18
*                 1.��������һ�����������ղ������ݵ�BUG
*     1.3.1  2019.12.24
*                 1.��������ͷ��β�����ݴ��, bitcopy
*     1.3.2  2020.2.13
*                 1.��������״̬������ Online_State_Deal,
*******************************************************************************/


/*******************************************************
*
* Function name ��Read_Pack
* Description   : ͨ�ý�� ���ڽ�������
*                 �ڴ����жϷ���������
* Parameter     ��
*        @pPackInfo_t      ���ݰ���Ϣ�ṹ��ָ��
*                               �����ͷ�ļ�
*        @ch               �յ���һ���ֽ�����
*       
* Return        ��1 �������
                  0 ����δ���
**********************************************************/



uint8_t Pack_Deal_API_Read_Pack(pTYPE_Pack_Info_t pPackInfo_t, uint8_t ch)
{
    switch (pPackInfo_t -> count)
    {
        case 0:
            // ֡ͷ1
            if (ch == pPackInfo_t -> frame_head1)
            {
                // ��ȷ
                if(pPackInfo_t -> frame_head2 == FRAME_NONE)
                    // ������֡ͷ2
                    pPackInfo_t -> count = 2; // ת����������
                else
                    // ����֡ͷ2
                    pPackInfo_t -> count = 1;  //ת��֡ͷ2�ж�
            }
            else
            {
                // ����
                pPackInfo_t -> count = 0;     // ת��֡ͷ1�ж�
            }
            
            //19.12.18 add�����
            pPackInfo_t -> i = 0;
            break;
            
        case 1:
            // ֡ͷ2
            
            // ��ȷ
            if (ch == pPackInfo_t -> frame_head2)
                pPackInfo_t -> count = 2; // ת����������
            
            // ���ݻ���֡ͷ1��,(����֡β2�ж�Ϊ֡ͷ1)
            else if (ch == pPackInfo_t -> frame_head1)
                pPackInfo_t -> count = 1; // ����֡ͷ2�ж�
            // ����
            else
                pPackInfo_t -> count = 0; // ת��֡ͷ1�ж�

            
            break;
            
            
        case 2:
            // ��������
            pPackInfo_t -> pack[pPackInfo_t -> i] = ch;
            pPackInfo_t -> i ++;
            if (pPackInfo_t -> i >= pPackInfo_t -> data_size)
            {
                // �����������
                pPackInfo_t -> count = 3; // ת��֡β1�ж�
            }
            break;
            
        case 3:
            // ֡β1�ж�
            if (ch == pPackInfo_t -> frame_end1)
            {
                // ֡β1��ȷ
                if(pPackInfo_t -> frame_end2 == FRAME_NONE)
                {
                    // ������֡β2
                    pPackInfo_t -> count = 0;        // ������һ֡����
                    
                    return 1;
                }
                else
                {
                    // ����֡β2
                    pPackInfo_t -> count = 4;
                }                
            }
            else
            {
                // ֡β1����
                pPackInfo_t -> count = 0;   //���½�������
            }
            break;
        case 4:
            // ֡β2�ж�
            if (ch == pPackInfo_t -> frame_end2)
            {
                // ֡β2��ȷ
                pPackInfo_t -> count = 0;         // ������һ֡����
                return 1;
            }
            else
            {
                // ֡β2����
                pPackInfo_t -> count = 0; //���½�������
            }
            break;
        
        default:
            //��������
            pPackInfo_t -> count = 0;  //��������
    }
    return 0;
}

/*******************************************************
*
* Function name ��Data_Pack
* Description   : ���ݴ����Ϣ,�����Ӧ�Ľṹ�� �� ���ݰ����ͻ�����
*                 
* Parameter     ��
*        @pPackInfo_t      ���ݰ���Ϣ�ṹ��ָ��
*                               �����ͷ�ļ�
*        @p_t              ���ݰ���Ϣ�ṹ��ָ�� ǿ��ת�� Ϊuint8_t*���ָ��
*       
* Return        : None
                
**********************************************************/



void Pack_Deal_API_Data_Pack(pTYPE_Pack_Info_t pPackInfo_t, uint8_t* p_t)
{
    uint8_t i = 0;
    //��֡ͷ1
    uint8_t offset = 1;
    pPackInfo_t -> pack[0] = pPackInfo_t -> frame_head1;
    
    if (pPackInfo_t -> frame_head2 != FRAME_NONE)
    {   
        // ����֡ͷ2 �����
        offset = 2;
        pPackInfo_t -> pack[1] = pPackInfo_t -> frame_head2;
    }
    
    // ���ݴ��
    for(i = 0;i < pPackInfo_t -> data_size; i++)
    {
        pPackInfo_t -> pack [i + offset] = p_t[i];
    }
    
    // ��֡β1
    pPackInfo_t -> pack[i + offset] = pPackInfo_t -> frame_end1;
    i ++;
    if (pPackInfo_t -> frame_end2 == FRAME_NONE)
    {
        // ������֡β2, δβ��0
        pPackInfo_t -> pack[i + offset] = 0;
    }
    else
    {
        // ����֡β2, ��֡β2, ��δβ��0
        pPackInfo_t -> pack[i + offset] = pPackInfo_t -> frame_end2;
        i ++;
        pPackInfo_t -> pack[i + offset] = 0;
    }
    pPackInfo_t -> count = pPackInfo_t -> data_size + offset * 2;
    pPackInfo_t -> i = 0;
}

/*******************************************************
*
* Function name ��Pack_Deal_API_SetPackInfo
* Description   : �������ݰ��ṹ�����Ϣ
*                 
* Parameter     ��
*        @pPackInfo_t      ���ݰ���Ϣ�ṹ��ָ��
*                               �����ͷ�ļ�
*        @frame_head1
*        @frame_head2
*        @frame_end1
*        @frame_end2
*       
* Return        : None
                
**********************************************************/


void    Pack_Deal_API_SetPackInfo(pTYPE_Pack_Info_t pPackInfo_t, uint8_t data_size,
                                  uint8_t frame_head1, uint8_t frame_head2,
                                  uint8_t frame_end1,  uint8_t frame_end2,
                                  uint8_t* buff)
{                                 
    pPackInfo_t -> data_size   = data_size;    
    pPackInfo_t -> frame_head1 = frame_head1; 
    pPackInfo_t -> frame_head2 = frame_head2; 
    pPackInfo_t -> frame_end1  = frame_end1;  
    pPackInfo_t -> frame_end2  = frame_end2;  
    pPackInfo_t -> pack        =  buff;       
    
    pPackInfo_t -> i = 0;
    pPackInfo_t -> count = 0;
    
}

/*
******************************************************
*
* Function name Pack_Deal_API_Bit_Copy
* Description   : Bit���ݿ���
*                 
* Parameter     ��
*        @pNew      :��������ָ��                               
*        @pOld      :ԭ����ָ��
*        @dataSize  :Bit��С

*       
* Return        : None
                
**********************************************************/


void Pack_Deal_API_Bit_Copy(uint8_t *pNew, uint8_t *pOld, uint8_t dataSize)
{
    int i;
    for(i = 0 ; i < dataSize;i++)
    {
        pNew[i] = pOld[i];
    }
}

/*
******************************************************
*
* Function name Online_State_Deal
* Description   : ����״̬����
*                 һ��ÿ�����,����
*           ��״̬, ��ǰ����, ��С����, �õ�һ����״̬
*                 
* Parameter     ��
*        @oldSta     :��״̬                               
*        @frame_num  :��ǰ����
*        @min_frame  :��С����

*       
* Return        : ��״̬ 0x00, ��δ����
*                        0x01, ���ߺ����
*                        0x80, ��һ������
*                        0x40, ���ߺ�����
*                        other, err
**********************************************************/




uint8_t Online_State_Deal(uint8_t oldSta, uint8_t frame_num, uint8_t min_frame)
{
    
    if (frame_num < min_frame)
	{
         switch (oldSta) 
             {
                 case 0x00:        
                    return 0x00;
                 
                 case 0x01:        
                    return 0x01; 
                 
                 case 0x80:              
                    return 0x01;   
                 
                 case 0x40:        
                    return 0x01;      
                 
                 default:
                    return 0x08;
             }
	}
	else
	{        
        switch (oldSta) 
             {
                 case 0x00:        
                    return 0x80;
                 
                 case 0x01:        
                    return 0x40;  
                 
                 case 0x80:              
                    return 0x80;    
                 
                 case 0x40:        
                    return 0x40;
                 
                 default:
                    return 0x08;
             }
      
       
	}    
}






/*
һ��ʾ��

typedef struct TYPE_Rx_PID_t
{
    int num;
    float x;
    float y;
    float z;
}TYPE_Rx_PID_t, *pTYPE_Rx_PID_t;

TYPE_Rx_PID_t g_PID_1;

void UART3_Config(void)
{
    ....;
}

void USART3_IRQHandler(void)
{
    static uint8_t RxData;
    static uint8_t buff[50];
    static uint8_t buff2[50];
    
    static TYPE_Pack_Info_t pid_Pack_Info = {
        16, 0x0D, 0x0A, 0x0A, 0x0D, buff};
    
    static TYPE_Pack_Info_t pid_Pack_Info2;
    Pack_Deal_API_SetPackInfo(pid_Pack_Info2, 16, 0x0D, 0x0A, 0x0A, 0x0D, buff);
    
    if(USART_GetFlagStatus(USART3, USART_IT_RXNE) != RESET)
    {
        USART_ClearITPendingBit(USART3, USART_IT_RXNE);
        
        RxData = USART_ReceiveData(USART3);
        if (Pack_Deal_API_Read_Pack(&pid_Pack_Info, RxData))
        {
            //���յ�һ�����ݰ�
            //ǿ��ת�� ��ֵ
            g_PID_1 = *((pTYPE_Rx_PID_t) buff);
            
            // ���´��������
            Pack_Deal_API_Data_Pack(&pid_Pack_Info2,(uint8_t *) &g_PID_1);
            while(pid_Pack_Info2.i < pid_Pack_Info2.count)
            {
               while(USART_GetFlagStatus(USART3, USART_FLAG_TC) != SET);
               USART_SendData(USART3, pid_Pack_Info2.pack[pid_Pack_Info2.i ++]);
            }
        }
    }
}

*/
                                      













