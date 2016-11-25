/**  
*  @file           uart3_dma.h 
*  @brief          STM32F10x USART+DMA发送和接收，接收保存到数组USART3_RECEIVE_DATA中
*  @version 1.0     
*  @author         李其建  
*  @date           2016年11月24日 
*/ 

#ifndef __USART3_DMA_H
#define __USART3_DMA_H
#include "sys.h" 

struct JY901_buffer
{
	unsigned char  len;   //数据长度
	unsigned char  buf[64];  //接收数据缓存区
};
extern struct JY901_buffer JY901_rx;

extern u8 USART3_TX_BUSY; //0：空闲 1:正在发送

void USART3_DMA_Init(u32 baud);
void USART3_DMA_Send_Once_Data(u8 *data,u16 size);
void USART3_Receive_Check(void);
#endif



