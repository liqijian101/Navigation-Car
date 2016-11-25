/**  
*  @file           uart2_dma.h 
*  @brief          STM32F10x USART+DMA发送和接收，接收保存到数组USART2_RECEIVE_DATA中
*  @version 1.0     
*  @author         李其建  
*  @date           2016年11月24日 
*/ 

#ifndef __USART2_DMA_H
#define __USART2_DMA_H	
#include "sys.h" 

struct uart2_buffer
{
	unsigned char  len;   //数据长度
	unsigned char  buf[64];  //接收数据缓存区
};
extern struct uart2_buffer uart2_rx,uart2_tx;

extern u8 USART2_TX_BUSY; //0：空闲 1:正在发送

void USART2_DMA_Init(u32 baud);
void USART2_DMA_Send_Once_Data(u8 *data,u16 size);
void USART2_Receive_Check(void);
#endif
