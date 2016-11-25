/**  
*  @file           uart1_dma.h 
*  @brief          STM32F10x USART+DMA发送和接收，接收保存到数组USART1_RECEIVE_DATA中
*  @version 1.0     
*  @author         李其建  
*  @date           2016年11月24日 
*/ 

#ifndef __USART1_DMA_H
#define __USART1_DMA_H
#include "sys.h" 

struct uart1_buffer
{
	unsigned char  len;   //数据长度
	unsigned char  buf[64];  //接收数据缓存区
};
extern struct uart1_buffer uart1_rx,uart1_tx;

extern u8 USART1_TX_BUSY; //0：空闲 1:正在发送

void USART1_DMA_Init(u32 baud);
void USART1_DMA_Send_Once_Data(u8 *data,u16 size);
void USART1_Receive_Check(void);
#endif


