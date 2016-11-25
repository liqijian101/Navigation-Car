/**  
*  @file           USART3_dma.h 
*  @brief          STM32F10x USART+DMA发送和接收，接收保存到数组USART3_RECEIVE_DATA中
*  @version 1.0     
*  @author         李其建  
*  @date           2016年11月24日 
*/ 

#include "usart3_dma.h"
#include "sys.h"
#include <string.h>
/* UART GPIO define. */
#define USART3_GPIO_TX       GPIO_Pin_10
#define USART3_GPIO_RX       GPIO_Pin_11
#define USART3_GPIO          GPIOB
#define USART3_GPIO_RCC      RCC_APB2Periph_GPIOB
#define RCC_APBPeriph_USART3 RCC_APB1Periph_USART3
#define USART3_TX_DMA        DMA1_Channel2
#define USART3_RX_DMA        DMA1_Channel3
#define USART3_TX_DMA_IRQn	 DMA1_Channel2_IRQn
#define USART3_DMA_RCC       RCC_AHBPeriph_DMA1



#define USART3_DATA_LEN  64  //接收和发送数据的最大长度
/*private*/
u8 USART3_SEND_DATA[USART3_DATA_LEN];  
u8 USART3_RECEIVE_DATA[USART3_DATA_LEN]; 
u8 USART3_TX_BUSY=0; //0：空闲 1:正在发送
/*public*/
struct JY901_buffer JY901_rx;
	  

/**
* @Description: UART1_DMA初始化
* @param baud - 串口波特率.    
*/ 
void USART3_DMA_Init(u32 baud)  
{
	//定义中断结构体  
    NVIC_InitTypeDef NVIC_InitStructure ;  
    //定义IO初始化结构体  
    GPIO_InitTypeDef GPIO_InitStructure;  
    //定义串口结构体    
    USART_InitTypeDef USART_InitStructure;  
    //定义DMA结构体  
    DMA_InitTypeDef DMA_InitStructure;  

/*串口IO设置*/
    //第2步：打开GPIO和USART3部件的时钟
	RCC_APB2PeriphClockCmd(USART3_GPIO_RCC | RCC_APB2Periph_AFIO, ENABLE);	
	//第2步：将USART3 Tx的GPIO配置为推挽复用模式 */
	GPIO_InitStructure.GPIO_Pin = USART3_GPIO_TX;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(USART3_GPIO, &GPIO_InitStructure);

	//第3步：将USART3 Rx的GPIO配置为浮空输入模式
	//由于CPU复位后，GPIO缺省都是浮空输入模式，因此下面这个步骤不是必须的
	//但是，我还是建议加上便于阅读，并且防止其它地方修改了这个口线的设置参数
	GPIO_InitStructure.GPIO_Pin = USART3_GPIO_RX;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;;
	GPIO_Init(USART3_GPIO, &GPIO_InitStructure);


/*基本串口参数设置*/
 	//打开串口对应的外设时钟    
    RCC_APB2PeriphClockCmd(RCC_APBPeriph_USART3, ENABLE);  

	USART_InitStructure.USART_BaudRate = baud;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//开校验时,校验位算在数据位中,因此9个数据位
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
  	USART_Init(USART3, &USART_InitStructure);
	
	USART_Cmd(USART3, ENABLE);
	//CPU的小缺陷：串口配置好，如果直接Send，则第1个字节发送不出去
  	//如下语句解决第1个字节无法正确发送出去的问题 
  	USART_ClearFlag(USART3, USART_FLAG_TC);     //清发送完成标志，Transmission Complete flag 
/*串口中断配置*/
  	NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
  	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
  	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
  	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  	NVIC_Init(&NVIC_InitStructure);
	
	USART_ITConfig(USART3,USART_IT_TC,DISABLE);  
    USART_ITConfig(USART3,USART_IT_RXNE,DISABLE);  
    USART_ITConfig(USART3,USART_IT_TXE,DISABLE); 
	USART_ITConfig(USART3,USART_IT_IDLE,ENABLE);//关闭空闲中断 

  	//Enable USART3 DMA Rx Tx request 
  	USART_DMACmd(USART3, USART_DMAReq_Rx, ENABLE); 
	USART_DMACmd(USART3, USART_DMAReq_Tx, ENABLE);



/*串口发送DMA配置*/    
    //启动DMA时钟 
	RCC_AHBPeriphClockCmd(USART3_DMA_RCC, ENABLE);
	//DMA通道配置
  	DMA_DeInit(USART3_TX_DMA);
	   
  	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)(&USART3->DR);	 	//外设地址
  	DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)USART3_SEND_DATA;		//内存地址																	
  	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;						//DMA传输方向												
  	DMA_InitStructure.DMA_BufferSize = USART3_DATA_LEN;						//设置DMA在传输时缓冲区的长度
  	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;		//设置DMA的外设递增模式，一个外设 	
  	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;					//设置DMA的内存递增模式 	
  	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte; //外设数据字长	 
  	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;			//内存数据字长	 
  	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;							//设置DMA的传输模式
  	DMA_InitStructure.DMA_Priority = DMA_Priority_High;						//设置DMA的优先级别
  	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
  	DMA_Init(USART3_TX_DMA, &DMA_InitStructure);							//配置DMA的通道
  
  	DMA_ITConfig(USART3_TX_DMA, DMA_IT_TC, ENABLE);						    //使能传输完成中断 

    DMA_Cmd(USART3_TX_DMA, DISABLE);
	//DMA发送中断设置 
 	NVIC_InitStructure.NVIC_IRQChannel = USART3_TX_DMA_IRQn;
 	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
  	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
  	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  	NVIC_Init(&NVIC_InitStructure);
   
   
/*串口接收DMA配置*/    
    //启动DMA时钟 
	RCC_AHBPeriphClockCmd(USART3_DMA_RCC, ENABLE);
	//DMA通道配置 
  	DMA_DeInit(USART3_RX_DMA);  
  	DMA_InitStructure.DMA_PeripheralBaseAddr =(uint32_t)(&USART3->DR);		//外设地址
  	DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)USART3_RECEIVE_DATA;	//内存地址
  	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;						//DMA传输方向
  	DMA_InitStructure.DMA_BufferSize =USART3_DATA_LEN;						//设置DMA在传输时缓冲区的长度
  	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;		//设置DMA的外设递增模式，一个外设  
  	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;					//设置DMA的内存递增模式 
  	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;	//外设数据字长
  	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;		    //内存数据字长
  	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;							//设置DMA的传输模式
  	DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;					//设置DMA的优先级别
  	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
  	DMA_Init(USART3_RX_DMA, &DMA_InitStructure);							//配置DMA的通道   
  
  	DMA_Cmd(USART3_RX_DMA, ENABLE);
}
/**
* @Description:UART1_DMA向外发送数据
* @param data - 发送数据存放地址
* @param size - 发送数据字节数    
*/  
void USART3_DMA_Send_Once_Data(u8 *data,u16 size) 
{
	//等待空闲  
	while (USART3_TX_BUSY);
	USART3_TX_BUSY = 1;
	//复制数据  
	memcpy(USART3_SEND_DATA,data,size);
	//改变datasize前先要禁止通道工作
	DMA_Cmd(USART3_TX_DMA, DISABLE);
	//DMA1,传输数据量
	USART3_TX_DMA->CNDTR=size;
	DMA_Cmd(USART3_TX_DMA, ENABLE);
}
 
/**
* @Description:串口处理接收完成中断 
* @param  buf - 接收的数据 
* @return len - 接收的数据长度   
*/   
static u8 USART3_RX_Finish_IRQ(u8 *buf) 
{
	u16 len = 0;
	
	//清USART_IT_IDLE标志
	USART3->SR;
	USART3->DR;
	//关闭DMA,防止处理其间有数据	  
	DMA_Cmd(USART3_RX_DMA, DISABLE);
	//清除标志位 
	DMA_ClearITPendingBit(DMA1_IT_TC5);	
	//获得接收帧帧长 
	len=USART3_DATA_LEN-DMA_GetCurrDataCounter(USART3_RX_DMA);
	memcpy(buf,USART3_RECEIVE_DATA,len);
	//设置传输数据长度  	
	DMA_SetCurrDataCounter(USART3_RX_DMA,USART3_DATA_LEN);
	//打开DMA,处理完,重开DMA  
	DMA_Cmd(USART3_RX_DMA, ENABLE);
	
	return len;
}
 
/**
* @Description: 串口中断处理函数   单独使用本文件是用他，现在把他放到stm32f4xx_it.c中
*/ 
void USART3_IRQHandler(void)
{
	//发送完成中断处理  
	if(USART_GetITStatus(USART3, USART_IT_TC) != RESET) 
	{
		//关闭发送完成中断  
		USART_ITConfig(USART3,USART_IT_TC,DISABLE);
		//发送完成  
		USART3_TX_BUSY = 0;
	}
	//接收完成中断处理 
	if(USART_GetITStatus(USART3, USART_IT_IDLE) != RESET) 
	{
		JY901_rx.len = USART3_RX_Finish_IRQ(JY901_rx.buf);
	}
}

/*
* @Description: USART使用DMA发数据中断服务程序 
*/
void USART3_TX_DMA_IRQHandler(void) 
{
	if(DMA_GetITStatus(DMA1_IT_TC4) != RESET) 
	{
		//清除标志位  
		DMA_ClearITPendingBit(DMA1_IT_TC4);
		//关闭DMA  
		DMA_Cmd(USART3_TX_DMA, DISABLE);
		//打开发送完成中断,发送最后两个字节  
		USART_ITConfig(USART3,USART_IT_TC,ENABLE);
	}
}



/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/






