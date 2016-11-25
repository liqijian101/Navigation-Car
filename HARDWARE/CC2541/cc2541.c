/**
 *  @file           cc2541.h
 *  @brief          蓝牙CC2541搜索连接配对程序
 *  @version 1.0
 *  @author         李其建
 *  @date           2016年11月25日
 */



#include "cc2541.h"
#include "delay.h"
#include "USART2_DMA.h"
#include "USART1_DMA.h"
#include "led.h"

/**
 * @Description: CC2541连接检查程序
 * @note1 :若蓝牙模块不存在则陷在此程序中，
 */
void CC2541_Connected_Check(void)
{
Line1:
	USART2_DMA_Send_Once_Data("AT+INQ\r\n",8);  //搜索蓝牙设备
/*
 * 若有设备返回数据
 * OK\r\n       4
 * +INQS\r\n     7
 * +INQ:1 0x0015830066D9\r\n  23
 * +INQE\r\n    7
 * Devices Found 1\r\n  17

 * 若无设备返回数据
 * OK
 * +INQS
 * +INQE
 * Devices Found 0
 * 若已经连接上了，则发送at+inq后arduino回复	一个命令  “+Connected 9\r\r\n”	
 * 要求Arduino端蓝牙凡是收到 AT+INQ\r\n  则发送 “+Connected 9\r\r\n”
 */
	delay_ms(200);
	uart2_rx.len=0;
	USART2_DMA_Send_Once_Data("AT+INQ\r\n",8);  //搜索蓝牙设备
	delay_ms(200);
	if(uart2_rx.len>=4) 
	{	
		if(uart2_rx.buf[uart2_rx.len-4]=='1')	 //未连接，且有一个可连接设备
		{
			uart2_rx.len=0;
			USART2_DMA_Send_Once_Data("at+conn1\r\n",10);	 //未连接
			delay_ms(300);
			if(uart2_rx.len>=20 &&  uart2_rx.buf[uart2_rx.len-19]=='d')	 //连接成功
			{
				uart2_rx.len=0;
				USART1_DMA_Send_Once_Data("蓝牙已连接\r\n",12); 
			}
			else
			{
				uart2_rx.len=0;
				USART1_DMA_Send_Once_Data("蓝牙再次尝试连接\r\n",20);
				goto Line1;
			} 	
		}
		else if(uart2_rx.buf[uart2_rx.len-4]=='0')//未连接，且没有可连接设备
		{
			USART1_DMA_Send_Once_Data("请打开另一端蓝牙\r\n",18);  
			LED=!LED;
			uart2_rx.len=0;
			goto Line1;
		}										  
		else if(uart2_rx.buf[uart2_rx.len-4]=='9') 	   //已连接
		{
			USART1_DMA_Send_Once_Data("蓝牙已连接\r\n",12);  
			uart2_rx.len=0;
		}  
	}
	else//小车蓝牙模块没有连接。
	{
		USART1_DMA_Send_Once_Data("小车蓝牙模块没有连接\r\n",22);
		uart2_rx.len=0;  
		goto Line1;
	}
}

/**
 * @Description: CC2541数据发送函数，连接在uart2上
 * @param  encoder_left - 左轮编码器值		
 * @param  encoder_right- 右轮编码器值
 * @param  wzL  - z轴角速度低字节
 * @param  wzH  - z轴角速度高字节
 * @param  YawL - z轴方向角低字节
 * @param  YawH - z轴方向角高字节
 * @note1  :数据协议-0X55 0XAA  leL leH reL reH  wzL wzH  HyL HyH 0X0D
 * @note1  :注意角速度和磁场角度没有转化为实际值
 */
 void CC2541_Send_To_Arduino(short encoder_left,short encoder_right,unsigned char wzL,unsigned char wzH,unsigned char YawL,unsigned char YawH )
 {
	
	 uart2_tx.buf[2]=(unsigned char)((encoder_left& 0x00FF));
	 uart2_tx.buf[3]=(unsigned char)((encoder_left& 0xFF00)>>8);
	 
	 uart2_tx.buf[4]=(unsigned char)((encoder_right& 0x00FF));
	 uart2_tx.buf[5]=(unsigned char)((encoder_right& 0xFF00)>>8);

	 uart2_tx.buf[6]=wzL;
	 uart2_tx.buf[7]=wzH;
	 uart2_tx.buf[8]=YawL;
	 uart2_tx.buf[9]=YawH;
	 USART2_DMA_Send_Once_Data(uart2_tx.buf,11);  

 }

/**
 * @Description: CC2541数据接收处理
 * @note1  :接收数据协议-0X55 leL leH reL reH 0X0D
 */
extern int g_Target_Left	,g_Target_Right;
void CC2541_Receive_Check(void)
{
	if(uart2_rx.len==6)
	{
		if(uart2_rx.buf[0]==0X55)
		g_Target_Left=(short)(uart2_rx.buf[2]<<8 | uart2_rx.buf[1]);
		g_Target_Right=(short)(uart2_rx.buf[4]<<8 | uart2_rx.buf[3]);
		USART1_DMA_Send_Once_Data("Receive OK\r\n",12);	   
	}
	uart2_rx.len=0;
}



