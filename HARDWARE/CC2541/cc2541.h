/**
 *  @file           cc2541.h
 *  @brief          蓝牙CC2541搜索连接配对程序
 *  @version 1.0
 *  @author         李其建
 *  @date           2016年11月25日
 */


#ifndef __CC2541_H
#define __CC2541_H	 

void CC2541_Connected_Check(void);
void CC2541_Receive_Check(void);
void CC2541_Send_To_Arduino(short encoder_left,short encoder_right,unsigned char wzL,unsigned char wzH,unsigned char YawL,unsigned char YawH );	 				    
#endif
