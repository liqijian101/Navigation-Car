/**
 *  @file           encoder.h
 *  @brief          STM32F10x 编码器测速程序
 *  @version 1.0
 *  @author         李其建
 *  @date           2016年11月24日
 */
 

#ifndef __ENCODER_H
#define __ENCODER_H	
#include "sys.h" 
							   
void Encoder_Left_Init(void);
void Encoder_Right_Init(void);
int Read_Encoder(u8 TIMX);
#endif 















