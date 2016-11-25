/**
 *  @file           led.h
 *  @brief          STM32F10x LED灯
 *  @version 1.0
 *  @author         李其建
 *  @date           2016年11月24日
 */
#ifndef __LED_H
#define __LED_H	 
#include "sys.h"
/*public*/
#define LED PCout(13)

void LED_Init(void);//初始化

		 				    
#endif
