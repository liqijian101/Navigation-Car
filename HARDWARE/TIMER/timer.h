/**
 *  @file           timer.h
 *  @brief          STM32F10x 定时中断程序
 *  @version 1.0
 *  @author         李其建
 *  @date           2016年11月25日
 */


#ifndef __TIMER_H
#define __TIMER_H
#include "sys.h"
/**
 * *FLAG_TIM3定时器3中断状态
 *  0-main中未在处理数据
 *  1-定时中断发生，main可以处理数据
 * -1-main处理数据超时
 */
extern s8 FLAG_TIM3;
void TIM3_Int_Init(u16 Period,u16 Prescaler);
 
#endif
