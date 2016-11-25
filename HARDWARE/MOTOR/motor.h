/**
 *  @file           motor.h
 *  @brief          STM32F10x 双路直流有刷电机PWM驱动程序
 *  @version 1.0
 *  @author         李其建
 *  @date           2016年11月25日
 */

#ifndef __MOTOR_H
#define __MOTOR_H
#include "sys.h"

#define PWMA   TIM1->CCR3  //PB15
#define AIN2   PAout(12)
#define AIN1   PAout(11) 
#define STBY   PAout(8)
#define BIN1   PBout(14)
#define BIN2   PBout(12)
#define PWMB   TIM1->CCR1  //PB13

void Motor_PWM_Init(u16 Period,u16 Prescaler);

#endif
