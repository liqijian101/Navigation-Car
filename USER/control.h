/**
 *  @file           control.h
 *  @brief          电机PID控制程序
 *  @version 1.0
 *  @author         李其建
 *  @date           2016年11月25日
 */


#ifndef __CONTROL_H
#define __CONTROL_H
#include "sys.h"



void Motor_All_Control(int Encoder_Left,int Target_Left,int Encoder_Right,int Target_Right);


u8 Turn_Off(float angle, int voltage);
#endif
