/**
 *  @file           control.h
 *  @brief          ���PID���Ƴ���
 *  @version 1.0
 *  @author         ���佨
 *  @date           2016��11��25��
 */


#ifndef __CONTROL_H
#define __CONTROL_H
#include "sys.h"



void Motor_All_Control(int Encoder_Left,int Target_Left,int Encoder_Right,int Target_Right);


u8 Turn_Off(float angle, int voltage);
#endif
