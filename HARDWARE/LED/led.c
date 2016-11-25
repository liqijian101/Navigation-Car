/**
 *  @file           led.h
 *  @brief          STM32F10x LED灯
 *  @version 1.0
 *  @author         李其建
 *  @date           2016年11月24日
 */

#include "led.h"
/**
 * @Description: LED IO初始化  PC13输出
 */
void LED_Init(void)
{
 
 GPIO_InitTypeDef  GPIO_InitStructure;
 	
 RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);	 //使能PC端口时钟
	
 GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;				 //LED端口配置
 GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		 //推挽输出
 GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		 //IO口速度为50MHz
 GPIO_Init(GPIOC, &GPIO_InitStructure);					 //根据设定参数初始化GPIOC
 GPIO_SetBits(GPIOC,GPIO_Pin_13);						 //端口输出高
}
 
