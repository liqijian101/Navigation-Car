/**
 *  @file           timer.h
 *  @brief          STM32F10x 定时中断程序
 *  @version 1.0
 *  @author         李其建
 *  @date           2016年11月25日
 */

#include "timer.h"

/**
 * @Description: 通用定时器TIM3定时中断初始化
 * @param  Period    - 自动重装值
 * @param  Prescaler - 时钟预分频数
 * @note1 :中断时间(s)=(Period/Prescaler)/时钟(72M)
 * @note1 :这里时钟选择为APB1的2倍，而APB1为36M,详见时钟树图
 */
void TIM3_Int_Init(u16 Period,u16 Prescaler)
{
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE); 		//时钟使能

	TIM_TimeBaseStructure.TIM_Period = Period-1; 				//设置在下一个更新事件装入活动的自动重装载寄存器周期的值
	TIM_TimeBaseStructure.TIM_Prescaler =Prescaler-1; 			//设置用来作为TIMx时钟频率除数的预分频值
	TIM_TimeBaseStructure.TIM_ClockDivision = 0; 				//设置时钟分割:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; //TIM向上计数模式
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure); 			
 
	TIM_ITConfig(TIM3,TIM_IT_Update,ENABLE);   					//使能或者失能指定的TIM中断
	NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;  			//TIM3中断
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;  	//先占优先级0级
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;  		//从优先级3级
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; 			//IRQ通道被使能
	NVIC_Init(&NVIC_InitStructure);  							//根据NVIC_InitStruct中指定的参数初始化外设NVIC寄存器

	TIM_Cmd(TIM3, ENABLE);  									//使能TIMx外设							 
}


s8 	FLAG_TIM3=0;
static char	TIM3_Count=0;	   
/**
 * @Description: 通用定时器TIM3中断函数
 */
void TIM3_IRQHandler(void) 
{
	if (TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET) 			//检查指定的TIM中断发生与否:TIM 中断源 
	{
		TIM_ClearITPendingBit(TIM3, TIM_IT_Update);				//清除TIMx的中断待处理位:TIM 中断源 
		if(FLAG_TIM3==0) 
		{
			FLAG_TIM3=1;
		} else if(FLAG_TIM3==1)  
		{
			TIM3_Count++;
			if(TIM3_Count>20) 
			{
				TIM3_Count=0;
				FLAG_TIM3=-1;
			}
		}else
		{}
	}
}












