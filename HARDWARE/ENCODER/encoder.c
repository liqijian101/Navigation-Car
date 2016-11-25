/**
 *  @file           encoder.h
 *  @brief          STM32F10x 编码器测速程序
 *  @version 1.0
 *  @author         李其建
 *  @date           2016年11月24日
 */


#include "encoder.h" 

/* ENCODER GPIO define. */
#define ENCODER_L_GPIO_A        GPIO_Pin_0
#define ENCODER_L_GPIO_B        GPIO_Pin_1
#define ENCODER_L_GPIO          GPIOA
#define ENCODER_L_GPIO_RCC      RCC_APB2Periph_GPIOA
#define ENCODER_L_TIM_RCC		RCC_APB1Periph_TIM2		//注意与时钟函数
#define ENCODER_L_TIMER		    TIM2

#define ENCODER_R_GPIO_A        GPIO_Pin_6
#define ENCODER_R_GPIO_B        GPIO_Pin_7
#define ENCODER_R_GPIO          GPIOB
#define ENCODER_R_GPIO_RCC      RCC_APB2Periph_GPIOB
#define ENCODER_R_TIM_RCC 		RCC_APB1Periph_TIM4		//注意与时钟函数
#define ENCODER_R_TIMER		    TIM4

/**
 * @Description:编码器初始化
 */ 
void Encoder_Left_Init(void)
{
    GPIO_InitTypeDef         GPIO_InitStructure; 
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
//	TIM_ICInitTypeDef        TIM_ICInitStructure;
	
	RCC_APB1PeriphClockCmd(ENCODER_L_TIM_RCC,ENABLE);//开启TIM时钟
 	RCC_APB2PeriphClockCmd(ENCODER_L_GPIO_RCC,ENABLE);//开启GPIO时钟
	
	
	GPIO_InitStructure.GPIO_Pin = ENCODER_L_GPIO_A|ENCODER_L_GPIO_B;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IN_FLOATING;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(ENCODER_L_GPIO,&GPIO_InitStructure); 

	
    TIM_TimeBaseStructure.TIM_Period = 65535; //设置下一个更新事件装入活动的自动重装载寄存器周期的值 这里必须设置成65535   这样正转为正  反转为负数
	TIM_TimeBaseStructure.TIM_Prescaler = 0; //设置用来作为TIMx时钟频率除数的预分频值  不分频
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; //设置时钟分割:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM向上计数模式
	TIM_TimeBaseInit(ENCODER_L_TIMER, &TIM_TimeBaseStructure); 
	
	//设置定时器为编码器模式   IT1  IT2为上升沿和下降沿都计数	 四倍频
	TIM_EncoderInterfaceConfig(ENCODER_L_TIMER, TIM_EncoderMode_TI12,TIM_ICPolarity_BothEdge,TIM_ICPolarity_BothEdge);

//	TIM_ICStructInit(&TIM_ICInitStructure);
//  TIM_ICInitStructure.TIM_ICFilter = 10;  			 //输入滤波器
//	TIM_ICInit(ENCODER_L_TIMER, &TIM_ICInitStructure);
//  TIM_ClearFlag(ENCODER_L_TIMER, TIM_FLAG_Update);  	 //清楚所有标志位
//  TIM_ITConfig(ENCODER_L_TIMER, TIM_IT_Update, DISABLE); //禁止中断更新
	TIM_Cmd(ENCODER_L_TIMER, ENABLE);  //使能TIM
}

/**
 * @Description:编码器初始化
 */
void Encoder_Right_Init(void)
{
    GPIO_InitTypeDef         GPIO_InitStructure; 
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
//	TIM_ICInitTypeDef        TIM_ICInitStructure;
	
	RCC_APB1PeriphClockCmd(ENCODER_R_TIM_RCC,ENABLE);//开启TIM时钟
 	RCC_APB2PeriphClockCmd(ENCODER_R_GPIO_RCC,ENABLE);//开启GPIO时钟
	
	
	GPIO_InitStructure.GPIO_Pin = ENCODER_R_GPIO_A|ENCODER_R_GPIO_B;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IN_FLOATING;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(ENCODER_R_GPIO,&GPIO_InitStructure); 

	
    TIM_TimeBaseStructure.TIM_Period = 65535; //设置下一个更新事件装入活动的自动重装载寄存器周期的值 这里必须设置成65535   这样正转为正  反转为负数
	TIM_TimeBaseStructure.TIM_Prescaler = 0; //设置用来作为TIMx时钟频率除数的预分频值  不分频
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; //设置时钟分割:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM向上计数模式
	TIM_TimeBaseInit(ENCODER_R_TIMER, &TIM_TimeBaseStructure); 
	
	//设置定时器为编码器模式   IT1  IT2为上升沿和下降沿都计数	 四倍频
	TIM_EncoderInterfaceConfig(ENCODER_R_TIMER, TIM_EncoderMode_TI12,TIM_ICPolarity_BothEdge,TIM_ICPolarity_BothEdge);

//	TIM_ICStructInit(&TIM_ICInitStructure);
//  TIM_ICInitStructure.TIM_ICFilter = 10;  			 //输入滤波器
//	TIM_ICInit(ENCODER_R_TIMER, &TIM_ICInitStructure);
//  TIM_ClearFlag(ENCODER_R_TIMER, TIM_FLAG_Update);  	 //清楚所有标志位
//  TIM_ITConfig(ENCODER_R_TIMER, TIM_IT_Update, DISABLE); //禁止中断更新
	TIM_Cmd(ENCODER_R_TIMER, ENABLE);  //使能TIM
}

/**
 * @Description: 读取指定的编码器
 * @param  TIMX        - 定时器
 * @return Encoder_TIM - 编码器数
 */ 
int Read_Encoder(u8 TIMX) 
{
	static int Encoder_TIM;
	switch(TIMX) 
	{
		case 2: Encoder_TIM= (short)TIM2 -> CNT;
				TIM2 -> CNT=0;
		break;
		case 4: Encoder_TIM= (short)TIM4 -> CNT;
				TIM4 -> CNT=0;
		break;
		default:  Encoder_TIM=0;
	}
	return Encoder_TIM;
}		



