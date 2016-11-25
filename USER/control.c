/**
 *  @file           control.h
 *  @brief          电机PID控制程序
 *  @version 1.0
 *  @author         李其建
 *  @date           2016年11月25日
 */


#include "control.h"	
#include "motor.h"

float Kp=30.0f;
float Ki=10.0f;
/*private*/
static int int_abs( int a );
static int Motor_Left_Control(int Encoder,int Target);
static int Motor_Right_Control(int Encoder,int Target);
static void Set_Pwm(int moto1,int moto2);
static void Xianfu_Pwm(int moto1,int moto2);

/**
 * @Description: 左右轮电机控制
 * @param  Encoder_Left  - 左轮当前速度
 * @param  Target_Left   - 左轮目标速度
 * @param  Encoder_Right - 右轮当前速度
 * @param  Target_Right  - 右轮目标速度
 */

void Motor_All_Control(int Encoder_Left,int Target_Left,int Encoder_Right,int Target_Right)
{
	static int Motor1,Motor2;                           	 //电机PWM变量 	
	Motor1=Motor_Left_Control(Encoder_Left,Target_Left);   	 //速度环PID控制	 记住，速度反馈是正反馈，就是小车快的时候要慢下来就需要再跑快一点
  	Motor2=Motor_Right_Control(Encoder_Right,Target_Right); 
   	Xianfu_Pwm(Motor1,Motor2);                           	 //PWM限幅
 	Set_Pwm(Motor1,Motor2);                                  //赋值给PWM寄存器  
}
/**
 * @Description: 左轮增量PI控制器
 * @param  Encoder - 当前速度
 * @param  Target  - 目标速度
 * @return Left_Pwm -PID计算PWM
 */
int Motor_Left_Control(int Encoder,int Target)
{ 	
	 static int Left_Bias,Left_Pwm,Left_Last_bias;
	 Left_Bias=Encoder-Target;                				 //计算偏差
	 Left_Pwm+=Kp*(Left_Bias-Left_Last_bias)+Ki*Left_Bias;   //增量式PI控制器
	 Left_Last_bias=Left_Bias;	                			 //保存上一次偏差 
	 return Left_Pwm;                        				 //增量输出
}
/**
 * @Description: 右轮增量PI控制器
 * @param  Encoder - 当前速度
 * @param  Target  - 目标速度
 * @return Left_Pwm -PID计算PWM
 */
int Motor_Right_Control(int Encoder,int Target)
{ 		
	 static int Right_Bias,Right_Pwm,Right_Last_bias;
	 Right_Bias=Encoder-Target;                //计算偏差
	 Right_Pwm+=Kp*(Right_Bias-Right_Last_bias)+Ki*Right_Bias;   //增量式PI控制器
	 Right_Last_bias=Right_Bias;	                   //保存上一次偏差 
	 return Right_Pwm;                         //增量输出
}

/**
 * @Description: 赋值给PWM寄存器
 * @param  moto1 - 左轮PWM
 * @param  moto2 - 右轮PWM
 */
void Set_Pwm(int moto1,int moto2)
{
	if(moto1<0)		AIN1=0,	AIN2=1;
	else 	        AIN1=1, AIN2=0;	 //前进 
	PWMA=int_abs(moto1);
	if(moto2<0)		BIN1=1,	BIN2=0;
	else        	BIN1=0;	BIN2=1;	//前进	 
	PWMB=int_abs(moto2);	
}

/**
 * @Description: 限制PWM幅值
 * @param  Moto1 - 电机1
 * @param  Moto2 - 电机2
 * @param  Amplitude - PWM满幅是7200 限制在7100 
 */
const int Amplitude=7100;
void Xianfu_Pwm( int Moto1, int Moto2 )
{
	if ( Moto1 < -Amplitude )
		Moto1 = -Amplitude;
	else if ( Moto1 > Amplitude )
		Moto1 = Amplitude;
	if ( Moto2 < -Amplitude )
		Moto2 = -Amplitude;
	else if ( Moto2 > Amplitude )
		Moto2 = Amplitude;
}

/**************************************************************************
函数功能：异常关闭电机
入口参数：倾角和电压
返回  值：1：异常  0：正常
**************************************************************************/
//u8 Turn_Off(float angle, int voltage)
//{
//	    u8 temp;
//			if(angle<-40||angle>40||1==Flag_Stop||voltage<1110)//电池电压低于11.1V关闭电机
//			{	                                                 //===倾角大于40度关闭电机
//      temp=1;                                            //===Flag_Stop置1关闭电机
//			AIN1=0;                                            
//			AIN2=0;
//			BIN1=0;
//			BIN2=0;
//      }
//			else
//      temp=0;
//      return temp;			
//}
	

/**
 * @Description: 求整型的绝对值
 * @param  a    - 待求绝对值的数
 * @return temp - 绝对值结果
 */
static int int_abs( int a )
{
	int temp;
	if ( a < 0 )
		temp = -a;
	else temp = a;
	return(temp);
}


