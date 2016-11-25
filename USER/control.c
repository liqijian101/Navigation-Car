/**
 *  @file           control.h
 *  @brief          ���PID���Ƴ���
 *  @version 1.0
 *  @author         ���佨
 *  @date           2016��11��25��
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
 * @Description: �����ֵ������
 * @param  Encoder_Left  - ���ֵ�ǰ�ٶ�
 * @param  Target_Left   - ����Ŀ���ٶ�
 * @param  Encoder_Right - ���ֵ�ǰ�ٶ�
 * @param  Target_Right  - ����Ŀ���ٶ�
 */

void Motor_All_Control(int Encoder_Left,int Target_Left,int Encoder_Right,int Target_Right)
{
	static int Motor1,Motor2;                           	 //���PWM���� 	
	Motor1=Motor_Left_Control(Encoder_Left,Target_Left);   	 //�ٶȻ�PID����	 ��ס���ٶȷ�����������������С�����ʱ��Ҫ����������Ҫ���ܿ�һ��
  	Motor2=Motor_Right_Control(Encoder_Right,Target_Right); 
   	Xianfu_Pwm(Motor1,Motor2);                           	 //PWM�޷�
 	Set_Pwm(Motor1,Motor2);                                  //��ֵ��PWM�Ĵ���  
}
/**
 * @Description: ��������PI������
 * @param  Encoder - ��ǰ�ٶ�
 * @param  Target  - Ŀ���ٶ�
 * @return Left_Pwm -PID����PWM
 */
int Motor_Left_Control(int Encoder,int Target)
{ 	
	 static int Left_Bias,Left_Pwm,Left_Last_bias;
	 Left_Bias=Encoder-Target;                				 //����ƫ��
	 Left_Pwm+=Kp*(Left_Bias-Left_Last_bias)+Ki*Left_Bias;   //����ʽPI������
	 Left_Last_bias=Left_Bias;	                			 //������һ��ƫ�� 
	 return Left_Pwm;                        				 //�������
}
/**
 * @Description: ��������PI������
 * @param  Encoder - ��ǰ�ٶ�
 * @param  Target  - Ŀ���ٶ�
 * @return Left_Pwm -PID����PWM
 */
int Motor_Right_Control(int Encoder,int Target)
{ 		
	 static int Right_Bias,Right_Pwm,Right_Last_bias;
	 Right_Bias=Encoder-Target;                //����ƫ��
	 Right_Pwm+=Kp*(Right_Bias-Right_Last_bias)+Ki*Right_Bias;   //����ʽPI������
	 Right_Last_bias=Right_Bias;	                   //������һ��ƫ�� 
	 return Right_Pwm;                         //�������
}

/**
 * @Description: ��ֵ��PWM�Ĵ���
 * @param  moto1 - ����PWM
 * @param  moto2 - ����PWM
 */
void Set_Pwm(int moto1,int moto2)
{
	if(moto1<0)		AIN1=0,	AIN2=1;
	else 	        AIN1=1, AIN2=0;	 //ǰ�� 
	PWMA=int_abs(moto1);
	if(moto2<0)		BIN1=1,	BIN2=0;
	else        	BIN1=0;	BIN2=1;	//ǰ��	 
	PWMB=int_abs(moto2);	
}

/**
 * @Description: ����PWM��ֵ
 * @param  Moto1 - ���1
 * @param  Moto2 - ���2
 * @param  Amplitude - PWM������7200 ������7100 
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
�������ܣ��쳣�رյ��
��ڲ�������Ǻ͵�ѹ
����  ֵ��1���쳣  0������
**************************************************************************/
//u8 Turn_Off(float angle, int voltage)
//{
//	    u8 temp;
//			if(angle<-40||angle>40||1==Flag_Stop||voltage<1110)//��ص�ѹ����11.1V�رյ��
//			{	                                                 //===��Ǵ���40�ȹرյ��
//      temp=1;                                            //===Flag_Stop��1�رյ��
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
 * @Description: �����͵ľ���ֵ
 * @param  a    - �������ֵ����
 * @return temp - ����ֵ���
 */
static int int_abs( int a )
{
	int temp;
	if ( a < 0 )
		temp = -a;
	else temp = a;
	return(temp);
}


