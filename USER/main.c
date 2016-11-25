#include "led.h"
#include "delay.h"
#include "sys.h"
#include "usart1_dma.h"	 
#include "usart2_dma.h" 
#include "usart3_dma.h" 
#include "encoder.h" 
#include "motor.h"
#include "timer.h"
#include "control.h"
#include "cc2541.h"


int g_Encoder_Left,g_Encoder_Right;             //左右编码器的脉冲计数

int Voltage;                                //电池电压采样相关的变量

int g_Target_Left,g_Target_Right;		 //注意修改usart2.c中的extern
unsigned char  wzL,wzH,HyL,HyH,YawL,YawH;

void Wifi_Send_To_Phone(int encoder1,int encoder2);

void JY901_Receive_Check(void);
 int main(void)
 { 
    u8 count=0;
	uart2_tx.buf[0]=0x55;
	uart2_tx.buf[1]=0xAA;
	uart2_tx.buf[10]=0x0D;

	delay_init();	    		//延时函数初始化
	NVIC_Configuration();
	USART1_DMA_Init(115200);    //PA9:TX  PA10:RX	接WIFI模块	 与手机相连发送状态数据 
	USART2_DMA_Init(115200);	//PA2:TX  PA3 :RX	接蓝牙模块   与Arduino相连发送姿态数据
	USART3_DMA_Init(115200);	//PB11:RX           接九轴模块
	
	LED_Init();		  			//初始化与LED GPIO
	CC2541_Connected_Check();
	Encoder_Left_Init();		//编码器初始化
	Encoder_Right_Init();		//编码器初始化
	Motor_PWM_Init(7200,1);   	//初始化PWM 10KHZ
	STBY=1;						//使能电机控制
	TIM3_Int_Init(200,7200);	//10Khz的计数频率，计数到200为20ms    
	while(1)
	{
	
/*
 * 轮子直径D=65mm=0.065m	  采样周期T=20ms=0.02s
 * 冲数(X)转化为轮子行进速度(v)       最大行驶速度1.24m/s
 * 个/T   -->  m/s
 * v=X*pi*D/1560/T   -->v=0.006545*X
 *                  -->X=152.7887*v
 */
	   JY901_Receive_Check();
	   
	   if(FLAG_TIM3==1)
	   {
	   	CC2541_Receive_Check();	
		g_Encoder_Left=Read_Encoder(2);            								//读取编码器的值,注意是否需要取反
		g_Encoder_Right=Read_Encoder(4);           								//读取编码器的值,注意是否需要取反
		CC2541_Send_To_Arduino(g_Encoder_Left,g_Encoder_Right,wzL,wzH,YawL,YawH );	//先发送，以便利用DMA发送节约时间的优势
		//Voltage=Get_battery_volt();            								//获取电池电压	          
			
	
		Motor_All_Control(g_Encoder_Left,g_Target_Left,g_Encoder_Right,g_Target_Right);
	//	 Wifi_Send_To_Phone(Encoder_Left,Encoder_Right);
		count++;
		if(count==50){count=0;LED=!LED;}   //1s钟
		FLAG_TIM3=0;
	  }	  
	}
}





	


/**
 * @Description: wifi发送到手机APP的数据，连接在uart1上
 * @param  encoder_left - 左轮编码器值		
 * @param  encoder_right- 右轮编码器值
 * @note1  :数据协议
 */		
void Wifi_Send_To_Phone( int encoder_left, int encoder_right )
{
	if ( encoder_left < 0 )
	{
		encoder_left = -encoder_left; 
		uart1_tx.buf[0] = '-';
	}else uart1_tx.buf[0] = ' ';

	uart1_tx.buf[3] = (unsigned char) (encoder_left % 10) + 48;
	uart1_tx.buf[2] = (unsigned char) (encoder_left / 10 % 10) + 48;
	uart1_tx.buf[1] = (unsigned char) (encoder_left / 100 % 10) + 48;
	uart1_tx.buf[4] = '\r';

	if ( encoder_right < 0 )
	{
		encoder_right = -encoder_right; 
		uart1_tx.buf[5] = '-';
	}else uart1_tx.buf[5] = ' ';

	uart1_tx.buf[8] = (unsigned char) (encoder_right % 10) + 48;
	uart1_tx.buf[7] = (unsigned char) (encoder_right / 10 % 10) + 48;
	uart1_tx.buf[6] = (unsigned char) (encoder_right / 100 % 10) + 48;
	uart1_tx.buf[9] = '\n';

	USART1_DMA_Send_Once_Data( uart1_tx.buf, 10 );
}


/**
 * @Description: JY901(九轴)数据接收处理
 * @note1 - 角速度输出格式：0X55 0X52 wxL wxH wyL wyH wzL wzH TL TH SUM
 * @note2 - 角  度输出格式：0X55 0X53 RoL RoH PiL PiH YaL YaH TL TH SUM
 * @note3 - 地磁场输出格式：0X55 0X54 HxL HxH HyL HyH HzL HzH TL TH SUM
 */
void JY901_Receive_Check(void)
{
	if(JY901_rx.len==22)
	{
		if (JY901_rx.buf[0]==0x55&&JY901_rx.buf[1]==0x52) //数据头不对，则重新开始寻找0x55数据头
		{
			wzL=JY901_rx.buf[6];
			wzH=JY901_rx.buf[7];
		}
/*
 *      if (JY901_rx.buf[11]==0x55&&JY901_rx.buf[12]==0x54) //数据头不对，则重新开始寻找0x55数据头
 *      {
 *              HyL=JY901_rx.buf[15];
 *              HyH=JY901_rx.buf[16];
 *      }
 */
		if (JY901_rx.buf[11]==0x55&&JY901_rx.buf[12]==0x53) //数据头不对，则重新开始寻找0x55数据头
		{
			YawL=JY901_rx.buf[17];
			YawH=JY901_rx.buf[18];
		}
		JY901_rx.len=0;	
	}	
}


