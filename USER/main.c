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


int g_Encoder_Left,g_Encoder_Right;             //���ұ��������������

int Voltage;                                //��ص�ѹ������صı���

int g_Target_Left,g_Target_Right;		 //ע���޸�usart2.c�е�extern
unsigned char  wzL,wzH,HyL,HyH,YawL,YawH;

void Wifi_Send_To_Phone(int encoder1,int encoder2);

void JY901_Receive_Check(void);
 int main(void)
 { 
    u8 count=0;
	uart2_tx.buf[0]=0x55;
	uart2_tx.buf[1]=0xAA;
	uart2_tx.buf[10]=0x0D;

	delay_init();	    		//��ʱ������ʼ��
	NVIC_Configuration();
	USART1_DMA_Init(115200);    //PA9:TX  PA10:RX	��WIFIģ��	 ���ֻ���������״̬���� 
	USART2_DMA_Init(115200);	//PA2:TX  PA3 :RX	������ģ��   ��Arduino����������̬����
	USART3_DMA_Init(115200);	//PB11:RX           �Ӿ���ģ��
	
	LED_Init();		  			//��ʼ����LED GPIO
	CC2541_Connected_Check();
	Encoder_Left_Init();		//��������ʼ��
	Encoder_Right_Init();		//��������ʼ��
	Motor_PWM_Init(7200,1);   	//��ʼ��PWM 10KHZ
	STBY=1;						//ʹ�ܵ������
	TIM3_Int_Init(200,7200);	//10Khz�ļ���Ƶ�ʣ�������200Ϊ20ms    
	while(1)
	{
	
/*
 * ����ֱ��D=65mm=0.065m	  ��������T=20ms=0.02s
 * ����(X)ת��Ϊ�����н��ٶ�(v)       �����ʻ�ٶ�1.24m/s
 * ��/T   -->  m/s
 * v=X*pi*D/1560/T   -->v=0.006545*X
 *                  -->X=152.7887*v
 */
	   JY901_Receive_Check();
	   
	   if(FLAG_TIM3==1)
	   {
	   	CC2541_Receive_Check();	
		g_Encoder_Left=Read_Encoder(2);            								//��ȡ��������ֵ,ע���Ƿ���Ҫȡ��
		g_Encoder_Right=Read_Encoder(4);           								//��ȡ��������ֵ,ע���Ƿ���Ҫȡ��
		CC2541_Send_To_Arduino(g_Encoder_Left,g_Encoder_Right,wzL,wzH,YawL,YawH );	//�ȷ��ͣ��Ա�����DMA���ͽ�Լʱ�������
		//Voltage=Get_battery_volt();            								//��ȡ��ص�ѹ	          
			
	
		Motor_All_Control(g_Encoder_Left,g_Target_Left,g_Encoder_Right,g_Target_Right);
	//	 Wifi_Send_To_Phone(Encoder_Left,Encoder_Right);
		count++;
		if(count==50){count=0;LED=!LED;}   //1s��
		FLAG_TIM3=0;
	  }	  
	}
}





	


/**
 * @Description: wifi���͵��ֻ�APP�����ݣ�������uart1��
 * @param  encoder_left - ���ֱ�����ֵ		
 * @param  encoder_right- ���ֱ�����ֵ
 * @note1  :����Э��
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
 * @Description: JY901(����)���ݽ��մ���
 * @note1 - ���ٶ������ʽ��0X55 0X52 wxL wxH wyL wyH wzL wzH TL TH SUM
 * @note2 - ��  �������ʽ��0X55 0X53 RoL RoH PiL PiH YaL YaH TL TH SUM
 * @note3 - �شų������ʽ��0X55 0X54 HxL HxH HyL HyH HzL HzH TL TH SUM
 */
void JY901_Receive_Check(void)
{
	if(JY901_rx.len==22)
	{
		if (JY901_rx.buf[0]==0x55&&JY901_rx.buf[1]==0x52) //����ͷ���ԣ������¿�ʼѰ��0x55����ͷ
		{
			wzL=JY901_rx.buf[6];
			wzH=JY901_rx.buf[7];
		}
/*
 *      if (JY901_rx.buf[11]==0x55&&JY901_rx.buf[12]==0x54) //����ͷ���ԣ������¿�ʼѰ��0x55����ͷ
 *      {
 *              HyL=JY901_rx.buf[15];
 *              HyH=JY901_rx.buf[16];
 *      }
 */
		if (JY901_rx.buf[11]==0x55&&JY901_rx.buf[12]==0x53) //����ͷ���ԣ������¿�ʼѰ��0x55����ͷ
		{
			YawL=JY901_rx.buf[17];
			YawH=JY901_rx.buf[18];
		}
		JY901_rx.len=0;	
	}	
}


