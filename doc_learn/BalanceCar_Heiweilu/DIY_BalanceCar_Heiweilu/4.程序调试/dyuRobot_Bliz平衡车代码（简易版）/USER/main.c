 /**************************************************************************
 ��  �� ���������
�Ա���ַ��https://shop119207236.taobao.com
**************************************************************************/
#include "sys.h"
#include "control.h"
//extern int Balance_Pwm ;
//extern  int Encoder_TIM ;
//////////////////////////ȫ�ֱ����Ķ���////////////////////////////////////      
float pitch,roll,yaw; 								  			 //ŷ����(��̬��)-----������ ������ ƫ����
short aacx,aacy,aacz;													 //���ٶȴ�����ԭʼ����
short gyrox,gyroy,gyroz;											 //������ԭʼ����
int   Encoder_Left,Encoder_Right;         		 //���ұ��������������
int 	Moto1=0,Moto2=0;												 //������������ո��������PWM	
int main(void)	
{ 
	delay_init();	    	           //=====��ʱ������ʼ��	
	NVIC_Configuration();					 //=====�ж����ȼ�����,���а��������е��ж����ȼ�������,��������һ�����޸ġ�
	uart1_init(9600);	             //=====����1��ʼ��
	Encoder_Init_TIM2();           //=====��ʼ��������2
	Encoder_Init_TIM4();          //=====��ʼ��������4
	OLED_Init();                   //=====OLED��ʼ��
	OLED_Clear();									 //=====OLED����
	MPU_Init();					    			 //=====��ʼ��MPU6050
	mpu_dmp_init();								 //=====��ʼ��MPU6050��DMPģʽ					 
	TIM1_PWM_Init(7199,0);   			 //=====��ʼ��PWM 10KHZ,�������������
	delay_ms(1000);								 //=====��ʱ2s ���С���ϵ�������ת������
	delay_ms(1000);								 //=====��ʱ2s ���С���ϵ�������ת������
	MPU6050_EXTI_Init();					 //=====MPU6050 5ms��ʱ�жϳ�ʼ��
	Motor_Init();									 //=====��ʼ���������ӵ�Ӳ��IO�ӿ�
  OLED_ShowString(0,0,"ANGLE:",12);						 
  while(1)	
	{	
			delay_ms(100);
			OLED_Float(0,48,pitch,3);	 //��ʾ�Ƕ�
	   //OLED_Float(1,48,Balance_Pwm,3);  
      OLED_Float(1,48,Moto1,0);
		 //  OLED_Float(2,48,Moto2,3);//��ʾ���pwm
	   //	OLED_Float(1,48,Encoder_TIM,0);
	
	} 	
}



