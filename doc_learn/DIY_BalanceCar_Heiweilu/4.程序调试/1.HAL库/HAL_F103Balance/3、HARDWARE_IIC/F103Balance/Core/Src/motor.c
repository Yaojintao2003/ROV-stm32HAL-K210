#include "motor.h"

void Motor_Start(void)     //Motor���������ӵ�PB12~15��ʼ����main()����MX��ִ����.
{
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);  //����MX���ú�ԭ��ͼ��������Ҫ��������TIM1��PWM���ͨ��1
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_4);  //����TIM1��PWM���ͨ��1
}
/**************************************************************************
�������ܣ���ֵ��PWM�Ĵ���
��ڲ���������PWM������PWM
����  ֵ����
**************************************************************************/
void Motor_Rotaton(int MotorA,int MotorB)
{
	//1.�о������ţ���Ӧ����ת
	if(MotorA>0)	Ain1=1,Ain2=0;//��ת
	else 				Ain1=0,Ain2=1;//��ת
	//2.�о�PWMֵ
	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,GFP_abs(MotorA));   //�Ѵ��ν�����PWMֵ�͸���ʱ��1��channel_1ͨ��.
	
	if(MotorB>0)	Bin1=1,Bin2=0;
	else 				Bin1=0,Bin2=1;	
	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_4,GFP_abs(MotorB));
}

/**************************************************************************
�������ܣ�����ֵ����
��ڲ�����int
����  ֵ��unsigned int
Ŀ    �ģ�����ֱ�������ٶȻ��Լ�ת�򻷼��������PWM�п���Ϊ��ֵ
					��ֻ�ܸ�����ʱ��PWM�Ĵ���ֻ������ֵ������Ҫ��PWM���о���ֵ����
**************************************************************************/
int GFP_abs(int a)
{ 		   
	int temp;
	temp = a>0?a:(-a);
	return temp;
}

/*�޷�����:����PWM���󳬹����Ļ�е����*/
void Limit(int *motoA,int *motoB)
{
	if(*motoA>PWM_MAX)  *motoA=PWM_MAX;
	if(*motoA<PWM_MIN)  *motoA=PWM_MIN;
	
	if(*motoB>PWM_MAX)  *motoB=PWM_MAX;
	if(*motoB<PWM_MIN)  *motoB=PWM_MIN;
}
/**************************************************************************
�������ܣ��쳣�رյ��
��ڲ�������Ǻ͵�ѹ
����  ֵ����
**************************************************************************/
void Stop(float angle, float voltage)
{
		if(angle<-40||angle>40||voltage<11.1)	 //��ص�ѹ����11.1V�رյ��
		{	                                   //===��Ǵ���40�ȹرյ��																			 
				Moto1=0;
				Moto2=0;
		}		
}

/*htim2��htim3�����tim.c���Ѿ�������ˡ�
������ԭ��:ÿ�����A���B��ֱ���ĳ����ʱ��������channelȥ�ɼ������ֵ�1��0,AB��λ��90��(����)���Ը��������ź��ĸ����ĸ������жϷ���,
����ÿ���ź����������Ķ��ټ����������ֵ��ܳ��Ϳ��������ǰ���ߵľ���;����ټ��϶�ʱ���Ļ������Լ�����ٶȡ�
��ʱ�ɼ����ı�����������û�и���ë��,��˱�����ģʽ�Ķ�ʱ������ʱ�������˲�FPֵ.*/

void Encoder_Start(void)   //��������������. ����������ʼ��HAL_TIM_Encoder_Init()��HAL_TIM_Encoder_MspInit()��main()�����е��õ�MX_TIM2_Init()�С�
{
	__HAL_TIM_SET_COUNTER(&htim2,0);   //�ô��κ����ñ������ĳ�ʼֵΪ0(�漰����ת����Ҫ)
	__HAL_TIM_SET_COUNTER(&htim3,0);

	HAL_TIM_Encoder_Start_IT(&htim2,TIM_CHANNEL_ALL);   //�������������ж�ģʽ,������ʱ��ͨ��TI1��TI2��ÿ���������������źŲɼ�ͨ��.
	HAL_TIM_Encoder_Start_IT(&htim3,TIM_CHANNEL_ALL);

	HAL_TIM_Encoder_Start(&htim2,TIM_CHANNEL_ALL);  //��������������
	HAL_TIM_Encoder_Start(&htim3,TIM_CHANNEL_ALL);
	
}


//��ȡ����������ֵ
int Read_Speed(int TIMx)
{
	int Encoder_Value = 0;
	switch(TIMx)
	{
		case 2: 
			Encoder_Value =(short) __HAL_TIM_GET_COUNTER(&htim2);     //�����������������ֵ
			__HAL_TIM_SET_COUNTER(&htim2,0);                   //����֮��Ҫ����,�Ա��´μ�����ȡ.����ÿ����������ֵ��0,ֱ���õ�λʱ��Ļ��Ϳ��Եó��ٶ���Ϣ��.��Ҫ�鷳��Ҫ��ȥ��ֵ��.
			break;

		case 3: 
			Encoder_Value =(short) __HAL_TIM_GET_COUNTER(&htim3);
			__HAL_TIM_SET_COUNTER(&htim3,0);
			break;
		
		default:
			Encoder_Value = 0;
	}	
	return Encoder_Value;
}

void Encoder_IRQHandler(void)   //�˴�ֻ�����жϱ�־,ʵ�ʲ�û������������.
{
	HAL_TIM_IRQHandler(&htim2);   //IRQHandler()����ֻ�����жϱ�־,ʵ��ISR�жϷ�������Ǹú�����ȥ������Ӧ��callback()����ʵ�ֵ�.
	HAL_TIM_IRQHandler(&htim3);   //��������ҪISR������Զ�����Ӧ��callback()��������,��Ϊϵͳ�е�callback()��__weak�����Ŀպ���.
}
