#include "exti.h"
/**************************************************************************
��    �ߣ��������
�Ա���ַ��https://shop119207236.taobao.com
**************************************************************************/

/**************************************************************************
�������ܣ��ⲿ�жϳ�ʼ��
��ڲ�������
����  ֵ���� 
��    �ã�����������MPU6050����INT�ģ�ÿ��MPU6050���������ʱ������INT����Ӧ�ĵ�ƽ�����
					�����������ⲿ�ж���Ϊ�������ڡ�����MPU6050���ݵ�ʵʱ�ԡ�
**************************************************************************/
void MPU6050_EXTI_Init(void)
{  
		GPIO_InitTypeDef GPIO_InitStructure;
		EXTI_InitTypeDef EXTI_InitStructure;
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);//�ⲿ�жϣ���Ҫʹ��AFIOʱ��
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE); //ʹ��PB�˿�ʱ��
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;	            //�˿�����
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;         //��������
		GPIO_Init(GPIOB, &GPIO_InitStructure);					      //�����趨������ʼ��GPIOB 
  	GPIO_EXTILineConfig(GPIO_PortSourceGPIOB,GPIO_PinSource5);
  	EXTI_InitStructure.EXTI_Line=EXTI_Line5;
  	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;	
  	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;//�½��ش���
  	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  	EXTI_Init(&EXTI_InitStructure);	 	//����EXTI_InitStruct��ָ���Ĳ�����ʼ������EXTI�Ĵ���
}










