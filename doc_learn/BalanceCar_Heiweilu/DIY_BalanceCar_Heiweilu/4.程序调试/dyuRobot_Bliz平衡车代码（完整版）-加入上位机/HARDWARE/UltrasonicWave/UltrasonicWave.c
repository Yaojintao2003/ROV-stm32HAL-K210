#include "UltrasonicWave.h"


#define	TRIG_PIN       GPIO_Pin_3   //TRIG       
#define	ECHO_PIN       GPIO_Pin_2		//ECHO   

/*
 * ��������UltrasonicWave_Configuration
 * ����  ��������ģ��ĳ�ʼ��
 * ����  ����
 * ���  ����	
 */
void UltrasonicWave_Configuration(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;	
	EXTI_InitTypeDef EXTI_InitStructure;
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_AFIO, ENABLE);
    
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;					 //PA3��TRIG
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;		     //��Ϊ�������ģʽ
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	         
  GPIO_Init(GPIOA, &GPIO_InitStructure);	                 //��ʼ������GPIO 

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;				     //PA3��ECH0
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;		 //��Ϊ����
  GPIO_Init(GPIOA,&GPIO_InitStructure);						 //��ʼ��GPIOA
	
	 //GPIOA.2	  �ж����Լ��жϳ�ʼ������
 	GPIO_EXTILineConfig(GPIO_PortSourceGPIOA,GPIO_PinSource2);

 	EXTI_InitStructure.EXTI_Line=EXTI_Line2;
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;	
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure);		//����EXTI_InitStruct��ָ���Ĳ�����ʼ������EXTI�Ĵ���
}


void EXTI2_IRQHandler(void)
{
	delay_us(10);		                      //��ʱ10us
  if(EXTI_GetITStatus(EXTI_Line2) != RESET)
	{
			TIM_SetCounter(TIM3,0);																		
			TIM_Cmd(TIM3, ENABLE);                               				//����ʱ��
			while(GPIO_ReadInputDataBit(GPIOA,ECHO_PIN));	       				//�ȴ��͵�ƽ
			TIM_Cmd(TIM3, DISABLE);			                         				//��ʱ��3ʧ��
			UltrasonicWave_Distance=TIM_GetCounter(TIM3)*1.7;						//�������&&UltrasonicWave_Distance<150
		
//	if(UltrasonicWave_Distance>0)
//	{
//		printf("distance:%f cm",UltrasonicWave_Distance);
//	}
//		
			EXTI_ClearITPendingBit(EXTI_Line2);  //���EXTI2��·����λ
}

}

/*
 * ��������UltrasonicWave_StartMeasure
 * ����  ����ʼ��࣬����һ��>10us�����壬Ȼ��������صĸߵ�ƽʱ��
 * ����  ����
 * ���  ����	
 */
void UltrasonicWave_StartMeasure(void)
{
  GPIO_SetBits(GPIOA,TRIG_PIN); 		  //��>10US�ĸߵ�ƽ�TRIG_PORT,TRIG_PIN��������define����˵
  delay_us(20);		                      //��ʱ20US
  GPIO_ResetBits(GPIOA,TRIG_PIN);
	
}

/******************* (C) 1209 Lab *****END OF FILE************/
