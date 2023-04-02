
#include "HAL_conf.h"
#include "timer.h"


void tim1_complementary_pwm(uint16 period,uint8 dead_time)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	TIM_BDTRInitTypeDef TIM_BDTRInitStruct;
	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA | RCC_AHBPeriph_GPIOB, ENABLE);

    //break�ź���������
    GPIO_InitStructure.GPIO_Pin  =  GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource11, GPIO_AF_6);
    
	//��ʱ�� CH1 CH2 CH3���ų�ʼ�� ���ڿ�������
	GPIO_InitStructure.GPIO_Pin  =  GPIO_Pin_3|GPIO_Pin_5|GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource7, GPIO_AF_7);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource5, GPIO_AF_7);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource3, GPIO_AF_7);
	
	//�������ų�ʼ��ΪIO
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;						
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4|GPIO_Pin_6;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
    
    TIM_DeInit(TIM1);
	
	TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
	TIM_TimeBaseStructure.TIM_Period = period;				                //���ö�ʱ������ֵ 
	TIM_TimeBaseStructure.TIM_Prescaler = 0;						        //��ʱ��ʱ�Ӳ���Ƶ
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_CenterAligned1; //���Ķ���ģʽ  PWMƵ��=��ʱ������ʱ��Ƶ��/(2*����ֵ)
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; 				//����Ƶ
	TIM_TimeBaseStructure.TIM_RepetitionCounter = 1;						//ռ�ձȵĸ���Ƶ��=��ʱ������ʱ��Ƶ��/((TIM_RepetitionCounter+1)*����ֵ)
	TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure); 					


	TIM_OCStructInit(&TIM_OCInitStructure); 						        
	
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; 						//PWMģʽ1����ʱ������ֵС�ڱȽ�ֵ��ʱ�����������Ч��ƽ
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; 			//CHxͨ��ʹ��
	TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Enable;         //
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; 				//CHx  ��Ч��ƽΪ�ߵ�ƽ ��Ч��ƽΪ�͵�ƽ
	TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_High; 			//CHxN ��Ч��ƽΪ�ߵ�ƽ ��Ч��ƽΪ�͵�ƽ
	TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Reset; 			//CHx  ����״̬����͵�ƽ
	TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCNIdleState_Reset; 			//CHxN ����״̬����͵�ƽ

	TIM_OCInitStructure.TIM_Pulse = 0; 						
	TIM_OC1Init(TIM1, &TIM_OCInitStructure);				
	TIM_OCInitStructure.TIM_Pulse = 0; 						
	TIM_OC2Init(TIM1, &TIM_OCInitStructure);				
	TIM_OCInitStructure.TIM_Pulse = 0; 						
	TIM_OC3Init(TIM1, &TIM_OCInitStructure);				
	TIM_OCInitStructure.TIM_Pulse = 1;      //����ͨ��4�Ƚ�ֵ
	TIM_OC4Init(TIM1, &TIM_OCInitStructure);//CC4���ڴ���ADC�ɼ�

	TIM_OC1PreloadConfig(TIM1,TIM_OCPreload_Disable );  //Ԥװ��ʧ��
	TIM_OC2PreloadConfig(TIM1,TIM_OCPreload_Disable );  
	TIM_OC3PreloadConfig(TIM1,TIM_OCPreload_Disable );  
	TIM_OC4PreloadConfig(TIM1,TIM_OCPreload_Disable );  
    
    
	TIM1->PSC = 0x00;   // ��ʱ��ʱ�� = HSI/(PSC+1) = 96MHz/1 = 96MHz
	
	TIM_BDTRStructInit(&TIM_BDTRInitStruct);
	TIM_BDTRInitStruct.TIM_OSSRState = TIM_OSSRState_Disable;
	TIM_BDTRInitStruct.TIM_OSSIState = TIM_OSSIState_Disable;
	TIM_BDTRInitStruct.TIM_LOCKLevel = TIM_LOCKLevel_2;    
	TIM_BDTRInitStruct.TIM_DeadTime  = dead_time;        
	
	TIM_BDTRInitStruct.TIM_Break     = TIM_Break_Enable;                    //ɲ��ʹ��
	TIM_BDTRInitStruct.TIM_BreakPolarity   = TIM_BreakPolarity_High;        //ɲ������Ϊ�ߣ��ߵ�ƽ��ʱ��ɲ��
	TIM_BDTRInitStruct.TIM_AutomaticOutput = TIM_AutomaticOutput_Enable;    //��ɲ���ź���Ч��ʱ���Զ��������
	TIM_BDTRConfig(TIM1, &TIM_BDTRInitStruct); 

	TIM_ARRPreloadConfig(TIM1, ENABLE);    		//ʹ���Զ���װ��
    
    TIM_ITConfig(TIM1,TIM_IT_Break,DISABLE);	//�ر�ɲ���ж�
	TIM_ITConfig(TIM1,TIM_IT_Update,DISABLE);	//�رո����ж�

	NVIC_InitStructure.NVIC_IRQChannel = TIM1_BRK_UP_TRG_COM_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPriority = 0x01;  //�����ж����ȼ�
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;     //�ж�ʹ��
    NVIC_Init(&NVIC_InitStructure);
    
	TIM_Cmd(TIM1, ENABLE);  //ʹ�ܶ�ʱ��1

}

void tim1_complementary_pwm_control(uint8 status)
{
    if(status)  
    {
        TIM_CtrlPWMOutputs(TIM1, ENABLE); 
    }
    else
    {
        TIM_CtrlPWMOutputs(TIM1, DISABLE); 
    }
}


void tim3_init(uint16 prescaler, uint16 period)
{
	TIM_TimeBaseInitTypeDef TIM_StructInit;
	NVIC_InitTypeDef NVIC_StructInit;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

	TIM_TimeBaseStructInit(&TIM_StructInit);
	TIM_StructInit.TIM_Period = period;         //ARR�Ĵ���ֵ
	TIM_StructInit.TIM_Prescaler = prescaler;   //Ԥ��Ƶֵ

	TIM_StructInit.TIM_ClockDivision = TIM_CKD_DIV1;    //������Ƶֵ
	TIM_StructInit.TIM_CounterMode = TIM_CounterMode_Up;//����ģʽ
	TIM_StructInit.TIM_RepetitionCounter = 0;

	TIM_TimeBaseInit(TIM3, &TIM_StructInit);

	NVIC_StructInit.NVIC_IRQChannel = TIM3_IRQn;
	NVIC_StructInit.NVIC_IRQChannelPriority = 2;
	NVIC_StructInit.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_StructInit);
	TIM_ClearFlag(TIM3, TIM_FLAG_Update);
    
	TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);  //����ʱ�������ж�

	TIM_Cmd(TIM3, ENABLE);  //ʹ�ܶ�ʱ��
}



