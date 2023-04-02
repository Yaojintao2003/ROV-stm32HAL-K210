
#include "hal_conf.h"
#include "motor.h"
#include "pwm_input.h"



float duty_cycle;
uint8 duty_flag;//��ǰռ�ձ��Ƿ���Ч 1����Ч  0����Ч
int16 period, duty_low, duty_high;



void pwm_input_init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);
    GPIO_PinAFConfig(MOTOR_PWM_PORT, GPIO_PinSource10, GPIO_AF_6);
    GPIO_InitStructure.GPIO_Pin = MOTOR_PWM_PIN;      //PWM���벶������
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
    GPIO_Init(MOTOR_PWM_PORT, &GPIO_InitStructure);
    
    GPIO_InitStructure.GPIO_Pin = MOTOR_DIR_PIN;       //�������� ����Ϊ�͵�ƽʱ�����ת��Ϊ�ߵ�ƽ�ǵ����ת
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
    GPIO_Init(MOTOR_DIR_PORT, &GPIO_InitStructure);
    
    
    NVIC_InitTypeDef NVIC_InitStructure;
    NVIC_InitStructure.NVIC_IRQChannel = TIM16_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPriority = 0x00;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //IRQͨ��ʹ��
	NVIC_Init(&NVIC_InitStructure);                 //����ָ���Ĳ�����ʼ��NVIC�Ĵ���
    
    
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
    TIM_ICInitTypeDef  TIM16_ICInitStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM16, ENABLE);

    TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
    TIM_TimeBaseStructure.TIM_Period = 20000;
    TIM_TimeBaseStructure.TIM_Prescaler = SystemCoreClock / 1000000 - 1;
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM16, &TIM_TimeBaseStructure);

    TIM_ICStructInit(&TIM16_ICInitStructure);
    TIM16_ICInitStructure.TIM_Channel = TIM_Channel_1;
    TIM16_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_BothEdge;
    TIM16_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
    TIM16_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
    TIM16_ICInitStructure.TIM_ICFilter = 10;
    
    TIM_PWMIConfig(TIM16, &TIM16_ICInitStructure);

    TIM_ITConfig(TIM16, TIM_IT_CC1, ENABLE);        //ʹ��ͨ��һ�ж�
    TIM_ITConfig(TIM16, TIM_IT_Update, ENABLE);     //ʹ�ܸ����ж�
    TIM_ClearITPendingBit(TIM16, TIM_IT_CC1);
    TIM_ClearITPendingBit(TIM16, TIM_IT_Update);

    TIM_Cmd(TIM16, ENABLE);
}

uint8 pwm_dir_get(void)
{
    //��ȡ�����ٶȵ���������
    //0�����������ת  1�����������ת
    return GPIO_ReadInputDataBit(MOTOR_DIR_PORT, MOTOR_DIR_PIN);
}









