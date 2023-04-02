
#include "HAL_conf.h"
#include "bldc_config.h"
#include "comp.h"

void comp_init(void)
{
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOD, ENABLE);
    
    GPIO_InitTypeDef  GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Pin  =  GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;                           //������Ҫ����Ϊģ������
	GPIO_Init(GPIOD, &GPIO_InitStructure);

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_COMP, ENABLE);                    //����ʱ��
    
    COMP_InitTypeDef COMP_InitStructure;
    COMP_InitStructure.COMP_InvertingInput = COMP_InvertingInput_CRV;       //�Ƚ�����������ӵ�CRV
    COMP_InitStructure.COMP_NonInvertingInput = COMP_NonInvertingInput_IO0; //�Ƚ���ͬ������ӵ�IO0
    COMP_InitStructure.COMP_Output = COMP_Output_TIM1BKIN;                  //�Ƚ����������ʱ��1��ɲ������˿�
    COMP_InitStructure.COMP_OutputPol = COMP_OutputPol_NonInverted;         //���������
    COMP_InitStructure.COMP_Hysteresis = COMP_Hysteresis_No;                //�Ƚ����������ʱ
    COMP_InitStructure.COMP_Mode = COMP_Mode_MediumSpeed;                   //�е�����
    COMP_InitStructure.COMP_Filter = COMP_Filter_32_Period;                 //�Ƚ�������˲�ʱ��Ϊ64������
    
    SET_COMP_CRV(COMP_CRV_Sele_AVDD, BLDC_FAULT_THRESHOLD);                 //����CRV��ѹΪ2/20*VCC
    COMP_Init(COMP_Selection_COMP4, &COMP_InitStructure);
    COMP_Cmd(COMP_Selection_COMP4, ENABLE);                                 //�Ƚ���ʹ��
}

