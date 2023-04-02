
#include "HAL_conf.h"
#include "adc.h"

adc_struct adc_information;


void adc_init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	ADC_InitTypeDef  ADC_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1 | RCC_APB2Periph_ADC2, ENABLE);
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE); 
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE); 
	
	GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_3 | GPIO_Pin_6;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;		                    //ADC������Ҫ����Ϊģ������
	GPIO_Init(GPIOA, &GPIO_InitStructure);
    
    GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_2 | GPIO_Pin_10 | GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;		
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	
	ADC_StructInit(&ADC_InitStructure);
	ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;                  //adc�ֱ�������Ϊ12λ
	ADC_InitStructure.ADC_PRESCARE = ADC_PCLK2_PRESCARE_8;                  //adcʱ��=PCLK2/8
	ADC_InitStructure.ADC_Mode = ADC_Mode_Single_Period;                    //������ת��
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;			        //�����Ҷ���
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T1_CC4;   //ʹ�ö�ʱ��CC4ͨ������ADCת������ÿ��PWM���ڴ���һ��ADCת��
	ADC_Init(ADC1, &ADC_InitStructure);
	ADC_Cmd(ADC1, ENABLE);//ʹ��ADC
    
    ADC_RegularChannelConfig(ADC1, 3, 0, ADC_SampleTime_7_5Cycles);     //A3����  ���ĸ�߾���RC�˲�֮��ĵ���
	ADC_RegularChannelConfig(ADC1, 6, 1, ADC_SampleTime_7_5Cycles);     //A6����  ���ĸ���˲� ֮ǰ �ĵ���
    ADC_RegularChannelConfig(ADC1, 10, 2, ADC_SampleTime_7_5Cycles);    //B2����  ���A�����
    ADC_RegularChannelConfig(ADC1, 11, 3, ADC_SampleTime_7_5Cycles);    //B10���� ���B�����
    
	ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;                  //adc�ֱ�������Ϊ12λ
	ADC_InitStructure.ADC_PRESCARE = ADC_PCLK2_PRESCARE_8;                  //adcʱ��=PCLK2/8
	ADC_InitStructure.ADC_Mode = ADC_Mode_Single_Period;                    //������ת��
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;			        //�����Ҷ���
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T1_CC4;   //ʹ�ö�ʱ��CC4ͨ������ADCת������ÿ��PWM���ڴ���һ��ADCת��
	ADC_Init(ADC2, &ADC_InitStructure);
	ADC_Cmd(ADC2, ENABLE);//ʹ��ADC

    ADC_RegularChannelConfig(ADC2, 1, 0, ADC_SampleTime_7_5Cycles);     //B10���� ���B�����
    ADC_ExternalTrigConvCmd(ADC2, ENABLE);              //ADC�ⲿ����ʹ��
    
	ADC_ITConfig(ADC1,ADC_IT_EOC, ENABLE);              //ʹ��ADC�ж�
	ADC_ExternalTrigConvCmd(ADC1, ENABLE);              //ADC�ⲿ����ʹ��
	NVIC_InitStructure.NVIC_IRQChannel = ADC1_IRQn;     //�жϱ��
	NVIC_InitStructure.NVIC_IRQChannelPriority = 0x01;	//���ȼ����� 0:���, 3:���
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;		//�ж�ʹ��
	NVIC_Init(&NVIC_InitStructure);
}

void adc_read(void)
{
    adc_information.current_bus_filter = ADC1->ADDR3;   //��ȡ�˲�֮���ĸ�ߵ���
    adc_information.current_bus = ADC1->ADDR6;          //��ȡĸ�ߵ���
    adc_information.voltage_bus = ((ADC2->ADDR1)&0xffff)*5*400/4096; //��ȡ��Դ��ѹ ADC�����ϵĵ�ѹ = ADCֵ/4096*5  ��Դ��ѹ = ADC�����ϵĵ�ѹ / (R27/(R27+R20))
    adc_information.current_a = ADC1->ADDR10;           //��ȡA�����
    adc_information.current_b = ADC1->ADDR11;           //��ȡB�����
   
}
