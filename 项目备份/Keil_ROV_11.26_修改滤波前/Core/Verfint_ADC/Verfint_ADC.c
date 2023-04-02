#include "main.h"

extern ADC_HandleTypeDef	hadc1;
#define	Verfint_Hadc		hadc1
#define Temp_Hadc			hadc1

float Vrefint_Proportion = 0;
float CPU_Temprate = 0;

uint16_t adcx_get_chx_value(ADC_HandleTypeDef *ADCx, uint32_t ch)
{
    static ADC_ChannelConfTypeDef sConfig = {0};
    sConfig.Channel = ch;
    sConfig.Rank = 1;
    sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;	//ADC_SAMPLETIME_3CYCLES;

    if (HAL_ADC_ConfigChannel(ADCx, &sConfig) != HAL_OK)
    {
        Error_Handler();
    }

    HAL_ADC_Start(ADCx);
    HAL_ADC_PollForConversion(ADCx, 10);
	HAL_ADC_Stop(ADCx);
    return (uint16_t)HAL_ADC_GetValue(ADCx);
}


void Get_Vrefint_Proportion(void)
{
    uint8_t i = 0;
    uint32_t total_adc = 0;
    for(i = 0; i < 200; i++)
    {
        total_adc += adcx_get_chx_value(&Verfint_Hadc, ADC_CHANNEL_VREFINT);
    }

    Vrefint_Proportion = 200 * 1.2f / total_adc;
}

float Get_Temprate(void)
{
    uint16_t adcx = 0;
    float temperate;

    adcx = adcx_get_chx_value(&Temp_Hadc, ADC_CHANNEL_TEMPSENSOR);
    temperate = (float)adcx * Vrefint_Proportion;
	
    temperate = (temperate - 0.76f) * 400.0f + 25.0f;
	CPU_Temprate = temperate;


    return temperate;
}