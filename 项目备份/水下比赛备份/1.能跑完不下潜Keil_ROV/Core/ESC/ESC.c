/*
 * ESC.c
 *
 *  Created on: Jan 16, 2021
 *      Author: Royic
 */

#include "main.h"
#include "cmsis_os.h"

#include "ESC.h"

#include "stm32_hal_legacy.h"
#define USE_HAL_LEGACY

#define PSC40
//#define PSC100

#ifdef PSC40
	#define	MinStatus	2500
	#define MidStatus	3750
	#define FullStatus	5000
#endif

#ifdef PSC100
	#define	MinStatus	1000
	#define MidStatus	1500
	#define FullStatus	2000
#endif

#define ESC_INIT 1

extern TIM_HandleTypeDef htim3, htim9;

void ESC_Init(ESC_Type Flag)
{
	//��ʱ��ƽ������,����������ź�
	HAL_TIM_PWM_Stop(&htim9, TIM_CHANNEL_1);				
	HAL_TIM_PWM_Stop(&htim9, TIM_CHANNEL_2);

	HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);				
	HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_2);
	HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_3);
	HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_4);
	if(Flag)
		HAL_Delay(500);											//0.5s
	else
		osDelay(500);

	//ģ����ջ���ʼ�ϵ�״̬
	__HAL_TIM_SetCompare(&htim9, TIM_CHANNEL_1, MidStatus);	
	__HAL_TIM_SetCompare(&htim9, TIM_CHANNEL_2, MidStatus);

	__HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_1, MidStatus);	
	__HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_2, MidStatus);
	__HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_3, MidStatus);
	__HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_4, MidStatus);

	HAL_TIM_PWM_Start(&htim9, TIM_CHANNEL_1);				
	HAL_TIM_PWM_Start(&htim9, TIM_CHANNEL_2);

	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);				
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);

	if(Flag)
		HAL_Delay(5000);											//5s
	else
		osDelay(5000);

	//�������
	__HAL_TIM_SetCompare(&htim9, TIM_CHANNEL_1, FullStatus);
	__HAL_TIM_SetCompare(&htim9, TIM_CHANNEL_2, FullStatus);

	__HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_1, FullStatus);
	__HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_2, FullStatus);
	__HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_3, FullStatus);
	__HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_4, FullStatus);										
	if(Flag)
		HAL_Delay(2000);											//2s ��1��2��3~��1!
	else
		osDelay(2000);

	//���Ż���
	__HAL_TIM_SetCompare(&htim9, TIM_CHANNEL_1, MidStatus);
	__HAL_TIM_SetCompare(&htim9, TIM_CHANNEL_2, MidStatus);

	__HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_1, MidStatus);	
	__HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_2, MidStatus);
	__HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_3, MidStatus);
	__HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_4, MidStatus);									
	if(Flag)
		HAL_Delay(500);											//0.5s	��4
	else
		osDelay(500);
}
