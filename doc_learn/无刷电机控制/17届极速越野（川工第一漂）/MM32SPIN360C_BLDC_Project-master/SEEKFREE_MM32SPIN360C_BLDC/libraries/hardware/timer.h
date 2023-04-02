#ifndef _timer_H_
#define _timer_H_

#include "common.h"


#define FCY  ((uint32)96000000)                    	    //ϵͳʱ��
#define FPWM  ((uint16)40000)                       	//PWMƵ��
#define PWM_PRIOD_LOAD (uint16)(FCY/FPWM/2)    	        //PWM����װ��ֵ
#define DEADTIME_LOAD (9)	         			        //����װ��ֵ

#define TIM3_TCY      1                           		//��ʱ��3ʱ����λus  
#define TIM3_PSC_LOAD (uint16)(FCY/1000000*TIM3_TCY-1)  //��ʱ��3װ��ֵ
#define TIM3_PRIOD    999                          		//��ʱ��3����ֵ

void tim1_complementary_pwm(uint16 period,uint8 dead_time);
void tim1_complementary_pwm_control(uint8 status);
void tim3_init(uint16 prescaler, uint16 period);
#endif
