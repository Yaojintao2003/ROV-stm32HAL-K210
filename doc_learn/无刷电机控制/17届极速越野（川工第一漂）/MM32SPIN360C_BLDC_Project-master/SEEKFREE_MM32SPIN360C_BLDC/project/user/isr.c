
#include "HAL_conf.h"
#include "gpio.h"
#include "adc.h"
#include "timer.h"

#include "pwm_input.h"
#include "opamp.h"
#include "comp.h"
#include "hall.h"
#include "pid.h"
#include "motor.h"
#include "move_filter.h"
#include "bldc_config.h"
#include "isr.h"


void TIM16_IRQHandler(void)
{
    static uint8 last_trigger_status;//1:�ϴ�Ϊ�½��ش��� 2���ϴ�Ϊ�����ش���
    TIM16->CNT = 0;
    if (TIM16->SR & TIM_IT_CC1)
    {
        TIM16->SR = (uint32_t)~TIM_IT_CC1;//TIM_ClearITPendingBit(TIM16, TIM_IT_CC1);
        if(MOTOR_PWM_PORT->IDR & MOTOR_PWM_PIN)//������    //GPIO_ReadInputDataBit(MOTOR_PWM_PORT, MOTOR_PWM_PIN))
        {
            if(2 != last_trigger_status)//�ϴβ��������ش��� ���β���Ч
            {
                last_trigger_status = 2;
                //��ȡ�͵�ƽ����ʱ�䲢��������
                duty_low = TIM16->CCR1;//TIM_GetCapture1(TIM16);
                period = duty_low + duty_high;
                duty_flag = 1;
            }
            
        }
        else//�½���
        {
            if(1 != last_trigger_status)//�ϴβ����½��ش��� ���β���Ч
            {
                last_trigger_status = 1;
                //��ȡ�ߵ�ƽ����ʱ��
                duty_high = TIM16->CCR1;//TIM_GetCapture1(TIM16);
            }
        }
    }

    if (TIM16->SR & TIM_IT_Update) 
    {
        //��ʱ��������״̬���øߵ�ƽʱ��
        if(MOTOR_PWM_PORT->IDR & MOTOR_PWM_PIN)//GPIO_ReadInputDataBit(MOTOR_PWM_PORT, MOTOR_PWM_PIN))
        {
            period = 1000;
            duty_high = period;
            duty_low = 0;
        }
        else
        {
            period = 1000;
            duty_high = 0;
        }
        TIM16->SR = (uint32_t)~TIM_IT_Update;//TIM_ClearITPendingBit(TIM16, TIM_IT_Update);
        duty_flag = 2;//�����źų�ʱ
    }
}

int16 break_time_num=0;
void TIM3_IRQHandler(void)
{
    int8 dir;
	
	break_time_num++;
	if(break_time_num>2000)
		break_time_num=0;
	
	if (TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET) 
	{                           
		TIM_ClearITPendingBit(TIM3, TIM_IT_Update);     //����жϱ�־λ   

        //���ʹ�ܿ���״̬
        motor_en();
        
		//���ݽ��յ����źţ����㷽��
        dir = pwm_dir_get();
        dir = dir==0?1:(int8)-1;
        
        //����pwm�źŵ�ռ�ձ�
        if(duty_flag)
        {
            duty_flag = 0;
            if(500 < period)  
            {
                duty_cycle = (float)duty_high/period;
            }
        }
        
        
        //�����ٶ�
    #if BLDC_CLOSE_LOOP_ENABLE==1
        motor_control.set_speed = dir * motor_control.max_speed * duty_cycle;
    #else
        motor_control.set_speed = dir * PWM_PRIOD_LOAD * duty_cycle;
    #endif
        
    #if BLDC_SPEED_CURVE_ENABLE==1
        //����������� ͨ�����ò������Ե��ڼ��ٵ���Ӧ�ٶ�
        motor_speed_curve();
    #else
        closed_loop.target_speed = motor_control.set_speed;
    #endif

        motor_set_dir();
        
    #if BLDC_CLOSE_LOOP_ENABLE==1
        //����PI�ջ�����
        closed_loop_pi_calc(speed_filter.data_average);
    #else
        closed_loop.out = closed_loop.target_speed;
    #endif

        //�������
        motor_power_out();
    }
}


void ADC1_IRQHandler(void)
{
	//�����־λ
    ADC_ClearFlag(ADC1, ADC_IT_EOC);
    
    //��ȡadc��ֵ
    adc_read();
    
	//����ɨ��
	scan_hall_status();

    if(1 > commutation_delay--)
    {//��ʱʱ�䵽 ��ʼ����
        commutation_delay = 0;
        motor_commutation(next_hall_value);
    }
}


void TIM1_BRK_UP_TRG_COM_IRQHandler(void)
{
	if(TIM_GetITStatus(TIM1, TIM_IT_Update) != RESET) 
	{
		TIM_ClearITPendingBit(TIM1, TIM_IT_Update);  	
	}
	
	if(TIM_GetITStatus(TIM1, TIM_IT_Break) != RESET) 
	{
		TIM_ClearITPendingBit(TIM1, TIM_IT_Break);  
	}
}


