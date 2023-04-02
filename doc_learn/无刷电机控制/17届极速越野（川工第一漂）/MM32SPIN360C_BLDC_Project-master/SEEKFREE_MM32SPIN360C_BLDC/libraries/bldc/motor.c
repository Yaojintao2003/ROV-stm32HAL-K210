/*-------------------- Includes -----------------------*/
#include "HAL_conf.h"
#include "hall.h"
#include "adc.h"
#include "timer.h"
#include "gpio.h"
#include "move_filter.h"
#include "pwm_input.h"
#include "bldc_config.h"
#include "motor.h"


motor_struct motor_control;


//-------------------------------------------------------------------------------------------------------------------
//  @brief      �ٶ����߼��㺯��
//  @param      void
//  @return     void					
//  @since      
//-------------------------------------------------------------------------------------------------------------------
void motor_speed_curve(void)
{
    if(FORWARD == motor_control.dir)
    {   //���õ��ٶȱȱջ���Ŀ���ٶȴ���ʹ�ò���ֵ���ջ�Ŀ���ٶ��𲽱ƽ������ٶ�
        if(motor_control.set_speed > closed_loop.target_speed)
        {
            closed_loop.target_speed += motor_control.increase_step;
            if(closed_loop.target_speed > motor_control.set_speed)
            {
                closed_loop.target_speed = motor_control.set_speed;
            }
        }
        //���õ��ٶȱ�Ŀ���ٶȵͣ���ʹ�ò���ֵ
        else
        {
            closed_loop.target_speed = motor_control.set_speed;
            if(0 > closed_loop.target_speed) 
            {
                closed_loop.target_speed = 0;
            }
        }
    }
    else//��ת
    {   //���õ��ٶȱȱջ���Ŀ���ٶȴ���ʹ�ò���ֵ���ջ�Ŀ���ٶ��𲽱ƽ������ٶ�
        if(motor_control.set_speed < closed_loop.target_speed)
        {
            closed_loop.target_speed -= motor_control.increase_step;
            if(closed_loop.target_speed < motor_control.set_speed)
            {
                closed_loop.target_speed = motor_control.set_speed;
            }
        }
        //���õ��ٶȱ�Ŀ���ٶȵͣ���ʹ�ò���ֵ
        else
        {
            closed_loop.target_speed = motor_control.set_speed;
            if(0 < closed_loop.target_speed) 
            {
                closed_loop.target_speed = 0;
            }
        }
    }
    
    
    //�޷�
    if(closed_loop.target_speed > motor_control.max_speed)
    {
        closed_loop.target_speed = motor_control.max_speed;
    }
    if(closed_loop.target_speed < motor_control.min_speed)
    {
        closed_loop.target_speed = motor_control.min_speed;
    }
}


void motor_set_dir(void)
{
    //���ٶ�Ϊ0��ʱ��ż���Ƿ���Ҫ�л�����
    if(speed_filter.data_average == 0)
    {
        if(!pwm_dir_get())
        {
            motor_control.dir = FORWARD;
        }
        else
        {
            motor_control.dir = REVERSE;
        }
    }
    
    //������ʵ�����еķ�����Ϣ
    motor_dir_out();
}
extern int16 break_time_num;
//-------------------------------------------------------------------------------------------------------------------
//  @brief      ɲ��
//  @param      void
//  @return     void					
//  @since      
//-------------------------------------------------------------------------------------------------------------------
void motor_brake(void)
{
	  if(break_time_num<=break_time)
		{
			PWMUVWH_OFF_UVWL_ON;//���ɲ��ʹ��������ȫ��������ȫ��
		}
		else if(break_time_num>break_time)
		{
			if(break_time_num>(break_time_duty*break_time))
				break_time_num=0;
			PWMUVWH_OFF_UVWL_OFF;
		}
    TIM1->CCR1 = 0;
    TIM1->CCR2 = 0;
    TIM1->CCR3 = 0;
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      ����ռ�ձ�
//  @param      void
//  @return     void					
//  @since      
//-------------------------------------------------------------------------------------------------------------------
void motor_duty_out(uint16 duty)
{
    TIM1->CCR1 = duty;
    TIM1->CCR2 = duty;
    TIM1->CCR3 = duty;
}
//-------------------------------------------------------------------------------------------------------------------
//  @brief      �������
//  @param      void
//  @return     void					
//  @since      
//-------------------------------------------------------------------------------------------------------------------
void motor_power_out(void)
{
    int16 duty;
    duty = closed_loop.out;

    if(motor_control.dir == FORWARD)
    {//�����ת
        if(0 > duty)
        {
            duty = 0;
        }
    }
    else
    {//�����ת
        if(0 >= duty)
        {
            duty = -duty;
        }
        else
        {
            duty = 0;
        }
    }
    
    
    if(0 == closed_loop.target_speed && motor_control.brake_flag)
    {//��ɲ����־λ������ʱ�������õ��ٶ�Ϊ0 ��ʱ��Ҫ����ɲ��
        motor_brake();
    }
    else if(MOTOR_DISABLE == motor_control.en_status)
    {//�����ʹ�ܿ��عرյ�ʱ����Ҫ����ɲ��
        motor_brake();
    }
    else
    {
        motor_duty_out(duty);
    }
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      ������ຯ��
//  @param      except_hall �����´λ�����ֵ
//  @return     void					
//  @since      
//-------------------------------------------------------------------------------------------------------------------
void motor_commutation(uint8 except_hall)
{
    if(0 == closed_loop.target_speed && motor_control.brake_flag)
    {
        motor_brake();
    }
    else if(MOTOR_DISABLE == motor_control.en_status)
    {
        motor_brake();
    }
    else
    {
        switch(except_hall)
        {
            case 1:
                PWMVH_ON_WL_ON//1
                break;
            
            case 2:		
                PWMUH_ON_VL_ON//2
                break;
            
            case 3:		
                PWMUH_ON_WL_ON//3
                break;
            
            case 4:		
                PWMWH_ON_UL_ON//4
                break;
            
            case 5:		
                PWMVH_ON_UL_ON//5
                break;
            
            case 6:		
                PWMWH_ON_VL_ON//6
                break;

            default:
                PWMH_OFF_PWML_OFF
                break;
        }
    }
    
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      ��ʱ��2���PWM��ʼ��
//  @param      void 
//  @return     void					
//  @since      
//-------------------------------------------------------------------------------------------------------------------
void tim2_pwm_init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
    TIM_OCInitTypeDef  TIM_OCInitStructure;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);

    GPIO_InitStructure.GPIO_Pin = MOTOR_SPEED_OUT_PIN;          
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;     
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(MOTOR_SPEED_OUT_PORT, &GPIO_InitStructure);
    GPIO_PinAFConfig(MOTOR_SPEED_OUT_PORT, GPIO_PinSource12, GPIO_AF_4);

    TIM_TimeBaseStructure.TIM_Period = 1000;          
    TIM_TimeBaseStructure.TIM_Prescaler = 8-1;         
    TIM_TimeBaseStructure.TIM_ClockDivision = 0;        
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; 
    TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);     

    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2;   
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse = 0;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low; 
    TIM_OC2Init(TIM2, &TIM_OCInitStructure);

    TIM_OC2PreloadConfig(TIM2, TIM_OCPreload_Enable);
    TIM_ARRPreloadConfig(TIM2, ENABLE);      

    TIM_Cmd(TIM2, ENABLE);                    
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      ���ʹ��״̬��ѯ
//  @param      frequency   ��Ҫ���õ�Ƶ�� 
//  @return     void					
//  @since      
//-------------------------------------------------------------------------------------------------------------------
void tim2_pwm_frequency(uint16 frequency)
{
    uint16 speed_out_period;

    if(0 == frequency)
    {
        TIM_SetCompare2(TIM2, 0);           //Ƶ��Ϊ0��ʱ�򣬽�ռ�ձ�����Ϊ0����
    }
    else
    {
        speed_out_period = 96000000/8/frequency;      //����Ƶ�ʵ�����ֵ
        TIM_SetAutoreload(TIM2, speed_out_period);    //��������ֵ
        TIM_SetCompare2(TIM2, speed_out_period/2);    //����ռ�ձ� 
    }
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      ����ٶ����
//  @param      void 
//  @return     void					
//  @since      ÿ�λ����ʱ��תIO���ⲿ�������ñ������ӿ�ֱ�Ӳɼ�����
//              �ٶ����������ⲿ�������������ӿڵ�Aͨ�� ������������Bͨ��
//-------------------------------------------------------------------------------------------------------------------
void motor_speed_out(void)
{
    uint32 temp;
    
    if(REVERSE == motor_control.dir)
    {
        temp = -speed_filter.data_average;
    }
    else
    {
        temp = speed_filter.data_average;
    }
    
    if(65535 < temp)    
    {
        temp = 65535;
    }
    
    tim2_pwm_frequency((uint16)temp);
//    static uint8 speed_temp;
//    speed_temp = !speed_temp;
//    GPIO_WriteBit(MOTOR_SPEED_OUT_PORT, MOTOR_SPEED_OUT_PIN, (BitAction)speed_temp);
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      ���ת���������
//  @param      void 
//  @return     void					
//  @since      
//-------------------------------------------------------------------------------------------------------------------
void motor_dir_out(void)
{
    GPIO_WriteBit(MOTOR_DIR_OUT_PORT, MOTOR_DIR_OUT_PIN, (BitAction)motor_control.dir);
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      ���ʹ��״̬��ѯ
//  @param      void 
//  @return     void					
//  @since      
//-------------------------------------------------------------------------------------------------------------------
void motor_en(void)
{
    if(GPIO_ReadInputDataBit(MOTOR_EN_STATUS_PORT, MOTOR_EN_STATUS_PIN))
    {
        motor_control.en_status = MOTOR_DISABLE;
    }
    else
    {
        motor_control.en_status = MOTOR_ENABLE;
    }
}



//-------------------------------------------------------------------------------------------------------------------
//  @brief      ����ٶ��뷽����� ��ʼ��
//  @param      void 
//  @return     void					
//  @since      
//-------------------------------------------------------------------------------------------------------------------
void motor_information_out_init(void)
{
    GPIO_InitTypeDef  GPIO_InitStructure;
	
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB|RCC_AHBPeriph_GPIOD, ENABLE); 
    
    tim2_pwm_init();
//    GPIO_InitStructure.GPIO_Pin  =  MOTOR_SPEED_OUT_PIN;
//	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
//	GPIO_Init(MOTOR_SPEED_OUT_PORT, &GPIO_InitStructure);
    
    GPIO_InitStructure.GPIO_Pin  =  MOTOR_DIR_OUT_PIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(MOTOR_DIR_OUT_PORT, &GPIO_InitStructure);
    
    GPIO_InitStructure.GPIO_Pin  =  MOTOR_EN_STATUS_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
	GPIO_Init(MOTOR_EN_STATUS_PORT, &GPIO_InitStructure);
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      ����ٶ����߳�ʼ��
//  @param      void 
//  @return     void					
//  @since      
//-------------------------------------------------------------------------------------------------------------------
void motor_speed_curve_init(void)
{
    #if BLDC_BRAKE_ENABLE==1
        motor_control.brake_flag = 1;   //ɲ��ʹ��
    #else
        motor_control.brake_flag = 0;   //ɲ���ر�
    #endif
    motor_control.dir = FORWARD;                    //����Ĭ�ϵķ���
    
    motor_control.set_speed = 0;
    #if(BLDC_CLOSE_LOOP_ENABLE)
    motor_control.max_speed = BLDC_MAX_SPEED;       //���������ת�ٶ�
    motor_control.min_speed = -BLDC_MAX_SPEED;      //�������ת�ٶ�
    motor_control.increase_step = BLDC_STEP_SPEED;  //���ü���ʱ�Ĳ���ֵ
    #endif
}


