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
//  @brief      速度曲线计算函数
//  @param      void
//  @return     void					
//  @since      
//-------------------------------------------------------------------------------------------------------------------
void motor_speed_curve(void)
{
    if(FORWARD == motor_control.dir)
    {   //设置的速度比闭环的目标速度大，则使用步进值将闭环目标速度逐步逼近设置速度
        if(motor_control.set_speed > closed_loop.target_speed)
        {
            closed_loop.target_speed += motor_control.increase_step;
            if(closed_loop.target_speed > motor_control.set_speed)
            {
                closed_loop.target_speed = motor_control.set_speed;
            }
        }
        //设置的速度比目标速度低，则不使用步进值
        else
        {
            closed_loop.target_speed = motor_control.set_speed;
            if(0 > closed_loop.target_speed) 
            {
                closed_loop.target_speed = 0;
            }
        }
    }
    else//反转
    {   //设置的速度比闭环的目标速度大，则使用步进值将闭环目标速度逐步逼近设置速度
        if(motor_control.set_speed < closed_loop.target_speed)
        {
            closed_loop.target_speed -= motor_control.increase_step;
            if(closed_loop.target_speed < motor_control.set_speed)
            {
                closed_loop.target_speed = motor_control.set_speed;
            }
        }
        //设置的速度比目标速度低，则不使用步进值
        else
        {
            closed_loop.target_speed = motor_control.set_speed;
            if(0 < closed_loop.target_speed) 
            {
                closed_loop.target_speed = 0;
            }
        }
    }
    
    
    //限幅
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
    //当速度为0的时候才检测是否需要切换方向
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
    
    //输出电机实际运行的方向信息
    motor_dir_out();
}
extern int16 break_time_num;
//-------------------------------------------------------------------------------------------------------------------
//  @brief      刹车
//  @param      void
//  @return     void					
//  @since      
//-------------------------------------------------------------------------------------------------------------------
void motor_brake(void)
{
	  if(break_time_num<=break_time)
		{
			PWMUVWH_OFF_UVWL_ON;//如果刹车使能则下桥全开，上桥全关
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
//  @brief      更新占空比
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
//  @brief      输出动力
//  @param      void
//  @return     void					
//  @since      
//-------------------------------------------------------------------------------------------------------------------
void motor_power_out(void)
{
    int16 duty;
    duty = closed_loop.out;

    if(motor_control.dir == FORWARD)
    {//电机正转
        if(0 > duty)
        {
            duty = 0;
        }
    }
    else
    {//电机反转
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
    {//当刹车标志位开启的时候并且设置的速度为0 此时需要进行刹车
        motor_brake();
    }
    else if(MOTOR_DISABLE == motor_control.en_status)
    {//当电机使能开关关闭的时候需要进行刹车
        motor_brake();
    }
    else
    {
        motor_duty_out(duty);
    }
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      电机换相函数
//  @param      except_hall 期望下次霍尔的值
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
//  @brief      定时器2输出PWM初始化
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
//  @brief      电机使能状态查询
//  @param      frequency   需要设置的频率 
//  @return     void					
//  @since      
//-------------------------------------------------------------------------------------------------------------------
void tim2_pwm_frequency(uint16 frequency)
{
    uint16 speed_out_period;

    if(0 == frequency)
    {
        TIM_SetCompare2(TIM2, 0);           //频率为0的时候，将占空比设置为0即可
    }
    else
    {
        speed_out_period = 96000000/8/frequency;      //计算频率的周期值
        TIM_SetAutoreload(TIM2, speed_out_period);    //设置周期值
        TIM_SetCompare2(TIM2, speed_out_period/2);    //设置占空比 
    }
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      电机速度输出
//  @param      void 
//  @return     void					
//  @since      每次换相的时候翻转IO，外部控制器用编码器接口直接采集即可
//              速度引脚连接外部控制器编码器接口的A通道 方向引脚连接B通道
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
//  @brief      电机转动方向输出
//  @param      void 
//  @return     void					
//  @since      
//-------------------------------------------------------------------------------------------------------------------
void motor_dir_out(void)
{
    GPIO_WriteBit(MOTOR_DIR_OUT_PORT, MOTOR_DIR_OUT_PIN, (BitAction)motor_control.dir);
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      电机使能状态查询
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
//  @brief      电机速度与方向输出 初始化
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
//  @brief      电机速度曲线初始化
//  @param      void 
//  @return     void					
//  @since      
//-------------------------------------------------------------------------------------------------------------------
void motor_speed_curve_init(void)
{
    #if BLDC_BRAKE_ENABLE==1
        motor_control.brake_flag = 1;   //刹车使能
    #else
        motor_control.brake_flag = 0;   //刹车关闭
    #endif
    motor_control.dir = FORWARD;                    //设置默认的方向
    
    motor_control.set_speed = 0;
    #if(BLDC_CLOSE_LOOP_ENABLE)
    motor_control.max_speed = BLDC_MAX_SPEED;       //设置最大正转速度
    motor_control.min_speed = -BLDC_MAX_SPEED;      //设置最大反转速度
    motor_control.increase_step = BLDC_STEP_SPEED;  //设置加速时的步进值
    #endif
}


