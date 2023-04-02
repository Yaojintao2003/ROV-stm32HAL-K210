
#include "timer.h"
#include "motor.h"
#include "pid.h"


closed_loop_struct closed_loop;



//-------------------------------------------------------------------------------------------------------------------
//  @brief      绝对值函数
//  @param      void 
//  @return     void					
//  @since      
//-------------------------------------------------------------------------------------------------------------------
int32 myabs(int32 x)
{
    return (x>=0?x:-x);
}

float limit_ab(float dat, int a, int b)
{
    if(a > dat) return a;
    if(b < dat) return b;
    return dat;
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      PI闭环计算
//  @param      read_speed  当前速度 
//  @return     void					
//  @since      
//-------------------------------------------------------------------------------------------------------------------
void closed_loop_pi_calc(int32 read_speed)
{
    closed_loop.real_speed = read_speed;
    
    closed_loop.error = closed_loop.target_speed - closed_loop.real_speed;

    closed_loop.pout = closed_loop.error * (closed_loop.kp + (float)myabs(closed_loop.error/1000)/1800);
    
    //积分系数根据误差进行动态调节
    closed_loop.iout += closed_loop.error * (closed_loop.ki + (float)myabs(closed_loop.error/1000)/38000);
    
    //积分限幅
    closed_loop.iout = limit_ab(closed_loop.iout, -closed_loop.out_max, closed_loop.out_max);

    //如果目标速度为0或者电机被关闭则清除积分
    if((0 == closed_loop.target_speed )|| (MOTOR_DISABLE == motor_control.en_status))
    {
        closed_loop.iout = 0;
    }
    
    //正转的时候对积分进行限幅
    if(0 < closed_loop.target_speed)
    {
        closed_loop.iout = limit_ab(closed_loop.iout, (float)closed_loop.target_speed/motor_control.max_speed*closed_loop.out_max/2, closed_loop.out_max);
    }
    //反转的时候对积分进行限幅
    else if(0 > closed_loop.target_speed)
    {
        closed_loop.iout = limit_ab(closed_loop.iout, -closed_loop.out_max, (float)closed_loop.target_speed/motor_control.max_speed*closed_loop.out_max/2);
    }

    closed_loop.out = closed_loop.iout + closed_loop.pout;
    
    //输出限幅
    closed_loop.out = limit_ab(closed_loop.out, -closed_loop.out_max, closed_loop.out_max);
}


//-------------------------------------------------------------------------------------------------------------------
//  @brief      PI闭环计算初始化
//  @param      void   
//  @return     void					
//  @since      
//-------------------------------------------------------------------------------------------------------------------
void closed_loop_pi_init(void)
{
    closed_loop.out_max = PWM_PRIOD_LOAD;
    closed_loop.kp = 0.001;
    closed_loop.ki = 0.00001;
    closed_loop.out = 0;
    closed_loop.real_speed = 0;
}
