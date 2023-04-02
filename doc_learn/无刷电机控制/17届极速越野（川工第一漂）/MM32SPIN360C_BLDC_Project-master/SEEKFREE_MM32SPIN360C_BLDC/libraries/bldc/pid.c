
#include "timer.h"
#include "motor.h"
#include "pid.h"


closed_loop_struct closed_loop;



//-------------------------------------------------------------------------------------------------------------------
//  @brief      ����ֵ����
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
//  @brief      PI�ջ�����
//  @param      read_speed  ��ǰ�ٶ� 
//  @return     void					
//  @since      
//-------------------------------------------------------------------------------------------------------------------
void closed_loop_pi_calc(int32 read_speed)
{
    closed_loop.real_speed = read_speed;
    
    closed_loop.error = closed_loop.target_speed - closed_loop.real_speed;

    closed_loop.pout = closed_loop.error * (closed_loop.kp + (float)myabs(closed_loop.error/1000)/1800);
    
    //����ϵ�����������ж�̬����
    closed_loop.iout += closed_loop.error * (closed_loop.ki + (float)myabs(closed_loop.error/1000)/38000);
    
    //�����޷�
    closed_loop.iout = limit_ab(closed_loop.iout, -closed_loop.out_max, closed_loop.out_max);

    //���Ŀ���ٶ�Ϊ0���ߵ�����ر����������
    if((0 == closed_loop.target_speed )|| (MOTOR_DISABLE == motor_control.en_status))
    {
        closed_loop.iout = 0;
    }
    
    //��ת��ʱ��Ի��ֽ����޷�
    if(0 < closed_loop.target_speed)
    {
        closed_loop.iout = limit_ab(closed_loop.iout, (float)closed_loop.target_speed/motor_control.max_speed*closed_loop.out_max/2, closed_loop.out_max);
    }
    //��ת��ʱ��Ի��ֽ����޷�
    else if(0 > closed_loop.target_speed)
    {
        closed_loop.iout = limit_ab(closed_loop.iout, -closed_loop.out_max, (float)closed_loop.target_speed/motor_control.max_speed*closed_loop.out_max/2);
    }

    closed_loop.out = closed_loop.iout + closed_loop.pout;
    
    //����޷�
    closed_loop.out = limit_ab(closed_loop.out, -closed_loop.out_max, closed_loop.out_max);
}


//-------------------------------------------------------------------------------------------------------------------
//  @brief      PI�ջ������ʼ��
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
