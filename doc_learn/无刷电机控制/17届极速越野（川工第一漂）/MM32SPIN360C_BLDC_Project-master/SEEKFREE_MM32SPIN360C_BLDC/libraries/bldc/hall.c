
#include "HAL_conf.h"
#include "move_filter.h"
#include "motor.h"
#include "gpio.h"
#include "hall.h"



uint8 hall_value_now;     //��ǰ������ֵ
uint8 hall_value_last;    //�ϴλ�����ֵ

uint8 next_hall_value;    //��һ�λ�����ֵ

//����Ӳ��˳��Ϊ2 3 1 5 4 6
uint8 hall_steps_normal[2][8] = 
{
    //�����е�0��û�����ã�ֻ������ռλ
    {0,3,6,2,5,1,4,0},  //��ת��ʱ�����˳��
    {0,5,3,1,6,4,2,0},  //��ת��ʱ�����˳��
};

uint8 hall_steps_advance[2][8] = 
{
    //�����е�0��û�����ã�ֻ������ռλ
    {0,2,4,6,1,3,5,0},  //��ת��ʱ�򣬳�ǰ�������˳��
    {0,4,1,5,2,6,3,0},  //��ת��ʱ�򣬳�ǰ�������˳��
};

uint8 hall_steps_break[8] = 
{
    //�����е�0��û�����ã�ֻ������ռλ
    0,1,2,3,4,5,6,0,  //��ת��ʱ�򣬳�ǰ�������˳��
};

uint8  hall_index = 0;
uint16 commutation_time_save[6] = {COMMUTATION_TIMEOUT, COMMUTATION_TIMEOUT, COMMUTATION_TIMEOUT, COMMUTATION_TIMEOUT, COMMUTATION_TIMEOUT, COMMUTATION_TIMEOUT};  //����ʱ�� ����6��
uint32 commutation_time_sum = 0;    //���λ���ʱ���ܺ�
uint16 commutation_time = COMMUTATION_TIMEOUT;    //ͳ�Ʊ��λ�������ʱ��
uint32 commutation_delay = COMMUTATION_TIMEOUT;   //�ӳٻ�����ʱʱ��
uint16 commutation_delay_ratio = 3; //������ʱʱ�� = commutation_delay_ratio*commutation_time_sum>>7 ��ֵԽС����Խ��ǰ


//-------------------------------------------------------------------------------------------------------------------
//  @brief      ��ȡ��ǰ����ֵ
//  @param      void   
//  @return     void					
//  @since      
//-------------------------------------------------------------------------------------------------------------------
void read_hall_value(void)
{
	uint8 hall_a;
	uint8 hall_b;
	uint8 hall_c;
	
	hall_a = GPIO_ReadInputDataBit(HALL_PORT, HALL_A_PIN);
	hall_b = GPIO_ReadInputDataBit(HALL_PORT, HALL_B_PIN);
	hall_c = GPIO_ReadInputDataBit(HALL_PORT, HALL_C_PIN);
	hall_value_now = hall_a*4 + hall_b*2 + hall_c; 
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      ���㵱ǰ���ٶ�
//  @param      void   
//  @return     void					
//  @since      
//-------------------------------------------------------------------------------------------------------------------
void calc_speed(void)
{
    //ת�ټ���
    uint8 i;
    int32 speed;
    commutation_time_sum = 0;
    for(i=0;i<6;i++)
    {
        commutation_time_sum += commutation_time_save[i];
    }
    
    //���ת�ټ���˵��
    //2.commutation_time_sum��ͳ�Ƶ������6�λ������ٴ�ADC�ж�
        //2.1 ����ADC�Ƕ�ʱ��1��PWM���ڴ����ģ���ΪADC�ж�Ƶ����PWM������һ����Ƶ��40khz
        //2.2 �������ù�ϵADC1_IRQHandler->scan_hall_status->calc_speed
        //2.3 commutation_timeΪͳ�ƻ���ʱ��ı���
    //3.ͨ�����ת�����Ƕ���RPM��ʾ��RPM��ʾÿ���ӵ����ת��
    //3.���תһȦ��Ҫ����Ĵ������� ���������*6
    //4.��˵��ת�ٵ���60*ADC�ж�Ƶ��/���������/commutation_time_sum���������Եõ����ÿ���ӵ�ת��
    
    speed = ADC_NUM_PM/POLEPAIRS/commutation_time_sum;
    
    if(REVERSE == motor_control.dir)//�����ת��ʱ����Ҫ���ٶ�ȡ��
    {
        speed = -speed;
    }
    move_filter_calc(&speed_filter, speed);
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      ������ֵ
//  @param      void   
//  @return     void					
//  @since      ������ֵ�������ٶȼ��㺯�����ó��´������Ļ���ֵ
//-------------------------------------------------------------------------------------------------------------------
void scan_hall_status(void)
{
	read_hall_value();
	
    commutation_time++;

    //���೬ʱ����
    //������೬����250ms����Ϊ���ֶ�ת��Ӧ�ü�ʱ���ٶ�����Ϊ0
    if(commutation_time >= COMMUTATION_TIMEOUT)		
	{
		commutation_time = COMMUTATION_TIMEOUT;
		commutation_time_save[0] = COMMUTATION_TIMEOUT;
		commutation_time_save[1] = COMMUTATION_TIMEOUT;
		commutation_time_save[2] = COMMUTATION_TIMEOUT;
		commutation_time_save[3] = COMMUTATION_TIMEOUT;
		commutation_time_save[4] = COMMUTATION_TIMEOUT;
		commutation_time_save[5] = COMMUTATION_TIMEOUT;
		
        //����ƽ���˲���ʼ��
        move_filter_init(&speed_filter);

        //������ٶ�����
        tim2_pwm_frequency(0);
	}
    
    if( (hall_value_now != hall_value_last) &&
        !commutation_delay)
    {
        
        hall_index++;
        if(hall_index >= 6)
        {
            hall_index = 0;
        }
        commutation_time_save[hall_index] = commutation_time;
        commutation_time = 0;
        
        //ת���ٶ�
        calc_speed();
        
        //ÿ����ɻ��࣬�ٶ�������ŷ�תһ�ε�ƽ״̬
        motor_speed_out();

        if((speed_filter.data_average > 4000) || (speed_filter.data_average < -4000))
        {
            //�ٶȴ���һ����ʱ�򣬻����ӳٽϴ�
            //��˲��ó�ǰ�������ʱ�ķ�ʽȥƥ����ѵĻ����
            next_hall_value = hall_steps_advance[motor_control.dir][hall_value_now];
            commutation_delay = (commutation_delay_ratio*commutation_time_sum)>>7;
        }
        else
        {
            //�ٶȽϵ����賬ǰ����
            next_hall_value = hall_steps_normal[motor_control.dir][hall_value_now];
            commutation_delay = 0;
        }
        
        hall_value_last = hall_value_now;
	}
}

void hall_init(void)
{
    GPIO_InitTypeDef  GPIO_InitStructure;
	
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE); 
    
    GPIO_InitStructure.GPIO_Pin  =  HALL_A_PIN | HALL_B_PIN | HALL_C_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_Init(HALL_PORT, &GPIO_InitStructure);
    
    //��ȡһ�µ�ǰ�Ļ���ֵ
    read_hall_value();
}

