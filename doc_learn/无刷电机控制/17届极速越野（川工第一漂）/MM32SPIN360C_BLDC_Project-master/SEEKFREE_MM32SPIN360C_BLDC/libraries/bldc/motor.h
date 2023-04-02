
#ifndef _motor_H
#define _motor_H


#include "common.h"
#include "pid.h"




//������ת���ٶ��������  ���Խ����������Ĳ���Ƶ��Խ�� �����Ų����������
#define MOTOR_SPEED_OUT_PORT    GPIOA
#define MOTOR_SPEED_OUT_PIN     GPIO_Pin_12 

//������ת�������������      0�������ת  1�������ת   ���ڸ�֪�ⲿ����ģ������ǰ�����з���
#define MOTOR_DIR_OUT_PORT      GPIOD
#define MOTOR_DIR_OUT_PIN       GPIO_Pin_2

//������ʹ�ܿ�������          0�����ֹͣ����  1��������������ź�������ת
#define MOTOR_EN_STATUS_PORT    GPIOB
#define MOTOR_EN_STATUS_PIN     GPIO_Pin_14











//========PWM1 2 3ͨ�����ʹ��λ����===================================== 
#define     PWMU_Enb            TIM1->CCER |= (uint16_t)(((uint16_t)TIM_CCER_CC1E)) 
#define     PWMU_Dis            TIM1->CCER &= (uint16_t)(~((uint16_t)TIM_CCER_CC1E)) 
#define     PWMV_Enb            TIM1->CCER |= (uint16_t)(((uint16_t)TIM_CCER_CC2E)) 
#define     PWMV_Dis            TIM1->CCER &= (uint16_t)(~((uint16_t)TIM_CCER_CC2E)) 
#define     PWMW_Enb            TIM1->CCER |= (uint16_t)(((uint16_t)TIM_CCER_CC3E)) 
#define     PWMW_Dis            TIM1->CCER &= (uint16_t)(~((uint16_t)TIM_CCER_CC3E))  
        
#define     PWMUVW_Enb          TIM1->CCER |= (uint16_t)(((uint16_t)(TIM_CCER_CC1E|TIM_CCER_CC2E|TIM_CCER_CC3E)))
#define     PWMUVW_Dis          TIM1->CCER &= (uint16_t)(~((uint16_t)(TIM_CCER_CC1E|TIM_CCER_CC2E|TIM_CCER_CC3E)))  
#define     PWMUVWN_Enb         TIM1->CCER |= (uint16_t)(((uint16_t)(TIM_CCER_CC1NE|TIM_CCER_CC2NE|TIM_CCER_CC3NE)))
#define     PWMUVWN_Dis         TIM1->CCER &= (uint16_t)(~((uint16_t)(TIM_CCER_CC1NE|TIM_CCER_CC2NE|TIM_CCER_CC3NE))) 

//========PWM1 2 3ͨ���������ʹ��λ����===================================== 
#define     UL_ON               GPIO_SetBits(GPIOB,GPIO_Pin_6);
#define     VL_ON               GPIO_SetBits(GPIOB,GPIO_Pin_4);
#define     WL_ON               GPIO_SetBits(GPIOA,GPIO_Pin_15);
#define     UL_OFF              GPIO_ResetBits(GPIOB,GPIO_Pin_6);
#define     VL_OFF              GPIO_ResetBits(GPIOB,GPIO_Pin_4);
#define     WL_OFF              GPIO_ResetBits(GPIOA,GPIO_Pin_15);
    
#define     PWMUH_ON_VL_ON		UL_OFF; WL_OFF; PWMUVW_Dis; PWMU_Enb; VL_ON;  //ע��I/O����ʱ�������ʱ��Ĺرմ�
#define     PWMUH_ON_WL_ON		UL_OFF; VL_OFF; PWMUVW_Dis; PWMU_Enb; WL_ON;
#define     PWMVH_ON_WL_ON  	UL_OFF; VL_OFF; PWMUVW_Dis; PWMV_Enb; WL_ON;
#define     PWMVH_ON_UL_ON   	VL_OFF; WL_OFF; PWMUVW_Dis; PWMV_Enb; UL_ON;
#define     PWMWH_ON_UL_ON   	VL_OFF; WL_OFF; PWMUVW_Dis; PWMW_Enb; UL_ON;
#define     PWMWH_ON_VL_ON   	UL_OFF; WL_OFF; PWMUVW_Dis; PWMW_Enb; VL_ON;
#define     PWMUVWH_OFF_UVWL_ON PWMUVW_Dis; UL_ON; VL_ON; WL_ON;

#define     PWMUVWH_OFF_UVWL_OFF PWMUVW_Dis; UL_OFF; VL_OFF; WL_OFF;  //������ȫ��


#define     PWMH_OFF_PWML_ON    PWMUVW_Dis; 	PWMUVWN_Enb;
#define     PWMH_ON_PWML_OFF    PWMUVWN_Dis;	PWMUVW_Enb;
#define     PWMH_OFF_PWML_OFF   PWMUVWN_Dis;	PWMUVW_Dis;





typedef enum
{
    FORWARD,    //��ת
    REVERSE,    //��ת
}MOTOR_DIR_enum;

typedef enum
{
    MOTOR_DISABLE,  //�����ر�
    MOTOR_ENABLE,   //����ʹ��
}MOTOR_EN_STATUS_enum;

typedef struct
{
    MOTOR_EN_STATUS_enum en_status; //ָʾ���ʹ��״̬
    uint8 brake_flag;   //ָʾ��ǰɲ���Ƿ���Ч    1������ɲ��  0����������    
    MOTOR_DIR_enum  dir;//�����ת���� FORWARD����ת  REVERSE����ת     BRAKE��ɲ��
    int32 set_speed;    //���õ��ٶ�
    int32 max_speed;    //�ٶ����ֵ
    int32 min_speed;    //�ٶ���Сֵ
    int16 increase_step;//�ٶ����ӵĲ���ֵ  ���ٵ�ʱ��ֵԽ��set_speed��ֵ�ͻ�Խ��ƽ�except_speed
}motor_struct;


     



extern motor_struct motor_control;




void motor_speed_curve(void);
void motor_set_dir(void);
void motor_power_out(void);
void motor_commutation(uint8 except_hall);
void motor_speed_curve_init(void);

void tim2_pwm_frequency(uint16 frequency);
void motor_speed_out(void);
void motor_dir_out(void);
void motor_en(void);
void motor_information_out_init(void);



#endif
