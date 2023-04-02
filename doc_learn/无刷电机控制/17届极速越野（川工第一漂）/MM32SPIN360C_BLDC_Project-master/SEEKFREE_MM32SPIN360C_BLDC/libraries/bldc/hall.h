
#ifndef _hall_H
#define _hall_H

#include "timer.h"
#include "common.h"

//������������
#define	POLEPAIRS		    2

//����һ������ADC�жϴ���
#define ADC_NUM_PM          60*FPWM

#define COMMUTATION_TIMEOUT 5000

#define HALL_PORT           GPIOA
#define HALL_A_PIN          GPIO_Pin_0
#define HALL_B_PIN          GPIO_Pin_1
#define HALL_C_PIN          GPIO_Pin_2



extern uint8 hall_value_now; //��ǰ������ֵ
extern uint8 hall_value_last;//�ϴλ�����ֵ

extern uint8 next_hall_value;

extern uint32 commutation_delay;
extern uint16 commutation_time;
extern uint16 commutation_time_save[6];
extern uint32 commutation_time_sum;

extern void scan_hall_status(void);
extern void read_hall_value(void);
extern void hall_init(void);
#endif
