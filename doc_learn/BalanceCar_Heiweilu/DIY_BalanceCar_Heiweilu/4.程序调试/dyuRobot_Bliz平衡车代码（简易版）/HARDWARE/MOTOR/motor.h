#ifndef __MOTOR_H
#define __MOTOR_H
#include <sys.h>	

 /**************************************************************************
 作  者 ：大鱼电子
淘宝地址：https://shop119207236.taobao.com
**************************************************************************/

#define PWMA   TIM1->CCR1  //PA8

#define AIN2   PBout(15)
#define AIN1   PBout(14)
#define BIN1   PBout(13)
#define BIN2   PBout(12)

#define PWMB   TIM1->CCR4  //PA11

void Motor_Init(void);
void Set_Pwm(int moto1,int moto2);
int myabs(int a);
void Xianfu_Pwm(void);
void Turn_Off(float angle);
#endif
