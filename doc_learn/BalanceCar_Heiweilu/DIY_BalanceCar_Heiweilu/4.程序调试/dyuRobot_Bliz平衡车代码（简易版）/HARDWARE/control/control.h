#ifndef __CONTROL_H
#define __CONTROL_H
#include "sys.h"
 /**************************************************************************
 ��  �� ���������
�Ա���ַ��https://shop119207236.taobao.com
**************************************************************************/
void EXTI9_5_IRQHandler(void);
int balance_UP(float Angle,float Mechanical_balance,float Gyro);
int velocity(int encoder_left,int encoder_right);
#endif
