#ifndef __CONTROL_H
#define __CONTROL_H
#include "sys.h"
 /**************************************************************************
 作  者 ：大鱼电子
淘宝地址：https://shop119207236.taobao.com
**************************************************************************/
#define PI 3.14159265
void EXTI9_5_IRQHandler(void);
int balance_UP(float Angle,float Mechanical_balance,float Gyro);
int velocity(int encoder_left,int encoder_right);
int turn(int encoder_left,int encoder_right,float gyro);
#endif
