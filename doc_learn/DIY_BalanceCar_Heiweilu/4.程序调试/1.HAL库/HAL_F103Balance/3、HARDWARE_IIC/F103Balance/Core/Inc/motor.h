#ifndef __MOTOR_H__
#define __MOTOR_H__

#include "main.h"

#define Ain1  PBout(14)
#define Ain2  PBout(15)
#define Bin1  PBout(13)
#define Bin2  PBout(12)

void Motor_Start(void);
void Motor_Rotaton(int MotorA,int MotorB);
int GFP_abs(int a);
void Limit(int *motoA,int *motoB);
void Stop(float angle, float voltage);
void Encoder_Start(void);    //开启编码器
int Read_Speed(int TIMx);   //从编码器读取电机转速数据
void Encoder_IRQHandler(void);  //清中断标志.

#endif
