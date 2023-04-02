 /**************************************************************************
 作  者 ：大鱼电子
淘宝地址：https://shop119207236.taobao.com
**************************************************************************/
#include "sys.h"
#include "control.h"
//extern int Balance_Pwm ;
//extern  int Encoder_TIM ;
//////////////////////////全局变量的定义////////////////////////////////////      
float pitch,roll,yaw; 								  			 //欧拉角(姿态角)-----俯仰角 翻滚角 偏航角
short aacx,aacy,aacz;													 //加速度传感器原始数据
short gyrox,gyroy,gyroz;											 //陀螺仪原始数据
int   Encoder_Left,Encoder_Right;         		 //左右编码器的脉冲计数
int 	Moto1=0,Moto2=0;												 //计算出来的最终赋给电机的PWM	
int main(void)	
{ 
	delay_init();	    	           //=====延时函数初始化	
	NVIC_Configuration();					 //=====中断优先级分组,其中包含了所有的中断优先级的配置,方便管理和一次性修改。
	uart1_init(9600);	             //=====串口1初始化
	Encoder_Init_TIM2();           //=====初始化编码器2
	Encoder_Init_TIM4();          //=====初始化编码器4
	OLED_Init();                   //=====OLED初始化
	OLED_Clear();									 //=====OLED清屏
	MPU_Init();					    			 //=====初始化MPU6050
	mpu_dmp_init();								 //=====初始化MPU6050的DMP模式					 
	TIM1_PWM_Init(7199,0);   			 //=====初始化PWM 10KHZ,用于驱动电机。
	delay_ms(1000);								 //=====延时2s 解决小车上电轮子乱转的问题
	delay_ms(1000);								 //=====延时2s 解决小车上电轮子乱转的问题
	MPU6050_EXTI_Init();					 //=====MPU6050 5ms定时中断初始化
	Motor_Init();									 //=====初始化与电机连接的硬件IO接口
  OLED_ShowString(0,0,"ANGLE:",12);						 
  while(1)	
	{	
			delay_ms(100);
			OLED_Float(0,48,pitch,3);	 //显示角度
	   //OLED_Float(1,48,Balance_Pwm,3);  
      OLED_Float(1,48,Moto1,0);
		 //  OLED_Float(2,48,Moto2,3);//显示电机pwm
	   //	OLED_Float(1,48,Encoder_TIM,0);
	
	} 	
}



