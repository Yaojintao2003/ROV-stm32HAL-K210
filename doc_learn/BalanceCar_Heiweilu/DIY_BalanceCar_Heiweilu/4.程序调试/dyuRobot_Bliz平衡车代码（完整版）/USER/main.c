 /**************************************************************************
 作  者 ：大鱼电子
淘宝地址：https://shop119207236.taobao.com
**************************************************************************/
#include "sys.h"
/****************************全局变量*************************************/    
float Voltage;  															 //电池电压采样相关的变量
float pitch,roll,yaw; 								  			 //欧拉角(姿态角)
short aacx,aacy,aacz;													 //加速度传感器原始数据
short gyrox,gyroy,gyroz;											 //陀螺仪原始数据
float UltrasonicWave_Distance;                 //超声波测距
int   Encoder_Left,Encoder_Right;         		 //左右编码器的脉冲计数
int 	Moto1=0,Moto2=0;												 //计算出来的最终赋给电机的PWM
int 	Flag_Qian,Flag_Hou,Flag_Left,Flag_Right; //蓝牙遥控相关的变量
u8    flag_UltrasonicWave=0;									 //超声波是否超出安全距离标志位
u8    flag_fall=0;					 									 //摔倒标志位
u8    key=0;								 									 //按键的键值
u8    KEY_MODE=0;					 										 //模式0,显示名字===HELLO BLIZ
																							 //模式1,电量模式
																							 //模式2,表情模式
																							 //模式3,参数模式
u8 DIS_STATE=255;				 											 //用来实现只刷新一次屏幕的变量			
/***********************************************************************/
int main(void)	
{ 
	LED_Init();                    //=====初始化与 LED 连接的IO
	USB_Init();										 //=====初始化与 USB 连接的IO
	KEY_Init();                    //=====初始化与按键连接的IO
	delay_init();	    	           //=====延时函数初始化	
	uart1_init(128000);	           //=====串口1初始化
	uart3_init(9600);              //=====串口3初始化成能与蓝牙进行通信的波特率,蓝牙默认通信波特率9600
	delay_ms(100);
/*****************修改蓝牙的默认通信波特率以及蓝牙默认的名字******************/
	Uart3SendStr("AT\r\n");
	Uart3SendStr("AT+NAMEdyuBliz\r\n");//发送蓝牙模块指令--设置名字为：Bliz
	delay_ms(100);	
	Uart3SendStr("AT+BAUD8\r\n"); 		 //发送蓝牙模块指令,将波特率设置成115200
	delay_ms(100);		
/*****************************************************************************/		
	uart3_init(115200);            //=====初始化串口3
	NVIC_Configuration();					 //=====中断优先级分组,其中包含了所有的中断优先级的配置,方便管理和一次性修改。
	Adc_Init();                    //=====初始化ADC
	Encoder_Init_TIM2();           //=====初始化编码器2
	Encoder_Init_TIM4();           //=====初始化编码器4
	Timer3_Init(5000,7199);	    	 //=====超声波定时器初始化
	OLED_Init();                   //=====OLED初始化
	OLED_Clear();									 //=====OLED清屏
	MPU_Init();					    			 //=====初始化MPU6050
	mpu_dmp_init();								 //=====初始化MPU6050的DMP模式					 
	UltrasonicWave_Configuration();//=====初始化超声波的硬件IO口
	TIM1_PWM_Init(7199,0);   			 //=====初始化PWM 10KHZ,用于驱动电机。 
	delay_ms(1000);								 //=====延时2s 解决小车上电轮子乱转的问题
	delay_ms(1000);								 //=====延时2s 解决小车上电轮子乱转的问题
	Motor_Init();									 //=====初始化与电机连接的硬件IO接口 
	MPU6050_EXTI_Init();					 //=====MPU6050 5ms定时中断初始化
  while(1)	
	{
			KEY_mode_Select();				 //=====模式选择以及屏幕表情显示
	} 	
}



