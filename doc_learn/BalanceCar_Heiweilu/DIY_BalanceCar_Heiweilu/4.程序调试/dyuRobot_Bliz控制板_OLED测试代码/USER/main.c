 /**************************************************************************
 作  者 ：大鱼电子
淘宝地址：https://shop119207236.taobao.com
**************************************************************************/
#include "sys.h"
/***************OLED的IO接口**************************
								GND   电源地
								VCC   接5V或3.3v电源
								SCL   接PB8（SCL）
								SDA   接PB9（SDA） 
****************OLED的IO接口**************************/
int main(void)	
{ 
	delay_init();	    	           //=====延时函数初始化	
	NVIC_Configuration();					 //=====中断优先级分组
	uart1_init(9600);	             //=====串口1初始化
	LED_Init();                    //=====初始化与 LED 连接的IO
	KEY_Init();                    //=====按键初始化
	OLED_Init();                   //=====OLED初始化
	OLED_Clear();									 //=====OLED清屏
  while(1)	
	{
			delay_ms(50);				 			 					//=====50ms刷新一次屏幕即可,不需要一直刷新。
			OLED_ShowString(0,4,"DAYUTC OLED Test",12);
			LED=~LED;
	} 	
}

