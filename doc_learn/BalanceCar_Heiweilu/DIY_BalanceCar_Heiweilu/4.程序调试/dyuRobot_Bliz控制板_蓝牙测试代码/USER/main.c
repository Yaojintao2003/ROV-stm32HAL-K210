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

/*关于蓝牙的IO口,本测试程序的蓝牙挂载在串口3上*/

u8 recieve_bluetooth_DATA=0;		 //	蓝牙接受数据标志

int main(void)	
{ 
	delay_init();	    	           //=====延时函数初始化	
	NVIC_Configuration();					 //=====中断优先级分组
	uart1_init(9600);	          	 //=====串口1初始化
	uart3_init(9600);              //=====串口3初始化成能与蓝牙进行通信的波特率,蓝牙默认通信波特率9600
	delay_ms(100);
/*****************修改蓝牙的默认通信波特率以及蓝牙默认的名字******************/
	Uart3SendStr("AT\r\n");
	Uart3SendStr("AT+NAMEdyuBliz\r\n");//发送蓝牙模块指令--设置名字为：Bliz
	delay_ms(100);	
	Uart3SendStr("AT+BAUD8\r\n"); 		 //发送蓝牙模块指令,将波特率设置成115200
	delay_ms(100);		
/*****************************************************************************/	
	uart3_init(115200);            //=====初始化串口3,此时的蓝牙默认通信波特率是115200
	LED_Init();                    //=====初始化与 LED 连接的IO
	KEY_Init();                    //=====按键初始化
	OLED_Init();                   //=====OLED初始化
	OLED_Clear();									 //=====OLED清屏
  while(1)	
	{
			delay_ms(50);						 	 //=====50ms刷一次屏幕,频率就是20HZ,不需要一直刷。
			OLED_ShowString(0,2,"Bluetooth_Test",12);
			OLED_ShowString(0,4,"Recieve:",12);
			OLED_Num2(12,4,recieve_bluetooth_DATA);
			printf("Recieve:%d\n",recieve_bluetooth_DATA);
		  /*也可以使用电脑上位机查看,如果没有购买OLED屏幕的客户打开电脑上位机,使用串口打印到上位机即可查看***/
			LED=~LED;										//表明程序一直处于运行中
	} 	
}

