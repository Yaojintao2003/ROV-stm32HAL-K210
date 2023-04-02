 /**************************************************************************
 作  者 ：大鱼电子
淘宝地址：https://shop119207236.taobao.com
**************************************************************************/
#include "sys.h"

int	Encoder_Left,Encoder_Right; 

int main(void)	
{ 
	delay_init();	    	           //=====延时函数初始化	
	NVIC_Configuration();					 //=====中断优先级分组
	uart1_init(9600);	             //=====串口1初始化
	LED_Init();                    //=====初始化与 LED 连接的IO
	KEY_Init();                    //=====按键初始化
	OLED_Init();                   //=====OLED初始化
	OLED_Clear();									 //=====OLED清屏
	Encoder_Init_TIM2();					 //=====编码器2初始化
	Encoder_Init_TIM4();					 //=====编码器4初始化
	TIM3_Int_Init(99,7199);				 //=====10ms中断一次。电机全速运行时编码器最大值为90		
//	TIM3_Int_Init(49,7199);			   //=====5ms中断一次。电机全速运行时编码器最大值为45
  while(1)	
	{
			delay_ms(50);				 			 						//=====50ms刷一次屏幕即可,不需要一直刷新。
			OLED_ShowString(0,0,"DAYUTC--OLED",12);
			OLED_ShowString(0,4,"L: ",12);
			OLED_ShowString(60,4,"R: ",12);
			OLED_Num2(4,4,Encoder_Left);					//显示右边电机的编码器值
			OLED_Num2(14,4,Encoder_Right);				//显示左边电机的编码器值
			printf("Encoder_Left=%d,Encoder_Right=%d\n",Encoder_Left,Encoder_Right);
		  /*也可以使用电脑上位机查看,如果没有购买OLED屏幕的客户打开电脑上位机,使用串口打印到上位机即可查看***/
			LED=~LED;															//表明程序处于运行中				
	} 				
}
