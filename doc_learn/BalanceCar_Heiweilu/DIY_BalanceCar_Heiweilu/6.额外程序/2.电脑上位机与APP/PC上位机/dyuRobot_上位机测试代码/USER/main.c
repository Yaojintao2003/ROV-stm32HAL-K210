 /**************************************************************************
 作  者 ：大鱼电子
淘宝地址：https://shop119207236.taobao.com
**************************************************************************/
#include "sys.h"
#include "DataScope_DP.h"

/****************************上位机变量*********************************/   
unsigned char i;          										 //计数变量
unsigned char Send_Count; 										 //串口需要发送的字节数
/**********************************************************************/

int main(void)	
{ 
	float j;
	delay_init();	    	           //=====延时函数初始化	
	NVIC_Configuration();					 //=====中断优先级分组
	uart1_init(128000);	           //=====串口1初始化
	LED_Init();                    //=====初始化与 LED 连接的IO
	KEY_Init();                    //=====按键初始化
	OLED_Init();                   //=====OLED初始化
	OLED_Clear();									 //=====OLED清屏
  while(1)	
	{
			j+=0.1;
			if(j>3.14)  j=-3.14; 
			DataScope_Get_Channel_Data(10*sin(j), 1 );
			DataScope_Get_Channel_Data(10*cos(j), 2 );
			DataScope_Get_Channel_Data(2*j, 3 );
			Send_Count=DataScope_Data_Generate(3);
			for( i = 0 ; i < Send_Count; i++) 
			{
			while((USART1->SR&0X40)==0);  
			USART1->DR = DataScope_OutPut_Buffer[i]; 
			}
			delay_ms(50); //20HZ 
	} 	
}

