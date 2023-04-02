#include "exti.h"
/**************************************************************************
作    者：大鱼电子
淘宝地址：https://shop119207236.taobao.com
**************************************************************************/

/**************************************************************************
函数功能：外部中断初始化
入口参数：无
返回  值：无 
作    用：是用来配置MPU6050引脚INT的，每当MPU6050有数据输出时，引脚INT有相应的电平输出。
					依次来触发外部中断作为控制周期。保持MPU6050数据的实时性。
**************************************************************************/
void MPU6050_EXTI_Init(void)
{  
		GPIO_InitTypeDef GPIO_InitStructure;
		EXTI_InitTypeDef EXTI_InitStructure;
		NVIC_InitTypeDef NVIC_InitStructure;
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);//外部中断，需要使能AFIO时钟
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE); //使能PB端口时钟
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;	            //端口配置
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;         //上拉输入
		GPIO_Init(GPIOB, &GPIO_InitStructure);					      //根据设定参数初始化GPIOB 
  	GPIO_EXTILineConfig(GPIO_PortSourceGPIOB,GPIO_PinSource5);
  	EXTI_InitStructure.EXTI_Line=EXTI_Line5;
  	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;	
  	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;//下降沿触发
  	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  	EXTI_Init(&EXTI_InitStructure);	 	//根据EXTI_InitStruct中指定的参数初始化外设EXTI寄存器
	
		//////////////////外部中断5优先级配置也就是MPU6050 INT引脚的配置///////////因为是控制中断，故此优先级应是最高。
		NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;				//使能外部中断通道
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x00;	//抢占优先级0， 
		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x01;					//子优先级1
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;								//使能外部中断通道
		NVIC_Init(&NVIC_InitStructure); 
}










