#include "sys.h" 
 /**************************************************************************
 作  者 ：大鱼电子
淘宝地址：https://shop119207236.taobao.com
**************************************************************************/
//设置NVIC 
//NVIC_PreemptionPriority:抢占优先级
//NVIC_SubPriority       :响应优先级
//NVIC_Channel           :中断编号
//NVIC_Group             :中断分组 0~4
//注意优先级不能超过设定的组的范围!否则会有意想不到的错误
//组划分:
//组0:0位抢占优先级,4位响应优先级
//组1:1位抢占优先级,3位响应优先级
//组2:2位抢占优先级,2位响应优先级
//组3:3位抢占优先级,1位响应优先级
//组4:4位抢占优先级,0位响应优先级
//NVIC_SubPriority和NVIC_PreemptionPriority的原则是,数值越小,越优先	   
void NVIC_Configuration(void)
{
		NVIC_InitTypeDef  NVIC_InitStructure;	
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);	//设置NVIC中断分组2:2位抢占优先级，2位响应优先级

//		//////////////////外部中断5优先级配置也就是MPU6050 INT引脚的配置///////////因为是控制中断，故此优先级应是最高。
//		NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;				//使能外部中断通道
//		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x00;	//抢占优先级0， 
//		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x01;					//子优先级1
//		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;								//使能外部中断通道
//		NVIC_Init(&NVIC_InitStructure); 
//	
//		//////////////////定时器3中断优先级配置也就是超声波计时的定时器的配置//////////////////		
//		NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;  //TIM3中断
//		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x01;  //先占优先级1级
//		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x01;  //从优先级3级
//		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //IRQ通道被使能
//		NVIC_Init(&NVIC_InitStructure);  //根据NVIC_InitStruct中指定的参数初始化外设NVIC寄存器
//	
//		//////////////////外部中断2优先级配置也就是超声波引脚的配置//////////////////
//		NVIC_InitStructure.NVIC_IRQChannel = EXTI2_IRQn;							//使能外部中断通道
//		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x01;	//抢占优先级2 
//		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x02;					//子优先级2 
//		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;								//使能外部中断通道
//		NVIC_Init(&NVIC_InitStructure);  	  //根据NVIC_InitStruct中指定的参数初始化外设NVIC寄存器
//	
//		//////////////////Usart1 NVIC 中断优先级配置////////////////////////////////////
//		NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
//		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0x02 ;//抢占优先级2
//		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x02;			//抢占优先级2
//		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;						//IRQ通道使能
//		NVIC_Init(&NVIC_InitStructure);														//根据NVIC_InitStruct中指定的参数初始化外设NVIC寄存器USART1

//		//////////////////Usart3 NVIC 中断优先级配置也就是蓝牙串口配置//////////////////
//		NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
//		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0x02 ;//抢占优先级2
//		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x02;		//子优先级2
//		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
//		NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存器
		 
}