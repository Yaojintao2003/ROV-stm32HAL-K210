#include "motor.h"
 /**************************************************************************
 作  者 ：大鱼电子
淘宝地址：https://shop119207236.taobao.com
**************************************************************************/
void Motor_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE); //使能PB端口时钟
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12|GPIO_Pin_13|GPIO_Pin_14|GPIO_Pin_15;	//端口配置
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;      //推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;     //50MHZ
  GPIO_Init(GPIOB, &GPIO_InitStructure);					      //根据设定参数初始化GPIOB 
	AIN1=0,AIN2=0;
	BIN1=0,BIN1=0;
}


/**************************************************************************
函数功能：赋值给PWM寄存器
入口参数：左轮PWM、右轮PWM
返回  值：无
**************************************************************************/
void Set_Pwm(int moto1,int moto2)
{
    	if(moto1<0)			AIN2=1,			AIN1=0;
			else 	          AIN2=0,			AIN1=1;
			PWMA=myabs(moto1);
		  if(moto2<0)	BIN1=0,			BIN2=1;
			else        BIN1=1,			BIN2=0;
			PWMB=myabs(moto2);	
}



/**************************************************************************
函数功能：绝对值函数
入口参数：int
返回  值：unsigned int
目    的：经过直立环和速度环以及转向环计算出来的PWM有可能为负值
					而只能赋给定时器PWM寄存器只能是正值。故需要对PWM进行绝对值处理
**************************************************************************/
int myabs(int a)
{ 		   
	  int temp;
		if(a<0)  temp=-a;  
	  else temp=a;
	  return temp;
}


/**************************************************************************
函数功能：限制PWM赋值 
入口参数：无
返回  值：无
**************************************************************************/
void Xianfu_Pwm(void)
{
	 //===PWM满幅是7200 限制在7000
    if(Moto1<-7000 ) Moto1=-7000 ;
		if(Moto1>7000 )  Moto1=7000 ;
	  if(Moto2<-7000 ) Moto2=-7000 ;
		if(Moto2>7000 )  Moto2=7000 ;
}


/**************************************************************************
函数功能：异常关闭电机
入口参数：倾角和电压
返回  值：无
**************************************************************************/
void Turn_Off(float angle, float voltage)
{
		if(angle<-40||angle>40||voltage<3.6)	 //电池电压低于3.6V关闭电机
		{	                                   //===倾角大于40度关闭电机																			 
				Moto1=0;
				Moto2=0;
				flag_fall=1;
		}		
		else 				
				flag_fall=0;
}
	
