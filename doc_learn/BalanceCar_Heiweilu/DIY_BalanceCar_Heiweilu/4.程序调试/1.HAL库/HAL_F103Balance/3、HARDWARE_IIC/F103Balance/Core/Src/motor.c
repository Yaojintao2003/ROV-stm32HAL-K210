#include "motor.h"

void Motor_Start(void)     //Motor控制所连接的PB12~15初始化在main()函数MX中执行了.
{
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);  //根据MX配置和原理图：这里需要开启的是TIM1的PWM输出通道1
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_4);  //开启TIM1的PWM输出通道1
}
/**************************************************************************
函数功能：赋值给PWM寄存器
入口参数：左轮PWM、右轮PWM
返回  值：无
**************************************************************************/
void Motor_Rotaton(int MotorA,int MotorB)
{
	//1.研究正负号，对应正反转
	if(MotorA>0)	Ain1=1,Ain2=0;//正转
	else 				Ain1=0,Ain2=1;//反转
	//2.研究PWM值
	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,GFP_abs(MotorA));   //把传参进来的PWM值送给定时器1的channel_1通道.
	
	if(MotorB>0)	Bin1=1,Bin2=0;
	else 				Bin1=0,Bin2=1;	
	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_4,GFP_abs(MotorB));
}

/**************************************************************************
函数功能：绝对值函数
入口参数：int
返回  值：unsigned int
目    的：经过直立环和速度环以及转向环计算出来的PWM有可能为负值
					而只能赋给定时器PWM寄存器只能是正值。故需要对PWM进行绝对值处理
**************************************************************************/
int GFP_abs(int a)
{ 		   
	int temp;
	temp = a>0?a:(-a);
	return temp;
}

/*限幅函数:避免PWM过大超过马达的机械极限*/
void Limit(int *motoA,int *motoB)
{
	if(*motoA>PWM_MAX)  *motoA=PWM_MAX;
	if(*motoA<PWM_MIN)  *motoA=PWM_MIN;
	
	if(*motoB>PWM_MAX)  *motoB=PWM_MAX;
	if(*motoB<PWM_MIN)  *motoB=PWM_MIN;
}
/**************************************************************************
函数功能：异常关闭电机
入口参数：倾角和电压
返回  值：无
**************************************************************************/
void Stop(float angle, float voltage)
{
		if(angle<-40||angle>40||voltage<11.1)	 //电池电压低于11.1V关闭电机
		{	                                   //===倾角大于40度关闭电机																			 
				Moto1=0;
				Moto2=0;
		}		
}

/*htim2和htim3句柄在tim.c中已经定义过了。
编码器原理:每个电机A相和B相分别用某个定时器的两个channel去采集编码轮的1和0,AB相位差90度(正交)可以根据两个信号哪个先哪个后来判断方向,
根据每个信号脉冲数量的多少及整个编码轮的周长就可以算出当前行走的距离;如果再加上定时器的话还可以计算出速度。
有时采集到的编码脉冲难免没有干扰毛刺,因此编码器模式的定时器配置时可设置滤波FP值.*/

void Encoder_Start(void)   //启动两个编码器. 而编码器初始化HAL_TIM_Encoder_Init()和HAL_TIM_Encoder_MspInit()在main()函数中调用的MX_TIM2_Init()中。
{
	__HAL_TIM_SET_COUNTER(&htim2,0);   //用带参宏设置编码器的初始值为0(涉及正反转的需要)
	__HAL_TIM_SET_COUNTER(&htim3,0);

	HAL_TIM_Encoder_Start_IT(&htim2,TIM_CHANNEL_ALL);   //开启编码器的中断模式,两个定时器通道TI1和TI2是每个编码器的两个信号采集通道.
	HAL_TIM_Encoder_Start_IT(&htim3,TIM_CHANNEL_ALL);

	HAL_TIM_Encoder_Start(&htim2,TIM_CHANNEL_ALL);  //开启两个编码器
	HAL_TIM_Encoder_Start(&htim3,TIM_CHANNEL_ALL);
	
}


//获取编码器计数值
int Read_Speed(int TIMx)
{
	int Encoder_Value = 0;
	switch(TIMx)
	{
		case 2: 
			Encoder_Value =(short) __HAL_TIM_GET_COUNTER(&htim2);     //保存编码器计数器的值
			__HAL_TIM_SET_COUNTER(&htim2,0);                   //保存之后要清零,以便下次继续读取.另外每次清零后采样值减0,直接用单位时间的话就可以得出速度信息了.不要麻烦还要减去初值了.
			break;

		case 3: 
			Encoder_Value =(short) __HAL_TIM_GET_COUNTER(&htim3);
			__HAL_TIM_SET_COUNTER(&htim3,0);
			break;
		
		default:
			Encoder_Value = 0;
	}	
	return Encoder_Value;
}

void Encoder_IRQHandler(void)   //此处只是清中断标志,实际并没有做其他操作.
{
	HAL_TIM_IRQHandler(&htim2);   //IRQHandler()函数只是清中断标志,实际ISR中断服务程序是该函数又去调用相应的callback()函数实现的.
	HAL_TIM_IRQHandler(&htim3);   //因此如果需要ISR则必须自定义相应的callback()函数才行,因为系统中的callback()是__weak声明的空函数.
}
