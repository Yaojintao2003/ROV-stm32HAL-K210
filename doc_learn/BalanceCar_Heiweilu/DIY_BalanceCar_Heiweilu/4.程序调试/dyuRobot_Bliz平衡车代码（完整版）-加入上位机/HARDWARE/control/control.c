#include "control.h"	
 /**************************************************************************
 作  者 ：大鱼电子
淘宝地址：https://shop119207236.taobao.com
**************************************************************************/
/**************************************************************************
函数功能：所有的控制代码都在这里面
         5ms定时中断由MPU6050的INT引脚触发
         严格保证采样和数据处理的时间同步	
				 在MPU6050的采样频率设置中，设置成100HZ，即可保证6050的数据是10ms更新一次。
				 读者可在imv_mpu.h文件第26行的宏定义进行修改(#define DEFAULT_MPU_HZ  (100))
**************************************************************************/
int Balance_Pwm,Velocity_Pwm,Turn_Pwm;

float Mechanical_angle=-6; // DIY小轮版本机械中值。

float balance_UP_KP=280; 	 // 小车直立环KP
float balance_UP_KD=0.8;

float velocity_KP=90;
float velocity_KI=0.45;

u8 UltrasonicWave_Voltage_Counter=0;

void EXTI9_5_IRQHandler(void) 
{    
	if(PBin(5)==0)		
	{		
		EXTI->PR=1<<5;                                           //===清除LINE5上的中断标志位   
		mpu_dmp_get_data(&pitch,&roll,&yaw);										 //===得到欧拉角（姿态角）的数据
		MPU_Get_Gyroscope(&gyrox,&gyroy,&gyroz);								 //===得到陀螺仪数据
		Encoder_Left=-Read_Encoder(2);                           //===读取编码器的值，因为两个电机的旋转了180度的，所以对其中一个取反，保证输出极性一致
		Encoder_Right=Read_Encoder(4);                           //===读取编码器的值
		UltrasonicWave_Voltage_Counter++;
		if(UltrasonicWave_Voltage_Counter==10)									 //===100ms读取一次超声波的数据以及电压
		{
			UltrasonicWave_Voltage_Counter=0;
			Voltage=Get_battery_volt();		                         //===读取电池电压		
			UltrasonicWave_StartMeasure();												 //===读取超声波的值
		}
		Balance_Pwm =balance_UP(pitch,Mechanical_angle,gyroy);   //===平衡环PID控制	
		Velocity_Pwm=velocity(Encoder_Left,Encoder_Right);       //===速度环PID控制	 
  	if(1==Flag_Left||1==Flag_Right)    
		Turn_Pwm =turn(Encoder_Left,Encoder_Right,gyroz);        //===转向环PID控制
		else Turn_Pwm=(-0.5)*gyroz;
		Moto1=Balance_Pwm-Velocity_Pwm-Turn_Pwm;                 //===计算左轮电机最终PWM
		Moto2=Balance_Pwm-Velocity_Pwm+Turn_Pwm;                 //===计算右轮电机最终PWM
	  Xianfu_Pwm();  																					 //===PWM限幅
		USB_TEST();																							 //===检查USB是否插入
		Turn_Off(pitch,Voltage);																 //===检查角度以及电压是否正常
		Set_Pwm(Moto1,Moto2);                                    //===赋值给PWM寄存器  
	}       	
} 

/**************************************************************************
函数功能：直立PD控制
入口参数：角度、机械平衡角度（机械中值）、角速度
返回  值：直立控制PWM
作    者：大鱼电子
**************************************************************************/
int balance_UP(float Angle,float Mechanical_balance,float Gyro)
{  
   float Bias;
	 int balance;
	 Bias=Angle-Mechanical_balance;    							 //===求出平衡的角度中值和机械相关
	 balance=balance_UP_KP*Bias+balance_UP_KD*Gyro;  //===计算平衡控制的电机PWM  PD控制   kp是P系数 kd是D系数 
	 return balance;
}

/**************************************************************************
函数功能：速度PI控制
入口参数：电机编码器的值
返回  值：速度控制PWM
作    者：大鱼电子
**************************************************************************/
int velocity(int encoder_left,int encoder_right)
{  
    static float Velocity,Encoder_Least,Encoder,Movement;
	  static float Encoder_Integral;
		if(1==Flag_Qian)				
		{
			flag_UltrasonicWave=0;//超声波标志位  0标志安全距离  1标志危险距离  
			Movement=20;
		}
    else if(1==Flag_Hou)		
		{
			flag_UltrasonicWave=0;
			Movement=-20;
		}
		else if(UltrasonicWave_Distance<10&&UltrasonicWave_Distance>3)	
		{
			flag_UltrasonicWave=1;
			Movement=UltrasonicWave_Distance*(-2);		
		}
		else 	
		{	
			flag_UltrasonicWave=0;
			Movement=0;
		}
   //=============速度PI控制器=======================//	
		Encoder_Least =(Encoder_Left+Encoder_Right)-0;                    //===获取最新速度偏差==测量速度（左右编码器之和）-目标速度（此处为零） 
		Encoder *= 0.8;		                                                //===一阶低通滤波器       
		Encoder += Encoder_Least*0.2;	                                    //===一阶低通滤波器    
		Encoder_Integral +=Encoder;                                       //===积分出位移 积分时间：10ms
		Encoder_Integral=Encoder_Integral-Movement;                       //===接收遥控器数据，控制前进后退
		if(Encoder_Integral>10000)  	Encoder_Integral=10000;             //===积分限幅
		if(Encoder_Integral<-10000)		Encoder_Integral=-10000;            //===积分限幅	
		Velocity=Encoder*velocity_KP+Encoder_Integral*velocity_KI;        //===速度控制	
	  if(pitch<-40||pitch>40) 			Encoder_Integral=0;     						//===电机关闭后清除积分
	  return Velocity;
}
/**************************************************************************
函数功能：转向PD控制
入口参数：电机编码器的值、Z轴角速度
返回  值：转向控制PWM
作    者：大鱼电子
**************************************************************************/
int turn(int encoder_left,int encoder_right,float gyro)//转向控制
{
	 static float Turn_Target,Turn,Encoder_temp,Turn_Convert=0.9,Turn_Count;
	  float Turn_Amplitude=44,Kp=20,Kd=0;     
	  //=============遥控左右旋转部分=======================//
  	if(1==Flag_Left||1==Flag_Right)                      //这一部分主要是根据旋转前的速度调整速度的起始速度，增加小车的适应性
		{
			if(++Turn_Count==1)
			Encoder_temp=myabs(encoder_left+encoder_right);      
			Turn_Convert=55/Encoder_temp;
			if(Turn_Convert<0.6)Turn_Convert=0.6;
			if(Turn_Convert>3)Turn_Convert=3;
		}	
	  else
		{
			Turn_Convert=0.9;
			Turn_Count=0;
			Encoder_temp=0;
		}			
		if(1==Flag_Left)	           Turn_Target-=Turn_Convert;
		else if(1==Flag_Right)	     Turn_Target+=Turn_Convert; 
		else Turn_Target=0;
    if(Turn_Target>Turn_Amplitude)  Turn_Target=Turn_Amplitude;    //===转向	速度限幅
	  if(Turn_Target<-Turn_Amplitude) Turn_Target=-Turn_Amplitude;
		if(Flag_Qian==1||Flag_Hou==1)  Kd=0.5;        
		else Kd=0;   //转向的时候取消陀螺仪的纠正 有点模糊PID的思想
  	//=============转向PD控制器=======================//
		Turn=-Turn_Target*Kp-gyro*Kd;                 //===结合Z轴陀螺仪进行PD控制
	  return Turn;
}
