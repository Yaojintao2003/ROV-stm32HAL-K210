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

float Mechanical_angle=0; // 无超声波、电池平躺着的小车机械中值

float balance_UP_KP=1000; 	 // 小车直立环KP(第一次kp调1000 、参数乘以0.6       1000    //600
float balance_UP_KD=-1.2;              //-1.2                                    -1.2                          //-1.08
float velocity_KP=0;      // 1.0
float velocity_KI=0;    //0.005

void EXTI9_5_IRQHandler(void) 
{    
	if(PBin(5)==0)		
	{		
		EXTI->PR=1<<5;                                           //===清除LINE5上的中断标志位   
		mpu_dmp_get_data(&pitch,&roll,&yaw);										 //===得到欧拉角（姿态角）的数据
		MPU_Get_Gyroscope(&gyrox,&gyroy,&gyroz);								 //===得到陀螺仪数据
		Encoder_Left=-Read_Encoder(2);                           //===读取编码器的值，因为两个电机的旋转了180度的，所以对其中一个取反，保证输出极性一致
		Encoder_Right=Read_Encoder(4);                           //===读取编码器的值
		Balance_Pwm =balance_UP(pitch,Mechanical_angle,gyroy);   //===直立环PID控制	
		Velocity_Pwm=velocity(Encoder_Left,Encoder_Right);       //===速度环PID控制	 
		//Turn_Pwm=(-0.5)*gyroz;																	 //===用来校正小车在原地转圈的现象
		Moto1=Balance_Pwm-Velocity_Pwm-Turn_Pwm  ;                 //===计算左轮电机最终PWM
		Moto2=Balance_Pwm-Velocity_Pwm+Turn_Pwm  ;            		//===计算右轮电机最终PWM	
	  Xianfu_Pwm();  																					 //===PWM限幅
		Turn_Off(pitch);																         //===检查角度是否正常
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
   //=============速度PI控制器=======================//	
		Encoder_Least =(Encoder_Left+Encoder_Right)-0;                    //===获取最新速度偏差==测量速度（左右编码器之和）-目标速度（此处为零） 
		Encoder *= 0.7;		                                                //===一阶低通滤波器       
		Encoder += Encoder_Least*0.3;	                                    //===一阶低通滤波器    
		Encoder_Integral +=Encoder;                                       //===积分出位移 积分时间：10ms
		Encoder_Integral=Encoder_Integral-Movement;                       //===接收遥控器数据，控制前进后退
		if(Encoder_Integral>10000)  		Encoder_Integral=10000;             //===积分限幅
		if(Encoder_Integral<-10000)		Encoder_Integral=-10000;            //===积分限幅	
		Velocity=Encoder*velocity_KP+Encoder_Integral*velocity_KI;        //===速度控制	
	  if(pitch<-40||pitch>40) 				Encoder_Integral=0;     						//===电机关闭后清除积分
	  return Velocity;
}
