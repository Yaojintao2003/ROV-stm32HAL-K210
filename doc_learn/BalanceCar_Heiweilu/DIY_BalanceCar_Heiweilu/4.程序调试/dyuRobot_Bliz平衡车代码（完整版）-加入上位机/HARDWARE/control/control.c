#include "control.h"	
 /**************************************************************************
 ��  �� ���������
�Ա���ַ��https://shop119207236.taobao.com
**************************************************************************/
/**************************************************************************
�������ܣ����еĿ��ƴ��붼��������
         5ms��ʱ�ж���MPU6050��INT���Ŵ���
         �ϸ�֤���������ݴ����ʱ��ͬ��	
				 ��MPU6050�Ĳ���Ƶ�������У����ó�100HZ�����ɱ�֤6050��������10ms����һ�Ρ�
				 ���߿���imv_mpu.h�ļ���26�еĺ궨������޸�(#define DEFAULT_MPU_HZ  (100))
**************************************************************************/
int Balance_Pwm,Velocity_Pwm,Turn_Pwm;

float Mechanical_angle=-6; // DIYС�ְ汾��е��ֵ��

float balance_UP_KP=280; 	 // С��ֱ����KP
float balance_UP_KD=0.8;

float velocity_KP=90;
float velocity_KI=0.45;

u8 UltrasonicWave_Voltage_Counter=0;

void EXTI9_5_IRQHandler(void) 
{    
	if(PBin(5)==0)		
	{		
		EXTI->PR=1<<5;                                           //===���LINE5�ϵ��жϱ�־λ   
		mpu_dmp_get_data(&pitch,&roll,&yaw);										 //===�õ�ŷ���ǣ���̬�ǣ�������
		MPU_Get_Gyroscope(&gyrox,&gyroy,&gyroz);								 //===�õ�����������
		Encoder_Left=-Read_Encoder(2);                           //===��ȡ��������ֵ����Ϊ�����������ת��180�ȵģ����Զ�����һ��ȡ������֤�������һ��
		Encoder_Right=Read_Encoder(4);                           //===��ȡ��������ֵ
		UltrasonicWave_Voltage_Counter++;
		if(UltrasonicWave_Voltage_Counter==10)									 //===100ms��ȡһ�γ������������Լ���ѹ
		{
			UltrasonicWave_Voltage_Counter=0;
			Voltage=Get_battery_volt();		                         //===��ȡ��ص�ѹ		
			UltrasonicWave_StartMeasure();												 //===��ȡ��������ֵ
		}
		Balance_Pwm =balance_UP(pitch,Mechanical_angle,gyroy);   //===ƽ�⻷PID����	
		Velocity_Pwm=velocity(Encoder_Left,Encoder_Right);       //===�ٶȻ�PID����	 
  	if(1==Flag_Left||1==Flag_Right)    
		Turn_Pwm =turn(Encoder_Left,Encoder_Right,gyroz);        //===ת��PID����
		else Turn_Pwm=(-0.5)*gyroz;
		Moto1=Balance_Pwm-Velocity_Pwm-Turn_Pwm;                 //===�������ֵ������PWM
		Moto2=Balance_Pwm-Velocity_Pwm+Turn_Pwm;                 //===�������ֵ������PWM
	  Xianfu_Pwm();  																					 //===PWM�޷�
		USB_TEST();																							 //===���USB�Ƿ����
		Turn_Off(pitch,Voltage);																 //===���Ƕ��Լ���ѹ�Ƿ�����
		Set_Pwm(Moto1,Moto2);                                    //===��ֵ��PWM�Ĵ���  
	}       	
} 

/**************************************************************************
�������ܣ�ֱ��PD����
��ڲ������Ƕȡ���еƽ��Ƕȣ���е��ֵ�������ٶ�
����  ֵ��ֱ������PWM
��    �ߣ��������
**************************************************************************/
int balance_UP(float Angle,float Mechanical_balance,float Gyro)
{  
   float Bias;
	 int balance;
	 Bias=Angle-Mechanical_balance;    							 //===���ƽ��ĽǶ���ֵ�ͻ�е���
	 balance=balance_UP_KP*Bias+balance_UP_KD*Gyro;  //===����ƽ����Ƶĵ��PWM  PD����   kp��Pϵ�� kd��Dϵ�� 
	 return balance;
}

/**************************************************************************
�������ܣ��ٶ�PI����
��ڲ����������������ֵ
����  ֵ���ٶȿ���PWM
��    �ߣ��������
**************************************************************************/
int velocity(int encoder_left,int encoder_right)
{  
    static float Velocity,Encoder_Least,Encoder,Movement;
	  static float Encoder_Integral;
		if(1==Flag_Qian)				
		{
			flag_UltrasonicWave=0;//��������־λ  0��־��ȫ����  1��־Σ�վ���  
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
   //=============�ٶ�PI������=======================//	
		Encoder_Least =(Encoder_Left+Encoder_Right)-0;                    //===��ȡ�����ٶ�ƫ��==�����ٶȣ����ұ�����֮�ͣ�-Ŀ���ٶȣ��˴�Ϊ�㣩 
		Encoder *= 0.8;		                                                //===һ�׵�ͨ�˲���       
		Encoder += Encoder_Least*0.2;	                                    //===һ�׵�ͨ�˲���    
		Encoder_Integral +=Encoder;                                       //===���ֳ�λ�� ����ʱ�䣺10ms
		Encoder_Integral=Encoder_Integral-Movement;                       //===����ң�������ݣ�����ǰ������
		if(Encoder_Integral>10000)  	Encoder_Integral=10000;             //===�����޷�
		if(Encoder_Integral<-10000)		Encoder_Integral=-10000;            //===�����޷�	
		Velocity=Encoder*velocity_KP+Encoder_Integral*velocity_KI;        //===�ٶȿ���	
	  if(pitch<-40||pitch>40) 			Encoder_Integral=0;     						//===����رպ��������
	  return Velocity;
}
/**************************************************************************
�������ܣ�ת��PD����
��ڲ����������������ֵ��Z����ٶ�
����  ֵ��ת�����PWM
��    �ߣ��������
**************************************************************************/
int turn(int encoder_left,int encoder_right,float gyro)//ת�����
{
	 static float Turn_Target,Turn,Encoder_temp,Turn_Convert=0.9,Turn_Count;
	  float Turn_Amplitude=44,Kp=20,Kd=0;     
	  //=============ң��������ת����=======================//
  	if(1==Flag_Left||1==Flag_Right)                      //��һ������Ҫ�Ǹ�����תǰ���ٶȵ����ٶȵ���ʼ�ٶȣ�����С������Ӧ��
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
    if(Turn_Target>Turn_Amplitude)  Turn_Target=Turn_Amplitude;    //===ת��	�ٶ��޷�
	  if(Turn_Target<-Turn_Amplitude) Turn_Target=-Turn_Amplitude;
		if(Flag_Qian==1||Flag_Hou==1)  Kd=0.5;        
		else Kd=0;   //ת���ʱ��ȡ�������ǵľ��� �е�ģ��PID��˼��
  	//=============ת��PD������=======================//
		Turn=-Turn_Target*Kp-gyro*Kd;                 //===���Z�������ǽ���PD����
	  return Turn;
}
