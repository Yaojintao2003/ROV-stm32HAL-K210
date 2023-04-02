#include "control.h"


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
int velocity(int encoder_left,int encoder_right,int target)
{  
	static float Velocity,Encoder_Least,Encoder;
	static float Encoder_Integral;
	//=============�ٶ�PI������=======================//	
	Encoder_Least =(Encoder_Left+Encoder_Right);//-target;                    //===��ȡ�����ٶ�ƫ��==�����ٶȣ����ұ�����֮�ͣ�-Ŀ���ٶ� 
	Encoder *= 0.8;		                                                //===һ�׵�ͨ�˲���       
	Encoder += Encoder_Least*0.2;	                                    //===һ�׵�ͨ�˲���    
	Encoder_Integral +=Encoder;                                       //===���ֳ�λ�� ����ʱ�䣺10ms
	Encoder_Integral=Encoder_Integral-target;                       //===����ң�������ݣ�����ǰ������
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
int turn(int encoder_left,int encoder_right,float gyro,int target)//ת�����
{
	 static float Turn_Target,Encoder_temp,Turn_Convert=0.9,Turn_Count;
	  float Turn_Amplitude=66,Kp=20,Kd=0;
	  //=============ң��������ת����=======================//
	  //��һ������Ҫ�Ǹ�����תǰ���ٶȵ����ٶȵ���ʼ�ٶȣ�����С������Ӧ��
  	if(target!=0)
		{
			if(++Turn_Count==1)
			Encoder_temp=GFP_abs(encoder_left+encoder_right);      
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
		if(target>0)	         Turn_Target+=Turn_Convert;
		else if(target<0)	     Turn_Target-=Turn_Convert; 
		else Turn_Target=0;
    if(Turn_Target>Turn_Amplitude)  Turn_Target=Turn_Amplitude;    //===ת��	�ٶ��޷�
	  if(Turn_Target<-Turn_Amplitude) Turn_Target=-Turn_Amplitude;
		if(target!=0)  Kd=0.5;        
		else Kd=0;   //ת���ʱ��ȡ�������ǵľ��� �е�ģ��PID��˼��
  	//=============ת��PD������=======================//
		return -Turn_Target*Kp-gyro*Kd;                 //===���Z�������ǽ���PD����
}
