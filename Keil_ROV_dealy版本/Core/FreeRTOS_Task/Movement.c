/*
 * Movement.c
 *
 *  Created on: 2021年3月4日
 *      Author: Royic
 */

#include "main.h"
#include "cmsis_os.h"

#include "../PID/PID.h"
#include "../ESC/ESC.h"

#define	MinStatus	2500
#define MidStatus	3750
#define FullStatus	5000

#define HMotor_PA2	(&htim9)
#define HMotor_PA3	(&htim9)
#define HMotor_PA6	(&htim3)
#define HMotor_PA7	(&htim3)
#define HMotor_PB0	(&htim3)
#define HMotor_PB1	(&htim3)

#define CMotor_PA2	TIM_CHANNEL_1
#define CMotor_PA3	TIM_CHANNEL_2
#define CMotor_PA6	TIM_CHANNEL_1
#define CMotor_PA7	TIM_CHANNEL_2
#define CMotor_PB0	TIM_CHANNEL_3
#define CMotor_PB1	TIM_CHANNEL_4

float Target_Yaw = 0;
extern float Target_High; 
extern int Base_Speed;

extern TIM_HandleTypeDef htim3, htim9;
void Motor_Set(unsigned char MotorID,int AddSpeed)
{
    switch(MotorID)
    {
    case Motor_AW:
        __HAL_TIM_SetCompare(HMotor_PB1, CMotor_PB1, MidStatus+AddSpeed);
        break;
    case Motor_DW:
        __HAL_TIM_SetCompare(HMotor_PB0, CMotor_PB0, MidStatus+AddSpeed);
        break;
    case Motor_AS:
        __HAL_TIM_SetCompare(HMotor_PA7, CMotor_PA7, MidStatus-AddSpeed);		//反了
        break;
    case Motor_DS:
        __HAL_TIM_SetCompare(HMotor_PA6, CMotor_PA6, MidStatus+AddSpeed);		
        break;	
    case Motor_LE :
        __HAL_TIM_SetCompare(HMotor_PA3, CMotor_PA3, MidStatus-AddSpeed);		//反了
        break;
    case Motor_RI:
        __HAL_TIM_SetCompare(HMotor_PA2, CMotor_PA2, MidStatus+AddSpeed);		
        break;
    }
}

#define All_KP 10 //10
#define All_Gyro_KP 0.375 //0.375

#define Up_Speed 0 //-500
//int Base_Speed = 175;
//int Base_Speed = 250;


#define All_Key 1

#if All_Key

#define Pitch_Key	1
#define Roll_Key	1
#define Yaw_Key		1
#define High_Key	1
int Base_Speed = 270; //270 300 350 325 310


#else

#define Pitch_Key	0
#define Roll_Key	0
#define Yaw_Key		0
#define High_Key	0
int Base_Speed = 0;

#endif


#if Pitch_Key
#define Pitch_KP All_KP*8
#define Pitch_Gyro_KP All_Gyro_KP
#else
#define Pitch_KP 0
#define Pitch_Gyro_KP 0
#endif
#define Pitch_KI 0
#define Pitch_KD (All_KP/100.)
#define Pitch_Gyro_KI 0
#define Pitch_Gyro_KD (2*All_Gyro_KP/100.)



#if Roll_Key
#define Roll_KP All_KP
#define Roll_Gyro_KP All_Gyro_KP
#else
#define Roll_KP 0
#define Roll_Gyro_KP 0
#endif
#define Roll_KI 0
#define Roll_KD (All_KP/200.)
#define Roll_Gyro_KI 0
#define Roll_Gyro_KD (2*All_Gyro_KP/100.)



#if Yaw_Key
#define Yaw_KP All_KP*8 //4
#define Yaw_Gyro_KP All_Gyro_KP*2	//2
#else
#define Yaw_KP 0
#define Yaw_Gyro_KP 0
#endif
#define Yaw_KI 0
#define Yaw_KD (All_KP/100.) //(All_KP/100.)
#define Yaw_Gyro_KI 0
#define Yaw_Gyro_KD (2*All_Gyro_KP/100.)

#if High_Key
#define High_KP All_KP*4
#else
#define High_KP 0
#endif
#define High_KI 0
#define High_KD (High_KP/1000.)

#define Line_x_KP	5
#define Line_x_KI	0
#define Line_x_KD	0.5

#define Line_Angle_KP	3 //10
#define Line_Angle_KI	0
#define Line_Angle_KD	0.1

//#define High_AccelZ_KP -All_Gyro_KP/2.
//#define High_AccelZ_KI 0
//#define High_AccelZ_KD (2*All_Gyro_KP/1000.)

pid_type_def Pitch_PID, Pitch_Gyro_PID, \
			 Roll_PID,  Roll_Gyro_PID,  \
			 Yaw_PID, 	Yaw_Gyro_PID,   \
			 High_PID, \
			 Line_x_PID, Line_Angle_PID;
const static fp32 pitch_pid[3]	= {Pitch_KP, Pitch_KI, Pitch_KD};
const static fp32 pitch_gyro_pid[3]	= {Pitch_Gyro_KP, Pitch_Gyro_KI, Pitch_Gyro_KD};

const static fp32 roll_pid[3]	= {Roll_KP, Roll_KI, Roll_KD};
const static fp32 roll_gyro_pid[3]	= {Roll_Gyro_KP, Roll_Gyro_KI, Roll_Gyro_KD};

const static fp32 yaw_pid[3]	= {Yaw_KP, Yaw_KI, Yaw_KD};
const static fp32 yaw_gyro_pid[3]	= {Yaw_Gyro_KP, Yaw_Gyro_KI, Yaw_Gyro_KD};

const static fp32 high_pid[3]	= {High_KP, High_KI, High_KD};

const static fp32 line_x_pid[3] = {Line_x_KP, Line_x_KI, Line_x_KD};
const static fp32 line_angle_pid[3] = {Line_Angle_KP, Line_Angle_KI, Line_Angle_KD};
//const static fp32 high_accelz_pid[3]	= {High_AccelZ_KP, High_AccelZ_KI, High_AccelZ_KD};
//extern uint8_t openmvdata[8];

extern float Pitch,Roll,Yaw,Pressure;
extern short gyro[];
extern int16_t AccelZ;
int AW_Speed=0,DW_Speed=0,AS_Speed=0,DS_Speed=0;
void Move_Task(void *argument)
{
	extern int32_t Line_Angle, Line_x;

    PID_init(&Pitch_PID, PID_POSITION, pitch_pid, FullStatus-MidStatus, (FullStatus-MidStatus)/10);
	PID_init(&Pitch_Gyro_PID, PID_POSITION, pitch_gyro_pid, FullStatus-MidStatus, (FullStatus-MidStatus)/10);

    PID_init(&Roll_PID, PID_POSITION, roll_pid, FullStatus-MidStatus, (FullStatus-MidStatus)/10);
	PID_init(&Roll_Gyro_PID, PID_POSITION, roll_gyro_pid, FullStatus-MidStatus, (FullStatus-MidStatus)/10);

    PID_init(&Yaw_PID, PID_POSITION, yaw_pid, FullStatus-MidStatus, (FullStatus-MidStatus)/10);
	PID_init(&Yaw_Gyro_PID, PID_POSITION, yaw_gyro_pid, FullStatus-MidStatus, (FullStatus-MidStatus)/10);

    PID_init(&High_PID, PID_POSITION, high_pid, FullStatus-MidStatus, (FullStatus-MidStatus)/10);
//	PID_init(&High_AccelZ_PID, PID_POSITION, high_accelz_pid, FullStatus-MidStatus, (FullStatus-MidStatus)/10);


	PID_init(&Line_x_PID, PID_POSITION, line_x_pid, FullStatus-MidStatus, (FullStatus-MidStatus)/10);
	PID_init(&Line_Angle_PID, PID_POSITION, line_angle_pid, FullStatus-MidStatus, (FullStatus-MidStatus)/10);
    for(;;)
    {

		PID_calc(&High_PID, Pressure, Target_High);

		PID_calc(&Pitch_PID, Pitch, 0);
		PID_calc(&Pitch_Gyro_PID, gyro[1], Pitch_PID.out);

		PID_calc(&Roll_PID, Roll, 0);
		PID_calc(&Roll_Gyro_PID, gyro[0], Roll_PID.out);

		PID_calc(&Line_x_PID, Line_x, 0);
		PID_calc(&Line_Angle_PID, Line_Angle, 0);
		
		Target_Yaw = Line_x_PID.out + Line_Angle_PID.out;

		PID_calc(&Yaw_PID, Yaw, Target_Yaw);
		PID_calc(&Yaw_Gyro_PID, gyro[2], Yaw_PID.out);

//		extern int32_t Line_Angle;
//		PID_calc(&Yaw_PID, Yaw_Kalman.out, 0);
//		PID_calc(&Yaw_Gyro_PID, gyro[2], Yaw_PID.out);

	//	AW_Speed=-High_PID.out;
	//	DW_Speed=-High_PID.out;
	/*	AW_Speed=High_PID.out;
		DW_Speed=-High_PID.out;
		AS_Speed=High_PID.out;
		DS_Speed=-High_PID.out;
		*/
		AW_Speed=High_PID.out;
		DW_Speed=High_PID.out;
		AS_Speed=-High_PID.out;
		DS_Speed=High_PID.out;
//		AW_Speed=-High_AccelZ_PID.out;
//		DW_Speed=-High_AccelZ_PID.out;
//		AS_Speed=-High_AccelZ_PID.out;
//		DS_Speed=-High_AccelZ_PID.out;
//		AW_Speed=Up_Speed;
//		DW_Speed=Up_Speed;
//		AS_Speed=Up_Speed;
//		DS_Speed=Up_Speed;

//yjt	//	AW_Speed=AW_Speed+(Pitch_Gyro_PID.out)+(-Roll_Gyro_PID.out);
		AW_Speed=AW_Speed+(Pitch_Gyro_PID.out)+(Roll_Gyro_PID.out);
//yjt	//	DW_Speed=DW_Speed+(Pitch_Gyro_PID.out)+(+Roll_Gyro_PID.out);
		DW_Speed=DW_Speed+(Pitch_Gyro_PID.out)+(-Roll_Gyro_PID.out);
		AS_Speed=AS_Speed+(-Pitch_Gyro_PID.out)+(-Roll_Gyro_PID.out);
		DS_Speed=DS_Speed+(-Pitch_Gyro_PID.out)+(+Roll_Gyro_PID.out);

		Motor_Set(Motor_AW,AW_Speed);
		Motor_Set(Motor_DW,DW_Speed);
		Motor_Set(Motor_AS,AS_Speed);
		Motor_Set(Motor_DS,DS_Speed);

//		Motor_Set(Motor_LE,Base_Speed-Yaw_Gyro_PID.out);
//		Motor_Set(Motor_RI,Base_Speed+Yaw_Gyro_PID.out);
	//YJT	//Motor_Set(Motor_LE,Base_Speed+Line_Angle_PID.out-Line_x_PID.out);
	//YJT	//Motor_Set(Motor_RI,Base_Speed-Line_Angle_PID.out+Line_x_PID.out);
/*
	HAL_TIM_PWM_Stop(&htim9, TIM_CHANNEL_2);				
	HAL_TIM_PWM_Stop(&htim9, TIM_CHANNEL_1);
	HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_2);				
	HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_3);

	HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);				
	HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_4);
*/

        osDelay(5);
    }

}
