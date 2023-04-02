/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "bsp_can.h"
#include "CAN_receive.h"
#include "usart.h"
#include "tim.h"


//1、首先要在你的main 文件中 包含“stdio.h” （标准输入输出头文件）
#include <stdio.h>

//2、在main文件中重定义<fputc>函数 如下：
// 发送数据
int fputc(int ch, FILE *f)
{
  HAL_UART_Transmit(&huart7,(uint8_t *)&ch, 1, 0xFFFF);// USART1 可以换成 USART2 等
  while (!(UART7->SR & UART_FLAG_TXE));
  return (ch);
}
// 接收数据
int GetKey (void) {
  while (!(UART7->SR & UART_FLAG_RXNE));
   return ((int)(UART7->DR & 0x1FF));
}


/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
unsigned char Usart_TX_Buf[40];
uint16_t date1;//stm32接收到串口的数据
uint8_t Usart1_RX_Buf;  //串口接收数据缓存buf
uint8_t Rx_UART_off=0;//串口接受结束标志
//
uint8_t RxCounter1=0;//数据缓冲区的索引
uint8_t RxBuffer1[5]={0};//存放数据的接收缓存区
uint8_t RxState = 0;	//接收标志位

   const motor_measure_t* motor_2006_1;   
//	motor_measure_t* get_trigger_motor_measure_point();
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
typedef struct
{
  float target_val;
  //目标值
  float err;
  //偏差值
	float err_last;
	//上一个偏差值
	float Kp,Ki, Kd;
	//比例、积分、微分系数
	float integral;
	//积分值
	float output_val;
	//输出值.
}PID;
	PID pid;
void PID_Init(uint32_t val )
{
	PID pid;
  pid.target_val=val;
   //目标值
  pid.err_last;//上一个偏差值
  pid. Kp=0.5;
  pid.Ki=2;
  pid.Kd= 2; //比例、积分、微分系数
  pid.integral=12;
  //积分值
  pid.output_val=0; //输 出值
	
}

float PID_realize(float actual_val)
{/*计算目标值与实际值的误差*/
		pid.err = pid.target_val - actual_val;
		/*积分项*/
		pid.integral += pid.err;
		/*PID算法实现*/
		pid.output_val = pid.Kp * pid.err +
		pid.Ki * pid.integral +
		pid.Kd*(pid.err - pid.err_last);
		/*误差传递*/
		pid.err_last = pid.err;
		/*返回当前实际值*/
			return pid.output_val;
}

typedef enum
{
	W_KEY_NULL,         //没有按键按下
	W_KEY_DuoJi,        //控制舵机运动角度
	W_KEY_motor,        //控制电机运动速度，返回电机运动速度
	W_KEY_motor_angle,   //控制电机运动角度，返回角度值
	W_KEY_START_PAUSE   //启动/暂停键按下
}RMBLUEE_KEY;//串口发送模拟按键
	RMBLUEE_KEY RMBLUEE_Key = W_KEY_NULL;

RMBLUEE_KEY check_key_press()
{

	int idx = date1;//串口节后数据值
	if (idx != -1)
	{
		switch(idx)
		{
			case 0: RMBLUEE_Key = W_KEY_DuoJi; break; //控制舵机运动角度
			case 1: RMBLUEE_Key = W_KEY_motor; break; //控制电机运动速度，返回电机运动速度
			case 2: RMBLUEE_Key = W_KEY_motor_angle; break;  //控制电机运动角度，返回角度值
			case 3: RMBLUEE_Key = W_KEY_START_PAUSE; break; //启动/暂停键按下
			default: break;
		}
		//printf("%s idx:%d -> RMBLUEE_Key:%d\r\n", __func__, idx, washerKey);
	}
	
	return RMBLUEE_Key;
}


//舵机工作函数
void DuoJi_do()
{
	uint8_t DuoJi=0;
	Rx_UART_off=0;//接收结束标志置0
	while(Rx_UART_off==0)
    printf("please write Duoji jiaodu(1000-2000)...\r\n"); 
    DuoJi=date1;
	  Rx_UART_off=1;//接收结束标志置1
    HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_1);
  __HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_1,DuoJi);//0°此时对应1500、、20000总周期

}

/*控制电机运动速度，返回电机运动速度*/
 void motor_do()
{
	 static uint8_t motor_speed=0;
	if(RMBLUEE_Key==W_KEY_motor)//区分是否更改电机运动状态
		{
		
    Rx_UART_off=0;//接收结束标志置0
	  while(Rx_UART_off==0)
    printf("please write motor_speed([0,20000])...\r\n"); 
	 
	   Rx_UART_off=1;//接收结束标志置1
	
    //shoot: (0x207) 2006电机控制电流, 范围 [-10000,10000]
	  motor_speed=date1;//串口接收的数据为无符号的,发送数据+10000，这里-10000
		CAN_cmd_gimbal(0, 0,(motor_speed-10000), 0);//控制电机转速
		 }
	  PID_Init(motor_speed);//这里应填入电流换算成的速度大小，此处简写
	  motor_2006_1 = get_trigger_motor_measure_point();//返回拨弹电机 2006电机数据指针
	  float a=PID_realize((motor_2006_1->speed_rpm));
	  CAN_cmd_gimbal(0, 0,a, 0);//控制电机转速
		  if(RMBLUEE_Key==W_KEY_motor)//区分是否更改电机运动状态，及显示
			{
    printf("motor_speed is...\r\n"); //返回实际速度
	 	HAL_UART_Transmit(&huart7,(uint8_t *)&a,3,0xFFFF);//输出电机的速度
			}
     
}
/*控制电机运动角度，返回角度值*/
		
	void	motor_angle_do_()
{
		 static uint8_t motor_angle=0;
	   static uint8_t motor_angle_1=0;
	   static int8_t motor_angle_2=0;
   	 static uint8_t t=0;
	if(RMBLUEE_Key==W_KEY_motor_angle)//区分是否更改电机运动状态
    {
    Rx_UART_off=0;//接收结束标志置0
		while(Rx_UART_off==0)
    printf("please write motor_angle([0,360])...\r\n"); 
	  motor_angle=date1;
	  Rx_UART_off=1;//接收结束标志置1
		}
    //假设电流度15000下速度是+2m/s
  	//假设电流度-15000下速度是-2m/s
		
		motor_angle_1=motor_2006_1->ecd;//当前角度值
		motor_angle_2=	motor_angle-motor_angle_1;//角度差值
		if(motor_angle_2<0)
		 motor_angle_2=motor_angle_2+360;//
		
    t=((motor_angle_2/360)*3.14*0.2)/2;
    PID_Init(15000);
	  float a=PID_realize((motor_2006_1->speed_rpm));//对速度PID速度稳定，*时间得角度
	  for(t=t;t>=0;t--)
				{  
  					HAL_Delay(1000);//转t秒得到角度
					  CAN_cmd_gimbal(0, 0,a, 0);//控制电机转速
				}
					if(RMBLUEE_Key==W_KEY_motor_angle)//区分是否更改电机运动状态及显示
					{
	    motor_2006_1 = get_trigger_motor_measure_point();//返回拨弹电机 2006电机数据指针
      printf("motor_angle is...\r\n"); //返回实际角度
			HAL_UART_Transmit(&huart7,(uint8_t *)&(motor_2006_1->ecd),3,0xFFFF);//输出电机的角度
					}
}
//
void RMBLUEE_run_loop()
{  
	check_key_press();
	switch(RMBLUEE_Key)
	{
		/*没信息输入*/
		case W_KEY_NULL:
			break;

		/*控制舵机运动角度*/
		case W_KEY_DuoJi:
			DuoJi_do();
			break;

		/*控制电机运动速度，返回电机运动速度*/
		case W_KEY_motor:
			motor_do();
			break;

		/*控制电机运动角度，返回角度值*/
		case W_KEY_motor_angle:
			motor_angle_do_();
			break;

		default: break;
	} 
}
/*————————————————
版权声明：本文为CSDN博主「码农爱学习」的原创文章，遵循CC 4.0 BY-SA版权协议，转载请附上原文出处链接及本声明。
原文链接：https://blog.csdn.net/hbsyaaa/article/details/125862779*/
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */


/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
osThreadId defaultTaskHandle;
osThreadId myTask02Handle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);
void StartTask02(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize )
{
  *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
  *ppxIdleTaskStackBuffer = &xIdleStack[0];
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
  /* place for user code */
}
/* USER CODE END GET_IDLE_TASK_MEMORY */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of myTask02 */
  osThreadDef(myTask02, StartTask02, osPriorityIdle, 0, 128);
  myTask02Handle = osThreadCreate(osThread(myTask02), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
	

  for(;;)
  {
	
		  
		printf("Pleace write command(0,1,2,3)...\r\n");
		
		RMBLUEE_run_loop();
		

		
    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_StartTask02 */
/**
* @brief Function implementing the myTask02 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask02 */
void StartTask02(void const * argument)
{
  /* USER CODE BEGIN StartTask02 */
  /* Infinite loop */
  for(;;)
  {//双线程+状态机作用，在串口控制其他状态的时候电机维持原来设定速度运动
		   motor_do();
	//	motor_angle_do_();电机角度不需要多线程，一次就转到位置了；
       osDelay(1);
  }
  /* USER CODE END StartTask02 */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */


/**************************************************************************
函数功能：USART7接收中断回调 
引脚：		 TX_PF7,RX_PF6
**************************************************************************/
//串口中断回调函数
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) 
{

		uint8_t i;
		/*static uint8_t RxCounter1=0;//数据缓冲区的索引
		static uint8_t RxBuffer1[5]={0};//存放数据的接收缓存区
		static uint8_t RxState = 0;	//接收标志位*/
			//int date1= (short)(byte[0]<<8+byte[1]);有符号的在机器中为补码，应该这样处理，但不太确定
		if(RxState==0&&Usart1_RX_Buf==0xA5)  //0xA5帧头
				{
					RxState=1;//状态1
					RxBuffer1[RxCounter1++]=Usart1_RX_Buf;//RxBuffer1[0]==0xA5 RxCounter1==1
				}
		
				else if(RxState==1)//开始接收有效数据
				{
					RxBuffer1[RxCounter1++]=Usart1_RX_Buf;//全部接收完，RxCounter1==5

					if(RxCounter1>=5||Usart1_RX_Buf == 0x5A)       //RxBuffer1接受满了,接收数据结束
					{
						  RxState=2;
						date1=(RxBuffer1[RxCounter1-2]<<8)+(RxBuffer1[RxCounter1-3]);
					}
				}
				else if(RxState==3)		//检测是否接受到结束标志
				{
						if(RxBuffer1[RxCounter1-1] == 0x5A)
						{
									RxCounter1 = 0;
									RxState = 0;
                  Rx_UART_off=1;//串口接收结束标志
							
						}
						else   //接收错误
						{
									RxState = 0;
									RxCounter1=0;
									for(i=0;i<5;i++)
									{
											RxBuffer1[i]=0x00;      //将存放数据数组清零

									}
						}
				} 
	
				else   //接收异常
				{
						RxState = 0;
						RxCounter1=0;
						for(i=0;i<5;i++)
						{
								RxBuffer1[i]=0x00;      //将存放数据数组清零

						}
				}
	HAL_UART_Receive_IT(&huart7,&Usart1_RX_Buf,1);

}





/* USER CODE END Application */
