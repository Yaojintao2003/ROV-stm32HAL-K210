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
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
unsigned char Usart_TX_Buf[40];
uint16_t date1;//stm32接收到串口的数据
uint16_t Tracking_speed;//寻迹速度
uint16_t start_speed = 0;//初始速度
uint8_t Usart1_RX_Buf;  //串口接收数据缓存buf
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

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
		const motor_measure_t* motor_2006_1;
	motor_2006_1 = get_trigger_motor_measure_point();//返回拨弹电机 2006电机数据指针

  for(;;)
  {

			HAL_UART_Transmit(&huart1,(uint8_t *)&motor_2006_1,1,0xFFFF);//输出电机的信息

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
  {
		
 //shoot: (0x207) 2006电机控制电流, 范围 [-10000,10000]
		CAN_cmd_gimbal(0, 0, 5000, 0);
		
    osDelay(1);
  }
  /* USER CODE END StartTask02 */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */


/**************************************************************************
函数功能：USART1接收中断回调 
引脚：		 TX_PA9,RX_PA10
**************************************************************************/
//串口中断回调函数
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) 
{

		uint8_t i;
		static uint8_t RxCounter1=0;//数据缓冲区的索引
		static uint8_t RxBuffer1[5]={0};//存放数据的接收缓存区
		static uint8_t RxState = 0;	//接收标志位
			
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
						//RxBuffer1[2]是openmv发送的第一个数据的低八位,RxBuffer1[3]是openmv发送的第一个数据的高八位
						//date1=(RxBuffer1[RxCounter1-4]<<8)+(RxBuffer1[RxCounter1-5]);//分为低八位和高八位，先发低后发高
						//date2=(RxBuffer1[RxCounter1-2]<<8)+(RxBuffer1[RxCounter1-3]);
						date1=(RxBuffer1[RxCounter1-2]<<8)+(RxBuffer1[RxCounter1-3]);
//						HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, 1);						
						
					}
				}
		
				else if(RxState==3)		//检测是否接受到结束标志
				{
						if(RxBuffer1[RxCounter1-1] == 0x5A)
						{

									RxCounter1 = 0;
									RxState = 0;

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

		


	HAL_UART_Receive_IT(&huart1,&Usart1_RX_Buf,1);

}

/* USER CODE END Application */
