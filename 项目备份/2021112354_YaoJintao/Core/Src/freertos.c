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


//1������Ҫ�����main �ļ��� ������stdio.h�� ����׼�������ͷ�ļ���
#include <stdio.h>

//2����main�ļ����ض���<fputc>���� ���£�
// ��������
int fputc(int ch, FILE *f)
{
  HAL_UART_Transmit(&huart7,(uint8_t *)&ch, 1, 0xFFFF);// USART1 ���Ի��� USART2 ��
  while (!(UART7->SR & UART_FLAG_TXE));
  return (ch);
}
// ��������
int GetKey (void) {
  while (!(UART7->SR & UART_FLAG_RXNE));
   return ((int)(UART7->DR & 0x1FF));
}


/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
unsigned char Usart_TX_Buf[40];
uint16_t date1;//stm32���յ����ڵ�����
uint8_t Usart1_RX_Buf;  //���ڽ������ݻ���buf
uint8_t Rx_UART_off=0;//���ڽ��ܽ�����־
//
uint8_t RxCounter1=0;//���ݻ�����������
uint8_t RxBuffer1[5]={0};//������ݵĽ��ջ�����
uint8_t RxState = 0;	//���ձ�־λ

   const motor_measure_t* motor_2006_1;   
//	motor_measure_t* get_trigger_motor_measure_point();
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
typedef struct
{
  float target_val;
  //Ŀ��ֵ
  float err;
  //ƫ��ֵ
	float err_last;
	//��һ��ƫ��ֵ
	float Kp,Ki, Kd;
	//���������֡�΢��ϵ��
	float integral;
	//����ֵ
	float output_val;
	//���ֵ.
}PID;
	PID pid;
void PID_Init(uint32_t val )
{
	PID pid;
  pid.target_val=val;
   //Ŀ��ֵ
  pid.err_last;//��һ��ƫ��ֵ
  pid. Kp=0.5;
  pid.Ki=2;
  pid.Kd= 2; //���������֡�΢��ϵ��
  pid.integral=12;
  //����ֵ
  pid.output_val=0; //�� ��ֵ
	
}

float PID_realize(float actual_val)
{/*����Ŀ��ֵ��ʵ��ֵ�����*/
		pid.err = pid.target_val - actual_val;
		/*������*/
		pid.integral += pid.err;
		/*PID�㷨ʵ��*/
		pid.output_val = pid.Kp * pid.err +
		pid.Ki * pid.integral +
		pid.Kd*(pid.err - pid.err_last);
		/*����*/
		pid.err_last = pid.err;
		/*���ص�ǰʵ��ֵ*/
			return pid.output_val;
}

typedef enum
{
	W_KEY_NULL,         //û�а�������
	W_KEY_DuoJi,        //���ƶ���˶��Ƕ�
	W_KEY_motor,        //���Ƶ���˶��ٶȣ����ص���˶��ٶ�
	W_KEY_motor_angle,   //���Ƶ���˶��Ƕȣ����ؽǶ�ֵ
	W_KEY_START_PAUSE   //����/��ͣ������
}RMBLUEE_KEY;//���ڷ���ģ�ⰴ��
	RMBLUEE_KEY RMBLUEE_Key = W_KEY_NULL;

RMBLUEE_KEY check_key_press()
{

	int idx = date1;//���ڽں�����ֵ
	if (idx != -1)
	{
		switch(idx)
		{
			case 0: RMBLUEE_Key = W_KEY_DuoJi; break; //���ƶ���˶��Ƕ�
			case 1: RMBLUEE_Key = W_KEY_motor; break; //���Ƶ���˶��ٶȣ����ص���˶��ٶ�
			case 2: RMBLUEE_Key = W_KEY_motor_angle; break;  //���Ƶ���˶��Ƕȣ����ؽǶ�ֵ
			case 3: RMBLUEE_Key = W_KEY_START_PAUSE; break; //����/��ͣ������
			default: break;
		}
		//printf("%s idx:%d -> RMBLUEE_Key:%d\r\n", __func__, idx, washerKey);
	}
	
	return RMBLUEE_Key;
}


//�����������
void DuoJi_do()
{
	uint8_t DuoJi=0;
	Rx_UART_off=0;//���ս�����־��0
	while(Rx_UART_off==0)
    printf("please write Duoji jiaodu(1000-2000)...\r\n"); 
    DuoJi=date1;
	  Rx_UART_off=1;//���ս�����־��1
    HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_1);
  __HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_1,DuoJi);//0���ʱ��Ӧ1500����20000������

}

/*���Ƶ���˶��ٶȣ����ص���˶��ٶ�*/
 void motor_do()
{
	 static uint8_t motor_speed=0;
	if(RMBLUEE_Key==W_KEY_motor)//�����Ƿ���ĵ���˶�״̬
		{
		
    Rx_UART_off=0;//���ս�����־��0
	  while(Rx_UART_off==0)
    printf("please write motor_speed([0,20000])...\r\n"); 
	 
	   Rx_UART_off=1;//���ս�����־��1
	
    //shoot: (0x207) 2006������Ƶ���, ��Χ [-10000,10000]
	  motor_speed=date1;//���ڽ��յ�����Ϊ�޷��ŵ�,��������+10000������-10000
		CAN_cmd_gimbal(0, 0,(motor_speed-10000), 0);//���Ƶ��ת��
		 }
	  PID_Init(motor_speed);//����Ӧ�����������ɵ��ٶȴ�С���˴���д
	  motor_2006_1 = get_trigger_motor_measure_point();//���ز������ 2006�������ָ��
	  float a=PID_realize((motor_2006_1->speed_rpm));
	  CAN_cmd_gimbal(0, 0,a, 0);//���Ƶ��ת��
		  if(RMBLUEE_Key==W_KEY_motor)//�����Ƿ���ĵ���˶�״̬������ʾ
			{
    printf("motor_speed is...\r\n"); //����ʵ���ٶ�
	 	HAL_UART_Transmit(&huart7,(uint8_t *)&a,3,0xFFFF);//���������ٶ�
			}
     
}
/*���Ƶ���˶��Ƕȣ����ؽǶ�ֵ*/
		
	void	motor_angle_do_()
{
		 static uint8_t motor_angle=0;
	   static uint8_t motor_angle_1=0;
	   static int8_t motor_angle_2=0;
   	 static uint8_t t=0;
	if(RMBLUEE_Key==W_KEY_motor_angle)//�����Ƿ���ĵ���˶�״̬
    {
    Rx_UART_off=0;//���ս�����־��0
		while(Rx_UART_off==0)
    printf("please write motor_angle([0,360])...\r\n"); 
	  motor_angle=date1;
	  Rx_UART_off=1;//���ս�����־��1
		}
    //���������15000���ٶ���+2m/s
  	//���������-15000���ٶ���-2m/s
		
		motor_angle_1=motor_2006_1->ecd;//��ǰ�Ƕ�ֵ
		motor_angle_2=	motor_angle-motor_angle_1;//�ǶȲ�ֵ
		if(motor_angle_2<0)
		 motor_angle_2=motor_angle_2+360;//
		
    t=((motor_angle_2/360)*3.14*0.2)/2;
    PID_Init(15000);
	  float a=PID_realize((motor_2006_1->speed_rpm));//���ٶ�PID�ٶ��ȶ���*ʱ��ýǶ�
	  for(t=t;t>=0;t--)
				{  
  					HAL_Delay(1000);//תt��õ��Ƕ�
					  CAN_cmd_gimbal(0, 0,a, 0);//���Ƶ��ת��
				}
					if(RMBLUEE_Key==W_KEY_motor_angle)//�����Ƿ���ĵ���˶�״̬����ʾ
					{
	    motor_2006_1 = get_trigger_motor_measure_point();//���ز������ 2006�������ָ��
      printf("motor_angle is...\r\n"); //����ʵ�ʽǶ�
			HAL_UART_Transmit(&huart7,(uint8_t *)&(motor_2006_1->ecd),3,0xFFFF);//�������ĽǶ�
					}
}
//
void RMBLUEE_run_loop()
{  
	check_key_press();
	switch(RMBLUEE_Key)
	{
		/*û��Ϣ����*/
		case W_KEY_NULL:
			break;

		/*���ƶ���˶��Ƕ�*/
		case W_KEY_DuoJi:
			DuoJi_do();
			break;

		/*���Ƶ���˶��ٶȣ����ص���˶��ٶ�*/
		case W_KEY_motor:
			motor_do();
			break;

		/*���Ƶ���˶��Ƕȣ����ؽǶ�ֵ*/
		case W_KEY_motor_angle:
			motor_angle_do_();
			break;

		default: break;
	} 
}
/*��������������������������������
��Ȩ����������ΪCSDN��������ũ��ѧϰ����ԭ�����£���ѭCC 4.0 BY-SA��ȨЭ�飬ת���븽��ԭ�ĳ������Ӽ���������
ԭ�����ӣ�https://blog.csdn.net/hbsyaaa/article/details/125862779*/
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
  {//˫�߳�+״̬�����ã��ڴ��ڿ�������״̬��ʱ����ά��ԭ���趨�ٶ��˶�
		   motor_do();
	//	motor_angle_do_();����ǶȲ���Ҫ���̣߳�һ�ξ�ת��λ���ˣ�
       osDelay(1);
  }
  /* USER CODE END StartTask02 */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */


/**************************************************************************
�������ܣ�USART7�����жϻص� 
���ţ�		 TX_PF7,RX_PF6
**************************************************************************/
//�����жϻص�����
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) 
{

		uint8_t i;
		/*static uint8_t RxCounter1=0;//���ݻ�����������
		static uint8_t RxBuffer1[5]={0};//������ݵĽ��ջ�����
		static uint8_t RxState = 0;	//���ձ�־λ*/
			//int date1= (short)(byte[0]<<8+byte[1]);�з��ŵ��ڻ�����Ϊ���룬Ӧ��������������̫ȷ��
		if(RxState==0&&Usart1_RX_Buf==0xA5)  //0xA5֡ͷ
				{
					RxState=1;//״̬1
					RxBuffer1[RxCounter1++]=Usart1_RX_Buf;//RxBuffer1[0]==0xA5 RxCounter1==1
				}
		
				else if(RxState==1)//��ʼ������Ч����
				{
					RxBuffer1[RxCounter1++]=Usart1_RX_Buf;//ȫ�������꣬RxCounter1==5

					if(RxCounter1>=5||Usart1_RX_Buf == 0x5A)       //RxBuffer1��������,�������ݽ���
					{
						  RxState=2;
						date1=(RxBuffer1[RxCounter1-2]<<8)+(RxBuffer1[RxCounter1-3]);
					}
				}
				else if(RxState==3)		//����Ƿ���ܵ�������־
				{
						if(RxBuffer1[RxCounter1-1] == 0x5A)
						{
									RxCounter1 = 0;
									RxState = 0;
                  Rx_UART_off=1;//���ڽ��ս�����־
							
						}
						else   //���մ���
						{
									RxState = 0;
									RxCounter1=0;
									for(i=0;i<5;i++)
									{
											RxBuffer1[i]=0x00;      //�����������������

									}
						}
				} 
	
				else   //�����쳣
				{
						RxState = 0;
						RxCounter1=0;
						for(i=0;i<5;i++)
						{
								RxBuffer1[i]=0x00;      //�����������������

						}
				}
	HAL_UART_Receive_IT(&huart7,&Usart1_RX_Buf,1);

}





/* USER CODE END Application */
