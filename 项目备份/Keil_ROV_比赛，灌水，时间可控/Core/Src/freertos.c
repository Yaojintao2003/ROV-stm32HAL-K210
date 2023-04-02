/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
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

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

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
/* Definitions for led_Task */
osThreadId_t led_TaskHandle;
const osThreadAttr_t led_Task_attributes = {
  .name = "led_Task",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityBelowNormal7,
};
/* Definitions for ukey_Task */
osThreadId_t ukey_TaskHandle;
const osThreadAttr_t ukey_Task_attributes = {
  .name = "ukey_Task",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for mpu6050_task */
osThreadId_t mpu6050_taskHandle;
const osThreadAttr_t mpu6050_task_attributes = {
  .name = "mpu6050_task",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal1,
};
/* Definitions for oled_task */
osThreadId_t oled_taskHandle;
const osThreadAttr_t oled_task_attributes = {
  .name = "oled_task",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for beep_task */
osThreadId_t beep_taskHandle;
const osThreadAttr_t beep_task_attributes = {
  .name = "beep_task",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for ms5837_read_tas */
osThreadId_t ms5837_read_tasHandle;
const osThreadAttr_t ms5837_read_tas_attributes = {
  .name = "ms5837_read_tas",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for move_task */
osThreadId_t move_taskHandle;
const osThreadAttr_t move_task_attributes = {
  .name = "move_task",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for turn_check_task */
osThreadId_t turn_check_taskHandle;
const osThreadAttr_t turn_check_task_attributes = {
  .name = "turn_check_task",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for ws2812_task */
osThreadId_t ws2812_taskHandle;
const osThreadAttr_t ws2812_task_attributes = {
  .name = "ws2812_task",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void LED_Task(void *argument);
extern void UKEY_Task(void *argument);
extern void MPU6050_Task(void *argument);
extern void OLED_Task(void *argument);
extern void BEEP_Task(void *argument);
extern void MS5837_Read_Task(void *argument);
extern void Move_Task(void *argument);
extern void Turn_Check_Task(void *argument);
extern void WS2812_Task(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

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
  /* creation of led_Task */
  led_TaskHandle = osThreadNew(LED_Task, NULL, &led_Task_attributes);

  /* creation of ukey_Task */
  ukey_TaskHandle = osThreadNew(UKEY_Task, NULL, &ukey_Task_attributes);

  /* creation of mpu6050_task */
  mpu6050_taskHandle = osThreadNew(MPU6050_Task, NULL, &mpu6050_task_attributes);

  /* creation of oled_task */
  oled_taskHandle = osThreadNew(OLED_Task, NULL, &oled_task_attributes);

  /* creation of beep_task */
  beep_taskHandle = osThreadNew(BEEP_Task, NULL, &beep_task_attributes);

  /* creation of ms5837_read_tas */
  ms5837_read_tasHandle = osThreadNew(MS5837_Read_Task, NULL, &ms5837_read_tas_attributes);

  /* creation of move_task */
  move_taskHandle = osThreadNew(Move_Task, NULL, &move_task_attributes);

  /* creation of turn_check_task */
  turn_check_taskHandle = osThreadNew(Turn_Check_Task, NULL, &turn_check_task_attributes);

  /* creation of ws2812_task */
  ws2812_taskHandle = osThreadNew(WS2812_Task, NULL, &ws2812_task_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_LED_Task */
/**
  * @brief  Function implementing the led_Task thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_LED_Task */
__weak void LED_Task(void *argument)
{
  /* USER CODE BEGIN LED_Task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END LED_Task */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
