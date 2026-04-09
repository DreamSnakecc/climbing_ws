/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "system_config.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
bool disable_allmotor = true;
bool corrEnable = true;
float corr_finished = true;

MotorTypeDef* motors_2006[3] = {&Motor_3,&Motor_4,&Motor_6};
MotorTypeDef* motors_3508[3] = {&Motor_1,&Motor_2,&Motor_5};

float velocity[3]={0};//rpm

float angle[3] = {0};
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
osThreadId defaultTaskHandle;
osThreadId myTask02Handle;
osThreadId myTask03Handle;
osThreadId myTask04Handle;
osThreadId myTask05Handle;
osThreadId myTask06Handle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
void StartTask(void const * argument);
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);
void Monitor_Task(void const * argument);
void Init_Task(void const * argument);
void Regulate_Task(void const * argument);
void Handle_Task(void const * argument);
void Usart_Task(void const * argument);

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
  osThreadDef(myTask02, Monitor_Task, osPriorityIdle, 0, 128);
  myTask02Handle = osThreadCreate(osThread(myTask02), NULL);

  /* definition and creation of myTask03 */
  osThreadDef(myTask03, Init_Task, osPriorityIdle, 0, 128);
  myTask03Handle = osThreadCreate(osThread(myTask03), NULL);

  /* definition and creation of myTask04 */
  osThreadDef(myTask04, Regulate_Task, osPriorityIdle, 0, 128);
  myTask04Handle = osThreadCreate(osThread(myTask04), NULL);

  /* definition and creation of myTask05 */
  osThreadDef(myTask05, Handle_Task, osPriorityIdle, 0, 128);
  myTask05Handle = osThreadCreate(osThread(myTask05), NULL);

  /* definition and creation of myTask06 */
  osThreadDef(myTask06, Usart_Task, osPriorityIdle, 0, 128);
  myTask06Handle = osThreadCreate(osThread(myTask06), NULL);

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
    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_Monitor_Task */
/**
* @brief Function implementing the myTask02 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Monitor_Task */
void Monitor_Task(void const * argument)
{
  /* USER CODE BEGIN Monitor_Task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Monitor_Task */
}

/* USER CODE BEGIN Header_Init_Task */
/**
* @brief Function implementing the myTask03 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Init_Task */
void Init_Task(void const * argument)
{
  /* USER CODE BEGIN Init_Task */
  /* Infinite loop */
  for(;;)
  {				
			
			for(int i = 0;i<4;i++)
			{
        //vesc_can_set_rpm(i + 1, Mad_Velocity[i]);// Set MAD motor RPM command
				comm_can_set_rpm(i + 1, Mad_Velocity[i]);
			}
			osDelay(10);
   }
  /* USER CODE END Init_Task */
}

/* USER CODE BEGIN Header_Regulate_Task */
/**
* @brief Function implementing the myTask04 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Regulate_Task */
void Regulate_Task(void const * argument)
{
  /* USER CODE BEGIN Regulate_Task */
  /* Infinite loop */
  for(;;)
  {
		switch(Motor_1.State)
		{
			case PIDPOSITION:				
				Motor_1.SpeedExpected =  ClassicPidRegulate(Motor_1.PositionExpected,Motor_1.PositionMeasure,&MotorPositionPid1);
			case PIDSPEED:						
				Motor_1.CurrentExpected = ClassicPidRegulate(Motor_1.SpeedExpected,Motor_1.SpeedMeasure,&MotorSpeedPid1);		
      case MOTOR_CURRENT: 					 
				Motor_1.PWM = Motor_1.CurrentExpected;//ClassicPidRegulate(Motor_1.CurrentExpected,Motor_1.CurrentMeasure,&MotorCurrentPid1);
				break; 
			case MOTOR_PWM:			
				break;
			default:
				Motor_1.PWM = 0;
				break;	
		}
		
		//CAN1_cmd_chassis(Motor_1.PWM,Motor_2.PWM,Motor_3.PWM,Motor_4.PWM);
  	//CAN2_cmd_chassis(Motor_5.PWM,Motor_6.PWM,0,0);
//		
    osDelay(1);
  }
  /* USER CODE END Regulate_Task */
}

/* USER CODE BEGIN Header_Handle_Task */
/**
* @brief Function implementing the myTask05 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Handle_Task */
void Handle_Task(void const * argument)
{
  /* USER CODE BEGIN Handle_Task */
  uint16_t pwmVal=0;   // PWM duty cycle
  uint8_t dir=1;
  /* Infinite loop */
  for(;;)
  {
    HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_1|TIM_CHANNEL_2|TIM_CHANNEL_3|TIM_CHANNEL_4);
		HAL_TIM_PWM_Start(&htim8,TIM_CHANNEL_1|TIM_CHANNEL_2|TIM_CHANNEL_3|TIM_CHANNEL_4);
		while (pwmVal< 500)
	  {
		  pwmVal++;
      __HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_1, pwmVal);    // Update compare value to change duty cycle
//		  TIM3->CCR1 = pwmVal;    Same effect as writing the register directly
		  HAL_Delay(1);
	  }
	  while (pwmVal)
	  {
		  pwmVal--;
      __HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_1, pwmVal);    // Update compare value to change duty cycle
//		  TIM3->CCR1 = pwmVal;     Same effect as writing the register directly
		  HAL_Delay(1);
	  }
	  HAL_Delay(200);
		osDelay(1);
  }
  /* USER CODE END Handle_Task */
}

/* USER CODE BEGIN Header_Usart_Task */
/**
* @brief Function implementing the myTask06 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Usart_Task */
void Usart_Task(void const * argument)
{
  /* USER CODE BEGIN Usart_Task */
  /* Infinite loop */
  for(;;)
  {
//		uint8_t UART8_BufferTX[10]="helloworld";
//   	HAL_UART_Transmit_DMA(&huart8, (uint8_t *)UART8_BufferTX,10);
     if(recv_end_flag == 1)  // Frame reception complete flag
		{
			
			Jetson_Decode();		
      recv_end_flag = 0;// Clear frame reception complete flag
      memset(ReceiveBuffer1,0,rx_len);
      rx_len = 0;// Clear received frame length
    }
    HAL_UART_Receive_DMA(&huart8,ReceiveBuffer1,BUFFERSIZE);// Restart UART8 DMA reception
		Jetson_Send_Vesc_Feedback();
    osDelay(20);
  }
  /* USER CODE END Usart_Task */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

  /* USER CODE BEGIN StartTask03 */



  /* USER CODE END StartTask03 */

/* USER CODE END Application */
