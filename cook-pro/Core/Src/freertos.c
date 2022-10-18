/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
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
#define CAN_RX_QUEUE_LEN	20
#define CAN_RX_QUEUE_SIZE	16
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
QueueHandle_t CAN_Rx_Queue;
/* USER CODE END Variables */
osThreadId IDLETaskHandle;
osThreadId DJIMotorStateUpdateTaskHandle;
osThreadId DJIMotorControlTaskHandle;
osThreadId DataTransmissiontaskHandle;
osThreadId _DEBUG_DJIControlTestTaskHandle;
osThreadId UpperOperationTaskHandle;
osThreadId NetherOperationTaskHandle;
osTimerId DJIMotorWatchTimerHandle;
osSemaphoreId DJIMotorContolPermissionHandle;
osSemaphoreId ReceivingReadyHandle;
osSemaphoreId SendingReadyHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);
void DJIMotorStateUpdateTaskFunc(void const * argument);
void DJIMotorControlTaskFunc(void const * argument);
void DataTransmissionTaskFunc(void const * argument);
void _DEBUG_DJIControlTestTaskFunc(void const * argument);
void UpperOperationTaskFunc(void const * argument);
void NetherOperationTaskFunc(void const * argument);
void DJIMotorWatchTimerCallBack(void const * argument);

extern void MX_USB_DEVICE_Init(void);
void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* GetTimerTaskMemory prototype (linked to static allocation support) */
void vApplicationGetTimerTaskMemory( StaticTask_t **ppxTimerTaskTCBBuffer, StackType_t **ppxTimerTaskStackBuffer, uint32_t *pulTimerTaskStackSize );

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

/* USER CODE BEGIN GET_TIMER_TASK_MEMORY */
static StaticTask_t xTimerTaskTCBBuffer;
static StackType_t xTimerStack[configTIMER_TASK_STACK_DEPTH];

void vApplicationGetTimerTaskMemory( StaticTask_t **ppxTimerTaskTCBBuffer, StackType_t **ppxTimerTaskStackBuffer, uint32_t *pulTimerTaskStackSize )
{
  *ppxTimerTaskTCBBuffer = &xTimerTaskTCBBuffer;
  *ppxTimerTaskStackBuffer = &xTimerStack[0];
  *pulTimerTaskStackSize = configTIMER_TASK_STACK_DEPTH;
  /* place for user code */
}
/* USER CODE END GET_TIMER_TASK_MEMORY */

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

  /* Create the semaphores(s) */
  /* definition and creation of DJIMotorContolPermission */
  osSemaphoreDef(DJIMotorContolPermission);
  DJIMotorContolPermissionHandle = osSemaphoreCreate(osSemaphore(DJIMotorContolPermission), 1);

  /* definition and creation of ReceivingReady */
  osSemaphoreDef(ReceivingReady);
  ReceivingReadyHandle = osSemaphoreCreate(osSemaphore(ReceivingReady), 1);

  /* definition and creation of SendingReady */
  osSemaphoreDef(SendingReady);
  SendingReadyHandle = osSemaphoreCreate(osSemaphore(SendingReady), 1);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* Create the timer(s) */
  /* definition and creation of DJIMotorWatchTimer */
  osTimerDef(DJIMotorWatchTimer, DJIMotorWatchTimerCallBack);
  DJIMotorWatchTimerHandle = osTimerCreate(osTimer(DJIMotorWatchTimer), osTimerPeriodic, NULL);

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  CAN_Rx_Queue = xQueueCreate((UBaseType_t)CAN_RX_QUEUE_LEN,(UBaseType_t)CAN_RX_QUEUE_SIZE);
  if(NULL == CAN_Rx_Queue)
  {
	  // Queue Create Error
  }
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of IDLETask */
  osThreadDef(IDLETask, StartDefaultTask, osPriorityIdle, 0, 128);
  IDLETaskHandle = osThreadCreate(osThread(IDLETask), NULL);

  /* definition and creation of DJIMotorStateUpdateTask */
  osThreadDef(DJIMotorStateUpdateTask, DJIMotorStateUpdateTaskFunc, osPriorityBelowNormal, 0, 256);
  DJIMotorStateUpdateTaskHandle = osThreadCreate(osThread(DJIMotorStateUpdateTask), NULL);

  /* definition and creation of DJIMotorControlTask */
  osThreadDef(DJIMotorControlTask, DJIMotorControlTaskFunc, osPriorityNormal, 0, 256);
  DJIMotorControlTaskHandle = osThreadCreate(osThread(DJIMotorControlTask), NULL);

  /* definition and creation of DataTransmissiontask */
  osThreadDef(DataTransmissiontask, DataTransmissionTaskFunc, osPriorityAboveNormal, 0, 256);
  DataTransmissiontaskHandle = osThreadCreate(osThread(DataTransmissiontask), NULL);

  /* definition and creation of _DEBUG_DJIControlTestTask */
  osThreadDef(_DEBUG_DJIControlTestTask, _DEBUG_DJIControlTestTaskFunc, osPriorityHigh, 0, 256);
  _DEBUG_DJIControlTestTaskHandle = osThreadCreate(osThread(_DEBUG_DJIControlTestTask), NULL);

  /* definition and creation of UpperOperationTask */
  osThreadDef(UpperOperationTask, UpperOperationTaskFunc, osPriorityRealtime, 0, 256);
  UpperOperationTaskHandle = osThreadCreate(osThread(UpperOperationTask), NULL);

  /* definition and creation of NetherOperationTask */
  osThreadDef(NetherOperationTask, NetherOperationTaskFunc, osPriorityHigh, 0, 256);
  NetherOperationTaskHandle = osThreadCreate(osThread(NetherOperationTask), NULL);

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
  /* init code for USB_DEVICE */
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_DJIMotorStateUpdateTaskFunc */
/**
* @brief Function implementing the DJIMotorStateUpdateTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_DJIMotorStateUpdateTaskFunc */
__weak void DJIMotorStateUpdateTaskFunc(void const * argument)
{
  /* USER CODE BEGIN DJIMotorStateUpdateTaskFunc */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END DJIMotorStateUpdateTaskFunc */
}

/* USER CODE BEGIN Header_DJIMotorControlTaskFunc */
/**
* @brief Function implementing the DJIMotorControlTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_DJIMotorControlTaskFunc */
__weak void DJIMotorControlTaskFunc(void const * argument)
{
  /* USER CODE BEGIN DJIMotorControlTaskFunc */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END DJIMotorControlTaskFunc */
}

/* USER CODE BEGIN Header_DataTransmissionTaskFunc */
/**
* @brief Function implementing the DataTransmissiontask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_DataTransmissionTaskFunc */
__weak void DataTransmissionTaskFunc(void const * argument)
{
  /* USER CODE BEGIN DataTransmissionTaskFunc */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END DataTransmissionTaskFunc */
}

/* USER CODE BEGIN Header__DEBUG_DJIControlTestTaskFunc */
/**
* @brief Function implementing the _DEBUG_DJIControlTestTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header__DEBUG_DJIControlTestTaskFunc */
__weak void _DEBUG_DJIControlTestTaskFunc(void const * argument)
{
  /* USER CODE BEGIN _DEBUG_DJIControlTestTaskFunc */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END _DEBUG_DJIControlTestTaskFunc */
}

/* USER CODE BEGIN Header_UpperOperationTaskFunc */
/**
* @brief Function implementing the UpperOperationTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_UpperOperationTaskFunc */
__weak void UpperOperationTaskFunc(void const * argument)
{
  /* USER CODE BEGIN UpperOperationTaskFunc */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END UpperOperationTaskFunc */
}

/* USER CODE BEGIN Header_NetherOperationTaskFunc */
/**
* @brief Function implementing the NetherOperationTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_NetherOperationTaskFunc */
__weak void NetherOperationTaskFunc(void const * argument)
{
  /* USER CODE BEGIN NetherOperationTaskFunc */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END NetherOperationTaskFunc */
}

/* DJIMotorWatchTimerCallBack function */
__weak void DJIMotorWatchTimerCallBack(void const * argument)
{
  /* USER CODE BEGIN DJIMotorWatchTimerCallBack */

  /* USER CODE END DJIMotorWatchTimerCallBack */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
