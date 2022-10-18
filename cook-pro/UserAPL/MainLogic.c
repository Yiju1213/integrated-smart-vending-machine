#include "MainLogic.h"
/* Defines --------------------------------------------- */
#define GRIDLEN 		80
#define LAYERLEN		110
#define BOXSAFELEN	80
/* Extern Members -------------------------------------- */
extern osSemaphoreId SendingReadyHandle;
extern osSemaphoreId ReceivingReadyHandle;
extern osSemaphoreId DJIMotorContolPermissionHandle;
/* Portable Members------------------------------------- */
/* Private Members ------------------------------------- */
void UpperOperationTaskFunc(void const * argument)
{
	xSemaphoreTake(SendingReadyHandle,portMAX_DELAY);
	
	// 先取走信号量，等到申请完电机再释放给ControlTask进行电机控制任务
	xSemaphoreTake(DJIMotorContolPermissionHandle,portMAX_DELAY);
	/** @Tips 进入任务调度再使能含有freertos API的中断 */
	HAL_CAN_ActivateNotification(&hcan1,CAN_IT_RX_FIFO0_MSG_PENDING);
	HAL_CAN_ActivateNotification(&hcan2,CAN_IT_RX_FIFO0_MSG_PENDING);
	// 延时5ms，保证获取OffsetAngle函数
	osDelay(5);
	// 机构电机申请
	MechMotorRegister();
	// 人工同步
	osDelay(3000);
	// 释放控制许可信号量
	xSemaphoreGive(DJIMotorContolPermissionHandle);
	
	
	
		
		SetRackPinionMove(-BOXSAFELEN,3,-100);
		osDelay(3000);
		
		// 等待可操作信号
		// ...
		// 定位暨提取操作
		// for(foodtaken = 0 ; foodtaken < foodtotal; foodtaken++ )
		SetXaxisMove(20,10,100);
		osDelay(10000);
		SetXaxisMove(-20,10,-100);
		osDelay(10000);
		
		SetRackPinionMove(BOXSAFELEN,3,100);
		Mag_ON();
		osDelay(5000);
		
		osDelay(3000);
		SetRackPinionMove(-GRIDLEN,3,-100);
		osDelay(3000);
		SetRackPinionMove(GRIDLEN,3,100);
		osDelay(3000);
		Mag_OFF();
		SetRackPinionMove(-BOXSAFELEN,3,-100);
		osDelay(2000);
		SetZaxisMove(LAYERLEN,15,100);
		osDelay(15000);
		SetRackPinionMove(BOXSAFELEN,3,100);
		Mag_ON();
		osDelay(3000);
		SetRackPinionMove(-GRIDLEN,3,-100);
		osDelay(3000);
		SetRackPinionMove(GRIDLEN,3,100);
		osDelay(3000);
		Mag_OFF();
		SetRackPinionMove(-BOXSAFELEN,3,-100);
		osDelay(2000);
		// 提取完应移动到可投放点
		SetZaxisMove(50,10,100);
		osDelay(10000);
		// XZ_Distance_Calculate
		// Xaxis/Zaxis Move
		// 送出可送出信号
		xSemaphoreGive(SendingReadyHandle);
		// 等待可接收信号
		xSemaphoreTake(ReceivingReadyHandle,portMAX_DELAY);
		SetBasketRotate(-4.8f/12,3,-50);
		osDelay(4000);
		SetBasketShake();
		osDelay(4000);
		SetBasketRotate(5.f/12,3,50);
		osDelay(4000);
		SetBasketShake();
		// 复位
		SetZaxisMove(-160,30,-100);
		
	for(;;)
	{
		osDelay(100);
		
	}
}
void NetherOperationTaskFunc(void const * argument)
{
	// 等待业务逻辑层创建好电机再释放可操作信号量
	xSemaphoreTake(DJIMotorContolPermissionHandle,portMAX_DELAY);
	xSemaphoreTake(ReceivingReadyHandle,portMAX_DELAY);
	
	// 等待可操作信号
		// ...
		Grasper_Unload();
		SetLiftScrewMove(-50,10,-100);
		osDelay(10000);
		// 留有抓取时间
		Grasper_Upload();
		osDelay(2000);
		SetLiftScrewMove(50,10,100);
		osDelay(10000);
		SetLiftSysRotate(-1.f/4,3,-50);
		osDelay(3000);
		// 给云台旋转直驱留有6000ms修正时间
		SetLiftScrewMove(-65,10,-100);
		osDelay(6000);
		
		// 送出准备接收信号
		xSemaphoreGive(ReceivingReadyHandle);
		// 等待可送出信号
		xSemaphoreTake(SendingReadyHandle,portMAX_DELAY);
		// 等待12s的菜品入锅操作
		osDelay(12000);
		
		//开始烹饪
		SetCoverRotate(-1.f/4,2,-50);
		osDelay(3000);
		SetCoverRest();
		SetShovelOn(20);
		osDelay(4000);
		SetShovelOff();
		SetCoverRotate(1.f/4,2,50);
		osDelay(2000);
		SetLiftScrewMove(65,10,100);
		osDelay(10000);
		SetLiftSysRotate(1.f/4,3,50);
		osDelay(3000);
		SetLiftScrewMove(-50,10,-100);
		osDelay(10000);
		Grasper_Unload();
		// 复位
		SetLiftScrewMove(50,10,100);
	
	for(;;)
	{
		osDelay(100);
		
	}
}




