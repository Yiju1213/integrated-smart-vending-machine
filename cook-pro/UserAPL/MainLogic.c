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
	
	// ��ȡ���ź������ȵ������������ͷŸ�ControlTask���е����������
	xSemaphoreTake(DJIMotorContolPermissionHandle,portMAX_DELAY);
	/** @Tips �������������ʹ�ܺ���freertos API���ж� */
	HAL_CAN_ActivateNotification(&hcan1,CAN_IT_RX_FIFO0_MSG_PENDING);
	HAL_CAN_ActivateNotification(&hcan2,CAN_IT_RX_FIFO0_MSG_PENDING);
	// ��ʱ5ms����֤��ȡOffsetAngle����
	osDelay(5);
	// �����������
	MechMotorRegister();
	// �˹�ͬ��
	osDelay(3000);
	// �ͷſ�������ź���
	xSemaphoreGive(DJIMotorContolPermissionHandle);
	
	
	
		
		SetRackPinionMove(-BOXSAFELEN,3,-100);
		osDelay(3000);
		
		// �ȴ��ɲ����ź�
		// ...
		// ��λ����ȡ����
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
		// ��ȡ��Ӧ�ƶ�����Ͷ�ŵ�
		SetZaxisMove(50,10,100);
		osDelay(10000);
		// XZ_Distance_Calculate
		// Xaxis/Zaxis Move
		// �ͳ����ͳ��ź�
		xSemaphoreGive(SendingReadyHandle);
		// �ȴ��ɽ����ź�
		xSemaphoreTake(ReceivingReadyHandle,portMAX_DELAY);
		SetBasketRotate(-4.8f/12,3,-50);
		osDelay(4000);
		SetBasketShake();
		osDelay(4000);
		SetBasketRotate(5.f/12,3,50);
		osDelay(4000);
		SetBasketShake();
		// ��λ
		SetZaxisMove(-160,30,-100);
		
	for(;;)
	{
		osDelay(100);
		
	}
}
void NetherOperationTaskFunc(void const * argument)
{
	// �ȴ�ҵ���߼��㴴���õ�����ͷſɲ����ź���
	xSemaphoreTake(DJIMotorContolPermissionHandle,portMAX_DELAY);
	xSemaphoreTake(ReceivingReadyHandle,portMAX_DELAY);
	
	// �ȴ��ɲ����ź�
		// ...
		Grasper_Unload();
		SetLiftScrewMove(-50,10,-100);
		osDelay(10000);
		// ����ץȡʱ��
		Grasper_Upload();
		osDelay(2000);
		SetLiftScrewMove(50,10,100);
		osDelay(10000);
		SetLiftSysRotate(-1.f/4,3,-50);
		osDelay(3000);
		// ����̨��תֱ������6000ms����ʱ��
		SetLiftScrewMove(-65,10,-100);
		osDelay(6000);
		
		// �ͳ�׼�������ź�
		xSemaphoreGive(ReceivingReadyHandle);
		// �ȴ����ͳ��ź�
		xSemaphoreTake(SendingReadyHandle,portMAX_DELAY);
		// �ȴ�12s�Ĳ�Ʒ�������
		osDelay(12000);
		
		//��ʼ���
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
		// ��λ
		SetLiftScrewMove(50,10,100);
	
	for(;;)
	{
		osDelay(100);
		
	}
}




