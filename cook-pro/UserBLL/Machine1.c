/* Includes -------------------------------------------- */
#include "Machine1.h"
/* Defines --------------------------------------------- */
#define dataRecord	300
#define DELAY_FOR_MANUAL 3000
#define SERVO_UPLOAD 75
#define SERVO_UNLOAD 150
#define pi 3.141592
/* Extern Members -------------------------------------- */
extern osSemaphoreId DJIMotorContolPermissionHandle;
/* Portable Members------------------------------------- */
/* Private Members ------------------------------------- */
/** �������Ƶ��ָ��(pMotor_�����_���ƻ���/�������������ʽ) */
static DJIMotor_t	*pPickupSys_XaxisScrew,
					*pPickupSys_ZaxisScrew1,
					*pPickupSys_ZaxisScrew2,
					*pPickupSys_RackPinion,
					*pPickupSys_BasketAngle,
					*pCookSys_PotCoverAngle,
					*pCookSys_ShovelSpoonSpin,
					*pSpinLiftSys_Screw,
					*pSpinLiftSys_SysAngle;
static ServoMotor_t	*pServoMotor1, *pServoMotor2, *pServoMotor3;
static char str[48];
/* Tasks ----------------------------------------------- */
/** @brief DJI����ջ��������Ʋ������� */
void DataTransmissionTaskFunc(void const * argument)
{
	// �˹�ͬ��
	osDelay(DELAY_FOR_MANUAL);
		DJIMotorMechParamTransmit(pSpinLiftSys_SysAngle,cntAngle);
		osDelay(50);
	for(;;)
	{
		osDelay(200);
		/** @CircleBEGIN */
		// ���ݴ���
		
		
		
		float delta;
		delta = pPickupSys_ZaxisScrew1->MechParam.TotalAngle - pPickupSys_ZaxisScrew2->MechParam.TotalAngle;
		sprintf(str,"%.2f ",delta);
		CDC_Transmit_FS((uint8_t*)str,sizeof(str));
		
		// ��������
		
		/** @CircleEND */
	}
}
/** @brief DJI����ջ�˳����Ʋ������� */
void _DEBUG_DJIControlTestTaskFunc(void const * argument)
{
	BaseType_t xReturn;
	float RackPinionTar,XaxisTar,ZaxisTar,SpinLiftSysAnlgeTar,LiftScrewTar,BasketTar,CoverTar;
	
	
	/*
	PickupSys_SetXaxisMove();
	PickupSys_SetZaxisMove();
	PickupSys_SetBasketRotate();
	PickupSys_SetRackPinionMove();
	SpinLiftSys_SetLiftMove();
	SpinLiftSys_SetSysRotate();
	CookSys_SetSpoonRotate();
	CookSys_SetCoverRotate();
	*/
	
	// �˹�ͬ��
	osDelay(DELAY_FOR_MANUAL);
	// �ͷſ�������ź���
	xReturn = xSemaphoreGive(DJIMotorContolPermissionHandle);
	if(xReturn != pdTRUE)
	{
		// Error
	}
	// X�Ḻ������
	/*
	SetServoMotor(pServoMotor1,SERVO_UNLOAD);
	LiftScrewTar = 50;
	LiftScrewTar = PosTarget_MMtransform(SpinLiftSys_Screw,LiftScrewTar);
	DJIMotor_SetTargetPosWithTime(pSpinLiftSys_Screw,LiftScrewTar,10,300);
	osDelay(12000);
	LiftScrewTar = -50;
	LiftScrewTar = PosTarget_MMtransform(SpinLiftSys_Screw,LiftScrewTar);
	DJIMotor_SetTargetPosWithTime(pSpinLiftSys_Screw,LiftScrewTar,10,-300);
	osDelay(12000);
	SetServoMotor(pServoMotor1,SERVO_UPLOAD);
	LiftScrewTar = 20;
	LiftScrewTar = PosTarget_MMtransform(SpinLiftSys_Screw,LiftScrewTar);
	DJIMotor_SetTargetPosWithTime(pSpinLiftSys_Screw,LiftScrewTar,6,300);
	osDelay(8000);
	SpinLiftSysAnlgeTar = -1*(1.f/4);
	DJIMotor_SetTargetPosWithTime(pSpinLiftSys_SysAngle,SpinLiftSysAnlgeTar,3,-50);
	osDelay(5000);
	LiftScrewTar = -100;
	LiftScrewTar = PosTarget_MMtransform(SpinLiftSys_Screw,LiftScrewTar);
	DJIMotor_SetTargetPosWithTime(pSpinLiftSys_Screw,LiftScrewTar,15,-300);
	osDelay(20000);
	// ���Ϲ���ǰ���ʳƷת��
	RackPinionTar = -80;
	RackPinionTar = PosTarget_MMtransform(PickUpSys_RackPinion,RackPinionTar);
	DJIMotor_SetTargetPosWithTime(pPickupSys_RackPinion,RackPinionTar,3,-100);
	osDelay(4000);
	XaxisTar = 100;
	XaxisTar = PosTarget_MMtransform(PickUpSys_XaxisScrew,XaxisTar);
	DJIMotor_SetTargetPosWithTime(pPickupSys_XaxisScrew,XaxisTar,15,300);
	osDelay(15000);
	XaxisTar = -100;
	XaxisTar = PosTarget_MMtransform(PickUpSys_XaxisScrew,XaxisTar);
	DJIMotor_SetTargetPosWithTime(pPickupSys_XaxisScrew,XaxisTar,15,-300);
	osDelay(15000);
	RackPinionTar = 80;
	RackPinionTar = PosTarget_MMtransform(PickUpSys_RackPinion,RackPinionTar);
	DJIMotor_SetTargetPosWithTime(pPickupSys_RackPinion,RackPinionTar,3,100);
	HAL_GPIO_WritePin(MAG_GPIO_Port,MAG_Pin,GPIO_PIN_SET);
	osDelay(4000);
	RackPinionTar = -80;
	RackPinionTar = PosTarget_MMtransform(PickUpSys_RackPinion,RackPinionTar);
	DJIMotor_SetTargetPosWithTime(pPickupSys_RackPinion,RackPinionTar,3,-100);
	osDelay(4000);
	RackPinionTar = 80;
	RackPinionTar = PosTarget_MMtransform(PickUpSys_RackPinion,RackPinionTar);
	DJIMotor_SetTargetPosWithTime(pPickupSys_RackPinion,RackPinionTar,3,100);
	HAL_GPIO_WritePin(MAG_GPIO_Port,MAG_Pin,GPIO_PIN_RESET);
	osDelay(4000);
	RackPinionTar = -80;
	RackPinionTar = PosTarget_MMtransform(PickUpSys_RackPinion,RackPinionTar);
	DJIMotor_SetTargetPosWithTime(pPickupSys_RackPinion,RackPinionTar,3,-100);
	osDelay(4000);
	ZaxisTar = 120;
	ZaxisTar = PosTarget_MMtransform(PickUpSys_ZaxisScrew,ZaxisTar);
	DJIMotor_SetTargetPosWithTime(pPickupSys_ZaxisScrew1,ZaxisTar,20,300);
	DJIMotor_SetTargetPosWithTime(pPickupSys_ZaxisScrew2,ZaxisTar,20,300);
	osDelay(21000);
	RackPinionTar = 80;
	RackPinionTar = PosTarget_MMtransform(PickUpSys_RackPinion,RackPinionTar);
	DJIMotor_SetTargetPosWithTime(pPickupSys_RackPinion,RackPinionTar,3,100);
	HAL_GPIO_WritePin(MAG_GPIO_Port,MAG_Pin,GPIO_PIN_SET);
	osDelay(4000);
	RackPinionTar = -80;
	RackPinionTar = PosTarget_MMtransform(PickUpSys_RackPinion,RackPinionTar);
	DJIMotor_SetTargetPosWithTime(pPickupSys_RackPinion,RackPinionTar,3,-100);
	osDelay(4000);
	RackPinionTar = 80;
	RackPinionTar = PosTarget_MMtransform(PickUpSys_RackPinion,RackPinionTar);
	DJIMotor_SetTargetPosWithTime(pPickupSys_RackPinion,RackPinionTar,3,100);
	osDelay(4000);
	HAL_GPIO_WritePin(MAG_GPIO_Port,MAG_Pin,GPIO_PIN_RESET);
	BasketTar = -4.8f/12;
	DJIMotor_SetTargetPosWithTime(pPickupSys_BasketAngle,BasketTar,3,-50);
	osDelay(4000);
	DJIMotor_SetTargetCur(pPickupSys_BasketAngle,0x0);
	osDelay(4000);
	BasketTar = 5.f/12;
	DJIMotor_SetTargetPosWithTime(pPickupSys_BasketAngle,BasketTar,3,50);
	osDelay(4000);
	DJIMotor_SetTargetCur(pPickupSys_BasketAngle,0x0);
	RackPinionTar = -80;
	RackPinionTar = PosTarget_MMtransform(PickUpSys_RackPinion,RackPinionTar);
	DJIMotor_SetTargetPosWithTime(pPickupSys_RackPinion,RackPinionTar,3,-100);
	osDelay(4000);
	ZaxisTar = -120;
	ZaxisTar = PosTarget_MMtransform(PickUpSys_ZaxisScrew,ZaxisTar);
	DJIMotor_SetTargetPosWithTime(pPickupSys_ZaxisScrew1,ZaxisTar,20,-300);
	DJIMotor_SetTargetPosWithTime(pPickupSys_ZaxisScrew2,ZaxisTar,20,-300);
	osDelay(21000);
	// ���Ϲ��ӿ�ʼ���
	CoverTar = -1.f/4;
	DJIMotor_SetTargetPosWithTime(pCookSys_PotCoverAngle,CoverTar,2,-50);
	osDelay(10000);
	DJIMotor_SetTargetVel(pCookSys_ShovelSpoonSpin,30);
	osDelay(5000);
	DJIMotor_SetTargetCur(pCookSys_ShovelSpoonSpin,0x0);
	CoverTar = 1.f/4;
	DJIMotor_SetTargetPosWithTime(pCookSys_PotCoverAngle,CoverTar,2,50);
	osDelay(10000);
	LiftScrewTar = 100;
	LiftScrewTar = PosTarget_MMtransform(SpinLiftSys_Screw,LiftScrewTar);
	DJIMotor_SetTargetPosWithTime(pSpinLiftSys_Screw,LiftScrewTar,15,300);
	osDelay(20000);
	SpinLiftSysAnlgeTar = 1*(1.f/4);
	DJIMotor_SetTargetPosWithTime(pSpinLiftSys_SysAngle,SpinLiftSysAnlgeTar,3,50);
	osDelay(5000);
	LiftScrewTar = -20;
	LiftScrewTar = PosTarget_MMtransform(SpinLiftSys_Screw,LiftScrewTar);
	DJIMotor_SetTargetPosWithTime(pSpinLiftSys_Screw,LiftScrewTar,6,-300);
	osDelay(8000);
	SetServoMotor(pServoMotor1,SERVO_UNLOAD);
	*/
	//SetCoverRotate(-1.f/4,2,-50);
	for(;;)
	{
		
		osDelay(6000);
		osDelay(5);
	}
}
/* Functions ------------------------------------------- */
/** @brief ��Բ�ͬ���������ƶ�λ��������ת��λ������ת�� */
float PosTarget_MMtransform(MechName_t name, float mechTarget)
{
	/** @Caution mechTarget��λ�Ǻ��� */
	switch(name)
	{
		case SpinLiftSys_Screw:			return (mechTarget / 4.f);
		case PickUpSys_RackPinion:		return (mechTarget / (74.f*pi));
		case PickUpSys_ZaxisScrew:		return (mechTarget / 5.f);
		case PickUpSys_XaxisScrew:		return (mechTarget / 4.f);
		default:	return 0;
	}
}

/** @brief ����������� */
void MechMotorRegister(void)
{
	// CAN1-1 ��λģ��X��˿�˴���
	DJIMotorInit_t pMotorInit_XaxisScrew 	= 	{CAN1Bus, M2006,	1.5,	0.2,	0,	12.5,	0.006,	0};
	pPickupSys_XaxisScrew	=	DJIMotorAllocate(&pMotorInit_XaxisScrew);
	// CAN1-2,3 ��λģ��Z��˫˿�˴���
	DJIMotorInit_t pMotorInit_ZaxisScrew1 	= 	{CAN1Bus, M2006,	1.7,	0.17,	0,	12.5,	0.006,	0};
	DJIMotorInit_t pMotorInit_ZaxisScrew2 	= 	{CAN1Bus, M2006,	1.7,	0.17,	0,	12.5,	0.006,	0};
	pPickupSys_ZaxisScrew1	=	DJIMotorAllocate(&pMotorInit_ZaxisScrew1);
	pPickupSys_ZaxisScrew2	=	DJIMotorAllocate(&pMotorInit_ZaxisScrew2);
	// CAN1-4 ������ת��ֱ��
	DJIMotorInit_t pMotorInit_BasketAngle	= 	{CAN1Bus, M3508,	15.1,	0.32,	1.3,	20,		0.05,	0};
	pPickupSys_BasketAngle	=	DJIMotorAllocate(&pMotorInit_BasketAngle);
	// CAN1-5 ���ֳ�������
	DJIMotorInit_t pMotorInit_RackPinion	= 	{CAN1Bus, M2006,	6,	0.25,	0,	12.5,	0.006,	0};
	pPickupSys_RackPinion	=	DJIMotorAllocate(&pMotorInit_RackPinion);
	// CAN2-1 һ�廯��̨̧��˿�˴���
	DJIMotorInit_t pMotorInit_SpinLift_Screw	=	{CAN2Bus, M2006,	1.5,	0.15,	0,	12.5,	0.006,	0};
	pSpinLiftSys_Screw	=	DJIMotorAllocate(&pMotorInit_SpinLift_Screw);
	// CAN2-2 һ�廯��̨��תֱ��
	DJIMotorInit_t pMotorInit_SpinLift_SysAngle	=	{CAN2Bus, M3508,	14,	0.54,	0,	16,		0.04,	0};
	pSpinLiftSys_SysAngle	=	DJIMotorAllocate(&pMotorInit_SpinLift_SysAngle);
	// CAN2-3 ������ת
	DJIMotorInit_t pMotorInit_Spoon	=	{CAN2Bus, M2006,	1.5,	0.17,	0,	12.5,	0.006,	0};
	pCookSys_ShovelSpoonSpin = DJIMotorAllocate(&pMotorInit_Spoon);
	// CAN2-4 ������תֱ��
	DJIMotorInit_t pMotorInit_Cover	=	{CAN2Bus, M3508,	10,	0.4,	0,	30,		0.05,	0};
	pCookSys_PotCoverAngle = DJIMotorAllocate(&pMotorInit_Cover);
	// �������
	pServoMotor1 = CreateServoMotor(&htim5,TIM_CHANNEL_4);
	//pServoMotor2 = CreateServoMotor(&htim5,TIM_CHANNEL_3);
	//pServoMotor3 = CreateServoMotor(&htim5,TIM_CHANNEL_2);
}

/** @brief X��˿��λ���趨 */
void SetXaxisMove(float pTar, uint32_t limTime, float vStart)
{
	float shaftTar;
	shaftTar = PosTarget_MMtransform(PickUpSys_XaxisScrew,pTar);
	DJIMotor_SetTargetPosWithTime(pPickupSys_XaxisScrew,shaftTar,limTime,vStart);
}
/** @brief Z��˿��λ���趨 */
void SetZaxisMove(float pTar, uint32_t limTime, float vStart)
{
	float shaftTar;
	shaftTar = PosTarget_MMtransform(PickUpSys_ZaxisScrew,pTar);
	DJIMotor_SetTargetPosWithTime(pPickupSys_ZaxisScrew1,shaftTar,limTime,vStart);
	DJIMotor_SetTargetPosWithTime(pPickupSys_ZaxisScrew2,shaftTar,limTime,vStart);
}
/** @brief ���ֳ���λ���趨 */
void SetRackPinionMove(float pTar, uint32_t limTime, float vStart)
{
	float shaftTar;
	shaftTar = PosTarget_MMtransform(PickUpSys_RackPinion,pTar);
	DJIMotor_SetTargetPosWithTime(pPickupSys_RackPinion,shaftTar,limTime,vStart);
}
/** @brief ����ת���趨 */
void SetBasketRotate(float pTar, uint32_t limTime, float vStart)
{
	float shaftTar;
	shaftTar = pTar;
	DJIMotor_SetTargetPosWithTime(pPickupSys_BasketAngle,shaftTar,limTime,vStart);
}
void SetBasketShake(void)
{
	DJIMotor_SetTargetCur(pPickupSys_BasketAngle,0x0);
}
/** @brief ̧��˿��λ���趨 */
void SetLiftScrewMove(float pTar, uint32_t limTime, float vStart)
{
	float shaftTar;
	shaftTar = PosTarget_MMtransform(SpinLiftSys_Screw,pTar);
	DJIMotor_SetTargetPosWithTime(pSpinLiftSys_Screw,shaftTar,limTime,vStart);
}
/** @brief һ�廯��̨ת���趨 */
void SetLiftSysRotate(float pTar, uint32_t limTime, float vStart)
{
	float shaftTar;
	shaftTar = pTar;
	DJIMotor_SetTargetPosWithTime(pSpinLiftSys_SysAngle,shaftTar,limTime,vStart);
}
/** @brief ����ת���趨 */
void SetCoverRotate(float pTar, uint32_t limTime, float vStart)
{
	float shaftTar;
	shaftTar = pTar;
	DJIMotor_SetTargetPosWithTime(pCookSys_PotCoverAngle,shaftTar,limTime,vStart);
}
void SetCoverRest(void)
{
	DJIMotor_SetTargetCur(pCookSys_PotCoverAngle,0x0);
}
/** @brief ���׷��������趨 */
void SetShovelOn(float vTar)
{
	DJIMotor_SetTargetVel(pCookSys_ShovelSpoonSpin,vTar);
}
void SetShovelOff(void)
{
	DJIMotor_SetTargetCur(pCookSys_ShovelSpoonSpin,0x0);
}
/** @brief ����������趨 */
void Mag_ON(void)
{
	HAL_GPIO_WritePin(MAG_GPIO_Port,MAG_Pin,GPIO_PIN_SET);
}
void Mag_OFF(void)
{
	HAL_GPIO_WritePin(MAG_GPIO_Port,MAG_Pin,GPIO_PIN_RESET);
}
/** @brief ץ�ֿ����趨 */
void Grasper_Upload(void)
{
	SetServoMotor(pServoMotor1,SERVO_UPLOAD);
}
void Grasper_Unload(void)
{
	SetServoMotor(pServoMotor1,SERVO_UNLOAD);
}
