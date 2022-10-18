#ifndef DJIMOTOR_H
#define DJIMOTOR_H
/* Includes ---------------------------------------------- */
#include "main.h"
#include "cmsis_os.h"
#include "freertos.h"
#include "can.h"
#include "usb_device.h"
#include "usbd_cdc_if.h"
#include "math.h"
#include "stdint.h"
/* Additional Configuration ------------------------------ */
#define USE_IntegralLimit 0
#define USE_UpdateWithinISR 0
/* Trajectory Planning Struct ---------------------------- */
typedef struct
{
	float a[6];
}QuintPolyCoef_t;
/* DJIMotor Params Struct--------------------------------- */
typedef enum
{
	M2006 = 1,
	M3508
}MotorModel_t;
typedef enum
{
	CAN1Bus = 1,
	CAN2Bus
}CANBus_t;
typedef enum
{
	Cur = 1,
	Vel,
	PosNoTime,
	PosWithTime,
}ControlForm_t;
typedef struct
{
	int16_t GivenCurrent;		// �趨����ֵ
	float Current;				// ʵʱ��������ֵ
	int16_t	Torque;				// ʵʱ����ת��
	uint16_t OffsetAngle;		// ��ʼƫ�ƽǶ�
	uint16_t Angle;				// ʵʱ�����Ƕ�
	uint16_t LastAngle;			// �ϴβ����Ƕ�
	int16_t SpeedRpm;			// ʵʱ����ת��
	int16_t Round;				// ��ת����Ȧ��
	float TotalAngle;			// ת����ת�ܽǶȣ���Ȧ���ƣ�
	float LastExpectTotalAngle;	// �ϴ�λ�ÿ���ת�Ӷ��ܽǶ�����ֵ
}MechParam_t;
typedef struct
{
	float		Target;				// ����Ŀ��ֵ
	float		Kp;					// ����ϵ��
	float		Ki;					// ����ϵ��
	float		Kd;					// ΢��ϵ��
	float		Err;				// ��� 
	float		LastErr;     		// �ϴ����
	/** @Caution �����޷����Կ���ֹͣ����������λ������ */
  #if (USE_IntegralLimit == 1)
	uint16_t 	IntegralLimit;		// �����޷�
  #endif
	uint16_t	MaxOutput;			// ����޷�
	float		Pout;				// ���������
	float		Iout;				// ���������
	float		Dout;				// ΢�������
	float 		Output;				// �������
}PIDParam_t;
typedef struct
{
	uint8_t 	ID;					// ������
	MotorModel_t Model;				// ����ͺ�
	CANBus_t Bus;					// CAN����
	uint16_t CAN_RxID;				// ���ձ���ID
	uint16_t CAN_TxID;				// ���ͱ���ID
	MechParam_t MechParam;			// �����е����
	PIDParam_t	PIDParamPos;		// λ�û�PID����
	PIDParam_t	PIDParamVel;		// �ٶȻ�PID����
	ControlForm_t ControlForm;		// ���������ʽ
	uint32_t	TimeStart;			// ������ʼʱ��
	uint32_t	TimeLimit;			// ����ʱ��
	QuintPolyCoef_t	CoefArray;		// ����ʽϵ����ṹ��
}DJIMotor_t;
/* DJIMotor Init Struct ---------------------------------- */
typedef struct
{
	CANBus_t bus;
	MotorModel_t model;
	float kp_vel;
	float ki_vel;
	float kd_vel;
	float kp_pos;
	float ki_pos;
	float kd_pos;
}DJIMotorInit_t;
/* CAN Message Struct ------------------------------------ */
typedef struct
{
	CAN_RxHeaderTypeDef Header;
	uint8_t Data[8];
}RxMsg_t;
typedef struct
{
	uint32_t ID;
	uint8_t Data[8];
	CANBus_t Bus;
}DJIMsg_t;
typedef struct
{
	CAN_TxHeaderTypeDef Header;
	uint8_t Data[8];
}TxMsg_t;
/* CAN Related Function ---------------------------------- */
void CAN_ConfigAndStart(void);
void CAN_CurrentTransimit(CAN_HandleTypeDef* hcan, int16_t* ControlCurrent);
/* DJI Motor Initialization Function --------------------- */
DJIMotor_t* DJIMotorAllocate(DJIMotorInit_t* This);
/* DJI Motor Control Function ---------------------------- */
void DJIMotor_SetTargetCur(DJIMotor_t* This, int16_t setCur);
void DJIMotor_SetTargetVel(DJIMotor_t* This, float setShaftVel);
void DJIMotor_SetTargetPosWithTime(DJIMotor_t* This, float setShaftPos, uint32_t setTime, float startVel);
void DJIMotor_SetTargetPos(DJIMotor_t* This, float setRotorPos);
void DJIMotor_SetTarget_HalfRound(DJIMotor_t* This);
void DJIMotor_MechParamUpdate(DJIMotor_t* This,DJIMsg_t* msg);
void DJIMotorControl(DJIMotor_t* pSpecific);
void DJIMotorControl_PosNoTime(DJIMotor_t* This);
void DJIMotorControl_PosWithTime(DJIMotor_t* This);
void DJIMotorControl_Vel(DJIMotor_t* This);
void DJIMotorControl_Cur(DJIMotor_t* This);
void DJIMotor_EmergencyStop(void);
/* Culculate Related Function----------------------------- */
float PID_Culculate(PIDParam_t *pid, float measure);
float TargetRRtransform(MotorModel_t model,float target);
QuintPolyCoef_t TrajPlan_QuinticPoly(float sf_delta, float tf,float startVel);
float VelPolyFunc(int power, QuintPolyCoef_t Coef, float t);
/* VPC Related ------------------------------------------- */
typedef enum
{
	speed = 1,
	offsetAngle,
	velTarget,
	posTarget,
	cntAngle,
	givenCur,
	velOutput,
	bus,
	data
}ParamEnum_t;
void DJIMotorMechParamTransmit(DJIMotor_t* This, ParamEnum_t mechparam);
#endif
