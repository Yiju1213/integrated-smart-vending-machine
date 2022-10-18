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
	int16_t GivenCurrent;		// 设定电流值
	float Current;				// 实时测量电流值
	int16_t	Torque;				// 实时测量转矩
	uint16_t OffsetAngle;		// 初始偏移角度
	uint16_t Angle;				// 实时测量角度
	uint16_t LastAngle;			// 上次测量角度
	int16_t SpeedRpm;			// 实时测量转速
	int16_t Round;				// 所转总整圈数
	float TotalAngle;			// 转子所转总角度（按圈数计）
	float LastExpectTotalAngle;	// 上次位置控制转子端总角度期望值
}MechParam_t;
typedef struct
{
	float		Target;				// 控制目标值
	float		Kp;					// 比例系数
	float		Ki;					// 积分系数
	float		Kd;					// 微分系数
	float		Err;				// 误差 
	float		LastErr;     		// 上次误差
	/** @Caution 积分限幅可以看作停止条件或者限位条件？ */
  #if (USE_IntegralLimit == 1)
	uint16_t 	IntegralLimit;		// 积分限幅
  #endif
	uint16_t	MaxOutput;			// 输出限幅
	float		Pout;				// 比例项输出
	float		Iout;				// 积分项输出
	float		Dout;				// 微分项输出
	float 		Output;				// 本次输出
}PIDParam_t;
typedef struct
{
	uint8_t 	ID;					// 电机序号
	MotorModel_t Model;				// 电机型号
	CANBus_t Bus;					// CAN总线
	uint16_t CAN_RxID;				// 接收报文ID
	uint16_t CAN_TxID;				// 发送报文ID
	MechParam_t MechParam;			// 电机机械参数
	PIDParam_t	PIDParamPos;		// 位置环PID参数
	PIDParam_t	PIDParamVel;		// 速度环PID参数
	ControlForm_t ControlForm;		// 电机控制形式
	uint32_t	TimeStart;			// 控制起始时间
	uint32_t	TimeLimit;			// 控制时间
	QuintPolyCoef_t	CoefArray;		// 多项式系数组结构体
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
