/**
  ******************************************************************************
  * @file           : DJIMotor.c
  * @author         : WYR
  * @brief          : DJI电机闭环控制模块
  ******************************************************************************
  * @attention
  ******************************************************************************
*/
/* Includes -------------------------------------------- */
#include "DJIMotor.h"
/* Defines --------------------------------------------- */
#define DJIMOTOR_SINGLEBUS_NUM 6
#define DJIMotorRxIDStart	0x200
#define DJIMotorRxIDEnd		0x208
#define ALARM_TIME  5
#define PosDeadRev	0.25
#define ABS(x)		((x>0)? (x): -(x)) 
#define S0_DELTA	0
#define VEL0		0
#define VELF		0
#define ACC0		0
#define ACCF		0
#define Quintic		5
#define PI	3.141592
#define M2006RR		36.f
#define M3508RR		3591/187.f
#define MIN_TO_SEC	60
/* Extern Members -------------------------------------- */
extern QueueHandle_t CAN_Rx_Queue;
extern osTimerId DJIMotorWatchTimerHandle;
extern osSemaphoreId DJIMotorContolPermissionHandle;
/* Portable Members ------------------------------------ */
/* Private Members ------------------------------------- */
static DJIMotor_t prvCAN1DJIMotor[DJIMOTOR_SINGLEBUS_NUM];
static DJIMotor_t prvCAN2DJIMotor[DJIMOTOR_SINGLEBUS_NUM];
static uint8_t prvDJIMotorCAN1Watcher[DJIMOTOR_SINGLEBUS_NUM];
static uint8_t prvDJIMotorCAN2Watcher[DJIMOTOR_SINGLEBUS_NUM];
static uint8_t prvDJIMotorCAN1Index = 0;
static uint8_t prvDJIMotorCAN2Index = 0;
static char str[48];
static float pendCnt;
static float debugData;
/* Tasks ----------------------------------------------- */
/** @brief DJI电机状态更新任务 */
void DJIMotorStateUpdateTaskFunc(void const * argument)
{
	BaseType_t xReturn;
	DJIMsg_t DJIMsg;
    for(;;)
    {
#if (USE_UpdateWithinISR == 0)
		// 无限期接收
		xReturn = xQueueReceive(CAN_Rx_Queue,(void*)&DJIMsg,portMAX_DELAY);
		// 接收成功
		if(pdPASS == xReturn)
		{
			// 判断是哪一条CAN总线，清空对应报警位，并调用电机信息更新函数
			switch(DJIMsg.Bus)
			{
				case CAN1Bus:
					prvDJIMotorCAN1Watcher[DJIMsg.ID] = 0;
					DJIMotor_MechParamUpdate(&prvCAN1DJIMotor[DJIMsg.ID],&DJIMsg);
					break;
				case CAN2Bus:
					prvDJIMotorCAN2Watcher[DJIMsg.ID] = 0;
					DJIMotor_MechParamUpdate(&prvCAN2DJIMotor[DJIMsg.ID],&DJIMsg);
					break;
			}
		}
		else
		{
			// Receive Error 接收出错
		}
#else
		osDelay(1000);
#endif	
    }
}

/** @brief DJI电机控制指令下发任务 */
void DJIMotorControlTaskFunc(void const * argument)
{
	uint8_t i;
	DJIMotor_t *pSpecific;
	// 每条CAN总线最多容纳8台DJIMOTOR，但每块电调中心板只能容纳6台DJIMOTOR
	int16_t currentArrayCAN1[DJIMOTOR_SINGLEBUS_NUM] = {0};
	int16_t currentArrayCAN2[DJIMOTOR_SINGLEBUS_NUM] = {0};
	// 等待业务逻辑层创建好电机再释放可操作信号量
	// xSemaphoreTake(DJIMotorContolPermissionHandle,portMAX_DELAY);
	/** @Tips 打开电机监控软件定时器，周期10ms，报警周期为5次 */
	// 这时候再开启，电机已经分配好了，两条总线上电机数是确定的（即指标数）
	//osTimerStart(DJIMotorWatchTimerHandle,100);
    for(;;)
    {
		// 1.1 CAN1总线上电机控制电流迭代
		for(i=0;i<prvDJIMotorCAN1Index;i++)
		{
			pSpecific = &prvCAN1DJIMotor[i];
			// 1.2 区分不同控制形式进行计算迭代
			DJIMotorControl(pSpecific);
			// 1.3 计算结果放入CAN1控制电流数组
			currentArrayCAN1[i] = pSpecific->MechParam.GivenCurrent;
		}
		// 1.4 发送CAN1控制电流
		if(prvDJIMotorCAN1Index > 0)
			CAN_CurrentTransimit(&hcan1,currentArrayCAN1);
		// 2.1 CAN2总线上电机控制电流迭代
		for(i=0;i<prvDJIMotorCAN2Index;i++)
		{
			pSpecific = &prvCAN2DJIMotor[i];
			// 2.2 区分不同控制形式进行计算迭代
			DJIMotorControl(pSpecific);
			// 2.3 将计算结果放入CAN2控制电流数组
			currentArrayCAN2[i] = pSpecific->MechParam.GivenCurrent;
		}
		// 2.4 发送CAN2控制电流
		if(prvDJIMotorCAN2Index > 0)
			CAN_CurrentTransimit(&hcan2,currentArrayCAN2);
        osDelay(10);
    }
}
/* Timers Callback Func -------------------------------- */
/** @brief DJI电机监测定时器回调函数 */
void DJIMotorWatchTimerCallBack(void const * argument)
{
	/** @Caution 快进快出 */
	uint8_t cnt;
	uint8_t times;
	for(cnt=0;cnt<prvDJIMotorCAN1Index;cnt++)
	{
		// 离线警报
		times = (prvDJIMotorCAN1Watcher[cnt]++);
		if(times >= ALARM_TIME)
		{
			HAL_GPIO_WritePin(LED_R_GPIO_Port,LED_R_Pin,GPIO_PIN_RESET);
			DJIMotor_EmergencyStop();
		}
		// 堵转警报
		
	}
	for(cnt=0;cnt<prvDJIMotorCAN2Index;cnt++)
	{
		times = (prvDJIMotorCAN2Watcher[cnt]++);
		if(times >= ALARM_TIME)
		{
			HAL_GPIO_WritePin(LED_R_GPIO_Port,LED_R_Pin,GPIO_PIN_RESET);
			DJIMotor_EmergencyStop();
		}
		// 堵转警报
	}
	
}
/* Functions ------------------------------------------- */
/** @brief CAN配置与启动函数 */
void CAN_ConfigAndStart(void)
{
	
	// 3. 使能CAN接收中断
	/** @Tips 不要在任务调度启动前使能会调用freeertos API的中断
	if(HAL_CAN_ActivateNotification(&hcan1,CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
	if(HAL_CAN_ActivateNotification(&hcan2,CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
	*/
}
/** @brief CAN接收中断回调函数 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	RxMsg_t RxMsg;
	DJIMsg_t DJIMsg;
	BaseType_t pxHigherPriorityTaskWoken;
	// 1. 接收数据帧
	HAL_CAN_GetRxMessage(hcan,CAN_RX_FIFO0,&(RxMsg.Header),RxMsg.Data);
	// 2. 报文简化
	DJIMsg.ID = RxMsg.Header.StdId - DJIMotorRxIDStart - 1;
	memcpy(DJIMsg.Data,RxMsg.Data,sizeof(RxMsg.Data));
	if(hcan == &hcan1)
		DJIMsg.Bus = CAN1Bus;
	if(hcan == &hcan2)
		DJIMsg.Bus = CAN2Bus;
	// 3. 中断中队列传输报文
	if((RxMsg.Header.StdId > DJIMotorRxIDStart) && \
	   (RxMsg.Header.StdId <= DJIMotorRxIDEnd))
	{
#if	(USE_UpdateWithinISR == 0)
		xQueueSendFromISR(CAN_Rx_Queue,(void*)&DJIMsg,&pxHigherPriorityTaskWoken);
		
		if(pxHigherPriorityTaskWoken)
			portYIELD_FROM_ISR(pxHigherPriorityTaskWoken);
#else
		switch(DJIMsg.Bus)
		{
			case CAN1Bus:
				prvDJIMotorCAN1Watcher[DJIMsg.ID] = 0;
				DJIMotor_MechParamUpdate(&prvCAN1DJIMotor[DJIMsg.ID],&DJIMsg);
				break;
			case CAN2Bus:
				prvDJIMotorCAN2Watcher[DJIMsg.ID] = 0;
				DJIMotor_MechParamUpdate(&prvCAN2DJIMotor[DJIMsg.ID],&DJIMsg);
				break;
		}
#endif
	}
	__HAL_CAN_ENABLE_IT(hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
}
/** @brief DJI电机机械参数更新 */
void DJIMotor_MechParamUpdate(DJIMotor_t* This, DJIMsg_t* pMsg)
{
	// 1. 角度信息更新
	This->MechParam.LastAngle = This->MechParam.Angle;
	This->MechParam.Angle = (uint16_t)(pMsg->Data[0]<<8 | pMsg->Data[1]);
	// 2. 转速信息更新
	This->MechParam.SpeedRpm = (int16_t)(pMsg->Data[2]<<8 | pMsg->Data[3]);
	if(This->Model == M2006)
	{
		// 3.1 转矩信息更新
		This->MechParam .Torque= (pMsg->Data[4]<<8 | pMsg->Data[5])*1.f;
	}
	if(This->Model == M3508)
	{
		// 3.2 实际电流信息更新
		This->MechParam.Current = (float)(pMsg->Data[4]<<8 | pMsg->Data[5])*1.f;
	}
	// 4. 圈数更新（当电机尚未分配给机构（Model == 0）时，不进行圈数更新！）
	if(This->Model != 0)
	{
		if(This->MechParam.Angle - This->MechParam.LastAngle > 4096)
			This->MechParam.Round --;
		else if (This->MechParam.Angle - This->MechParam.LastAngle < -4096)
			This->MechParam.Round ++;
	}
	// 5. 计算转子端总转过角度（单位/r）
	This->MechParam.TotalAngle = (This->MechParam.Round * 8192 + This->MechParam.Angle - This->MechParam.OffsetAngle)/8192.f;
}
/** @brief 创建一个DJI电机 */
DJIMotor_t* DJIMotorAllocate(DJIMotorInit_t* This)
{
	DJIMotor_t *pSpecific;
	uint8_t ID;
	// 1. 判断总线->拷贝当前总线设备指标->判断指标是否溢出---
	//    ->当前总线设备指标自增供下次使用->获得当前总线当前指标设备指针
	switch(This->bus)
	{
		case CAN1Bus:
			ID = prvDJIMotorCAN1Index;
			if(ID>=DJIMOTOR_SINGLEBUS_NUM)	return NULL;
			pSpecific = &prvCAN1DJIMotor[ID];
			prvDJIMotorCAN1Index++;
			break;
		case CAN2Bus:
			ID = prvDJIMotorCAN2Index;
			if(ID>=DJIMOTOR_SINGLEBUS_NUM)	return NULL;
			pSpecific = &prvCAN2DJIMotor[ID];
			prvDJIMotorCAN2Index++;
			break;
	}
	// 2. 设定设备总线
	pSpecific->Bus = This->bus;
	// 3. 设定电机型号
	pSpecific->Model = This->model;
	// 4. 设定CAN通信ID
	pSpecific->CAN_RxID = DJIMotorRxIDStart + ID + 1;
	if(ID<4)
		pSpecific->CAN_TxID = 0x200;
	else
		pSpecific->CAN_TxID = 0x1FF;
	// 4. 获取初始偏移角度
	pSpecific->MechParam.OffsetAngle = pSpecific->MechParam.Angle;
	/** @TIPS  */
	pSpecific->MechParam.LastAngle = pSpecific->MechParam.Angle;
	// 5. 设定PID参数
	pSpecific->PIDParamVel.Kp = This->kp_vel;
	pSpecific->PIDParamVel.Ki = This->ki_vel;
	pSpecific->PIDParamVel.Kd = This->kd_vel;
	pSpecific->PIDParamPos.Kp = This->kp_pos;
	pSpecific->PIDParamPos.Ki = This->ki_pos;
	pSpecific->PIDParamPos.Kd = This->kd_pos;
	if(This->model == M2006)
		pSpecific->PIDParamPos.MaxOutput = pSpecific->PIDParamVel.MaxOutput = (uint16_t)10000;
	else
		pSpecific->PIDParamPos.MaxOutput = pSpecific->PIDParamVel.MaxOutput = (uint16_t)16384;
	return (pSpecific);
}

/** @brief 设定目标电流 */
void DJIMotor_SetTargetCur(DJIMotor_t* This, int16_t setCur)
{
	This->ControlForm = Cur;
	// 假定setCur是控制电流
	This->MechParam.GivenCurrent = setCur;		/** @Caution 目标电流是给定控制电流还是实际电流 */
}

/** @brief 设定目标速度 */
void DJIMotor_SetTargetVel(DJIMotor_t* This, float setShaftVel)
{
	float setRotorVel = 0;
	// 1. 设定控制形式
	This->ControlForm = Vel;
	// 2. 目标量减速比转换
	setRotorVel = TargetRRtransform(This->Model,setShaftVel);
	if(setRotorVel == 0)
	{
		// RRtransform Error!
	}
	// 3. 设定转子端目标量
	This->PIDParamVel.Target = setRotorVel;
}

/** @brief 设定目标位置，该函数仅接受路径规划后调用，用于转回三环控制 */			  
void DJIMotor_SetTargetPos(DJIMotor_t* This, float setRotorPos)
{									  /** @Caution setRotorPos指代转子端目标圈数 */
	// 清除上一轮控制积分输出的影响
	This->PIDParamVel.Iout = This->PIDParamPos.Iout = 0;
	// 1. 设定控制形式
	This->ControlForm = PosNoTime;
	// 3. 设定转子端目标量
	This->PIDParamPos.Target = setRotorPos;
}

/** @brief 设定目标位置及运行时间 */			/** @Caution setPos应指代输出轴目标圈数 */
void DJIMotor_SetTargetPosWithTime(DJIMotor_t* This, float setShaftPos, uint32_t setTime, float startVel)
{
	float setRotorPos = 0;
	// 清除上一轮控制积分输出的影响
	This->PIDParamVel.Iout = This->PIDParamPos.Iout = 0;
	// 1. 设定控制形式
	This->ControlForm = PosWithTime;
	// 2. 起始时间归0等待新一次赋值，设定控制时间
	This->TimeLimit = setTime * 1000; /** @Tips 转成ms */
	This->TimeStart = 0;		/** @Caution 考虑时间重置情况 */
	// 3. 目标量(圈数)减速比转换
	setRotorPos = TargetRRtransform(This->Model,setShaftPos);
	if(setRotorPos == 0)
	{
		// RRtransform Error!
	}
	// 4. 取得规划路径常系数组结构体（结构体深拷贝）
	This->CoefArray = TrajPlan_QuinticPoly(setRotorPos,setTime,startVel);
	// 5. 存储转子端目标量绝对期望值，用于走完路径规划后转三环控制
	This->PIDParamPos.Target = setRotorPos + This->MechParam.LastExpectTotalAngle;
	// 6. 更新转子端目标量期望值用于下一次转子端目标量绝对期望值迭代
	This->MechParam.LastExpectTotalAngle = This->PIDParamPos.Target;
}

/** @brief 针对不同减速比电机进行输出轴目标值与转子目标值转换 */
float TargetRRtransform(MotorModel_t model,float target)
{
	switch(model)
	{
		case M2006: return (target * M2006RR);
		case M3508: return (target * M3508RR);
		default:	return	0;		/** @Caution 未处理 */
	}
}
/** @brief DJI电机控制 */
void DJIMotorControl(DJIMotor_t* pSpecific)
{
	switch (pSpecific->ControlForm)
	{
		case Cur:	DJIMotorControl_Cur(pSpecific);			break;
		case Vel:	DJIMotorControl_Vel(pSpecific);			break;
		case PosNoTime:		DJIMotorControl_PosNoTime(pSpecific);	break;
		case PosWithTime:	DJIMotorControl_PosWithTime(pSpecific);	break;
		default:	DJIMotor_SetTargetCur(pSpecific,0x0);
	}
}
/** @brief DJI电机电流环控制 */
void DJIMotorControl_Cur(DJIMotor_t* This)
{
	// Nothing needed
}
/** @brief DJI电机速度环控制 */
void DJIMotorControl_Vel(DJIMotor_t* This)
{
	// 1. 速度环输出迭代
	This->PIDParamVel.Output = PID_Culculate(&(This->PIDParamVel),This->MechParam.SpeedRpm);
	// 2. 速度环输出作为控制电流输入
	This->MechParam.GivenCurrent = (int16_t)This->PIDParamVel.Output;
}
/** @brief DJI电机位置环（无时间要求）控制 */
void DJIMotorControl_PosNoTime(DJIMotor_t* This)
{
	// 类似限位开关
	//if(ABS(This->PIDParamPos.Target - This->MechParam.TotalAngle) < PosDeadRev)
	//	This->PIDParamPos.Iout = This->PIDParamVel.Iout = 0;
	// 1. 位置环输出迭代
	This->PIDParamPos.Output = PID_Culculate(&(This->PIDParamPos),This->MechParam.TotalAngle);
	// 2. 位置环输出作为速度环输入
	This->PIDParamVel.Target = This->PIDParamPos.Output;
	// 3. 速度环输出迭代
	This->PIDParamVel.Output = PID_Culculate(&(This->PIDParamVel),This->MechParam.SpeedRpm);
	// 4. 速度环输出作为控制电流输入
	This->MechParam.GivenCurrent = (int16_t)This->PIDParamVel.Output;
}
/** @brief DJI电机位置（有时间要求）控制 */
void DJIMotorControl_PosWithTime(DJIMotor_t* This)
{
	// 第一次进入则赋初值
	if(This->TimeStart == 0)/** @Tips Tick是毫秒 */
		This->TimeStart = HAL_GetTick();
	// 取得当前时间
	uint32_t timeNow = HAL_GetTick();
	// 1. 判断是否达到控制时间，达到控制时间则转为三环控制
	if((timeNow - This->TimeStart) >= This->TimeLimit)
	{
		/** @Tips 将控制形式转为PosNoTime，将控制量设置为先前保持的转子端绝对控制量 */
		DJIMotor_SetTargetPos(This,This->PIDParamPos.Target);
		//DJIMotor_SetTargetCur(This,0x0);
	}
	else
	{
		// 2. 速度环输入跟随多项式速度曲线（时间应传入相对值）
		This->PIDParamVel.Target = VelPolyFunc(Quintic,This->CoefArray,(float)(timeNow - This->TimeStart));
		// 3. 速度环输出迭代
		This->PIDParamVel.Output = PID_Culculate(&(This->PIDParamVel),This->MechParam.SpeedRpm);
		// 4. 速度环输出作为控制电流输入
		This->MechParam.GivenCurrent = (int16_t)This->PIDParamVel.Output;
	}
}
/** @brief 紧急情况全总线电机停止工作 */
void DJIMotor_EmergencyStop(void)
{
	uint8_t cnt;
	for(cnt=0;cnt<prvDJIMotorCAN1Index;cnt++)
	{
		DJIMotor_SetTargetCur(&prvCAN1DJIMotor[cnt],0x0);
	}
	for(cnt=0;cnt<prvDJIMotorCAN2Index;cnt++)
	{		
		DJIMotor_SetTargetCur(&prvCAN2DJIMotor[cnt],0x0);
	}
}
/** @brief 五次多项式路径规划（相对当前位置），得到位置曲线常系数组 */
/** @Caurion 传入参数是否要标准化为转子端总角度？ */
QuintPolyCoef_t TrajPlan_QuinticPoly(float sf_delta, float tf, float startVel)
{
	QuintPolyCoef_t coef;
	float *a = coef.a;
	/*
	a[0] = S0_DELTA*1.f;
	a[1] = VEL0*1.f;
	a[2] = ACC0/2.f;
	a[3] = (20*(sf_delta-S0_DELTA)-( 8*VELF+12*VEL0)*tf-(3*ACC0-  ACCF)*pow(tf,2))*1.f/(2*pow(tf,3));
	a[4] = (30*(S0_DELTA-sf_delta)+(14*VELF+16*VEL0)*tf+(3*ACC0-2*ACCF)*pow(tf,2))*1.f/(2*pow(tf,4));
	a[5] = (12*(sf_delta-S0_DELTA)-( 6*VELF+ 6*VEL0)*tf-(  ACC0-  ACCF)*pow(tf,2))*1.f/(2*pow(tf,5));
	*/
	// 常系数赋值
	a[0] = 0;
	a[1] = startVel/MIN_TO_SEC;
	a[2] = 0;
	a[3] = ( 20*sf_delta-12*a[1]*tf)*1.f/(2*pow(tf,3));
	a[4] = (-30*sf_delta+16*a[1]*tf)*1.f/(2*pow(tf,4));
	a[5] = ( 12*sf_delta- 6*a[1]*tf)*1.f/(2*pow(tf,5));
	return coef;
}
/** @brief 速度曲线多项式计算（时函） */
/** @Tips 速度多项式是位置多项式的导数形式 */
float VelPolyFunc(int power, QuintPolyCoef_t Coef, float t)
{
	uint8_t cnt = 0;
	/** @Tips ms转成s */
	t /= 1000.f;
	float target = 0.f;
	float *a = Coef.a;
	for(cnt=0;cnt<=power;cnt++)
	{
		/** @Tips rpm */
		if(cnt == 0)
			continue;
		target += (60 * cnt * a[cnt] * pow(t,cnt-1));
	}
	return target;
}
/** @brief PID计算 */
float PID_Culculate(PIDParam_t *pid, float measure)
{
	// 1. 误差迭代
	pid->LastErr = pid->Err;
	pid->Err = pid->Target - measure;
	// 2. PID计算
	pid->Pout =	pid->Kp * pid->Err;
	pid->Iout += (float)(pid->Ki * pid->Err);
	pid->Dout =	pid->Kd * (pid->Err - pid->LastErr);
	// 3. 更新输出值
	pid->Output = pid->Pout + pid->Iout + pid->Dout;
	if(pid->Output > pid->MaxOutput)
		pid->Output = pid->MaxOutput;
	if(pid->Output < -(pid->MaxOutput))
		pid->Output = -(pid->MaxOutput);
	return pid->Output;
}
/** @brief 通过CAN将控制电流值发送给DJI电机 */
void CAN_CurrentTransimit(CAN_HandleTypeDef* hcan, int16_t* pCurrentArray)
{
	uint8_t cnt;
	TxMsg_t TxMsg;
	uint32_t TxMailbox;
	// 对于前四个电机（如果有）的报文
	// 报文格式补充
	TxMsg.Header.StdId = 0x200;
	//TxMsg->Header->ExtId = 0x200;
	TxMsg.Header.IDE	= CAN_ID_STD;
	TxMsg.Header.RTR	= CAN_RTR_DATA;
	TxMsg.Header.DLC	= 8;
	TxMsg.Header.TransmitGlobalTime = DISABLE;
	// 报文数据（控制电流）补充
	for(cnt=0;cnt<4;cnt++)
	{
		TxMsg.Data[2*cnt]	= (pCurrentArray[cnt]>>8);
		TxMsg.Data[2*cnt+1] = (pCurrentArray[cnt]);
	}
	// 报文发送：
	// 1.等待邮箱为空
	while(HAL_CAN_GetTxMailboxesFreeLevel(hcan) == 0);
	// 2.发送报文
	HAL_CAN_AddTxMessage(hcan,&(TxMsg.Header),TxMsg.Data,&TxMailbox);
	// 对于后四个电机（如果有）的报文
	// 报文格式补充
	TxMsg.Header.StdId = 0x1FF;
	//TxMsg->Header.ExtId = 0x1FF;
	TxMsg.Header.IDE 	 = CAN_ID_STD;
	TxMsg.Header.RTR	 = CAN_RTR_DATA;
	TxMsg.Header.DLC	 = 8;
	TxMsg.Header.TransmitGlobalTime = DISABLE;
	// 报文数据（控制电流）补充
	for(cnt=0;cnt < 4;cnt++)
	{
		TxMsg.Data[2*cnt]	= (pCurrentArray[4+cnt]>>8);
		TxMsg.Data[2*cnt+1] = (pCurrentArray[4+cnt]);
	}
	// 同样步骤进行报文发送
	while(HAL_CAN_GetTxMailboxesFreeLevel(hcan) == 0);    // Get Number of free Tx Mailboxes.
	HAL_CAN_AddTxMessage(hcan,&(TxMsg.Header),TxMsg.Data,&TxMailbox);
}

/** @brief  虚拟串口DJI电机参数传输 */
void DJIMotorMechParamTransmit(DJIMotor_t* This, ParamEnum_t mechparam)
{	
	float param = 0;
	switch(mechparam)
	{
		case speed:
			param = This->MechParam.SpeedRpm*1.f;
			break;
		case offsetAngle:
			param = This->MechParam.OffsetAngle*1.f;
			break;
		case velTarget:
			param = This->PIDParamVel.Target;
			break;
		case posTarget:
			param = This->PIDParamPos.Target;
			break;
		case cntAngle:
			param = This->MechParam.TotalAngle;
			break;
		case givenCur:
			param = This->MechParam.GivenCurrent;
			break;
		case velOutput:
			param = This->PIDParamVel.Target;
			break;
		case bus:
			param = This->Bus;
			break;
		case data:
			param = debugData;
			break;
		default:
			sprintf(str,"mechparam error!\n");
			CDC_Transmit_FS((uint8_t*)str,sizeof(str));
	}
	sprintf(str,"%.2f ",param);
	CDC_Transmit_FS((uint8_t*)str,sizeof(str));
}
