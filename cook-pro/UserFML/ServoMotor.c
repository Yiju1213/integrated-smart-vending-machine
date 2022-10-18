/**
  ******************************************************************************
  * @file           : ServoMotor.c
  * @author         : TWW
  * @brief          : 舵机控制模块
  ******************************************************************************
  * @attention
  ******************************************************************************
*/
#include "ServoMotor.h"
#define OPERATE_FREQUENCY 50	/** @Tips Make sure the PWM frequency is 50Hz？ */
#define ANGLE_RANGE	180
#define ARR_VALUE	20000
/** @Tips 脉冲宽度，单位毫秒ms */
#define PULSE_WIDTH_ZERO	0.5		
#define PULSE_WIDTH_FULL	2.5
/** @Tips 脉冲比较值 */
#define PULSE_ZERO	(PULSE_WIDTH_ZERO/(1000/OPERATE_FREQUENCY)*ARR_VALUE)
#define PULSE_FULL	(PULSE_WIDTH_FULL/(1000/OPERATE_FREQUENCY)*ARR_VALUE)
#define MAX_SERVO_MOTOR_NUMBER 15
static uint8_t prvServoMotorIndex;
static ServoMotor_t ServoMotor[MAX_SERVO_MOTOR_NUMBER];
/** @brief 申请舵机 */
ServoMotor_t* CreateServoMotor(TIM_HandleTypeDef *htim, uint32_t channel) 
{
	uint8_t i = prvServoMotorIndex;
	if(i >= MAX_SERVO_MOTOR_NUMBER)
	{
		// Error
		return NULL;
	}
	HAL_TIM_PWM_Start(htim,channel);
	// 设置舵机角度初始值90
    __HAL_TIM_SetCompare(htim,channel,1500);
	
	ServoMotor[i].Htim=htim;
	ServoMotor[i].Channel= channel;
	ServoMotor[i].Angle=90;
	
	prvServoMotorIndex++;
	return (&ServoMotor[i]);
}


void SetServoMotor(ServoMotor_t *SMName,uint16_t angle)  //设置舵机角度
{
	if(angle > 180) return;
	if(angle < 75)	return;
	else
	{
		//uint16_t pulse=angle * (PULSE_FULL - PULSE_ZERO)/ANGLE_RANGE + PULSE_ZERO;
		uint16_t pulse=angle * 2000/180 + 500;
		__HAL_TIM_SetCompare(SMName->Htim,SMName->Channel,pulse);
		SMName->Angle=angle;
	}
}
