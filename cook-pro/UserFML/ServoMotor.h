#ifndef SERVOMOTOR_H
#define SERVOMOTOR_H
#include "stdint.h"
#include "main.h"
#include "cmsis_os.h"
#include "freertos.h"
#include "tim.h"

typedef struct{
	TIM_HandleTypeDef *Htim; //定时器
	uint32_t Channel; //通道
	uint16_t Angle;  //角度
}ServoMotor_t;

ServoMotor_t* CreateServoMotor(TIM_HandleTypeDef *htim,uint32_t channel); //创建舵机
void SetServoMotor(ServoMotor_t *SMName,uint16_t angle);  //设置舵机角度

#endif
