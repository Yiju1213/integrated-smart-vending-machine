#ifndef SERVOMOTOR_H
#define SERVOMOTOR_H
#include "stdint.h"
#include "main.h"
#include "cmsis_os.h"
#include "freertos.h"
#include "tim.h"

typedef struct{
	TIM_HandleTypeDef *Htim; //��ʱ��
	uint32_t Channel; //ͨ��
	uint16_t Angle;  //�Ƕ�
}ServoMotor_t;

ServoMotor_t* CreateServoMotor(TIM_HandleTypeDef *htim,uint32_t channel); //�������
void SetServoMotor(ServoMotor_t *SMName,uint16_t angle);  //���ö���Ƕ�

#endif
