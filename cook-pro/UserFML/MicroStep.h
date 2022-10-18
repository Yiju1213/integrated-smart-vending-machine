#ifndef _MICROSTEP_H_
#define _MICROSTEP_H_
/* Includes ---------------------------------------------- */
#include "main.h"
#include "cmsis_os.h"
#include "freertos.h"
#include "usb_device.h"
#include "usbd_cdc_if.h"
#include "math.h"
#include "stdint.h"
#include "tim.h"
/* MicroStep Struct -------------------------------------- */
typedef struct 
{
	uint16_t Pul;
	uint16_t Rev;
	uint16_t PPR;
}MicroStep_t;
/* Functions --------------------------------------------- */
void MicroStep_Allocate(TIM_HandleTypeDef *htim, uint32_t Channel);
void MicroStep_PulseNotification(void);

#endif