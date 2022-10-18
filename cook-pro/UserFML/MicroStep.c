/**
  ******************************************************************************
  * @file           : MicroStep.c
  * @author         : WYR
  * @brief          : 42��������ģ�飨PA1-T-PUL PA2-U-DIR PA3-V-ENA��
  ******************************************************************************
  * @attention
  ******************************************************************************
*/
/* Includes -------------------------------------------- */
#include "MicroStep.h"
/* Defines --------------------------------------------- */
#define MicroStepPPR 200
/* Extern Members -------------------------------------- */

/* Portable Members ------------------------------------ */

/* Private Members ------------------------------------- */
	uint16_t MicroStepPul;
	uint16_t MicroStepRev;
	
/* Functions ------------------------------------------- */
void MicroStep_PulseNotification(void)
{
	MicroStepPul++;          		  // ÿ1ms����1��
    if(MicroStepPul == MicroStepPPR)   // ÿתһȦLED�Ʒ�תһ��
    {
        HAL_GPIO_TogglePin(LED_G_GPIO_Port,LED_G_Pin);
        MicroStepPul = 0;
		MicroStepRev ++;
    }
}
