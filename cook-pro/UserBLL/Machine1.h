#ifndef MACHINE_H_
#define MACHINE_H_
/* Includes -------------------------------------------- */
#include "DJIMotor.h"
#include "ServoMotor.h"
/* Additional Configuration ---------------------------- */
#define _DEBUG_MODE 1
#define _VEL_TEST	0
#define _POS_TEST	0
#define _SEQUENCE_TEST	1
/* Mechanism Name Enum -------------------------------------- */
typedef enum
{
	SpinLiftSys_Screw = 1,
	PickUpSys_RackPinion,
	PickUpSys_ZaxisScrew,
	PickUpSys_XaxisScrew
}MechName_t;
/* Mechanism & Motor PosTarget Transform  Function  ---- */
float PosTarget_MMtransform(MechName_t name, float mechTarget);
/* Mechanism Motor Functions --------------------------- */
void MechMotorRegister(void);
void SetXaxisMove(float pTar, uint32_t limTime, float vStart);
void SetZaxisMove(float pTar, uint32_t limTime, float vStart);
void SetRackPinionMove(float pTar, uint32_t limTime, float vStart);
void SetBasketRotate(float pTar, uint32_t limTime, float vStart);
void SetBasketShake(void);
void SetCoverRotate(float pTar, uint32_t limTime, float vStart);
void SetCoverRest(void);
void SetShovelOn(float vTar);
void SetShovelOff(void);
void SetLiftSysRotate(float pTar, uint32_t limTime, float vStart);
void SetLiftScrewMove(float pTar, uint32_t limTime, float vStart);
void Mag_ON(void);
void Mag_OFF(void);
void Grasper_Upload(void);
void Grasper_Unload(void);
#endif
