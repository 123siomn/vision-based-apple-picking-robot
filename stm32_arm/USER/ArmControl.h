#ifndef _ARM_CONTROL_H_
#define _ARM_CONTROL_H_

#include "include.h"

#define ARM_SERVO_MIN_ID       1
#define ARM_SERVO_MAX_ID       6
#define ARM_SERVO_MIN_PULSE    500
#define ARM_SERVO_MAX_PULSE    2500
#define ARM_SERVO_HOME_PULSE   1500
#define ARM_SERVO_MIN_TIME     20
#define ARM_SERVO_MAX_TIME     30000

typedef enum
{
	ARM_STATE_IDLE = 0,
	ARM_STATE_SERVO_MOVE,
	ARM_STATE_STOP
}ArmControlState_t;

void ArmControl_Stop(void);
uint8 ArmControl_SetServo(uint8 id, uint16 pulse, uint16 time);
void ArmControl_SetHome(uint16 time);
void ArmControl_Task(void);
ArmControlState_t ArmControl_GetState(void);

#endif
