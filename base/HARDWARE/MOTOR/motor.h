#ifndef  MOTOR_H__
#define  MOTOR_H__

#include  "main.h"



#define  AIN1_SET    HAL_GPIO_WritePin(AIN1_GPIO_Port,AIN1_Pin,GPIO_PIN_SET)
#define  AIN1_RESET  HAL_GPIO_WritePin(AIN1_GPIO_Port,AIN1_Pin,GPIO_PIN_RESET)

#define  BIN1_SET    HAL_GPIO_WritePin(BIN1_GPIO_Port,BIN1_Pin,GPIO_PIN_SET)
#define  BIN1_RESET  HAL_GPIO_WritePin(BIN1_GPIO_Port,BIN1_Pin,GPIO_PIN_RESET)


void Motor_Set(int Motor1,int Motor2);
int Motor_GetLastCmd1(void);
int Motor_GetLastCmd2(void);
void motorPidSetSpeed(float Motor1SetSpeed,float Motor2SetSpeed);
void motorPidReset(void);
float motorPidGetOutputLimit(void);
float motorPidGetErrSumLimit(void);
void motorPidSpeedUp(void);
void motorPidSpeedCut(void);
#endif


