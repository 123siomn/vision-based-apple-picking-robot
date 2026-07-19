#ifndef  MOTOR_H__
#define  MOTOR_H__

#include  "main.h"



#define  AIN1_SET    HAL_GPIO_WritePin(AIN1_GPIO_Port,AIN1_Pin,GPIO_PIN_SET)
#define  AIN1_RESET  HAL_GPIO_WritePin(AIN1_GPIO_Port,AIN1_Pin,GPIO_PIN_RESET)

#define  BIN1_SET    HAL_GPIO_WritePin(BIN1_GPIO_Port,BIN1_Pin,GPIO_PIN_SET)
#define  BIN1_RESET  HAL_GPIO_WritePin(BIN1_GPIO_Port,BIN1_Pin,GPIO_PIN_RESET)


/**
 * @brief  按给定 PWM 写入右轮电机1和左轮电机2。
 * @param  Motor1: 右轮 PWM，正数为前进
 * @param  Motor2: 左轮 PWM，正数为前进
 * @return None
 */
void Motor_Set(int Motor1, int Motor2);
int Motor_GetLastCmd1(void);
int Motor_GetLastCmd2(void);
#endif
