#ifndef __BASE_CONTROL_H__
#define __BASE_CONTROL_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

/* MOVE 指令输入限幅，保持树莓派协议仍使用 -5.0~5.0。 */
#define BASE_CONTROL_MAX_SPEED        5.0f
/* 当前底盘联调阶段使用开环 PWM，取值范围限制为 -99~99。 */
#define BASE_CONTROL_MAX_PWM          99
/* 超声波安全停车距离，单位 cm。 */
#define BASE_CONTROL_SAFE_DISTANCE_CM 20.0f

/* 底盘运行状态，由树莓派协议命令切换。 */
typedef enum
{
	BASE_STATE_IDLE = 0,
	BASE_STATE_MANUAL,
	BASE_STATE_LINE_TRACK,
	BASE_STATE_AVOID,
	BASE_STATE_STOP
} BaseControl_State_t;

/**
 * @brief  立即停止底盘运动，并进入 STOP 状态。
 * @param  None
 * @return None
 */
void BaseControl_Stop(void);

/**
 * @brief  设置底盘运行状态。
 * @param  state: 目标底盘状态
 * @return None
 */
void BaseControl_SetState(BaseControl_State_t state);

/**
 * @brief  获取当前底盘运行状态。
 * @param  None
 * @return 当前底盘状态
 */
BaseControl_State_t BaseControl_GetState(void);

/**
 * @brief  设置超声波安全停车开关。
 * @param  enable: 0 关闭，非 0 开启
 * @return None
 */
void BaseControl_SetSafetyEnable(uint8_t enable);

/**
 * @brief  获取超声波安全停车开关状态。
 * @param  None
 * @return 1 表示已开启，0 表示已关闭
 */
uint8_t BaseControl_GetSafetyEnable(void);

/**
 * @brief  直接使用开环 PWM 控制左右轮电机。
 * @param  rightPwm: 电机1 PWM，电机1为右轮
 * @param  leftPwm: 电机2 PWM，电机2为左轮
 * @return None
 */
void BaseControl_SetOpenLoopPwm(int16_t rightPwm, int16_t leftPwm);

/**
 * @brief  将树莓派 MOVE 指令中的左右轮数值转换为开环 PWM 并执行。
 * @param  leftSpeed: 树莓派协议中的左轮数值
 * @param  rightSpeed: 树莓派协议中的右轮数值
 * @return None
 */
void BaseControl_SetWheelOpenLoop(float leftSpeed, float rightSpeed);

/**
 * @brief  兼容旧速度控制函数名，当前内部改为开环 PWM 控制。
 * @param  leftSpeed: 树莓派协议中的左轮数值
 * @param  rightSpeed: 树莓派协议中的右轮数值
 * @return None
 */
void BaseControl_SetWheelSpeed(float leftSpeed, float rightSpeed);

/**
 * @brief  设置红外循迹开环基础 PWM。
 * @param  pwm: 循迹直行时的基础 PWM
 * @return None
 */
void BaseControl_SetLineBasePwm(int16_t pwm);

/**
 * @brief  获取红外循迹开环基础 PWM。
 * @param  None
 * @return 当前循迹基础 PWM
 */
int16_t BaseControl_GetLineBasePwm(void);

/**
 * @brief  根据当前底盘状态执行周期任务。
 * @param  None
 * @return None
 */
void BaseControl_Task(void);

#ifdef __cplusplus
}
#endif

#endif /* __BASE_CONTROL_H__ */
