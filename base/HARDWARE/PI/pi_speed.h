#ifndef __PI_SPEED_H__
#define __PI_SPEED_H__

#include "main.h"

/**
 * @brief  单轮 PI 速度控制器运行状态。
 * @note   本模块不依赖电机引脚、编码器定时器或串口，仅负责速度误差到 PWM 的计算。
 */
typedef struct
{
	float targetSpeed;
	float kp;
	float ki;
	float integral;
	float integralLimit;
	int16_t outputLimit;
	int16_t minActivePwm;
	int16_t outputPwm;
} PiSpeedController_t;

/**
 * @brief  初始化一个单轮 PI 速度控制器。
 * @param  controller: PI 控制器对象
 * @param  kp: 比例系数
 * @param  ki: 积分系数
 * @param  integralLimit: 积分项绝对值上限
 * @param  outputLimit: PWM 输出绝对值上限
 * @param  minActivePwm: 非零输出的最小有效 PWM，用于跨过电机死区
 * @return None
 */
void PiSpeed_Init(PiSpeedController_t *controller, float kp, float ki,
	float integralLimit, int16_t outputLimit, int16_t minActivePwm);

/**
 * @brief  清空目标速度、积分项和输出值。
 * @param  controller: PI 控制器对象
 * @return None
 */
void PiSpeed_Reset(PiSpeedController_t *controller);

/**
 * @brief  设置单轮目标速度。
 * @param  controller: PI 控制器对象
 * @param  targetSpeed: 目标速度，单位由上层编码器换算定义
 * @return None
 */
void PiSpeed_SetTarget(PiSpeedController_t *controller, float targetSpeed);

/**
 * @brief  使用当前实测速度执行一次 PI 运算。
 * @param  controller: PI 控制器对象
 * @param  measuredSpeed: 当前实测速度
 * @param  periodS: 本次控制周期，单位秒
 * @return 计算得到的 PWM 输出
 */
int16_t PiSpeed_Update(PiSpeedController_t *controller, float measuredSpeed, float periodS);

#endif /* __PI_SPEED_H__ */

