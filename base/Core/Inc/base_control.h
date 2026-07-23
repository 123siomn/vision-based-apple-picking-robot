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
/* 空载标定最大轮速约为 10 SPD，限制目标值可避免异常指令导致突加速。 */
#define BASE_CONTROL_PI_MAX_TARGET_SPEED 5.0f
/* 黑线摩擦较大时实测持续运行 PWM 约为 38，PI 输出低于此值时抬升到安全起转值。 */
#define BASE_CONTROL_PI_MIN_ACTIVE_PWM 40
/* 超声波安全停车距离，单位 cm。 */
#define BASE_CONTROL_SAFE_DISTANCE_CM 20.0f
/* 当前演示阶段关闭超声波安全停车，避免无关测距读数干扰循迹和视觉微调。 */
#define BASE_CONTROL_ENABLE_ULTRASONIC_SAFETY 0u

/* 底盘运行状态，由树莓派协议命令切换。 */
typedef enum
{
	BASE_STATE_IDLE = 0,
	BASE_STATE_MANUAL,
	BASE_STATE_LINE_TRACK,
	BASE_STATE_AVOID,
	BASE_STATE_PI_SPEED,
	BASE_STATE_RETURN_RIGHT,
	BASE_STATE_STOP
} BaseControl_State_t;

/* PI 正式运行时可执行的固定底盘动作，目标速度仅在底盘控制层维护。 */
typedef enum
{
	BASE_ACTION_STOP = 0,
	BASE_ACTION_TRACK,
	BASE_ACTION_FORWARD,
	BASE_ACTION_BACKWARD,
	BASE_ACTION_ROTATE_LEFT,
	BASE_ACTION_ROTATE_RIGHT,
	BASE_ACTION_RETURN_RIGHT,
	BASE_ACTION_PI_TEST
} BaseControl_Action_t;

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
 * @brief  开启或关闭独立 PI 轮速测试模式。
 * @param  enable: 非 0 开启，0 关闭；开启后保持停车，等待 PITARGET 设置目标速度。
 * @return None
 */
void BaseControl_SetPiEnable(uint8_t enable);

/**
 * @brief  获取独立 PI 轮速测试模式开关状态。
 * @param  None
 * @return 1 表示已开启，0 表示已关闭
 */
uint8_t BaseControl_GetPiEnable(void);

/**
 * @brief  设置左右轮 PI 目标速度。
 * @param  leftTargetSpeed: 左轮目标速度，单位与 STATUS 中 SPD_L 相同
 * @param  rightTargetSpeed: 右轮目标速度，单位与 STATUS 中 SPD_R 相同
 * @return 1 表示设置成功，0 表示 PI 模式未开启
 */
uint8_t BaseControl_SetWheelPiTarget(float leftTargetSpeed, float rightTargetSpeed);

/** @brief 启动 PI 红外循迹，左右轮目标速度由底盘内部决定。 */
void BaseControl_StartPiLineTrack(void);

/** @brief 执行一个固定速度的 PI 底盘动作。 */
uint8_t BaseControl_StartPiAction(BaseControl_Action_t action);

/** @brief 获取当前固定底盘动作，供 STATUS 回传。 */
BaseControl_Action_t BaseControl_GetAction(void);

/** @brief 在底盘主任务中按实际周期执行一次 PI 运算。 */
void BaseControl_UpdateWheelPi(float rightMeasuredSpeed, float leftMeasuredSpeed, float periodS);

/**
 * @brief  获取 PI 目标速度和实时输出，供 STATUS 回传与网页观察。
 * @param  leftTargetSpeed: 左轮目标速度输出地址，可为 NULL
 * @param  rightTargetSpeed: 右轮目标速度输出地址，可为 NULL
 * @param  leftPwm: 左轮当前 PI 输出 PWM 地址，可为 NULL
 * @param  rightPwm: 右轮当前 PI 输出 PWM 地址，可为 NULL
 * @return None
 */
void BaseControl_GetPiStatus(float *leftTargetSpeed, float *rightTargetSpeed,
	int16_t *leftPwm, int16_t *rightPwm);

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
