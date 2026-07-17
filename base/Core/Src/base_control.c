#include "base_control.h"
#include "motor.h"
#include "HC_SR04.h"
#include "usart.h"
#include <stdio.h>
#include <string.h>

#define BASE_LINE_BLACK_ACTIVE_LEVEL  GPIO_PIN_SET
/* 临时循迹诊断开关：1 时通过 USART1 周期输出传感器、PWM、编码器和速度。 */
#define BASE_LINE_DEBUG_ENABLE        0u
#define BASE_LINE_DEBUG_INTERVAL_MS   200u

static BaseControl_State_t g_eBaseControlState = BASE_STATE_IDLE;
static uint8_t g_ucBaseSafetyEnable = 0u;
static uint32_t g_ulBaseSafetyLastTick = 0u;
static uint32_t g_ulBaseLineTrackLastTick = 0u;
static uint32_t g_ulBaseLineDebugLastTick = 0u;
static int16_t g_iBaseLineBasePwm = 50;

/* 编码器与速度由定时中断更新，仅用于当前循迹诊断输出，不参与控制。 */
extern short Encode1Count;
extern short Encode2Count;
extern float Motor1Speed;
extern float Motor2Speed;

/**
 * @brief  限制开环 PWM 到电机驱动允许范围。
 * @param  pwm: 请求输出的 PWM，正数表示前进，负数表示后退
 * @return 限幅后的 PWM，范围为 -BASE_CONTROL_MAX_PWM~BASE_CONTROL_MAX_PWM
 */
static int16_t BaseControl_LimitPwm(int16_t pwm)
{
	if(pwm > BASE_CONTROL_MAX_PWM)
	{
		return BASE_CONTROL_MAX_PWM;
	}
	if(pwm < -BASE_CONTROL_MAX_PWM)
	{
		return -BASE_CONTROL_MAX_PWM;
	}
	return pwm;
}

/**
 * @brief  限制树莓派 MOVE 指令中的速度数值范围。
 * @param  speed: 树莓派协议中的单轮数值
 * @return 限幅后的数值，范围为 -BASE_CONTROL_MAX_SPEED~BASE_CONTROL_MAX_SPEED
 */
static float BaseControl_LimitSpeed(float speed)
{
	if(speed > BASE_CONTROL_MAX_SPEED)
	{
		return BASE_CONTROL_MAX_SPEED;
	}
	if(speed < -BASE_CONTROL_MAX_SPEED)
	{
		return -BASE_CONTROL_MAX_SPEED;
	}
	return speed;
}

/**
 * @brief  将树莓派 MOVE 指令中的速度数值映射为开环 PWM。
 * @param  speed: 树莓派协议中的单轮数值，范围为 -5.0~5.0
 * @return 映射后的 PWM，范围为 -99~99
 */
static int16_t BaseControl_SpeedToPwm(float speed)
{
	float limitedSpeed = BaseControl_LimitSpeed(speed);
	float pwm = limitedSpeed * (float)BASE_CONTROL_MAX_PWM / BASE_CONTROL_MAX_SPEED;

	if(pwm >= 0.0f)
	{
		return BaseControl_LimitPwm((int16_t)(pwm + 0.5f));
	}
	return BaseControl_LimitPwm((int16_t)(pwm - 0.5f));
}

/**
 * @brief  周期读取超声波距离，并在距离过近时停车。
 * @param  None
 * @return None
 */
static void BaseControl_SafetyTask(void)
{
#if (BASE_CONTROL_ENABLE_ULTRASONIC_SAFETY != 0u)
	float distance;
	uint32_t now = HAL_GetTick();

	if(g_ucBaseSafetyEnable == 0u)
	{
		return;
	}

	if((now - g_ulBaseSafetyLastTick) < 50u)
	{
		return;
	}
	g_ulBaseSafetyLastTick = now;

	distance = HC_SR04_Read();
	if((distance > 0.0f) && (distance <= BASE_CONTROL_SAFE_DISTANCE_CM))
	{
		BaseControl_Stop();
	}
#else
	/* 演示阶段不读取超声波，避免测距异常改变底盘输出。 */
	return;
#endif
}

/**
 * @brief  将红外对管 GPIO 电平转换为黑线检测状态。
 * @param  pinState: 红外对管数字输入电平
 * @return 1 表示检测到黑线，0 表示未检测到黑线
 */
static uint8_t BaseControl_IsLineBlack(GPIO_PinState pinState)
{
	return (pinState == BASE_LINE_BLACK_ACTIVE_LEVEL) ? 1u : 0u;
}

/**
 * @brief  读取四路红外循迹传感器状态。
 * @param  line: 输出数组，line[0]~line[3] 对应 1~4 号红外传感器
 * @return None
 */
static void BaseControl_ReadLineState(uint8_t line[4])
{
	line[0] = BaseControl_IsLineBlack(HAL_GPIO_ReadPin(HW_OUT_1_GPIO_Port, HW_OUT_1_Pin));
	line[1] = BaseControl_IsLineBlack(HAL_GPIO_ReadPin(HW_OUT_2_GPIO_Port, HW_OUT_2_Pin));
	line[2] = BaseControl_IsLineBlack(HAL_GPIO_ReadPin(HW_OUT_3_GPIO_Port, HW_OUT_3_Pin));
	line[3] = BaseControl_IsLineBlack(HAL_GPIO_ReadPin(HW_OUT_4_GPIO_Port, HW_OUT_4_Pin));
}

/**
 * @brief  周期输出当前循迹传感器、开环 PWM 和编码器速度，供串口助手诊断。
 * @param  line: 四路黑线检测结果
 * @param  action: 当前循迹动作文本
 * @param  rightPwm: 右轮 PWM
 * @param  leftPwm: 左轮 PWM
 * @return None
 */
static void BaseControl_PrintLineDebug(const uint8_t line[4], const char *action,
	int16_t rightPwm, int16_t leftPwm)
{
#if (BASE_LINE_DEBUG_ENABLE != 0u)
	char text[180];
	uint32_t now = HAL_GetTick();

	if((now - g_ulBaseLineDebugLastTick) < BASE_LINE_DEBUG_INTERVAL_MS)
	{
		return;
	}
	g_ulBaseLineDebugLastTick = now;

	(void)sprintf(text,
		"DBG LINE1 %s LINE2 %s LINE3 %s LINE4 %s ACT %s PWM_R %d PWM_L %d ENC_R %d ENC_L %d SPD_R %.2f SPD_L %.2f\r\n",
		(line[0] != 0u) ? "BLACK" : "WHITE",
		(line[1] != 0u) ? "BLACK" : "WHITE",
		(line[2] != 0u) ? "BLACK" : "WHITE",
		(line[3] != 0u) ? "BLACK" : "WHITE",
		action,
		(int)rightPwm,
		(int)leftPwm,
		(int)Encode1Count,
		(int)Encode2Count,
		Motor1Speed,
		Motor2Speed);
	(void)HAL_UART_Transmit(&huart1, (uint8_t *)text, (uint16_t)strlen(text), 100u);
#else
	(void)line;
	(void)action;
	(void)rightPwm;
	(void)leftPwm;
#endif
}

/**
 * @brief  执行测试通过的开环红外循迹逻辑。
 * @param  None
 * @return None
 */
static void BaseControl_LineTrackTask(void)
{
	uint8_t line[4];
	int16_t rightPwm;
	int16_t leftPwm;
	const char *action = "STRAIGHT";
	uint32_t now = HAL_GetTick();

	if((now - g_ulBaseLineTrackLastTick) < 20u)
	{
		return;
	}
	g_ulBaseLineTrackLastTick = now;

	BaseControl_ReadLineState(line);
	rightPwm = g_iBaseLineBasePwm;
	leftPwm = g_iBaseLineBasePwm;

	/* 实测规则：0=白底板，1=黑线；1号只读取不参与控制。 */
	/* 1/2号在右侧，2号检测黑线时向右修正；3/4号在左侧，3/4号检测黑线时向左修正。 */
	/* 临时强差速诊断：2 号黑线时右轮停止；3/4 号黑线时左轮停止。 */
	if(line[1] != 0u)
	{
		rightPwm = 0;
		leftPwm = g_iBaseLineBasePwm;
		action = "RIGHT_STOP";
	}
	else if((line[2] != 0u) || (line[3] != 0u))
	{
		rightPwm = g_iBaseLineBasePwm;
		leftPwm = 0;
		action = "LEFT_STOP";
	}

	BaseControl_SetOpenLoopPwm(rightPwm, leftPwm);
	BaseControl_PrintLineDebug(line, action, rightPwm, leftPwm);
}

/**
 * @brief  立即停止底盘运动，并进入 STOP 状态。
 * @param  None
 * @return None
 */
void BaseControl_Stop(void)
{
	g_eBaseControlState = BASE_STATE_STOP;
	Motor_Set(0, 0);
}

/**
 * @brief  设置底盘运行状态。
 * @param  state: 目标底盘状态
 * @return None
 */
void BaseControl_SetState(BaseControl_State_t state)
{
	g_eBaseControlState = state;
	if((state == BASE_STATE_STOP) || (state == BASE_STATE_IDLE))
	{
		Motor_Set(0, 0);
	}
}

/**
 * @brief  获取当前底盘运行状态。
 * @param  None
 * @return 当前底盘状态
 */
BaseControl_State_t BaseControl_GetState(void)
{
	return g_eBaseControlState;
}

/**
 * @brief  设置超声波安全停车开关。
 * @param  enable: 0 关闭，非 0 开启
 * @return None
 */
void BaseControl_SetSafetyEnable(uint8_t enable)
{
#if (BASE_CONTROL_ENABLE_ULTRASONIC_SAFETY != 0u)
	g_ucBaseSafetyEnable = (enable == 0u) ? 0u : 1u;
#else
	/* 即使收到旧版 SAFE 命令，也始终保持超声波功能关闭。 */
	(void)enable;
	g_ucBaseSafetyEnable = 0u;
#endif
	g_ulBaseSafetyLastTick = 0u;
}

/**
 * @brief  获取超声波安全停车开关状态。
 * @param  None
 * @return 1 表示已开启，0 表示已关闭
 */
uint8_t BaseControl_GetSafetyEnable(void)
{
	return g_ucBaseSafetyEnable;
}

/**
 * @brief  直接使用开环 PWM 控制左右轮电机。
 * @param  rightPwm: 电机1 PWM，电机1为右轮
 * @param  leftPwm: 电机2 PWM，电机2为左轮
 * @return None
 */
void BaseControl_SetOpenLoopPwm(int16_t rightPwm, int16_t leftPwm)
{
	Motor_Set((int)BaseControl_LimitPwm(rightPwm), (int)BaseControl_LimitPwm(leftPwm));
}

/**
 * @brief  将树莓派 MOVE 指令中的左右轮数值转换为开环 PWM 并执行。
 * @param  leftSpeed: 树莓派协议中的左轮数值
 * @param  rightSpeed: 树莓派协议中的右轮数值
 * @return None
 */
void BaseControl_SetWheelOpenLoop(float leftSpeed, float rightSpeed)
{
	int16_t rightPwm = BaseControl_SpeedToPwm(rightSpeed);
	int16_t leftPwm = BaseControl_SpeedToPwm(leftSpeed);

	g_eBaseControlState = BASE_STATE_MANUAL;
	BaseControl_SetOpenLoopPwm(rightPwm, leftPwm);
}

/**
 * @brief  兼容旧速度控制函数名，当前内部改为开环 PWM 控制。
 * @param  leftSpeed: 树莓派协议中的左轮数值
 * @param  rightSpeed: 树莓派协议中的右轮数值
 * @return None
 */
void BaseControl_SetWheelSpeed(float leftSpeed, float rightSpeed)
{
	BaseControl_SetWheelOpenLoop(leftSpeed, rightSpeed);
}

/**
 * @brief  设置红外循迹开环基础 PWM。
 * @param  pwm: 循迹直行时的基础 PWM
 * @return None
 */
void BaseControl_SetLineBasePwm(int16_t pwm)
{
	g_iBaseLineBasePwm = BaseControl_LimitPwm(pwm);
}

/**
 * @brief  获取红外循迹开环基础 PWM。
 * @param  None
 * @return 当前循迹基础 PWM
 */
int16_t BaseControl_GetLineBasePwm(void)
{
	return g_iBaseLineBasePwm;
}

/**
 * @brief  根据当前底盘状态执行周期任务。
 * @param  None
 * @return None
 */
void BaseControl_Task(void)
{
	BaseControl_SafetyTask();

	switch(g_eBaseControlState)
	{
		case BASE_STATE_IDLE:
		case BASE_STATE_MANUAL:
			break;

		case BASE_STATE_LINE_TRACK:
			BaseControl_LineTrackTask();
			break;

		case BASE_STATE_AVOID:
			break;

		case BASE_STATE_STOP:
			Motor_Set(0, 0);
			break;

		default:
			BaseControl_Stop();
			break;
	}
}
