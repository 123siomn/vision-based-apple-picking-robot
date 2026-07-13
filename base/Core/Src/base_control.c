#include "base_control.h"
#include "motor.h"
#include "HC_SR04.h"

#define BASE_LINE_TURN_PWM            15
#define BASE_LINE_BLACK_ACTIVE_LEVEL  GPIO_PIN_SET

static BaseControl_State_t g_eBaseControlState = BASE_STATE_IDLE;
static uint8_t g_ucBaseSafetyEnable = 0u;
static uint32_t g_ulBaseSafetyLastTick = 0u;
static uint32_t g_ulBaseLineTrackLastTick = 0u;
static int16_t g_iBaseLineBasePwm = 30;

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
 * @brief  执行测试通过的开环红外循迹逻辑。
 * @param  None
 * @return None
 */
static void BaseControl_LineTrackTask(void)
{
	uint8_t line[4];
	int16_t rightPwm;
	int16_t leftPwm;
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
	if(line[1] != 0u)
	{
		rightPwm = BaseControl_LimitPwm(g_iBaseLineBasePwm - BASE_LINE_TURN_PWM);
		leftPwm = BaseControl_LimitPwm(g_iBaseLineBasePwm + BASE_LINE_TURN_PWM);
	}
	else if((line[2] != 0u) || (line[3] != 0u))
	{
		rightPwm = BaseControl_LimitPwm(g_iBaseLineBasePwm + BASE_LINE_TURN_PWM);
		leftPwm = BaseControl_LimitPwm(g_iBaseLineBasePwm - BASE_LINE_TURN_PWM);
	}

	BaseControl_SetOpenLoopPwm(rightPwm, leftPwm);
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
	g_ucBaseSafetyEnable = (enable == 0u) ? 0u : 1u;
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
