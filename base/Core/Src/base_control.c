#include "base_control.h"
#include "motor.h"
#include "tim.h"
#include "pi_speed.h"
#include "HC_SR04.h"
#include "usart.h"
#include <stdio.h>
#include <string.h>

#define BASE_LINE_BLACK_ACTIVE_LEVEL  GPIO_PIN_SET
/* 临时循迹诊断开关：1 时通过 USART1 周期输出传感器、PWM、编码器和速度。 */
#define BASE_LINE_DEBUG_ENABLE        0u
#define BASE_LINE_DEBUG_INTERVAL_MS   200u
/* 独立 PI 初始参数，仅用于轮速测试模式，后续通过网页实测后再调大或调小。 */
#define BASE_CONTROL_PI_KP                 5.0f
#define BASE_CONTROL_PI_KI                 0.5f
#define BASE_CONTROL_PI_INTEGRAL_MAX       60.0f
#define BASE_CONTROL_PI_CORRECTION_LIMIT   30
#define BASE_CONTROL_SPEED_FILTER_ALPHA    0.35f
#define BASE_CONTROL_FF_MIN_SPEED           1.90f
#define BASE_CONTROL_FF_MID_SPEED           3.00f
#define BASE_CONTROL_FF_MAX_SPEED           5.00f
#define BASE_CONTROL_FF_MID_PWM            60
#define BASE_CONTROL_TRACK_SPEED            3.10f
#define BASE_CONTROL_TRACK_TURN_SPEED       3.10f
#define BASE_CONTROL_FORWARD_SPEED          2.73f
#define BASE_CONTROL_BACKWARD_SPEED         3.51f
#define BASE_CONTROL_BACKWARD_START_SPEED   2.00f
#define BASE_CONTROL_BACKWARD_RAMP_STEP     0.25f
#define BASE_CONTROL_ROTATE_SPEED           2.73f
/* 抓取完成后回到黑线时使用前进右转，左轮较快、右轮较慢。 */
#define BASE_CONTROL_RETURN_RIGHT_SPEED_R    2.30f
#define BASE_CONTROL_RETURN_RIGHT_SPEED_L    3.10f

static BaseControl_State_t g_eBaseControlState = BASE_STATE_IDLE;
static BaseControl_Action_t g_eBaseControlAction = BASE_ACTION_STOP;
static uint8_t g_ucBaseBackwardRampActive = 0u;
static uint8_t g_ucBaseSafetyEnable = 0u;
#if (BASE_CONTROL_ENABLE_ULTRASONIC_SAFETY != 0u)
static uint32_t g_ulBaseSafetyLastTick = 0u;
#endif
static uint32_t g_ulBaseLineTrackLastTick = 0u;
#if (BASE_LINE_DEBUG_ENABLE != 0u)
static uint32_t g_ulBaseLineDebugLastTick = 0u;
#endif
static int16_t g_iBaseLineBasePwm = 50;
static uint8_t g_ucBasePiEnable = 0u;
static uint8_t g_ucBasePiInitialized = 0u;
static PiSpeedController_t g_tBasePiRight;
static PiSpeedController_t g_tBasePiLeft;
static uint32_t g_ulBaseSpeedLastTick = 0u;
static uint32_t g_ulBasePiLastTick = 0u;
static uint8_t g_ucBaseSpeedFilterReady = 0u;

/* 编码器与速度由 BaseControl_Task 周期更新，STATUS 只读取并回传。 */
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
 * @brief  限制 PI 目标速度到允许范围。
 * @param  speed: 待限制的目标速度
 * @return 限幅后的目标速度
 */
static float BaseControl_LimitPiTargetSpeed(float speed)
{
	if(speed > BASE_CONTROL_PI_MAX_TARGET_SPEED)
	{
		return BASE_CONTROL_PI_MAX_TARGET_SPEED;
	}
	if(speed < -BASE_CONTROL_PI_MAX_TARGET_SPEED)
	{
		return -BASE_CONTROL_PI_MAX_TARGET_SPEED;
	}
	return speed;
}

/**
 * @brief  根据空载实测曲线，将目标轮速换算为基础 PWM。
 * @param  targetSpeed: 单轮目标速度，单位与 STATUS 中 SPD 相同
 * @return 基础 PWM；PI 输出将在此基础上做正负小幅修正
 */
static int16_t BaseControl_SpeedToFeedforwardPwm(float targetSpeed)
{
	float magnitude;
	float pwm;
	int16_t signedPwm;

	if(targetSpeed == 0.0f)
	{
		return 0;
	}

	magnitude = (targetSpeed >= 0.0f) ? targetSpeed : -targetSpeed;
	if(magnitude <= BASE_CONTROL_FF_MIN_SPEED)
	{
		pwm = (float)BASE_CONTROL_PI_MIN_ACTIVE_PWM;
	}
	else if(magnitude <= BASE_CONTROL_FF_MID_SPEED)
	{
		pwm = (float)BASE_CONTROL_PI_MIN_ACTIVE_PWM +
			(magnitude - BASE_CONTROL_FF_MIN_SPEED) *
			((float)BASE_CONTROL_FF_MID_PWM - (float)BASE_CONTROL_PI_MIN_ACTIVE_PWM) /
			(BASE_CONTROL_FF_MID_SPEED - BASE_CONTROL_FF_MIN_SPEED);
	}
	else
	{
		pwm = (float)BASE_CONTROL_FF_MID_PWM +
			(magnitude - BASE_CONTROL_FF_MID_SPEED) *
			((float)BASE_CONTROL_MAX_PWM - (float)BASE_CONTROL_FF_MID_PWM) /
			(BASE_CONTROL_FF_MAX_SPEED - BASE_CONTROL_FF_MID_SPEED);
	}

	signedPwm = BaseControl_LimitPwm((int16_t)(pwm + 0.5f));
	return (targetSpeed >= 0.0f) ? signedPwm : (int16_t)(-signedPwm);
}
/**
 * @brief  首次使用时初始化左右轮独立 PI 控制器。
 * @param  None
 * @return None
 */
static void BaseControl_InitPiIfNeeded(void)
{
	if(g_ucBasePiInitialized != 0u)
	{
		return;
	}

	PiSpeed_Init(&g_tBasePiRight, BASE_CONTROL_PI_KP, BASE_CONTROL_PI_KI,
		BASE_CONTROL_PI_INTEGRAL_MAX, BASE_CONTROL_PI_CORRECTION_LIMIT, 0);
	PiSpeed_Init(&g_tBasePiLeft, BASE_CONTROL_PI_KP, BASE_CONTROL_PI_KI,
		BASE_CONTROL_PI_INTEGRAL_MAX, BASE_CONTROL_PI_CORRECTION_LIMIT, 0);
	g_ucBasePiInitialized = 1u;
}

/**
 * @brief  清空独立 PI 模式的目标、积分和输出记录。
 * @param  None
 * @return None
 */
static void BaseControl_ResetPiState(void)
{
	BaseControl_InitPiIfNeeded();
	PiSpeed_Reset(&g_tBasePiRight);
	PiSpeed_Reset(&g_tBasePiLeft);
	/* 下次采样直接使用最新编码器速度，避免模式切换沿用旧滤波值。 */
	g_ucBaseSpeedFilterReady = 0u;
	g_ucBaseBackwardRampActive = 0u;
}

/**
 * @brief  退出独立 PI 模式，让开环、循迹或停车命令独占电机 PWM。
 * @param  None
 * @return None
 */
static void BaseControl_ExitPiMode(void)
{
	g_ucBasePiEnable = 0u;
	BaseControl_ResetPiState();
}

/**
 * @brief  在不改变控制模式的情况下直接写入电机 PWM。
 * @param  rightPwm: 右轮 PWM
 * @param  leftPwm: 左轮 PWM
 * @return None
 */
/**
 * @brief  在保持当前 PI 模式的情况下更新左右轮目标速度。
 * @param  rightTargetSpeed: 右轮目标速度
 * @param  leftTargetSpeed: 左轮目标速度
 * @return None
 */
static void BaseControl_SetPiTargetsInternal(float rightTargetSpeed, float leftTargetSpeed)
{
	BaseControl_InitPiIfNeeded();
	PiSpeed_SetTarget(&g_tBasePiRight, BaseControl_LimitPiTargetSpeed(rightTargetSpeed));
	PiSpeed_SetTarget(&g_tBasePiLeft, BaseControl_LimitPiTargetSpeed(leftTargetSpeed));
}
/**
 * @brief  以固定步长将当前 PI 目标平滑靠近最终目标。
 * @param  current: 当前生效目标速度
 * @param  target: 最终目标速度
 * @param  step: 每次 PI 周期允许的速度变化量
 * @return 下一次生效目标速度
 */
static float BaseControl_RampTarget(float current, float target, float step)
{
	if(current < target)
	{
		current += step;
		return (current > target) ? target : current;
	}
	if(current > target)
	{
		current -= step;
		return (current < target) ? target : current;
	}
	return target;
}
static void BaseControl_ApplyPwm(int16_t rightPwm, int16_t leftPwm)
{
	Motor_Set((int)BaseControl_LimitPwm(rightPwm), (int)BaseControl_LimitPwm(leftPwm));
}

/**
 * @brief  在主循环中周期读取编码器并更新 PI，避免高速中断影响 USART1 通信。
 * @param  None
 * @return None
 */
static void BaseControl_UpdateEncoderFeedback(void)
{
	uint32_t now = HAL_GetTick();
	uint32_t speedPeriodMs;
	uint32_t piPeriodMs;
	float rawRightSpeed;
	float rawLeftSpeed;

	if((now - g_ulBaseSpeedLastTick) < 10u)
	{
		return;
	}
	speedPeriodMs = now - g_ulBaseSpeedLastTick;
	g_ulBaseSpeedLastTick = now;

	Encode1Count = -(short)__HAL_TIM_GET_COUNTER(&htim4);
	Encode2Count = (short)__HAL_TIM_GET_COUNTER(&htim2);
	__HAL_TIM_SET_COUNTER(&htim4, 0);
	__HAL_TIM_SET_COUNTER(&htim2, 0);

	/* 按实际采样时间换算轮速，避免串口回复占用主循环时产生速度误差。 */
	/* 10 ms 窗口脉冲数较少，采用轻量一阶滤波减少编码器量化抖动。 */
	rawRightSpeed = (float)Encode1Count * 1000.0f / (float)speedPeriodMs / 9.6f / 11.0f / 4.0f;
	rawLeftSpeed = (float)Encode2Count * 1000.0f / (float)speedPeriodMs / 9.6f / 11.0f / 4.0f;
	if(g_ucBaseSpeedFilterReady == 0u)
	{
		Motor1Speed = rawRightSpeed;
		Motor2Speed = rawLeftSpeed;
		g_ucBaseSpeedFilterReady = 1u;
	}
	else
	{
		Motor1Speed += BASE_CONTROL_SPEED_FILTER_ALPHA * (rawRightSpeed - Motor1Speed);
		Motor2Speed += BASE_CONTROL_SPEED_FILTER_ALPHA * (rawLeftSpeed - Motor2Speed);
	}

	if((now - g_ulBasePiLastTick) >= 20u)
	{
		piPeriodMs = now - g_ulBasePiLastTick;
		g_ulBasePiLastTick = now;
		BaseControl_UpdateWheelPi(Motor1Speed, Motor2Speed, (float)piPeriodMs / 1000.0f);
	}
}
/**
 * @brief  限制树莓派 MOVE 指令中的速度数值范围。
 * @param  speed: 树莓派协议中的单轮数值
 * @return 限幅后的数值，范围为 -BASE_CONTROL_MAX_SPEED~BASE_CONTROL_MAX_SPEED
 */static float BaseControl_LimitSpeed(float speed)
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
	float rightTargetSpeed;
	float leftTargetSpeed;
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
	rightTargetSpeed = BASE_CONTROL_TRACK_SPEED;
	leftTargetSpeed = BASE_CONTROL_TRACK_SPEED;

	/* 0=白底板，1=黑线；1、2号位于右侧，检测黑线时执行右侧修正。 */
	if((line[0] != 0u) || (line[1] != 0u))
	{
		rightTargetSpeed = 0.0f;
		leftTargetSpeed = BASE_CONTROL_TRACK_TURN_SPEED;
		action = "RIGHT_STOP";
	}
	else if((line[2] != 0u) || (line[3] != 0u))
	{
		rightTargetSpeed = BASE_CONTROL_TRACK_TURN_SPEED;
		leftTargetSpeed = 0.0f;
		action = "LEFT_STOP";
	}

	BaseControl_SetPiTargetsInternal(rightTargetSpeed, leftTargetSpeed);
	rightPwm = BaseControl_SpeedToFeedforwardPwm(rightTargetSpeed);
	leftPwm = BaseControl_SpeedToFeedforwardPwm(leftTargetSpeed);
	BaseControl_PrintLineDebug(line, action, rightPwm, leftPwm);
}

/**
 * @brief  抓取完成后以右侧圆弧寻找黑线，检测到任一黑线后自动恢复 PI 循迹。
 * @param  None
 * @return None
 */
static void BaseControl_ReturnRightTask(void)
{
	uint8_t line[4];
	uint32_t now = HAL_GetTick();

	if((now - g_ulBaseLineTrackLastTick) < 20u)
	{
		return;
	}
	g_ulBaseLineTrackLastTick = now;
	BaseControl_ReadLineState(line);
	if((line[0] != 0u) || (line[1] != 0u) || (line[2] != 0u) || (line[3] != 0u))
	{
		BaseControl_StartPiLineTrack();
		return;
	}

	BaseControl_SetPiTargetsInternal(BASE_CONTROL_RETURN_RIGHT_SPEED_R,
		BASE_CONTROL_RETURN_RIGHT_SPEED_L);
}
void BaseControl_Stop(void)
{
	BaseControl_ExitPiMode();
	g_eBaseControlAction = BASE_ACTION_STOP;
	g_eBaseControlState = BASE_STATE_STOP;
	BaseControl_ApplyPwm(0, 0);
}

/**
 * @brief  设置底盘运行状态。
 * @param  state: 目标底盘状态
 * @return None
 */
void BaseControl_SetState(BaseControl_State_t state)
{
	if((state != BASE_STATE_PI_SPEED) && (state != BASE_STATE_LINE_TRACK) &&
		(state != BASE_STATE_RETURN_RIGHT))
	{
		BaseControl_ExitPiMode();
		g_eBaseControlAction = BASE_ACTION_STOP;
	}
	g_eBaseControlState = state;
	if((state == BASE_STATE_STOP) || (state == BASE_STATE_IDLE))
	{
		BaseControl_ApplyPwm(0, 0);
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
#if (BASE_CONTROL_ENABLE_ULTRASONIC_SAFETY != 0u)
	g_ulBaseSafetyLastTick = 0u;
#endif
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
	BaseControl_ExitPiMode();
	BaseControl_ApplyPwm(rightPwm, leftPwm);
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

	BaseControl_ExitPiMode();
	g_eBaseControlState = BASE_STATE_MANUAL;
	BaseControl_ApplyPwm(rightPwm, leftPwm);
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
 * @brief  开启或关闭独立 PI 轮速测试模式。
 * @param  enable: 非 0 开启，0 关闭；开启后保持停车，等待 PITARGET 设置目标速度。
 * @return None
 */
void BaseControl_SetPiEnable(uint8_t enable)
{
	BaseControl_ResetPiState();
	if(enable == 0u)
	{
		g_ucBasePiEnable = 0u;
		g_eBaseControlAction = BASE_ACTION_STOP;
		g_eBaseControlState = BASE_STATE_STOP;
		BaseControl_ApplyPwm(0, 0);
		return;
	}

	g_ucBasePiEnable = 1u;
	g_eBaseControlAction = BASE_ACTION_PI_TEST;
	g_eBaseControlState = BASE_STATE_STOP;
	BaseControl_ApplyPwm(0, 0);
}

/**
 * @brief  获取独立 PI 轮速测试模式开关状态。
 * @param  None
 * @return 1 表示已开启，0 表示已关闭
 */
uint8_t BaseControl_GetPiEnable(void)
{
	return g_ucBasePiEnable;
}

/**
 * @brief  设置左右轮 PI 目标速度。
 * @param  leftTargetSpeed: 左轮目标速度，单位与 STATUS 中 SPD_L 相同
 * @param  rightTargetSpeed: 右轮目标速度，单位与 STATUS 中 SPD_R 相同
 * @return 1 表示设置成功，0 表示 PI 模式未开启
 */
uint8_t BaseControl_SetWheelPiTarget(float leftTargetSpeed, float rightTargetSpeed)
{
	if(g_ucBasePiEnable == 0u)
	{
		return 0u;
	}

	/* PIMODE ON 时已清积分；同方向更新目标时保留积分，避免每次调速重新爬升。 */
	BaseControl_SetPiTargetsInternal(rightTargetSpeed, leftTargetSpeed);
	g_eBaseControlAction = BASE_ACTION_PI_TEST;
	g_eBaseControlState = BASE_STATE_PI_SPEED;
	return 1u;
}

/**
 * @brief  启动 PI 红外循迹，固定目标速度由底盘内部根据红外状态设置。
 * @param  None
 * @return None
 */
void BaseControl_StartPiLineTrack(void)
{
	BaseControl_ResetPiState();
	g_ucBasePiEnable = 1u;
	g_eBaseControlAction = BASE_ACTION_TRACK;
	g_eBaseControlState = BASE_STATE_LINE_TRACK;
	g_ulBaseLineTrackLastTick = 0u;
	BaseControl_SetPiTargetsInternal(0.0f, 0.0f);
}

/**
 * @brief  执行固定速度 PI 动作，树莓派只发送动作名，不传递 PWM 或速度值。
 * @param  action: 前进、后退、原地左转或原地右转
 * @return 1 表示动作有效，0 表示动作非法
 */
uint8_t BaseControl_StartPiAction(BaseControl_Action_t action)
{
	float rightTargetSpeed = 0.0f;
	float leftTargetSpeed = 0.0f;

	switch(action)
	{
		case BASE_ACTION_FORWARD:
			rightTargetSpeed = BASE_CONTROL_FORWARD_SPEED;
			leftTargetSpeed = BASE_CONTROL_FORWARD_SPEED;
			break;
		case BASE_ACTION_BACKWARD:
			rightTargetSpeed = -BASE_CONTROL_BACKWARD_START_SPEED;
			leftTargetSpeed = -BASE_CONTROL_BACKWARD_START_SPEED;
			break;
		case BASE_ACTION_ROTATE_LEFT:
			rightTargetSpeed = BASE_CONTROL_ROTATE_SPEED;
			leftTargetSpeed = -BASE_CONTROL_ROTATE_SPEED;
			break;
		case BASE_ACTION_ROTATE_RIGHT:
			rightTargetSpeed = -BASE_CONTROL_ROTATE_SPEED;
			leftTargetSpeed = BASE_CONTROL_ROTATE_SPEED;
			break;
		case BASE_ACTION_RETURN_RIGHT:
			rightTargetSpeed = BASE_CONTROL_RETURN_RIGHT_SPEED_R;
			leftTargetSpeed = BASE_CONTROL_RETURN_RIGHT_SPEED_L;
			break;
		default:
			return 0u;
	}

	BaseControl_ResetPiState();
	g_ucBasePiEnable = 1u;
	g_eBaseControlAction = action;
	g_eBaseControlState = (action == BASE_ACTION_RETURN_RIGHT) ?
		BASE_STATE_RETURN_RIGHT : BASE_STATE_PI_SPEED;
	BaseControl_SetPiTargetsInternal(rightTargetSpeed, leftTargetSpeed);
	if(action == BASE_ACTION_BACKWARD)
	{
		/* 目标已从 -2.00 SPD 起步，后续由 PI 周期平滑增加。 */
		g_ucBaseBackwardRampActive = 1u;
	}
	return 1u;
}

/**
 * @brief  获取当前底盘固定动作。
 * @param  None
 * @return 当前动作枚举
 */
BaseControl_Action_t BaseControl_GetAction(void)
{
	return g_eBaseControlAction;
}
/**
 * @brief  在底盘主任务中按实际周期执行一次 PI 运算。
 * @param  rightMeasuredSpeed: 电机1/右轮实测速度
 * @param  leftMeasuredSpeed: 电机2/左轮实测速度
 * @return None
 */
void BaseControl_UpdateWheelPi(float rightMeasuredSpeed, float leftMeasuredSpeed, float periodS)
{
	int16_t rightPwm;
	int16_t leftPwm;

	if((g_ucBasePiEnable == 0u) || ((g_eBaseControlState != BASE_STATE_PI_SPEED) &&
		(g_eBaseControlState != BASE_STATE_LINE_TRACK) &&
		(g_eBaseControlState != BASE_STATE_RETURN_RIGHT)))
	{
		return;
	}

	/* 基础 PWM 负责跨越死区并接近目标速度，PI 只输出有限修正量。 */
	rightPwm = (int16_t)(BaseControl_SpeedToFeedforwardPwm(g_tBasePiRight.targetSpeed) +
		PiSpeed_Update(&g_tBasePiRight, rightMeasuredSpeed, periodS));
	leftPwm = (int16_t)(BaseControl_SpeedToFeedforwardPwm(g_tBasePiLeft.targetSpeed) +
		PiSpeed_Update(&g_tBasePiLeft, leftMeasuredSpeed, periodS));
	BaseControl_ApplyPwm(rightPwm, leftPwm);
	/* 后退从约 -52 PWM 平滑起步，避免首次反向命令直接接近满 PWM。 */
	if(g_ucBaseBackwardRampActive != 0u)
	{
		float rightTarget = BaseControl_RampTarget(g_tBasePiRight.targetSpeed,
			-BASE_CONTROL_BACKWARD_SPEED, BASE_CONTROL_BACKWARD_RAMP_STEP);
		float leftTarget = BaseControl_RampTarget(g_tBasePiLeft.targetSpeed,
			-BASE_CONTROL_BACKWARD_SPEED, BASE_CONTROL_BACKWARD_RAMP_STEP);

		BaseControl_SetPiTargetsInternal(rightTarget, leftTarget);
		if((rightTarget <= -BASE_CONTROL_BACKWARD_SPEED) &&
			(leftTarget <= -BASE_CONTROL_BACKWARD_SPEED))
		{
			g_ucBaseBackwardRampActive = 0u;
		}
	}
}

/**
 * @brief  获取 PI 目标速度和实时输出，供 STATUS 回传与网页观察。
 * @param  leftTargetSpeed: 左轮目标速度输出地址，可为 NULL
 * @param  rightTargetSpeed: 右轮目标速度输出地址，可为 NULL
 * @param  leftPwm: 左轮当前 PI 输出 PWM 地址，可为 NULL
 * @param  rightPwm: 右轮当前 PI 输出 PWM 地址，可为 NULL
 * @return None
 */
void BaseControl_GetPiStatus(float *leftTargetSpeed, float *rightTargetSpeed,
	int16_t *leftPwm, int16_t *rightPwm)
{
	if(leftTargetSpeed != NULL)
	{
		*leftTargetSpeed = g_tBasePiLeft.targetSpeed;
	}
	if(rightTargetSpeed != NULL)
	{
		*rightTargetSpeed = g_tBasePiRight.targetSpeed;
	}
	if(leftPwm != NULL)
	{
		*leftPwm = g_tBasePiLeft.outputPwm;
	}
	if(rightPwm != NULL)
	{
		*rightPwm = g_tBasePiRight.outputPwm;
	}
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
	BaseControl_UpdateEncoderFeedback();
	BaseControl_SafetyTask();

	switch(g_eBaseControlState)
	{
		case BASE_STATE_IDLE:
		case BASE_STATE_MANUAL:
			break;

		case BASE_STATE_LINE_TRACK:
			BaseControl_LineTrackTask();
			break;

		case BASE_STATE_RETURN_RIGHT:
			BaseControl_ReturnRightTask();
			break;

		case BASE_STATE_AVOID:
		case BASE_STATE_PI_SPEED:
			break;

		case BASE_STATE_STOP:
			BaseControl_ApplyPwm(0, 0);
			break;

		default:
			BaseControl_Stop();
			break;
	}
}
