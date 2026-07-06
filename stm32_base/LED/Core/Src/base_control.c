#include "base_control.h"
#include "motor.h"
#include "HC_SR04.h"
#include "adc.h"
#include "pid.h"

extern tPid pidVoltage;

static BaseControl_State_t g_eBaseControlState = BASE_STATE_IDLE;
static float g_fBaseTargetLeftSpeed = 0.0f;
static float g_fBaseTargetRightSpeed = 0.0f;
static uint8_t g_ucBaseSafetyEnable = 0;
static uint32_t g_ulBaseSafetyLastTick = 0;
static uint32_t g_ulBaseLineTrackLastTick = 0;

static const float g_fLineVoltageMax[4] = {2.89f, 2.89f, 2.89f, 2.89f};

/**
* @brief  对底盘目标速度做限幅
* @note   防止树莓派命令异常时给电机 PID 写入过大的目标速度。
* @param  speed: 输入速度
* @return 限幅后的速度
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
* @brief  执行超声波安全停车检查
* @note   开启安全保护后周期读取 HC_SR04，距离过近时立即停车；负数超时值不触发停车。
* @param  无
* @return 无
*/
static void BaseControl_SafetyTask(void)
{
	float distance;
	uint32_t now = HAL_GetTick();

	if(g_ucBaseSafetyEnable == 0u)
	{
		return;
	}

	/* HC_SR04_Read() 不需要高频调用，这里限制为约 50ms 一次。 */
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
* @brief  采集四路灰度 ADC 电压
* @note   ADC1 通道顺序沿用 CubeMX 配置：PA5、PA7、PB0、PB1。
* @param  voltage: 用于保存四路电压值的数组
* @return 无
*/
static void BaseControl_ReadLineVoltage(float voltage[4])
{
	uint8_t i;

	for(i = 0; i < 4u; i++)
	{
		HAL_ADC_Start(&hadc1);
		if(HAL_ADC_PollForConversion(&hadc1, 50) == HAL_OK)
		{
			voltage[i] = (float)HAL_ADC_GetValue(&hadc1) / 4096.0f * 3.3f;
		}
		else
		{
			voltage[i] = 0.0f;
		}
	}
}

/**
* @brief  执行灰度循迹任务
* @note   使用四路 ADC 灰度差分误差和 pidVoltage，输出左右轮目标速度。
* @param  无
* @return 无
*/
static void BaseControl_LineTrackTask(void)
{
	float voltage[4];
	float normalized[4];
	float outerError;
	float turn;
	float leftSpeed;
	float rightSpeed;
	uint8_t i;
	uint32_t now = HAL_GetTick();

	/* 循迹控制不需要每轮主循环都算，限制为约 20ms 一次。 */
	if((now - g_ulBaseLineTrackLastTick) < 20u)
	{
		return;
	}
	g_ulBaseLineTrackLastTick = now;

	BaseControl_ReadLineVoltage(voltage);
	for(i = 0; i < 4u; i++)
	{
		if(voltage[i] > g_fLineVoltageMax[i])
		{
			voltage[i] = g_fLineVoltageMax[i];
		}
		normalized[i] = voltage[i] / g_fLineVoltageMax[i] * 100.0f;
	}

	/* 外侧差分误差：(4号 - 1号) / (4号 + 1号 + 1)，沿用旧灰度循迹思路。 */
	outerError = (normalized[3] - normalized[0]) / (normalized[3] + normalized[0] + 1.0f);
	turn = PID_realize(&pidVoltage, outerError);

	leftSpeed = BaseControl_LimitSpeed(1.5f + turn);
	rightSpeed = BaseControl_LimitSpeed(1.5f - turn);
	g_fBaseTargetLeftSpeed = leftSpeed;
	g_fBaseTargetRightSpeed = rightSpeed;
	motorPidSetSpeed(leftSpeed, rightSpeed);
}

/**
* @brief  底盘停车接口
* @note   进入停止状态，并把左右轮 PID 目标速度清零。
* @param  无
* @return 无
*/
void BaseControl_Stop(void)
{
	g_fBaseTargetLeftSpeed = 0.0f;
	g_fBaseTargetRightSpeed = 0.0f;
	g_eBaseControlState = BASE_STATE_STOP;
	motorPidSetSpeed(0.0f, 0.0f);
}

/**
* @brief  设置底盘左右轮目标速度
* @note   当前直接调用已有电机 PID 速度接口，并把状态切换为手动速度控制。
* @param  leftSpeed: 左轮目标速度
* @param  rightSpeed: 右轮目标速度
* @return 无
*/
void BaseControl_SetWheelSpeed(float leftSpeed, float rightSpeed)
{
	g_fBaseTargetLeftSpeed = BaseControl_LimitSpeed(leftSpeed);
	g_fBaseTargetRightSpeed = BaseControl_LimitSpeed(rightSpeed);
	g_eBaseControlState = BASE_STATE_MANUAL;
	motorPidSetSpeed(g_fBaseTargetLeftSpeed, g_fBaseTargetRightSpeed);
}

/**
* @brief  设置底盘控制状态
* @note   当前用于预留循迹、避障等状态入口。切到 STOP 时会立即停车。
* @param  state: 目标底盘控制状态
* @return 无
*/
void BaseControl_SetState(BaseControl_State_t state)
{
	g_eBaseControlState = state;
	if(state == BASE_STATE_STOP)
	{
		BaseControl_Stop();
	}
}

/**
* @brief  获取当前底盘控制状态
* @note   后续状态上报、调试显示和安全逻辑可通过该接口读取状态。
* @param  无
* @return 当前底盘控制状态
*/
BaseControl_State_t BaseControl_GetState(void)
{
	return g_eBaseControlState;
}

/**
* @brief  设置超声波安全停车开关
* @note   开启后 BaseControl_Task() 会周期读取 HC_SR04，距离过近时自动停车。
* @param  enable: 0 关闭，非 0 开启
* @return 无
*/
void BaseControl_SetSafetyEnable(uint8_t enable)
{
	g_ucBaseSafetyEnable = (enable == 0u) ? 0u : 1u;
	g_ulBaseSafetyLastTick = 0;
}

/**
* @brief  获取超声波安全停车开关状态
* @note   后续 STATUS 状态上报可使用该接口。
* @param  无
* @return 1: 已开启  0: 已关闭
*/
uint8_t BaseControl_GetSafetyEnable(void)
{
	return g_ucBaseSafetyEnable;
}

/**
* @brief  底盘控制周期任务
* @note   由 main() 主循环周期调用。循迹、避障、MPU6050 航向控制都在这里按状态分发。
* @param  无
* @return 无
*/
void BaseControl_Task(void)
{
	BaseControl_SafetyTask();

	switch(g_eBaseControlState)
	{
		case BASE_STATE_IDLE:
			break;

		case BASE_STATE_MANUAL:
			/* 手动模式下速度目标已在收到 MOVE 命令时写入 PID，这里暂不重复写入。 */
			break;

		case BASE_STATE_LINE_TRACK:
			BaseControl_LineTrackTask();
			break;

		case BASE_STATE_AVOID:
			/* 待补充：后续在这里调用超声波避障任务接口。 */
			break;

		case BASE_STATE_STOP:
			motorPidSetSpeed(0.0f, 0.0f);
			break;

		default:
			BaseControl_Stop();
			break;
	}
}
