#include "include.h"

static ArmControlState_t gArmControlState = ARM_STATE_IDLE;

/*
 * 函数功能：限制舵机运行时间范围
 * 说明：普通 PWM 舵机动作时间过短容易冲击机构，过长会影响任务响应。
 */
static uint16 ArmControl_LimitTime(uint16 time)
{
	if(time < ARM_SERVO_MIN_TIME)
	{
		return ARM_SERVO_MIN_TIME;
	}
	if(time > ARM_SERVO_MAX_TIME)
	{
		return ARM_SERVO_MAX_TIME;
	}
	return time;
}

/*
 * 函数功能：停止机械臂当前控制任务
 * 说明：停止命令只更新控制状态，不关闭 TIM3 PWM 输出，避免舵机失去保持力。
 */
void ArmControl_Stop(void)
{
	gArmControlState = ARM_STATE_STOP;
}

/*
 * 函数功能：设置单个 PWM 舵机目标脉宽和运动时间
 * 参数：id 为舵机编号 1~6；pulse 为目标脉宽 500~2500us；time 为运动时间 ms。
 * 返回：1 表示命令有效并已下发，0 表示参数非法。
 */
uint8 ArmControl_SetServo(uint8 id, uint16 pulse, uint16 time)
{
	if((id < ARM_SERVO_MIN_ID) || (id > ARM_SERVO_MAX_ID))
	{
		return 0;
	}
	if((pulse < ARM_SERVO_MIN_PULSE) || (pulse > ARM_SERVO_MAX_PULSE))
	{
		return 0;
	}

	ServoSetPluseAndTime(id, pulse, ArmControl_LimitTime(time));
	gArmControlState = ARM_STATE_SERVO_MOVE;
	return 1;
}

/*
 * 函数功能：让 1~6 号 PWM 舵机回到中位
 * 说明：用于上电复位、调试复位和树莓派 HOME 指令。
 */
void ArmControl_SetHome(uint16 time)
{
	uint8 i;
	uint16 moveTime = ArmControl_LimitTime(time);

	for(i = ARM_SERVO_MIN_ID; i <= ARM_SERVO_MAX_ID; i++)
	{
		ServoSetPluseAndTime(i, ARM_SERVO_HOME_PULSE, moveTime);
	}
	gArmControlState = ARM_STATE_SERVO_MOVE;
}

/*
 * 函数功能：机械臂控制周期任务
 * 说明：当前先预留为空，后续可在这里加入抓取状态机或安全限位检查。
 */
void ArmControl_Task(void)
{
}

/*
 * 函数功能：读取机械臂当前控制状态
 */
ArmControlState_t ArmControl_GetState(void)
{
	return gArmControlState;
}