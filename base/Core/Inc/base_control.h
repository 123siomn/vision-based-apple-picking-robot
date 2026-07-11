#ifndef __BASE_CONTROL_H__
#define __BASE_CONTROL_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

/* 底盘速度限幅，单位沿用原 motorPidSetSpeed() 的速度单位 */
#define BASE_CONTROL_MAX_SPEED 5.0f
/* 超声波安全停车默认距离，单位 cm */
#define BASE_CONTROL_SAFE_DISTANCE_CM 20.0f

/* 底盘控制状态，由树莓派命令切换，不再使用旧 g_ucMode 多模式。 */
typedef enum
{
	BASE_STATE_IDLE = 0,       // 空闲：不主动改变电机目标速度
	BASE_STATE_MANUAL,         // 手动速度控制：执行 MOVE 指令给出的左右轮速度
	BASE_STATE_LINE_TRACK,     // 循迹：预留给后续循迹任务接口
	BASE_STATE_AVOID,          // 避障：预留给后续超声波避障任务接口
	BASE_STATE_STOP            // 停止：保持左右轮目标速度为 0
} BaseControl_State_t;

void BaseControl_Stop(void);
void BaseControl_SetWheelSpeed(float leftSpeed, float rightSpeed);
void BaseControl_SetState(BaseControl_State_t state);
BaseControl_State_t BaseControl_GetState(void);
void BaseControl_SetSafetyEnable(uint8_t enable);
uint8_t BaseControl_GetSafetyEnable(void);
void BaseControl_Task(void);

#ifdef __cplusplus
}
#endif

#endif /* __BASE_CONTROL_H__ */
