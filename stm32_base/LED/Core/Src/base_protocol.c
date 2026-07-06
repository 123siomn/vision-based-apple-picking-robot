#include "base_protocol.h"
#include "base_control.h"
#include <stdio.h>
#include <string.h>

/**
* @brief  判断命令字符串是否以指定关键字开头
* @note   用于保持协议解析简单清晰，后续扩展命令时可替换为更完整的解析器。
* @param  line: 树莓派发来的完整命令行
* @param  key: 命令关键字
* @return 1: 匹配  0: 不匹配
*/
static uint8_t BaseProtocol_IsCommand(const char *line, const char *key)
{
	return (strncmp(line, key, strlen(key)) == 0) ? 1u : 0u;
}

/**
* @brief  处理树莓派发来的单行底盘命令
* @note   当前支持 STOP、MOVE L R、SAFE ON/OFF、IDLE、TRACK ON/OFF、AVOID ON。协议扩展集中放在本文件，不放进 usart.c。
* @param  line: USART1 接收到的一行命令，不包含换行符
* @return 无
*/
void BaseProtocol_HandleLine(const char *line)
{
	float leftSpeed = 0.0f;
	float rightSpeed = 0.0f;

	if(line == NULL)
	{
		return;
	}

	if(BaseProtocol_IsCommand(line, "STOP") != 0u)
	{
		BaseControl_Stop();
		return;
	}

	if(BaseProtocol_IsCommand(line, "IDLE") != 0u)
	{
		BaseControl_SetState(BASE_STATE_IDLE);
		return;
	}

	if(BaseProtocol_IsCommand(line, "SAFE ON") != 0u)
	{
		BaseControl_SetSafetyEnable(1u);
		return;
	}

	if(BaseProtocol_IsCommand(line, "SAFE OFF") != 0u)
	{
		BaseControl_SetSafetyEnable(0u);
		return;
	}

	if(BaseProtocol_IsCommand(line, "TRACK ON") != 0u)
	{
		BaseControl_SetState(BASE_STATE_LINE_TRACK);
		return;
	}

	if(BaseProtocol_IsCommand(line, "TRACK OFF") != 0u)
	{
		BaseControl_Stop();
		return;
	}

	if(BaseProtocol_IsCommand(line, "AVOID ON") != 0u)
	{
		BaseControl_SetState(BASE_STATE_AVOID);
		return;
	}

	if(sscanf(line, "MOVE %f %f", &leftSpeed, &rightSpeed) == 2)
	{
		BaseControl_SetWheelSpeed(leftSpeed, rightSpeed);
		return;
	}

	/* 未识别命令暂不执行动作，避免错误命令导致底盘误动。 */
}



