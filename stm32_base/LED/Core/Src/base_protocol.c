#include "base_protocol.h"
#include "base_control.h"
#include "usart.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#define BASE_PROTOCOL_RESPONSE_SIZE 96
#define BASE_PROTOCOL_TARGET "BASE"
#define BASE_PROTOCOL_DEFAULT_SEQ "000"

/*
 * 函数功能：计算数据帧 payload 的 8 位累加和
 * 说明：校验范围只包含 $ 之后、* 之前的 ASCII 字符，逗号也参与计算。
 */
static uint8_t BaseProtocol_CalcChecksum(const char *payload)
{
	uint8_t checksum = 0u;

	while(*payload != '\0')
	{
		checksum = (uint8_t)(checksum + (uint8_t)(*payload));
		payload++;
	}

	return checksum;
}

/*
 * 函数功能：把 0~15 的数值转换成一个大写十六进制字符
 */
static char BaseProtocol_HexToChar(uint8_t value)
{
	value &= 0x0Fu;
	if(value < 10u)
	{
		return (char)('0' + value);
	}
	return (char)('A' + value - 10u);
}

/*
 * 函数功能：把一个十六进制字符转换成 0~15 的数值
 * 返回值：1 表示转换成功，0 表示字符非法。
 */
static uint8_t BaseProtocol_CharToHex(char ch, uint8_t *value)
{
	if((ch >= '0') && (ch <= '9'))
	{
		*value = (uint8_t)(ch - '0');
		return 1u;
	}
	if((ch >= 'A') && (ch <= 'F'))
	{
		*value = (uint8_t)(ch - 'A' + 10);
		return 1u;
	}
	if((ch >= 'a') && (ch <= 'f'))
	{
		*value = (uint8_t)(ch - 'a' + 10);
		return 1u;
	}
	return 0u;
}

/*
 * 函数功能：解析两位 ASCII 十六进制校验码
 * 返回值：1 表示解析成功，0 表示校验码格式错误。
 */
static uint8_t BaseProtocol_ParseChecksum(const char *text, uint8_t *checksum)
{
	uint8_t high;
	uint8_t low;

	if((text[0] == '\0') || (text[1] == '\0') || (text[2] != '\0'))
	{
		return 0u;
	}
	if(BaseProtocol_CharToHex(text[0], &high) == 0u)
	{
		return 0u;
	}
	if(BaseProtocol_CharToHex(text[1], &low) == 0u)
	{
		return 0u;
	}

	*checksum = (uint8_t)((high << 4) | low);
	return 1u;
}

/*
 * 函数功能：通过 USART1 发送标准协议数据帧
 * 帧格式：$BASE,SEQ,TYPE,DETAIL*CS\r\n
 */
static void BaseProtocol_SendFrame(const char *seq, const char *type, const char *detail)
{
	char payload[BASE_PROTOCOL_RESPONSE_SIZE];
	char frame[BASE_PROTOCOL_RESPONSE_SIZE];
	uint8_t checksum;

	if(seq == NULL)
	{
		seq = BASE_PROTOCOL_DEFAULT_SEQ;
	}
	if(detail == NULL)
	{
		detail = "UNKNOWN";
	}

	(void)sprintf(payload, "%s,%s,%s,%s", BASE_PROTOCOL_TARGET, seq, type, detail);
	checksum = BaseProtocol_CalcChecksum(payload);
	(void)sprintf(frame, "$%s*%c%c\r\n",
		payload,
		BaseProtocol_HexToChar((uint8_t)(checksum >> 4)),
		BaseProtocol_HexToChar(checksum));

	(void)HAL_UART_Transmit(&huart1, (uint8_t *)frame, (uint16_t)strlen(frame), 100u);
}

/*
 * 函数功能：发送 ACK 应答帧
 * 说明：ACK 表示命令格式正确，并且已经交给底盘控制层执行。
 */
static void BaseProtocol_SendAck(const char *seq, const char *cmd)
{
	BaseProtocol_SendFrame(seq, "ACK", cmd);
}

/*
 * 函数功能：发送 ERR 应答帧
 * 说明：ERR 表示协议、目标、命令或参数错误。
 */
static void BaseProtocol_SendErr(const char *seq, const char *reason)
{
	BaseProtocol_SendFrame(seq, "ERR", reason);
}

/*
 * 函数功能：把底盘控制状态转换成状态字符串
 */
static const char *BaseProtocol_GetStateText(void)
{
	switch(BaseControl_GetState())
	{
		case BASE_STATE_IDLE:
			return "IDLE";
		case BASE_STATE_MANUAL:
			return "MANUAL";
		case BASE_STATE_LINE_TRACK:
			return "LINE_TRACK";
		case BASE_STATE_AVOID:
			return "AVOID";
		case BASE_STATE_STOP:
			return "STOP";
		default:
			return "UNKNOWN";
	}
}

/*
 * 函数功能：发送底盘状态帧
 * 说明：当前先返回状态和超声波安全开关，距离和障碍事件后续再扩展。
 */
static void BaseProtocol_SendStatus(const char *seq)
{
	char detail[BASE_PROTOCOL_RESPONSE_SIZE];

	(void)sprintf(detail, "STATE=%s,SAFE=%s",
		BaseProtocol_GetStateText(),
		(BaseControl_GetSafetyEnable() != 0u) ? "ON" : "OFF");
	BaseProtocol_SendFrame(seq, "STATUS", detail);
}

/*
 * 函数功能：执行 MOVE 命令
 * 支持格式：$BASE,SEQ,MOVE,left,right*CS
 */
static void BaseProtocol_HandleMove(const char *seq, char *leftText, char *rightText)
{
	float leftSpeed;
	float rightSpeed;

	if((leftText == NULL) || (rightText == NULL))
	{
		BaseProtocol_SendErr(seq, "PARAM");
		return;
	}

	leftSpeed = (float)atof(leftText);
	rightSpeed = (float)atof(rightText);
	if((leftSpeed > BASE_CONTROL_MAX_SPEED) || (leftSpeed < -BASE_CONTROL_MAX_SPEED) ||
		(rightSpeed > BASE_CONTROL_MAX_SPEED) || (rightSpeed < -BASE_CONTROL_MAX_SPEED))
	{
		BaseProtocol_SendErr(seq, "PARAM");
		return;
	}

	BaseControl_SetWheelSpeed(leftSpeed, rightSpeed);
	BaseProtocol_SendAck(seq, "MOVE");
}

/*
 * 函数功能：执行 SAFE 命令
 * 支持格式：$BASE,SEQ,SAFE,ON*CS 或 $BASE,SEQ,SAFE,OFF*CS
 */
static void BaseProtocol_HandleSafe(const char *seq, char *switchText)
{
	if(switchText == NULL)
	{
		BaseProtocol_SendErr(seq, "PARAM");
		return;
	}

	if(strcmp(switchText, "ON") == 0)
	{
		BaseControl_SetSafetyEnable(1u);
		BaseProtocol_SendAck(seq, "SAFE");
		return;
	}
	if(strcmp(switchText, "OFF") == 0)
	{
		BaseControl_SetSafetyEnable(0u);
		BaseProtocol_SendAck(seq, "SAFE");
		return;
	}

	BaseProtocol_SendErr(seq, "PARAM");
}

/*
 * 函数功能：执行 TRACK 命令
 * 支持格式：$BASE,SEQ,TRACK,ON*CS 或 $BASE,SEQ,TRACK,OFF*CS
 */
static void BaseProtocol_HandleTrack(const char *seq, char *switchText)
{
	if(switchText == NULL)
	{
		BaseProtocol_SendErr(seq, "PARAM");
		return;
	}

	if(strcmp(switchText, "ON") == 0)
	{
		BaseControl_SetState(BASE_STATE_LINE_TRACK);
		BaseProtocol_SendAck(seq, "TRACK");
		return;
	}
	if(strcmp(switchText, "OFF") == 0)
	{
		BaseControl_Stop();
		BaseProtocol_SendAck(seq, "TRACK");
		return;
	}

	BaseProtocol_SendErr(seq, "PARAM");
}

/*
 * 函数功能：执行 AVOID 命令
 * 支持格式：$BASE,SEQ,AVOID,ON*CS
 * 说明：当前只进入预留避障状态，具体避障动作后续再实现。
 */
static void BaseProtocol_HandleAvoid(const char *seq, char *switchText)
{
	if((switchText != NULL) && (strcmp(switchText, "ON") == 0))
	{
		BaseControl_SetState(BASE_STATE_AVOID);
		BaseProtocol_SendAck(seq, "AVOID");
		return;
	}

	BaseProtocol_SendErr(seq, "PARAM");
}

/*
 * 函数功能：解析并执行一帧底盘新协议命令
 * 支持命令：STOP、IDLE、MOVE、SAFE、TRACK、AVOID、STATUS。
 */
static void BaseProtocol_HandleFrame(char *line)
{
	char *payload;
	char *checksumText;
	char *target;
	char *seq;
	char *cmd;
	char *arg1;
	char *arg2;
	char *star;
	uint8_t receivedChecksum;
	uint8_t calculatedChecksum;

	if(line[0] != '$')
	{
		BaseProtocol_SendErr(BASE_PROTOCOL_DEFAULT_SEQ, "FORMAT");
		return;
	}

	star = strchr(line, '*');
	if(star == NULL)
	{
		BaseProtocol_SendErr(BASE_PROTOCOL_DEFAULT_SEQ, "FORMAT");
		return;
	}

	*star = '\0';
	payload = &line[1];
	checksumText = &star[1];

	if(BaseProtocol_ParseChecksum(checksumText, &receivedChecksum) == 0u)
	{
		BaseProtocol_SendErr(BASE_PROTOCOL_DEFAULT_SEQ, "CHECKSUM");
		return;
	}

	calculatedChecksum = BaseProtocol_CalcChecksum(payload);
	if(receivedChecksum != calculatedChecksum)
	{
		BaseProtocol_SendErr(BASE_PROTOCOL_DEFAULT_SEQ, "CHECKSUM");
		return;
	}

	target = strtok(payload, ",");
	seq = strtok(NULL, ",");
	cmd = strtok(NULL, ",");
	arg1 = strtok(NULL, ",");
	arg2 = strtok(NULL, ",");

	if((target == NULL) || (seq == NULL) || (cmd == NULL))
	{
		BaseProtocol_SendErr(BASE_PROTOCOL_DEFAULT_SEQ, "FORMAT");
		return;
	}

	if(strcmp(target, BASE_PROTOCOL_TARGET) != 0)
	{
		BaseProtocol_SendErr(seq, "TARGET");
		return;
	}

	if(strcmp(cmd, "STOP") == 0)
	{
		BaseControl_Stop();
		BaseProtocol_SendAck(seq, "STOP");
		return;
	}

	if(strcmp(cmd, "IDLE") == 0)
	{
		BaseControl_SetState(BASE_STATE_IDLE);
		BaseProtocol_SendAck(seq, "IDLE");
		return;
	}

	if(strcmp(cmd, "MOVE") == 0)
	{
		BaseProtocol_HandleMove(seq, arg1, arg2);
		return;
	}

	if(strcmp(cmd, "SAFE") == 0)
	{
		BaseProtocol_HandleSafe(seq, arg1);
		return;
	}

	if(strcmp(cmd, "TRACK") == 0)
	{
		BaseProtocol_HandleTrack(seq, arg1);
		return;
	}

	if(strcmp(cmd, "AVOID") == 0)
	{
		BaseProtocol_HandleAvoid(seq, arg1);
		return;
	}

	if(strcmp(cmd, "STATUS") == 0)
	{
		BaseProtocol_SendStatus(seq);
		return;
	}

	BaseProtocol_SendErr(seq, "CMD");
}

/*
 * 函数功能：处理树莓派发来的单行底盘数据帧
 * 说明：USART1 接收层只负责缓存一行，本函数负责协议校验、解析、执行和应答。
 */
void BaseProtocol_HandleLine(const char *line)
{
	char frame[BASE_PROTOCOL_RESPONSE_SIZE];

	if(line == NULL)
	{
		return;
	}

	(void)strncpy(frame, line, sizeof(frame) - 1u);
	frame[sizeof(frame) - 1u] = '\0';
	BaseProtocol_HandleFrame(frame);
}
