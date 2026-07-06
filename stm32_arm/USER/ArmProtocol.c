#include "include.h"
#include <stdio.h>
#include <string.h>

#define ARM_PROTOCOL_LINE_SIZE 64

static uint8 gArmProtocolLine[ARM_PROTOCOL_LINE_SIZE];
static uint8 gArmProtocolIndex = 0;
static uint8 gArmProtocolLineReady = FALSE;

/*
 * 函数功能：通过 USART1 发送一段文本应答
 * 说明：树莓派调试阶段可根据 OK/ERR 判断命令是否被机械臂接收。
 */
static void ArmProtocol_SendText(const char *text)
{
	while(*text != '\0')
	{
		Uart1SendData((BYTE)(*text));
		text++;
	}
}

/*
 * 函数功能：判断收到的文本命令是否匹配指定关键字
 */
static uint8 ArmProtocol_IsCommand(const char *line, const char *key)
{
	return (strncmp(line, key, strlen(key)) == 0) ? TRUE : FALSE;
}

/*
 * 函数功能：接收 USART1 的单个字节并拼接成一行命令
 * 说明：该函数在 USART1 中断中调用，只做缓存，不在中断里执行舵机动作。
 */
void ArmProtocol_ReceiveByte(uint8 data)
{
	if((data == '\n') || (data == '\r'))
	{
		if(gArmProtocolIndex > 0)
		{
			gArmProtocolLine[gArmProtocolIndex] = '\0';
			gArmProtocolLineReady = TRUE;
			gArmProtocolIndex = 0;
		}
		return;
	}

	if(gArmProtocolIndex < (ARM_PROTOCOL_LINE_SIZE - 1))
	{
		gArmProtocolLine[gArmProtocolIndex++] = data;
	}
	else
	{
		gArmProtocolIndex = 0;
		gArmProtocolLineReady = FALSE;
	}
}

/*
 * 函数功能：解析并执行树莓派下发的机械臂文本命令
 * 支持命令：STOP、HOME [time]、SERVO id pulse time。
 */
void ArmProtocol_Task(void)
{
	char *line;
	int id;
	int pulse;
	int time;

	if(gArmProtocolLineReady == FALSE)
	{
		return;
	}

	gArmProtocolLineReady = FALSE;
	line = (char *)gArmProtocolLine;

	if(ArmProtocol_IsCommand(line, "STOP"))
	{
		ArmControl_Stop();
		ArmProtocol_SendText("OK STOP\r\n");
		return;
	}

	if(ArmProtocol_IsCommand(line, "HOME"))
	{
		time = 1000;
		(void)sscanf(line, "HOME %d", &time);
		ArmControl_SetHome((uint16)time);
		ArmProtocol_SendText("OK HOME\r\n");
		return;
	}

	if(sscanf(line, "SERVO %d %d %d", &id, &pulse, &time) == 3)
	{
		if(ArmControl_SetServo((uint8)id, (uint16)pulse, (uint16)time))
		{
			ArmProtocol_SendText("OK SERVO\r\n");
		}
		else
		{
			ArmProtocol_SendText("ERR SERVO\r\n");
		}
		return;
	}

	ArmProtocol_SendText("ERR CMD\r\n");
}
