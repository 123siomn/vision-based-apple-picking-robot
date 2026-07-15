#include "include.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#define ARM_PROTOCOL_LINE_SIZE 64
#define ARM_PROTOCOL_RESPONSE_SIZE 96
#define ARM_PROTOCOL_TARGET "ARM"
#define ARM_PROTOCOL_DEFAULT_SEQ "000"

static uint8 gArmProtocolLine[ARM_PROTOCOL_LINE_SIZE];
static uint8 gArmProtocolIndex = 0;
static uint8 gArmProtocolLineReady = FALSE;

/*
 * 函数功能：通过 USART1 发送一段 ASCII 文本
 * 说明：该函数只负责逐字节发送，数据帧格式由上层组帧函数生成。
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
 * 函数功能：计算数据帧 payload 的 8 位累加和
 * 说明：校验范围只包含 $ 之后、* 之前的 ASCII 字符，逗号也参与计算。
 */
static uint8 ArmProtocol_CalcChecksum(const char *payload)
{
	uint8 checksum = 0;

	while(*payload != '\0')
	{
		checksum = (uint8)(checksum + (uint8)(*payload));
		payload++;
	}

	return checksum;
}

/*
 * 函数功能：把 0~15 的数值转换成一个大写十六进制字符
 */
static char ArmProtocol_HexToChar(uint8 value)
{
	value &= 0x0F;
	if(value < 10)
	{
		return (char)('0' + value);
	}
	return (char)('A' + value - 10);
}

/*
 * 函数功能：把一个十六进制字符转换成 0~15 的数值
 * 返回值：TRUE 表示转换成功，FALSE 表示字符非法。
 */
static uint8 ArmProtocol_CharToHex(char ch, uint8 *value)
{
	if((ch >= '0') && (ch <= '9'))
	{
		*value = (uint8)(ch - '0');
		return TRUE;
	}
	if((ch >= 'A') && (ch <= 'F'))
	{
		*value = (uint8)(ch - 'A' + 10);
		return TRUE;
	}
	if((ch >= 'a') && (ch <= 'f'))
	{
		*value = (uint8)(ch - 'a' + 10);
		return TRUE;
	}
	return FALSE;
}

/*
 * 函数功能：解析两位 ASCII 十六进制校验码
 * 返回值：TRUE 表示解析成功，FALSE 表示校验码格式错误。
 */
static uint8 ArmProtocol_ParseChecksum(const char *text, uint8 *checksum)
{
	uint8 high;
	uint8 low;

	if((text[0] == '\0') || (text[1] == '\0') || (text[2] != '\0'))
	{
		return FALSE;
	}
	if(ArmProtocol_CharToHex(text[0], &high) == FALSE)
	{
		return FALSE;
	}
	if(ArmProtocol_CharToHex(text[1], &low) == FALSE)
	{
		return FALSE;
	}

	*checksum = (uint8)((high << 4) | low);
	return TRUE;
}

/*
 * 函数功能：发送标准协议数据帧
 * 帧格式：$ARM,SEQ,TYPE,DETAIL*CS\r\n
 */
static void ArmProtocol_SendFrame(const char *seq, const char *type, const char *detail)
{
	char payload[ARM_PROTOCOL_RESPONSE_SIZE];
	char frame[ARM_PROTOCOL_RESPONSE_SIZE];
	uint8 checksum;

	if(seq == NULL)
	{
		seq = ARM_PROTOCOL_DEFAULT_SEQ;
	}
	if(detail == NULL)
	{
		detail = "UNKNOWN";
	}

	(void)sprintf(payload, "%s,%s,%s,%s", ARM_PROTOCOL_TARGET, seq, type, detail);
	checksum = ArmProtocol_CalcChecksum(payload);
	(void)sprintf(frame, "$%s*%c%c\r\n",
		payload,
		ArmProtocol_HexToChar((uint8)(checksum >> 4)),
		ArmProtocol_HexToChar(checksum));

	ArmProtocol_SendText(frame);
}

/*
 * 函数功能：发送 ACK 应答帧
 * 说明：ACK 表示命令格式正确，并且已经交给机械臂控制层执行。
 */
static void ArmProtocol_SendAck(const char *seq, const char *cmd)
{
	ArmProtocol_SendFrame(seq, "ACK", cmd);
}

/*
 * 函数功能：发送 ERR 应答帧
 * 说明：ERR 表示协议、目标、命令或参数错误。
 */
static void ArmProtocol_SendErr(const char *seq, const char *reason)
{
	ArmProtocol_SendFrame(seq, "ERR", reason);
}

/*
 * 函数功能：接收 USART1 的单个字节并拼接成一行数据帧
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
 * 函数功能：执行 STOP 命令
 */
static void ArmProtocol_HandleStop(const char *seq)
{
	ArmControl_Stop();
	ArmProtocol_SendAck(seq, "STOP");
}

/*
 * 函数功能：执行 HOME 命令
 * 支持格式：$ARM,SEQ,HOME,time*CS
 */
static void ArmProtocol_HandleHome(const char *seq, char *timeText)
{
	int time = 1000;

	if(timeText != NULL)
	{
		time = atoi(timeText);
	}
	if((time < ARM_SERVO_MIN_TIME) || (time > ARM_SERVO_MAX_TIME))
	{
		ArmProtocol_SendErr(seq, "PARAM");
		return;
	}

	ArmControl_SetHome((uint16)time);
	ArmProtocol_SendAck(seq, "HOME");
}

/*
 * 函数功能：执行 SERVO 命令
 * 支持格式：$ARM,SEQ,SERVO,id,pulse,time*CS
 */
static void ArmProtocol_HandleServo(const char *seq, char *idText, char *pulseText, char *timeText)
{
	int id;
	int pulse;
	int time;

	if((idText == NULL) || (pulseText == NULL) || (timeText == NULL))
	{
		ArmProtocol_SendErr(seq, "PARAM");
		return;
	}

	id = atoi(idText);
	pulse = atoi(pulseText);
	time = atoi(timeText);
	if((id < ARM_SERVO_MIN_ID) || (id > ARM_SERVO_MAX_ID))
	{
		ArmProtocol_SendErr(seq, "PARAM");
		return;
	}
	if((pulse < ARM_SERVO_MIN_PULSE) || (pulse > ARM_SERVO_MAX_PULSE))
	{
		ArmProtocol_SendErr(seq, "PARAM");
		return;
	}
	if((time < ARM_SERVO_MIN_TIME) || (time > ARM_SERVO_MAX_TIME))
	{
		ArmProtocol_SendErr(seq, "PARAM");
		return;
	}

	if(ArmControl_SetServo((uint8)id, (uint16)pulse, (uint16)time))
	{
		ArmProtocol_SendAck(seq, "SERVO");
	}
	else
	{
		ArmProtocol_SendErr(seq, "PARAM");
	}
}

/*
 * 函数功能：把机械臂控制状态转换成状态字符串
 */
static const char *ArmProtocol_GetStateText(void)
{
	switch(ArmControl_GetState())
	{
		case ARM_STATE_IDLE:
			return "IDLE";
		case ARM_STATE_SERVO_MOVE:
			return "MOVING";
		case ARM_STATE_STOP:
			return "STOP";
		default:
			return "UNKNOWN";
	}
}

/*
 * 函数功能：执行 STATUS 查询命令
 * 说明：保持原有 $ARM,SEQ,STATUS*CS 查询格式不变，在 STATUS 详情中增加机械臂电池电压。
 */
static void ArmProtocol_HandleStatus(const char *seq)
{
	char detail[ARM_PROTOCOL_RESPONSE_SIZE];

	(void)sprintf(detail, "STATE=%s,VBAT=%umV", ArmProtocol_GetStateText(), GetBatteryVoltage());
	ArmProtocol_SendFrame(seq, "STATUS", detail);
}

/*
 * 函数功能：解析并执行一帧机械臂新协议命令
 * 支持命令：$ARM,SEQ,STOP*CS、$ARM,SEQ,HOME,time*CS、$ARM,SEQ,SERVO,id,pulse,time*CS、$ARM,SEQ,STATUS*CS。
 */
static void ArmProtocol_HandleFrame(char *line)
{
	char *payload;
	char *checksumText;
	char *target;
	char *seq;
	char *cmd;
	char *arg1;
	char *arg2;
	char *arg3;
	char *star;
	uint8 receivedChecksum;
	uint8 calculatedChecksum;

	if(line[0] != '$')
	{
		ArmProtocol_SendErr(ARM_PROTOCOL_DEFAULT_SEQ, "FORMAT");
		return;
	}

	star = strchr(line, '*');
	if(star == NULL)
	{
		ArmProtocol_SendErr(ARM_PROTOCOL_DEFAULT_SEQ, "FORMAT");
		return;
	}

	*star = '\0';
	payload = &line[1];
	checksumText = &star[1];

	if(ArmProtocol_ParseChecksum(checksumText, &receivedChecksum) == FALSE)
	{
		ArmProtocol_SendErr(ARM_PROTOCOL_DEFAULT_SEQ, "CHECKSUM");
		return;
	}

	calculatedChecksum = ArmProtocol_CalcChecksum(payload);
	if(receivedChecksum != calculatedChecksum)
	{
		ArmProtocol_SendErr(ARM_PROTOCOL_DEFAULT_SEQ, "CHECKSUM");
		return;
	}

	target = strtok(payload, ",");
	seq = strtok(NULL, ",");
	cmd = strtok(NULL, ",");
	arg1 = strtok(NULL, ",");
	arg2 = strtok(NULL, ",");
	arg3 = strtok(NULL, ",");

	if((target == NULL) || (seq == NULL) || (cmd == NULL))
	{
		ArmProtocol_SendErr(ARM_PROTOCOL_DEFAULT_SEQ, "FORMAT");
		return;
	}

	if(strcmp(target, ARM_PROTOCOL_TARGET) != 0)
	{
		ArmProtocol_SendErr(seq, "TARGET");
		return;
	}

	if(strcmp(cmd, "STOP") == 0)
	{
		ArmProtocol_HandleStop(seq);
		return;
	}

	if(strcmp(cmd, "HOME") == 0)
	{
		ArmProtocol_HandleHome(seq, arg1);
		return;
	}

	if(strcmp(cmd, "SERVO") == 0)
	{
		ArmProtocol_HandleServo(seq, arg1, arg2, arg3);
		return;
	}

	if(strcmp(cmd, "STATUS") == 0)
	{
		ArmProtocol_HandleStatus(seq);
		return;
	}

	ArmProtocol_SendErr(seq, "CMD");
}

/*
 * 函数功能：机械臂协议周期任务
 * 说明：主循环调用该函数；收到完整一帧后在这里完成校验、解析、执行和应答。
 */
void ArmProtocol_Task(void)
{
	if(gArmProtocolLineReady == FALSE)
	{
		return;
	}

	gArmProtocolLineReady = FALSE;
	ArmProtocol_HandleFrame((char *)gArmProtocolLine);
}
