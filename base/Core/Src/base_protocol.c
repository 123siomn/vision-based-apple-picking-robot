#include "base_protocol.h"
#include "base_control.h"
#include "adc.h"
#include "usart.h"
#include "motor.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#define BASE_PROTOCOL_RESPONSE_SIZE 384
#define BASE_PROTOCOL_TARGET "BASE"
#define BASE_PROTOCOL_DEFAULT_SEQ "000"

/* 协议帧最长约 300 字节，使用静态缓冲区避免 STATUS 处理时占满 STM32 主栈。 */
static char g_acBaseProtocolPayload[BASE_PROTOCOL_RESPONSE_SIZE];
static char g_acBaseProtocolTxFrame[BASE_PROTOCOL_RESPONSE_SIZE];
static char g_acBaseProtocolStatusDetail[BASE_PROTOCOL_RESPONSE_SIZE];
static char g_acBaseProtocolRxFrame[BASE_PROTOCOL_RESPONSE_SIZE];

/* 编码器与轮速由 TIM1 中断更新，STATUS 只读取并回传，不参与控制。 */
extern short Encode1Count;
extern short Encode2Count;
extern float Motor1Speed;
extern float Motor2Speed;

/**
 * @brief  计算协议 payload 的 8 位累加和。
 * @param  payload: 位于 '$' 和 '*' 之间的字符串
 * @return 8 位累加校验值
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

/**
 * @brief  将 4 位数值转换为大写十六进制字符。
 * @param  value: 输入数值，仅使用低 4 位
 * @return ASCII 十六进制字符
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

/**
 * @brief  将一个 ASCII 十六进制字符转换为 4 位数值。
 * @param  ch: ASCII 十六进制字符
 * @param  value: 输出转换后的数值
 * @return 1 表示转换成功，0 表示字符非法
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

/**
 * @brief  解析两位 ASCII 十六进制校验码。
 * @param  text: 以 '\0' 结尾的校验码字符串
 * @param  checksum: 输出解析后的校验值
 * @return 1 表示解析成功，0 表示格式错误
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

/**
 * @brief  通过 USART1 发送一帧标准底盘应答。
 * @param  seq: 请求帧中的序号
 * @param  type: 应答类型，例如 ACK、ERR 或 STATUS
 * @param  detail: 应答详情字符串
 * @return None
 */
static void BaseProtocol_SendFrame(const char *seq, const char *type, const char *detail)
{
	uint8_t checksum;

	if(seq == NULL)
	{
		seq = BASE_PROTOCOL_DEFAULT_SEQ;
	}
	if(detail == NULL)
	{
		detail = "UNKNOWN";
	}

	(void)sprintf(g_acBaseProtocolPayload, "%s,%s,%s,%s", BASE_PROTOCOL_TARGET, seq, type, detail);
	checksum = BaseProtocol_CalcChecksum(g_acBaseProtocolPayload);
	(void)sprintf(g_acBaseProtocolTxFrame, "$%s*%c%c\r\n",
		g_acBaseProtocolPayload,
		BaseProtocol_HexToChar((uint8_t)(checksum >> 4)),
		BaseProtocol_HexToChar(checksum));

	(void)HAL_UART_Transmit(&huart1, (uint8_t *)g_acBaseProtocolTxFrame,
		(uint16_t)strlen(g_acBaseProtocolTxFrame), 100u);
}

/**
 * @brief  发送命令接收成功应答。
 * @param  seq: 请求帧中的序号
 * @param  cmd: 已接收的命令名
 * @return None
 */
static void BaseProtocol_SendAck(const char *seq, const char *cmd)
{
	BaseProtocol_SendFrame(seq, "ACK", cmd);
}

/**
 * @brief  发送命令错误应答。
 * @param  seq: 请求帧中的序号，无法获取时使用默认序号
 * @param  reason: 错误原因
 * @return None
 */
static void BaseProtocol_SendErr(const char *seq, const char *reason)
{
	BaseProtocol_SendFrame(seq, "ERR", reason);
}

/**
 * @brief  将底盘状态枚举转换为状态字符串。
 * @param  None
 * @return 状态字符串
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
		case BASE_STATE_PI_SPEED:
			return "PI_SPEED";
		case BASE_STATE_STOP:
			return "STOP";
		default:
			return "UNKNOWN";
	}
}

/**
 * @brief  将当前固定 PI 动作转换为 STATUS 回传文本。
 * @return 动作文本
 */
static const char *BaseProtocol_GetActionText(void)
{
	switch(BaseControl_GetAction())
	{
		case BASE_ACTION_TRACK:
			return "TRACK";
		case BASE_ACTION_FORWARD:
			return "FORWARD";
		case BASE_ACTION_BACKWARD:
			return "BACKWARD";
		case BASE_ACTION_ROTATE_LEFT:
			return "ROTATE_LEFT";
		case BASE_ACTION_ROTATE_RIGHT:
			return "ROTATE_RIGHT";
		case BASE_ACTION_PI_TEST:
			return "PI_TEST";
		case BASE_ACTION_STOP:
		default:
			return "STOP";
	}
}
/**
 * @brief  发送底盘状态应答。
 * @param  seq: 请求帧中的序号
 * @return None
 */
static void BaseProtocol_SendStatus(const char *seq, uint8_t includeDebug)
{
	char batteryText[20];
	uint8_t line1;
	uint8_t line2;
	uint8_t line3;
	uint8_t line4;
	const char *lineAction;
	int32_t batteryMv;
	float piLeftTarget;
	float piRightTarget;
	int16_t piLeftPwm;
	int16_t piRightPwm;

	line1 = (HAL_GPIO_ReadPin(HW_OUT_1_GPIO_Port, HW_OUT_1_Pin) == GPIO_PIN_SET) ? 1u : 0u;
	line2 = (HAL_GPIO_ReadPin(HW_OUT_2_GPIO_Port, HW_OUT_2_Pin) == GPIO_PIN_SET) ? 1u : 0u;
	line3 = (HAL_GPIO_ReadPin(HW_OUT_3_GPIO_Port, HW_OUT_3_Pin) == GPIO_PIN_SET) ? 1u : 0u;
	line4 = (HAL_GPIO_ReadPin(HW_OUT_4_GPIO_Port, HW_OUT_4_Pin) == GPIO_PIN_SET) ? 1u : 0u;

	if(line2 != 0u)
	{
		lineAction = "RIGHT_STOP";
	}
	else if((line3 != 0u) || (line4 != 0u))
	{
		lineAction = "LEFT_STOP";
	}
	else
	{
		lineAction = "STRAIGHT";
	}

	BaseControl_GetPiStatus(&piLeftTarget, &piRightTarget, &piLeftPwm, &piRightPwm);
	batteryMv = BaseAdc_ReadBatteryVoltageMv();
	if(batteryMv < 0)
	{
		(void)sprintf(batteryText, "ERR");
	}
	else
	{
		(void)sprintf(batteryText, "%ldmV", (long)batteryMv);
	}

	if(includeDebug != 0u)
	{
		(void)sprintf(g_acBaseProtocolStatusDetail,
			"STATE=%s,SAFE=%s,VBAT=%s,LINE=%u%u%u%u,ACT=%s,LINEACT=%s,ENC_R=%d,ENC_L=%d,SPD_R=%.2f,SPD_L=%.2f,PI=%s,TGTSPD_R=%.2f,TGTSPD_L=%.2f,PWM_R=%d,PWM_L=%d,PIOUT_R=%d,PIOUT_L=%d",
			BaseProtocol_GetStateText(),
			(BaseControl_GetSafetyEnable() != 0u) ? "ON" : "OFF",
			batteryText,
			(unsigned int)line1, (unsigned int)line2, (unsigned int)line3, (unsigned int)line4,
			BaseProtocol_GetActionText(), lineAction,
			(int)Encode1Count, (int)Encode2Count, Motor1Speed, Motor2Speed,
			(BaseControl_GetPiEnable() != 0u) ? "ON" : "OFF",
			piRightTarget, piLeftTarget,
			Motor_GetLastCmd1(), Motor_GetLastCmd2(),
			(int)piRightPwm, (int)piLeftPwm);
	}
	else
	{
		(void)sprintf(g_acBaseProtocolStatusDetail,
			"STATE=%s,SAFE=%s,VBAT=%s,LINE=%u%u%u%u,ACT=%s,LINEACT=%s,ENC_R=%d,ENC_L=%d,SPD_R=%.2f,SPD_L=%.2f,PI=%s,TGTSPD_R=%.2f,TGTSPD_L=%.2f",
			BaseProtocol_GetStateText(),
			(BaseControl_GetSafetyEnable() != 0u) ? "ON" : "OFF",
			batteryText,
			(unsigned int)line1, (unsigned int)line2, (unsigned int)line3, (unsigned int)line4,
			BaseProtocol_GetActionText(), lineAction,
			(int)Encode1Count, (int)Encode2Count, Motor1Speed, Motor2Speed,
			(BaseControl_GetPiEnable() != 0u) ? "ON" : "OFF",
			piRightTarget, piLeftTarget);
	}
	BaseProtocol_SendFrame(seq, "STATUS", g_acBaseProtocolStatusDetail);
}
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

	BaseControl_SetWheelOpenLoop(leftSpeed, rightSpeed);
	BaseProtocol_SendAck(seq, "MOVE");
}

/**
 * @brief  执行 SAFE 命令。
 * @param  seq: 请求帧中的序号
 * @param  switchText: ON 或 OFF
 * @return None
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
#if (BASE_CONTROL_ENABLE_ULTRASONIC_SAFETY == 0u)
		BaseProtocol_SendErr(seq, "DISABLED");
		return;
#else
		BaseControl_SetSafetyEnable(1u);
		BaseProtocol_SendAck(seq, "SAFE");
		return;
#endif
	}
	if(strcmp(switchText, "OFF") == 0)
	{
		BaseControl_SetSafetyEnable(0u);
		BaseProtocol_SendAck(seq, "SAFE");
		return;
	}

	BaseProtocol_SendErr(seq, "PARAM");
}

/**
 * @brief  执行 TRACK 命令。
 * @param  seq: 请求帧中的序号
 * @param  switchText: ON 或 OFF
 * @return None
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
		BaseControl_StartPiLineTrack();
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

/**
 * @brief  执行 TRACKPWM 命令，设置红外循迹直行时的开环基础 PWM。
 * @param  seq: 请求帧中的序号
 * @param  pwmText: PWM 整数字符串，范围 0~99
 * @return None
 */
static void BaseProtocol_HandleTrackPwm(const char *seq, char *pwmText)
{
	char *endText;
	long pwm;

	if((pwmText == NULL) || (*pwmText == '\0'))
	{
		BaseProtocol_SendErr(seq, "PARAM");
		return;
	}

	pwm = strtol(pwmText, &endText, 10);
	if((*endText != '\0') || (pwm < 0L) || (pwm > (long)BASE_CONTROL_MAX_PWM))
	{
		BaseProtocol_SendErr(seq, "PARAM");
		return;
	}

	BaseControl_SetLineBasePwm((int16_t)pwm);
	BaseProtocol_SendAck(seq, "TRACKPWM");
}

/**
 * @brief  执行 OPENPWM 命令，用于网页记录开环 PWM 与编码器速度的对应关系。
 * @param  seq: 请求帧中的序号
 * @param  leftText: 左轮 PWM 字符串，范围 -99~99
 * @param  rightText: 右轮 PWM 字符串，范围 -99~99
 * @return None
 */
static void BaseProtocol_HandleOpenPwm(const char *seq, char *leftText, char *rightText)
{
	char *leftEnd;
	char *rightEnd;
	long leftPwm;
	long rightPwm;

	if((leftText == NULL) || (rightText == NULL))
	{
		BaseProtocol_SendErr(seq, "PARAM");
		return;
	}

	leftPwm = strtol(leftText, &leftEnd, 10);
	rightPwm = strtol(rightText, &rightEnd, 10);
	if((*leftEnd != '\0') || (*rightEnd != '\0') ||
		(leftPwm > (long)BASE_CONTROL_MAX_PWM) || (leftPwm < -(long)BASE_CONTROL_MAX_PWM) ||
		(rightPwm > (long)BASE_CONTROL_MAX_PWM) || (rightPwm < -(long)BASE_CONTROL_MAX_PWM))
	{
		BaseProtocol_SendErr(seq, "PARAM");
		return;
	}

	BaseControl_SetState(BASE_STATE_MANUAL);
	BaseControl_SetOpenLoopPwm((int16_t)rightPwm, (int16_t)leftPwm);
	BaseProtocol_SendAck(seq, "OPENPWM");
}

/**
 * @brief  解析 PI 目标速度参数。
 * @param  text: 速度字符串
 * @param  value: 输出速度地址
 * @return 1 表示成功，0 表示格式或范围错误
 */
static uint8_t BaseProtocol_ParsePiTarget(const char *text, float *value)
{
	char *endText;
	double parsed;

	if((text == NULL) || (*text == '\0') || (value == NULL))
	{
		return 0u;
	}

	parsed = strtod(text, &endText);
	if((parsed != parsed) || (*endText != '\0') || (parsed > (double)BASE_CONTROL_PI_MAX_TARGET_SPEED) ||
		(parsed < -(double)BASE_CONTROL_PI_MAX_TARGET_SPEED))
	{
		return 0u;
	}

	*value = (float)parsed;
	return 1u;
}

/**
 * @brief  执行 PIMODE 命令，单独开启或关闭 PI 轮速测试模式。
 * @param  seq: 请求帧中的序号
 * @param  switchText: ON 或 OFF
 * @return None
 */
static void BaseProtocol_HandlePiMode(const char *seq, char *switchText)
{
	if(switchText == NULL)
	{
		BaseProtocol_SendErr(seq, "PARAM");
		return;
	}
	if(strcmp(switchText, "ON") == 0)
	{
		BaseControl_SetPiEnable(1u);
		BaseProtocol_SendAck(seq, "PIMODE");
		return;
	}
	if(strcmp(switchText, "OFF") == 0)
	{
		BaseControl_SetPiEnable(0u);
		BaseProtocol_SendAck(seq, "PIMODE");
		return;
	}

	BaseProtocol_SendErr(seq, "PARAM");
}

/**
 * @brief  执行 PITARGET 命令，设置左右轮的 PI 目标速度。
 * @param  seq: 请求帧中的序号
 * @param  leftText: 左轮目标速度字符串
 * @param  rightText: 右轮目标速度字符串
 * @return None
 */
static void BaseProtocol_HandlePiTarget(const char *seq, char *leftText, char *rightText)
{
	float leftTarget;
	float rightTarget;

	if((BaseProtocol_ParsePiTarget(leftText, &leftTarget) == 0u) ||
		(BaseProtocol_ParsePiTarget(rightText, &rightTarget) == 0u))
	{
		BaseProtocol_SendErr(seq, "PARAM");
		return;
	}
	if(BaseControl_SetWheelPiTarget(leftTarget, rightTarget) == 0u)
	{
		BaseProtocol_SendErr(seq, "PI_OFF");
		return;
	}

	BaseProtocol_SendAck(seq, "PITARGET");
}
/**
 * @brief  执行 AVOID 命令占位逻辑。
 * @param  seq: 请求帧中的序号
 * @param  switchText: ON 表示进入预留避障状态
 * @return None
 */
/**
 * @brief  执行固定 PI 底盘动作，命令不携带 PWM 或目标速度。
 * @param  seq: 请求序号
 * @param  action: 固定底盘动作
 * @param  name: 用于 ACK 的命令名
 * @return None
 */
static void BaseProtocol_HandleFixedAction(const char *seq, BaseControl_Action_t action,
	const char *name)
{
	if(BaseControl_StartPiAction(action) == 0u)
	{
		BaseProtocol_SendErr(seq, "PARAM");
		return;
	}
	BaseProtocol_SendAck(seq, name);
}
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

/**
 * @brief  解析并执行一帧标准底盘命令。
 * @param  line: 可修改的帧缓存
 * @return None
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
	if(strcmp(cmd, "TRACKPWM") == 0)
	{
		BaseProtocol_HandleTrackPwm(seq, arg1);
		return;
	}
	if(strcmp(cmd, "OPENPWM") == 0)
	{
		BaseProtocol_HandleOpenPwm(seq, arg1, arg2);
		return;
	}
	if(strcmp(cmd, "PIMODE") == 0)
	{
		BaseProtocol_HandlePiMode(seq, arg1);
		return;
	}
	if(strcmp(cmd, "PITARGET") == 0)
	{
		BaseProtocol_HandlePiTarget(seq, arg1, arg2);
		return;
	}
	if(strcmp(cmd, "FORWARD") == 0)
	{
		BaseProtocol_HandleFixedAction(seq, BASE_ACTION_FORWARD, "FORWARD");
		return;
	}
	if(strcmp(cmd, "BACKWARD") == 0)
	{
		BaseProtocol_HandleFixedAction(seq, BASE_ACTION_BACKWARD, "BACKWARD");
		return;
	}
	if(strcmp(cmd, "ROTATE_LEFT") == 0)
	{
		BaseProtocol_HandleFixedAction(seq, BASE_ACTION_ROTATE_LEFT, "ROTATE_LEFT");
		return;
	}
	if(strcmp(cmd, "ROTATE_RIGHT") == 0)
	{
		BaseProtocol_HandleFixedAction(seq, BASE_ACTION_ROTATE_RIGHT, "ROTATE_RIGHT");
		return;
	}
	if(strcmp(cmd, "AVOID") == 0)
	{
		BaseProtocol_HandleAvoid(seq, arg1);
		return;
	}
	if(strcmp(cmd, "STATUS") == 0)
	{
		BaseProtocol_SendStatus(seq, 0u);
		return;
	}
	if(strcmp(cmd, "STATUSDBG") == 0)
	{
		BaseProtocol_SendStatus(seq, 1u);
		return;
	}

	BaseProtocol_SendErr(seq, "CMD");
}

/**
 * @brief  处理 USART1 接收层交付的一行完整底盘命令。
 * @param  line: 以 '\0' 结尾的命令行字符串
 * @return None
 */
void BaseProtocol_HandleLine(const char *line)
{
	if(line == NULL)
	{
		return;
	}

	(void)strncpy(g_acBaseProtocolRxFrame, line, sizeof(g_acBaseProtocolRxFrame) - 1u);
	g_acBaseProtocolRxFrame[sizeof(g_acBaseProtocolRxFrame) - 1u] = '\0';
	BaseProtocol_HandleFrame(g_acBaseProtocolRxFrame);
}
