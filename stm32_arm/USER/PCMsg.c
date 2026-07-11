#include "include.h"

/*
 * 函数功能：初始化 USART1
 * 说明：USART1 固定作为树莓派与机械臂 STM32 的通信串口，使用 PA9/PA10，波特率 115200。
 */
void InitUart1(void)
{
	NVIC_InitTypeDef NVIC_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1|RCC_APB2Periph_GPIOA|RCC_APB2Periph_AFIO, ENABLE);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	USART_InitStructure.USART_BaudRate = 115200;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(USART1, &USART_InitStructure);

	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
	USART_Cmd(USART1, ENABLE);

	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}

/*
 * 函数功能：USART1 发送单字节数据
 * 说明：供 ArmProtocol.c 向树莓派返回 OK/ERR 文本应答。
 */
void Uart1SendData(BYTE dat)
{
	while((USART1->SR & 0X40) == 0);
	USART1->DR = (u8)dat;
	while((USART1->SR & 0X40) == 0);
}

/*
 * 函数功能：USART1 接收中断服务函数
 * 说明：接收树莓派文本协议字节，并交给 ArmProtocol_ReceiveByte() 缓存。
 */
void USART1_IRQHandler(void)
{
	u8 rxBuf;

	if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
	{
		rxBuf = USART_ReceiveData(USART1);
		ArmProtocol_ReceiveByte(rxBuf);
	}
}

/*
 * 函数功能：处理 USART1 树莓派协议任务
 * 说明：保留旧函数名，减少主循环改动；内部只调用新的 ArmProtocol_Task()。
 */
void TaskPCMsgHandle(void)
{
	ArmProtocol_Task();
}