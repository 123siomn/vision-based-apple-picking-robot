/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    usart.c
  * @brief   This file provides code for the configuration
  *          of the USART instances.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "usart.h"

/* USER CODE BEGIN 0 */
#include "base_protocol.h"

#define BASE_CMD_RX_BUFFER_SIZE 96U

static uint8_t g_ucUsart1ReceiveData;
static uint8_t g_ucaBaseCmdRxBuffer[BASE_CMD_RX_BUFFER_SIZE];
static uint8_t g_ucaBaseCmdLine[BASE_CMD_RX_BUFFER_SIZE];
static uint8_t g_ucBaseCmdRxIndex = 0U;
static volatile uint8_t g_ucBaseCmdLineReady = 0U;

/* USER CODE END 0 */

UART_HandleTypeDef huart1;

/* USART1 init function */

void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

void HAL_UART_MspInit(UART_HandleTypeDef* uartHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(uartHandle->Instance==USART1)
  {
  /* USER CODE BEGIN USART1_MspInit 0 */

  /* USER CODE END USART1_MspInit 0 */
    /* USART1 clock enable */
    __HAL_RCC_USART1_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**USART1 GPIO Configuration
    PA9     ------> USART1_TX
    PA10     ------> USART1_RX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_10;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* USART1 interrupt Init */
    HAL_NVIC_SetPriority(USART1_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(USART1_IRQn);
  /* USER CODE BEGIN USART1_MspInit 1 */

  /* USER CODE END USART1_MspInit 1 */
  }
}

void HAL_UART_MspDeInit(UART_HandleTypeDef* uartHandle)
{

  if(uartHandle->Instance==USART1)
  {
  /* USER CODE BEGIN USART1_MspDeInit 0 */

  /* USER CODE END USART1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_USART1_CLK_DISABLE();

    /**USART1 GPIO Configuration
    PA9     ------> USART1_TX
    PA10     ------> USART1_RX
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_9|GPIO_PIN_10);

    /* USART1 interrupt Deinit */
    HAL_NVIC_DisableIRQ(USART1_IRQn);
  /* USER CODE BEGIN USART1_MspDeInit 1 */

  /* USER CODE END USART1_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */
/**
  * @brief  开启底盘 USART1 单字节中断接收
  * @note   树莓派通过 USART1 下发命令，接收完成后在 HAL_UART_RxCpltCallback() 中继续接收下一字节。
  * @param  无
  * @retval 无
  */
void BaseCmd_StartReceive(void)
{
  (void)HAL_UART_Receive_IT(&huart1, &g_ucUsart1ReceiveData, 1U);
}

/**
  * @brief  接收树莓派发来的单字节数据
  * @note   本函数只拼接一行完整数据帧，不在中断上下文执行电机动作。
  * @param  data: USART1 收到的 1 个字节
  * @retval 无
  */
void BaseCmd_ReceiveByte(uint8_t data)
{
  uint8_t i;

  if((data == '\n') || (data == '\r'))
  {
    if(g_ucBaseCmdRxIndex == 0U)
    {
      return;
    }

    g_ucaBaseCmdRxBuffer[g_ucBaseCmdRxIndex] = '\0';
    for(i = 0U; i <= g_ucBaseCmdRxIndex; i++)
    {
      g_ucaBaseCmdLine[i] = g_ucaBaseCmdRxBuffer[i];
    }
    g_ucBaseCmdRxIndex = 0U;
    g_ucBaseCmdLineReady = 1U;
    return;
  }

  if(g_ucBaseCmdRxIndex < (BASE_CMD_RX_BUFFER_SIZE - 1U))
  {
    g_ucaBaseCmdRxBuffer[g_ucBaseCmdRxIndex++] = data;
  }
  else
  {
    /* 接收异常或帧过长时丢弃当前半包，等待下一行重新同步。 */
    g_ucBaseCmdRxIndex = 0U;
  }
}

/**
  * @brief  底盘命令处理任务
  * @note   主循环周期调用；把 USART1 收到的完整行交给 base_protocol.c 解析执行。
  * @param  无
  * @retval 无
  */
void BaseCmd_Task(void)
{
  if(g_ucBaseCmdLineReady == 0U)
  {
    return;
  }

  g_ucBaseCmdLineReady = 0U;
  BaseProtocol_HandleLine((const char *)g_ucaBaseCmdLine);
}

/**
  * @brief  UART 接收完成回调函数
  * @note   当前只使用 USART1 连接树莓派，收到 1 字节后立即恢复下一次接收。
  * @param  huart: 触发回调的 UART 句柄
  * @retval 无
  */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if(huart == &huart1)
  {
    BaseCmd_ReceiveByte(g_ucUsart1ReceiveData);
    BaseCmd_StartReceive();
  }
}

/**
  * @brief  UART 错误回调函数
  * @note   USART1 出现溢出等错误时清除标志并恢复接收，避免通信链路卡死。
  * @param  huart: 发生错误的 UART 句柄
  * @retval 无
  */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if(huart == &huart1)
  {
    if(__HAL_UART_GET_FLAG(huart, UART_FLAG_ORE) != RESET)
    {
      __HAL_UART_CLEAR_OREFLAG(huart);
    }
    BaseCmd_StartReceive();
  }
}

/* USER CODE END 1 */
