/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    adc.h
  * @brief   This file contains all the function prototypes for
  *          the adc.c file
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
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __ADC_H__
#define __ADC_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

extern ADC_HandleTypeDef hadc2;

/* USER CODE BEGIN Private defines */
#define BASE_ADC_VREF_MV                 3300UL
#define BASE_ADC_MAX_VALUE               4095UL
#define BASE_ADC_BATTERY_DIVIDER_RATIO   4UL
#define BASE_ADC_BATTERY_SAMPLE_COUNT    8U
#define BASE_ADC_POLL_TIMEOUT_MS         10U
/* USER CODE END Private defines */

void MX_ADC2_Init(void);

/* USER CODE BEGIN Prototypes */
/**
 * @brief  读取底盘电池电压。
 * @param  None
 * @return 电池电压，单位 mV；返回 -1 表示 ADC 读取失败
 */
int32_t BaseAdc_ReadBatteryVoltageMv(void);
/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __ADC_H__ */
