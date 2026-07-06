/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/**
  ******************************************************************************
  * 作者: 小车俱乐部 VCC
  * 联系方式: 1930299709@qq.com
  * 程序版本: V3.3.0
  * 硬件资料版本: V3.3.0
  *
  ******************************************************************************
  * 功能说明:
  * 本文件为小车主控逻辑入口，包含：
  * 1) 外设初始化（电机、编码器、串口、ADC、OLED、MPU6050）
  * 2) 多模式控制（显示、红外循迹、遥控、避障、跟随、航向控制、视觉追踪、双板协同抓取）
  * 3) 传感器采集与 PID 运算
  * 4) 与上位机/APP 及机械臂控制板的数据交互
  *
  *
  ******************************************************************************
  */  
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "oled.h"
#include "stdio.h"
#include "motor.h"
#include "niming.h"
#include "pid.h"

#include "cJSON.h"
#include <string.h>
#include "HC_SR04.h"



#include "mpu6050.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h" 
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
extern float Motor1Speed ;// 电机1当前速度
extern float Motor2Speed ;// 电机2当前速度

extern tPid pidMotor1Speed;// 电机1速度环 PID 参数结构体
extern tPid pidMotor2Speed;
extern tPid pidFollow;    // 超声波跟随 PID
extern tPid pidMPU6050YawMovement;  // MPU6050 航向角闭环控制 PID
extern uint8_t Usart1_ReadBuf[255];	// 串口1接收缓冲区
float p,i,d,a,b;// 通过 JSON 在线调参使用的临时变量
uint8_t OledString[50];// OLED 显示字符串缓冲区
extern float Mileage;// 里程

extern tPid pidVoltage;// 灰度电压循迹 PID 参数

extern tPid pidHW_Tracking;// 红外循迹 PID
extern tPid pidOpenmv_Tracking;// OpenMV 视觉循迹 PID

uint8_t g_ucaHW_Read[4] = {0};// 四路红外传感器原始状态
int8_t g_cThisState = 0;// 当前偏差状态
int8_t g_cLastState = 0; // 上一轮偏差状态
float g_fHW_PID_Out;// 红外循迹 PID 输出
float g_fHW_PID_Out1;// 电机1经 PID 修正后的目标速度
float g_fHW_PID_Out2;// 电机2经 PID 修正后的目标速度

uint8_t g_ucUsart3ReceiveData;  // 串口3中断单字节接收缓存
uint8_t g_ucUsart2ReceiveData;  // 串口2中断单字节接收缓存

uint8_t Usart3String[50];// 串口3发送字符串缓冲区
float g_fHC_SR04_Read;// 超声波测距当前读数(cm)
float g_fFollow_PID_Out;// 跟随模式 PID 输出值


float pitch,roll,yaw; // 姿态角：俯仰、横滚、航向

float  g_fMPU6050YawMovePidOut = 0.00f; // 航向控制 PID 原始输出
float  g_fMPU6050YawMovePidOut1 = 0.00f; // 航向控制下电机1目标速度
float  g_fMPU6050YawMovePidOut2 = 0.00f; // 航向控制下电机2目标速度

uint8_t g_ucMode = 0; 

float g_fVoltage[4];// 4路灰度传感器 ADC 电压值（PA5、PA7、PB0、PB1）

float g_fVoltageMax[4]={2.89,2.89,2.89,2.89};// 各通道归一化参考最大电压（标定值）
int   g_iVoltageGuiYi[4];// 归一化结果，范围 0~100

float g_fVoltageOuter;// 外侧差分误差（4号与1号）
float g_fVoltageInterior;// 内侧差分误差（3号与2号）

float g_fVoltagePidOut;// 灰度循迹 PID 输出

extern int g_lHW_State;// 视觉状态字（由 OpenMV/外部通信更新）

// Mode 0: display, 1: IR track, 2: remote drive, 3: obstacle avoid, 4: follow, 5: yaw PID, 6: camera track, 7: dual-STM32 auto grab
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// 双STM32自动抓取状态机
// 这部分是我新增的核心逻辑，建议你先从这里开始看。
// 整个流程是：搜索目标 -> 对准目标 -> 接近目标 -> 通知机械臂抓取 -> 后退。
// 当 g_ucMode == 7 时，小车就运行这里的代码。
typedef enum
{
	DUAL_SM_SEARCH = 0,      // 搜索目标：原地慢慢转，等待摄像头重新看到目标
	DUAL_SM_ALIGN,           // 对准目标：根据视觉偏差调整左右轮速度
	DUAL_SM_APPROACH,        // 接近目标：向前走，同时继续修正方向
	DUAL_SM_WAIT_ARM,        // 等机械臂完成动作：此时小车保持静止
	DUAL_SM_BACKUP,          // 后退离开抓取点：给后续动作留空间
	DUAL_SM_DONE             // 流程结束：小车保持停止
} tDualSmState;

// 当前状态，初始化为“搜索目标”
static tDualSmState g_tDualSmState = DUAL_SM_SEARCH;
// 记录某个状态开始时的时间，用于后面做超时判断
static uint32_t g_ulDualSmTick = 0;
// 记录“已经稳定对准目标”开始的时间
static uint32_t g_ulAlignStableTick = 0;
// 记录上一轮的模式值，用于判断是否刚刚离开 mode 7
static uint8_t g_ucLastMode = 0xFF;

// 对速度做限幅，避免电机速度设置过大
static float dualClamp(float value, float minVal, float maxVal)
{
	// 小于最小值时，直接返回最小值
	if(value < minVal) return minVal;
	// 大于最大值时，直接返回最大值
	if(value > maxVal) return maxVal;
	// 在正常范围内时，原样返回
	return value;
}

// 按机械臂板已有协议发送命令：运行动作组
static void Arm_SendFullActionRun(uint8_t actionNum, uint16_t times)
{
	// 这里准备一个7字节的数据包
	uint8_t packet[7];
	// 第1个帧头字节
	packet[0] = 0x55;
	// 第2个帧头字节
	packet[1] = 0x55;
	// 长度字节，表示后面命令和参数的有效长度
	packet[2] = 0x05;
	// 命令字 0x06：表示“运行整套动作组”
	packet[3] = 0x06;
	// 动作组编号，例如后面调用时会传 100
	packet[4] = actionNum;
	// 运行次数低字节
	packet[5] = (uint8_t)(times & 0xFFu);
	// 运行次数高字节
	packet[6] = (uint8_t)((times >> 8) & 0xFFu);
	// 通过 USART1 把命令发给机械臂板
	HAL_UART_Transmit(&huart1, packet, sizeof(packet), 100);
}

// 告诉机械臂板停止当前动作组
static void Arm_SendFullActionStop(void)
{
	// 这里准备一个4字节的停止命令
	uint8_t packet[4];
	// 第1个帧头字节
	packet[0] = 0x55;
	// 第2个帧头字节
	packet[1] = 0x55;
	// 长度字节
	packet[2] = 0x02;
	// 命令字 0x07：表示“停止动作组”
	packet[3] = 0x07;
	// 通过 USART1 把停止命令发给机械臂板
	HAL_UART_Transmit(&huart1, packet, sizeof(packet), 100);
}

// 把状态机恢复到初始状态：重新开始搜索目标
static void DualSm_Reset(void)
{
	// 把状态重新设为“搜索目标”
	g_tDualSmState = DUAL_SM_SEARCH;
	// 记录当前时间，作为状态开始时间
	g_ulDualSmTick = HAL_GetTick();
	// 清零“稳定对准计时器”
	g_ulAlignStableTick = 0;
	// 让小车先停下，避免切换状态时乱动
	motorPidSetSpeed(0,0);
}

// mode 7 的主流程函数：自动寻找、对准、接近并触发抓取
static void DualSm_Task(void)
{
	// 读取当前系统时间，单位是毫秒
	uint32_t now = HAL_GetTick();
	// turn 用来保存 PID 算出来的转向修正量
	float turn;
	// left 用来保存左轮目标速度
	float left;
	// right 用来保存右轮目标速度
	float right;

	// 根据当前状态，执行不同阶段的动作
	switch(g_tDualSmState)
	{
		case DUAL_SM_SEARCH:
			// 搜索阶段：如果摄像头没看到目标，小车就原地慢慢转动继续找
			if(usartCamera_HasRecentFrame(400) == 0)
			{
				// 左轮前进、右轮后退，让小车原地旋转
				motorPidSetSpeed(0.45f,-0.45f);
			}
			else
			{
				// 一旦重新看到目标，就切换到“对准目标”阶段
				g_tDualSmState = DUAL_SM_ALIGN;
				// 记录切状态时的时间
				g_ulDualSmTick = now;
				// 清空稳定计时器，重新开始计算
				g_ulAlignStableTick = 0;
			}
			break;

		case DUAL_SM_ALIGN:
			// 对准阶段：根据视觉偏差，让目标尽量出现在画面中间
			if(usartCamera_HasRecentFrame(400) == 0)
			{
				// 如果对准过程中目标丢了，就退回搜索状态
				g_tDualSmState = DUAL_SM_SEARCH;
				// 更新时间戳
				g_ulDualSmTick = now;
				// 本轮直接结束
				break;
			}

			// 用视觉偏差做 PID 运算，得到转向修正量
			turn = PID_realize(&pidOpenmv_Tracking, (float)g_cThisState);
			// 左轮速度 = 基础速度 + 修正量
			left = dualClamp(0.45f + turn, 0.0f, 1.2f);
			// 右轮速度 = 基础速度 - 修正量
			right = dualClamp(0.45f - turn, 0.0f, 1.2f);
			// 把新的左右轮速度发给电机
			motorPidSetSpeed(left,right);

			// 当偏差已经比较小，说明目标基本在小车前方中间
			if((g_cThisState >= -1) && (g_cThisState <= 1))
			{
				// 如果这是第一次进入“稳定对准”状态，就记下开始时间
				if(g_ulAlignStableTick == 0)
				{
					g_ulAlignStableTick = now;
				}
				// 如果已经连续稳定 350ms，就认为已经对准成功
				else if((now - g_ulAlignStableTick) >= 350)
				{
					// 切换到接近目标阶段
					g_tDualSmState = DUAL_SM_APPROACH;
					// 更新状态开始时间
					g_ulDualSmTick = now;
				}
			}
			else
			{
				// 只要偏差又变大，就把稳定计时清零重新来
				g_ulAlignStableTick = 0;
			}
			break;

		case DUAL_SM_APPROACH:
			// 接近阶段：向前移动，同时继续根据视觉做方向修正
			if(usartCamera_HasRecentFrame(400) == 0)
			{
				// 如果接近过程中目标丢失，也退回搜索状态
				g_tDualSmState = DUAL_SM_SEARCH;
				g_ulDualSmTick = now;
				break;
			}

			// 接近时也继续做 PID 修正，避免小车冲偏
			turn = PID_realize(&pidOpenmv_Tracking, (float)g_cThisState);
			// 接近阶段基础速度更大一些，所以这里是 0.90f
			left = dualClamp(0.90f + turn, 0.0f, 1.8f);
			// 右轮速度同理
			right = dualClamp(0.90f - turn, 0.0f, 1.8f);
			// 按新的左右轮速度继续前进
			motorPidSetSpeed(left,right);

			// 如果超声波距离已经小于等于 18cm，说明到抓取位置附近了
			if(HC_SR04_Read() <= 18.0f)
			{
				// 先让小车停车，避免机械臂抓取时车还在动
				motorPidSetSpeed(0,0);
				// 通知机械臂板运行动作组 100，执行 1 次
				Arm_SendFullActionRun(100,1);
				// 切换到“等待机械臂完成”阶段
				g_tDualSmState = DUAL_SM_WAIT_ARM;
				// 记录开始等待的时间
				g_ulDualSmTick = now;
			}
			break;

		case DUAL_SM_WAIT_ARM:
			// 等待机械臂动作完成阶段：小车保持不动
			motorPidSetSpeed(0,0);
			// 等待 4500ms，默认机械臂已经执行完动作组
			if((now - g_ulDualSmTick) >= 4500)
			{
				// 进入后退阶段
				g_tDualSmState = DUAL_SM_BACKUP;
				// 记录后退开始时间
				g_ulDualSmTick = now;
			}
			break;

		case DUAL_SM_BACKUP:
			// 后退阶段：离开抓取点，防止挡住后续动作
			motorPidSetSpeed(-0.8f,-0.8f);
			// 后退 800ms 后停止
			if((now - g_ulDualSmTick) >= 800)
			{
				// 先停车
				motorPidSetSpeed(0,0);
				// 把状态设为完成
				g_tDualSmState = DUAL_SM_DONE;
				// 更新时间戳
				g_ulDualSmTick = now;
			}
			break;

		case DUAL_SM_DONE:
			// 完成阶段：保持停车，等待你切换模式或重新触发
			motorPidSetSpeed(0,0);
			break;

		default:
			// 如果状态异常，就直接复位，避免程序跑飞
			DualSm_Reset();
			break;
	}
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART1_UART_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM4_Init();
  MX_ADC2_Init();
  MX_USART3_UART_Init();
  MX_USART2_UART_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */
  OLED_Init();			// 初始化 OLED
  OLED_Clear()  	; 
  HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);// 启动 TIM1 CH1 PWM 输出
  HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_4);// 启动 TIM1 CH4 PWM 输出
  HAL_TIM_Encoder_Start(&htim2,TIM_CHANNEL_ALL);// 启动 TIM2 编码器接口
  HAL_TIM_Encoder_Start(&htim4,TIM_CHANNEL_ALL);// 启动 TIM4 编码器接口
  HAL_TIM_Base_Start_IT(&htim2);				// 开启 TIM2 基本定时中断
  HAL_TIM_Base_Start_IT(&htim4);                // 开启 TIM4 基本定时中断
  
  HAL_TIM_Base_Start_IT(&htim1);                // 开启 TIM1 基本定时中断
  __HAL_UART_ENABLE_IT(&huart1,UART_IT_RXNE);	// 使能串口1接收中断
  PID_init();// 初始化全部 PID 参数
  DualSm_Reset();// Reset mode-7 state machine at startup
  HAL_UART_Receive_IT(&huart3,&g_ucUsart3ReceiveData,1);  // 开启串口3中断接收
  
  HAL_UART_Receive_IT(&huart2,&g_ucUsart2ReceiveData,1);  // 开启串口2中断接收
		
  HAL_Delay(500);// 延时0.5秒，等待 MPU6050 上电稳定
  
  MPU_Init(); // 初始化 MPU6050
  
  __HAL_UART_ENABLE_IT(&huart2, UART_IT_ERR);// 使能 UART2 错误中断

//  while(MPU_Init()!=0);// 如需阻塞式初始化，可等待 MPU6050 初始化成功
//  while(mpu_dmp_init()!=0);// 如需 DMP 功能，可在此处初始化

//  cJSON *cJsonData ,*cJsonVlaue;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	sprintf((char *)OledString," g_ucMode:%d",g_ucMode);// 在 OLED 上显示当前模式
	OLED_ShowString(0,6,OledString,12);	// OLED 第6行显示模式号
	
	sprintf((char *)Usart3String," g_ucMode:%d",g_ucMode);// 通过串口3回传当前模式给 APP
	HAL_UART_Transmit(&huart3,( uint8_t *)Usart3String,strlen(( const  char  *)Usart3String),50);// 发送字符串，长度由 strlen 计算

	// 如果上一轮还是 mode 7，而这一轮已经不是 mode 7，
	// 说明用户刚刚退出了自动抓取模式
	if((g_ucLastMode == 7) && (g_ucMode != 7))
	{
		// 先通知机械臂板停止动作
		Arm_SendFullActionStop();
		// 再把小车这边的状态机恢复到初始状态
		DualSm_Reset();
	}
	// 保存当前模式值，供下一轮循环判断是否发生模式切换
	g_ucLastMode = g_ucMode;

	// mode 7：双STM32自动抓取模式
	// 进入这个模式后，直接执行状态机，跳过下面旧的演示模式代码
	if(g_ucMode == 7)
	{
		// 调用一次状态机，让自动抓取流程往前推进一步
		DualSm_Task();
		// 稍微延时一下，避免主循环跑得太快
		HAL_Delay(10);
		// 直接进入下一轮 while(1)
		continue;
	}

		
	
	// 采集4路灰度传感器 ADC 电压值
	for(int i=0;i<4;i++)
	{
		HAL_ADC_Start(&hadc1);
		HAL_ADC_PollForConversion(&hadc1,50);
		g_fVoltage[i]=(float)HAL_ADC_GetValue(&hadc1)/4096*3.300; // 12位ADC: 0~4095，换算成 0~3.3V
	}  
	
    /** 灰度值归一化 **/  
	for(int i=0;i<4;i++)
	{
		if(g_fVoltage[i] > g_fVoltageMax[i]) g_fVoltage[i] = g_fVoltageMax[i];// 超过标定上限时截断
		g_iVoltageGuiYi[i] = g_fVoltage[i]/g_fVoltageMax[i]*100;// 映射到 0~100
	}
	
	/* 计算差分误差：(a-b)/(a+b)，结果更稳定且与亮度绝对值弱相关 */
	/* 分母 +1 是防止 a+b=0 导致除0异常 */
	g_fVoltageOuter = (float)((g_iVoltageGuiYi[3]-g_iVoltageGuiYi[0])/(float)(g_iVoltageGuiYi[3]+g_iVoltageGuiYi[0] +1));// 外侧误差（4-1）
	g_fVoltageInterior = (float)((g_iVoltageGuiYi[2]-g_iVoltageGuiYi[1])/(float)(g_iVoltageGuiYi[2]+g_iVoltageGuiYi[1] +1));// 内侧误差（3-2）

	
//	/* 阈值法循迹（旧方案，保留参考）*/
//	if(0.75 > g_fVoltageOuter > 0.5)
//	{
//		motorPidSetSpeed(1,0.8);// 偏左时，左轮稍快右轮稍慢
//	}
//	else if(0.75 <= g_fVoltageOuter)// 偏左较大
//	{
//		motorPidSetSpeed(1.2,0.2);// 偏左严重时加大差速
//	}
//	else if(-0.75 < g_fVoltageOuter < -0.5)
//	{
//		motorPidSetSpeed(0.8,1);// 偏右时，右轮稍快左轮稍慢
//	}
//	else if( -0.75 >= g_fVoltageOuter)// 偏右较大
//	{
//		motorPidSetSpeed(0.2,1.2);// 偏右严重时加大差速
//	}
//	else{		
//		motorPidSetSpeed(1,1);// 居中直行	
//	}
//		
	
	g_fVoltagePidOut = PID_realize(&pidVoltage,g_fVoltageOuter);// 灰度误差经 PID 输出转向修正量

	g_fHW_PID_Out1 = 1.5 + g_fVoltagePidOut;// 左轮速度=基础速度+转向修正
	g_fHW_PID_Out2 = 1.5 - g_fVoltagePidOut;// 右轮速度=基础速度-转向修正
	if(g_fHW_PID_Out1 >5) g_fHW_PID_Out1 =5;// 输出限幅到 0~5
	if(g_fHW_PID_Out1 <0) g_fHW_PID_Out1 =0;
	if(g_fHW_PID_Out2 >5) g_fHW_PID_Out2 =5;// 输出限幅到 0~5
	if(g_fHW_PID_Out2 <0) g_fHW_PID_Out2 =0;
	
	motorPidSetSpeed(g_fHW_PID_Out1,g_fHW_PID_Out2);// 下发左右轮目标速度
	
	
	sprintf((char*)OledString, "O:%.2f  I:%.2f  ", g_fVoltageOuter,g_fVoltageInterior);// O:外侧误差 I:内侧误差
	OLED_ShowString(0,1,OledString,12);// 在 OLED 上显示误差
	
	sprintf((char*)OledString, "G1:%d  G2:%d   ", g_iVoltageGuiYi[0],g_iVoltageGuiYi[1]);// 显示归一化灰度1/2
	OLED_ShowString(0,2,OledString,12);// OLED 第2行显示
		
	sprintf((char *)OledString,"G3:%d  G4:%d   ",g_iVoltageGuiYi[2],g_iVoltageGuiYi[3]);// 显示归一化灰度3/4
	OLED_ShowString(0,3,OledString,12);// OLED 第3行显示
	
	sprintf((char *)OledString,"v1:%.2f v2:%.2f ",g_fVoltage[0],g_fVoltage[1]);// 显示电压1/2
	OLED_ShowString(0,4,OledString,12);// OLED 第4行显示
	
	sprintf((char *)OledString,"v3:%.2f v4:%.2f ",g_fVoltage[2],g_fVoltage[3]);// 显示电压3/4
	OLED_ShowString(0,5,OledString,12);// OLED 第5行显示
	
	
	sprintf((char *)OledString,"v1:%.2f v2:%.2f \r\n",g_fVoltage[0],g_fVoltage[1]);// 兼容旧格式字符串（含换行）
	OLED_ShowString(0,4,OledString,12);// OLED 刷新
	
	sprintf((char *)OledString,"v3:%.2f v4:%.2f \r\n",g_fVoltage[2],g_fVoltage[3]);// 兼容旧格式字符串（含换行）
	OLED_ShowString(0,5,OledString,12);// OLED 刷新  
	  
	
	  
	if(g_ucMode == 0)
	{
	// 模式0：信息显示/空闲模式
		sprintf((char*)OledString, "V1:%.2fV2:%.2f", Motor1Speed,Motor2Speed);// 显示双电机实时速度
		OLED_ShowString(0,0,OledString,12);// OLED 第0行显示
		
//		sprintf((char*)OledString, "Mileage:%.2f", Mileage);// 显示里程
//		OLED_ShowString(0,1,OledString,12);// OLED 第1行显示
//		
//		sprintf((char*)OledString, "U:%.2fV", adcGetBatteryVoltage());// 显示电池电压
//		OLED_ShowString(0,2,OledString,12);// OLED 第2行显示
//		
//		sprintf((char *)OledString,"HC_SR04:%.2fcm\r\n",HC_SR04_Read());// 显示超声波距离
//		OLED_ShowString(0,3,OledString,12);// OLED 第3行显示
		
//		sprintf((char *)OledString,"p:%.2f r:%.2f \r\n",pitch,roll);// 显示俯仰/横滚角
//		OLED_ShowString(0,4,OledString,12);// OLED 第4行显示
//		
//		sprintf((char *)OledString,"y:%.2f  \r\n",yaw);// 显示航向角
//		OLED_ShowString(0,5,OledString,12);// OLED 第5行显示
		
	// 向 APP 连续回传状态信息
		sprintf((char*)Usart3String, "V1:%.2fV2:%.2f", Motor1Speed,Motor2Speed);// 电机速度
		HAL_UART_Transmit(&huart3,( uint8_t *)Usart3String,strlen(( const  char  *)Usart3String),50);// 发送字符串，长度由 strlen 计算
		// 多包发送：速度、里程、电池电压、超声波、姿态角
		sprintf((char*)Usart3String, "Mileage:%.2f", Mileage);// 里程
		HAL_UART_Transmit(&huart3,( uint8_t *)Usart3String,strlen(( const  char  *)Usart3String),50);// 发送字符串，长度由 strlen 计算
		
		sprintf((char*)Usart3String, "U:%.2fV", adcGetBatteryVoltage());// 电池电压
		HAL_UART_Transmit(&huart3,( uint8_t *)Usart3String,strlen(( const  char  *)Usart3String),50);// 发送字符串，长度由 strlen 计算
		
		sprintf((char *)Usart3String,"HC_SR04:%.2fcm\r\n",HC_SR04_Read());// 超声波距离
		HAL_UART_Transmit(&huart3,( uint8_t *)Usart3String,strlen(( const  char  *)Usart3String),50);// 发送字符串，长度由 strlen 计算
		
		sprintf((char *)Usart3String,"p:%.2f r:%.2f \r\n",pitch,roll);// MPU6050 俯仰角/横滚角
		HAL_UART_Transmit(&huart3,( uint8_t *)Usart3String,strlen(( const  char  *)Usart3String),50);// 发送字符串，长度由 strlen 计算
		
		sprintf((char *)Usart3String,"y:%.2f  \r\n",yaw);// MPU6050 航向角
		HAL_UART_Transmit(&huart3,( uint8_t *)Usart3String,strlen(( const  char  *)Usart3String),50);// 发送字符串，长度由 strlen 计算
	
		// 读取 MPU6050 姿态（如需实时刷新可打开下面代码）
//		while(mpu_dmp_get_data(&pitch,&roll,&yaw)!=0){}  // 阻塞等待直到读到有效姿态数据
		
		// 显示模式下默认停车，目标速度置 0
		motorPidSetSpeed(0,0);
	}
	if(g_ucMode == 1)
	{
	///**** 红外循迹 PID 模式 ******************/
//	g_ucaHW_Read[0] = READ_HW_OUT_1;// 建议在中断或固定周期采样后再参与 if 判断
//	g_ucaHW_Read[1] = READ_HW_OUT_2;
//	g_ucaHW_Read[2] = READ_HW_OUT_3;
//	g_ucaHW_Read[3] = READ_HW_OUT_4;

	if(g_ucaHW_Read[0] == 0&&g_ucaHW_Read[1] == 0&&g_ucaHW_Read[2] == 0&&g_ucaHW_Read[3] == 0 )
	{
//		printf("居中\r\n");// 可用于串口调试当前红外状态
		g_cThisState = 0;// 居中
	}
	else if(g_ucaHW_Read[0] == 0&&g_ucaHW_Read[1] == 1&&g_ucaHW_Read[2] == 0&&g_ucaHW_Read[3] == 0 )// 注意必须写 else if
	{
//		printf("偏左1\r\n");
		g_cThisState = -1;// 轻微偏左
	}
	else if(g_ucaHW_Read[0] == 1&&g_ucaHW_Read[1] == 0&&g_ucaHW_Read[2] == 0&&g_ucaHW_Read[3] == 0 )
	{
//		printf("偏左2\r\n");
		g_cThisState = -2;// 中度偏左
	}
	else if(g_ucaHW_Read[0] == 1&&g_ucaHW_Read[1] == 1&&g_ucaHW_Read[2] == 0&&g_ucaHW_Read[3] == 0)
	{
//		printf("偏左3\r\n");
		g_cThisState = -3;// 严重偏左
	}
	else if(g_ucaHW_Read[0] == 0&&g_ucaHW_Read[1] == 0&&g_ucaHW_Read[2] == 1&&g_ucaHW_Read[3] == 0 )
	{
//		printf("偏右1\r\n");
		g_cThisState = 1;// 轻微偏右	
	}
	else if(g_ucaHW_Read[0] == 0&&g_ucaHW_Read[1] == 0&&g_ucaHW_Read[2] == 0&&g_ucaHW_Read[3] == 1 )
	{
//		printf("偏右2\r\n");
		g_cThisState = 2;// 中度偏右
	}
	else if(g_ucaHW_Read[0] == 0&&g_ucaHW_Read[1] == 0&&g_ucaHW_Read[2] == 1&&g_ucaHW_Read[3] == 1)
	{
//	    printf("偏右3\r\n");
		g_cThisState = 3;// 严重偏右
	}
	g_fHW_PID_Out = PID_realize(&pidHW_Tracking,g_cThisState);// 将偏差输入 PID，得到转向修正量

	g_fHW_PID_Out1 = 3 + g_fHW_PID_Out;// 左轮速度=基础速度+修正
	g_fHW_PID_Out2 = 3 - g_fHW_PID_Out;// 右轮速度=基础速度-修正
	if(g_fHW_PID_Out1 >5) g_fHW_PID_Out1 =5;// 限幅 0~5
	if(g_fHW_PID_Out1 <0) g_fHW_PID_Out1 =0;
	if(g_fHW_PID_Out2 >5) g_fHW_PID_Out2 =5;// 限幅 0~5
	if(g_fHW_PID_Out2 <0) g_fHW_PID_Out2 =0;
	if(g_cThisState != g_cLastState)// 状态变化时再更新速度，减少抖动
	{
		motorPidSetSpeed(g_fHW_PID_Out1,g_fHW_PID_Out2);// 下发速度
	}
	
	g_cLastState = g_cThisState;// 保存本轮状态	

	}
	if(g_ucMode == 2)
	{
		
		//*************** 遥控模式 ***********************//
		// 具体逻辑在串口/按键回调里更新目标速度
	}
	if(g_ucMode == 3)
	{
		//****** 超声波避障模式 *********************//
//// 前方无障碍则前进
		if(HC_SR04_Read() > 25)// 距离大于25cm
		{
			motorPidSetSpeed(1,1);// 直行
			HAL_Delay(100);
		}
		else{	// 前方有障碍
			motorPidSetSpeed(-1,1);// 左转避障	
			HAL_Delay(500);
			if(HC_SR04_Read() > 25)// 转向后距离恢复
			{
				motorPidSetSpeed(1,1);// 继续前进
				HAL_Delay(100);
			}
			else{// 仍有障碍
				motorPidSetSpeed(1,-1);// 右转尝试绕行
				HAL_Delay(1000);
				if(HC_SR04_Read() >25)// 绕行后距离恢复
				{
					 motorPidSetSpeed(1,1);// 继续前进
					HAL_Delay(100);
				}
				else{
					motorPidSetSpeed(-1,-1);// 后退
					HAL_Delay(1000);
					motorPidSetSpeed(-1,1);// 再次左转
					HAL_Delay(50);
				}
			}
		}
	}
	if(g_ucMode == 4)
	{
	//********** 跟随 PID 模式 ***********//
		g_fHC_SR04_Read=HC_SR04_Read();// 读取目标距离
		if(g_fHC_SR04_Read < 60){  // 仅在60cm内开启跟随
			g_fFollow_PID_Out = PID_realize(&pidFollow,g_fHC_SR04_Read);// 距离误差经 PID 输出速度
			if(g_fFollow_PID_Out > 6) g_fFollow_PID_Out = 6;// 输出限幅
			if(g_fFollow_PID_Out < -6) g_fFollow_PID_Out = -6;
			motorPidSetSpeed(g_fFollow_PID_Out,g_fFollow_PID_Out);// 双轮同速前进/后退
		}
		else motorPidSetSpeed(0,0);// 超过60cm则停车
		HAL_Delay(10);// 适当降低控制频率
	}
	if(g_ucMode == 5)
	{
	//************* MPU6050 航向角 PID 模式 *****************//

		sprintf((char *)Usart3String,"pitch:%.2f roll:%.2f yaw:%.2f\r\n",pitch,roll,yaw);// 回传姿态角
		HAL_UART_Transmit(&huart3,( uint8_t *)Usart3String,strlen(( const  char  *)Usart3String),0xFFFF);// 串口发送，长度由 strlen 计算	
	   
//	    mpu_dmp_get_data(&pitch,&roll,&yaw);// 返回0表示成功读取到DMP姿态
//		while(mpu_dmp_get_data(&pitch,&roll,&yaw)!=0){}  // 轮询直到拿到有效数据
		
		
		g_fMPU6050YawMovePidOut = PID_realize(&pidMPU6050YawMovement,yaw);// 航向角输入 PID，输出转向修正

		g_fMPU6050YawMovePidOut1 = 1.5 + g_fMPU6050YawMovePidOut;// 左轮速度=基础速度+修正
		g_fMPU6050YawMovePidOut2 = 1.5 - g_fMPU6050YawMovePidOut;
		if(g_fMPU6050YawMovePidOut1 >3.5) g_fMPU6050YawMovePidOut1 =3.5;// 限幅
		if(g_fMPU6050YawMovePidOut1 <0) g_fMPU6050YawMovePidOut1 =0;
		if(g_fMPU6050YawMovePidOut2 >3.5) g_fMPU6050YawMovePidOut2 =3.5;// 限幅
		if(g_fMPU6050YawMovePidOut2 <0) g_fMPU6050YawMovePidOut2 =0;
		motorPidSetSpeed(g_fMPU6050YawMovePidOut1,g_fMPU6050YawMovePidOut2);// 输出到双电机闭环
	
	}
	if(g_ucMode == 6)
	{

		sprintf((char*)OledString, "lHW:%d  ", g_lHW_State);// 显示视觉状态字
		OLED_ShowString(0,0,OledString,12);// OLED 第0行显示
		
		g_fHW_PID_Out = PID_realize(&pidOpenmv_Tracking,g_cThisState);// 视觉偏差输入 PID，得到转向修正

		g_fHW_PID_Out1 = 0.5 + g_fHW_PID_Out;// 左轮速度=基础速度+修正
		g_fHW_PID_Out2 = 0.5 - g_fHW_PID_Out;// 右轮速度=基础速度-修正
		if(g_fHW_PID_Out1 >1.2) g_fHW_PID_Out1 =1.2;// 限幅 0~1.2
		if(g_fHW_PID_Out1 <0) g_fHW_PID_Out1 =0;
		if(g_fHW_PID_Out2 >1.2) g_fHW_PID_Out2 =1.2;// 限幅 0~1.2
		if(g_fHW_PID_Out2 <0) g_fHW_PID_Out2 =0;
		if(g_cThisState != g_cLastState)// 状态变化时再更新速度，减少抖动
		{
			motorPidSetSpeed(g_fHW_PID_Out1,g_fHW_PID_Out2);// 下发速度
		}
		
		g_cLastState = g_cThisState;// 保存当前状态	

	}
	
   }
	
	
///****    ??PID????******************/
//	g_ucaHW_Read[0] = READ_HW_OUT_1;//????????????????if?????
//	g_ucaHW_Read[1] = READ_HW_OUT_2;
//	g_ucaHW_Read[2] = READ_HW_OUT_3;
//	g_ucaHW_Read[3] = READ_HW_OUT_4;

//	if(g_ucaHW_Read[0] == 0&&g_ucaHW_Read[1] == 0&&g_ucaHW_Read[2] == 0&&g_ucaHW_Read[3] == 0 )
//	{
////		printf("????\r\n");//?????????????????
//		g_cThisState = 0;//??
//	}
//	else if(g_ucaHW_Read[0] == 0&&g_ucaHW_Read[1] == 1&&g_ucaHW_Read[2] == 0&&g_ucaHW_Read[3] == 0 )//??else if??????
//	{
////		printf("????\r\n");
//		g_cThisState = -1;//????
//	}
//	else if(g_ucaHW_Read[0] == 1&&g_ucaHW_Read[1] == 0&&g_ucaHW_Read[2] == 0&&g_ucaHW_Read[3] == 0 )
//	{
////		printf("????\r\n");
//		g_cThisState = -2;//????
//	}
//	else if(g_ucaHW_Read[0] == 1&&g_ucaHW_Read[1] == 1&&g_ucaHW_Read[2] == 0&&g_ucaHW_Read[3] == 0)
//	{
////		printf("????\r\n");
//		g_cThisState = -3;//????
//	}
//	else if(g_ucaHW_Read[0] == 0&&g_ucaHW_Read[1] == 0&&g_ucaHW_Read[2] == 1&&g_ucaHW_Read[3] == 0 )
//	{
////		printf("????\r\n");
//		g_cThisState = 1;//????	
//	}
//	else if(g_ucaHW_Read[0] == 0&&g_ucaHW_Read[1] == 0&&g_ucaHW_Read[2] == 0&&g_ucaHW_Read[3] == 1 )
//	{
////		printf("????\r\n");
//		g_cThisState = 2;//????
//	}
//	else if(g_ucaHW_Read[0] == 0&&g_ucaHW_Read[1] == 0&&g_ucaHW_Read[2] == 1&&g_ucaHW_Read[3] == 1)
//	{
////	    printf("????\r\n");
//		g_cThisState = 3;//????
//	}
//	g_fHW_PID_Out = PID_realize(&pidHW_Tracking,g_cThisState);//PID???????? ?????????????

//	g_fHW_PID_Out1 = 3 + g_fHW_PID_Out;//??1??=????+??PID????
//	g_fHW_PID_Out2 = 3 - g_fHW_PID_Out;//??1??=????-??PID????
//	if(g_fHW_PID_Out1 >5) g_fHW_PID_Out1 =5;//???? ?????0-5??
//	if(g_fHW_PID_Out1 <0) g_fHW_PID_Out1 =0;
//	if(g_fHW_PID_Out2 >5) g_fHW_PID_Out2 =5;
//	if(g_fHW_PID_Out2 <0) g_fHW_PID_Out2 =0;
//	if(g_cThisState != g_cLastState)//??????????????????????????????????????????
//	{
//		motorPidSetSpeed(g_fHW_PID_Out1,g_fHW_PID_Out2);//???????????
//	}
//	
//	g_cLastState = g_cThisState;//??????????	



////?????(??)????
////***************?????****************************//
//	sprintf((char *)Usart3String,"V1:%.2fV2:%.2f\r\n",Motor1Speed,Motor2Speed);//???????? ????/?
//	HAL_UART_Transmit(&huart3,( uint8_t *)Usart3String,strlen(( const  char  *)Usart3String),50);//?????????????? strlen:???????
//	
//	sprintf((char *)Usart3String,"Mileage%.2f\r\n",Mileage);//?????? ??cm
//	HAL_UART_Transmit(&huart3,( uint8_t *)Usart3String,strlen(( const  char  *)Usart3String),50);//?????????????? strlen:???????
//	
//	sprintf((char *)Usart3String,"U:%.2fV\r\n",adcGetBatteryVoltage());//??????
//	HAL_UART_Transmit(&huart3,( uint8_t *)Usart3String,strlen(( const  char  *)Usart3String),50);//?????????????? strlen:???????	
//	
//	sprintf((char *)Usart3String,"HC_SR04:%.2fcm\r\n",HC_SR04_Read());//???????
//	HAL_UART_Transmit(&huart3,( uint8_t *)Usart3String,strlen(( const  char  *)Usart3String),0xFFFF);//????????? strlen:???????	
//	
//   	sprintf((char *)Usart3String,"pitch:%.2f roll:%.2f yaw:%.2f\r\n",pitch,roll,yaw);//??6050?? ??? ??? ???
//	HAL_UART_Transmit(&huart3,( uint8_t *)Usart3String,strlen(( const  char  *)Usart3String),0xFFFF);//????????? strlen:???????	
//   
//   //mpu_dmp_get_data(&pitch,&roll,&yaw);//???:0,DMP???????
//    while(mpu_dmp_get_data(&pitch,&roll,&yaw)!=0){}  //????????????????
//	
//	
//	HAL_Delay(5);//???????????HC_SR04_Read()

////*************MPU6050??? PID????*****************//

//   	sprintf((char *)Usart3String,"pitch:%.2f roll:%.2f yaw:%.2f\r\n",pitch,roll,yaw);//??6050?? ??? ??? ???
//	HAL_UART_Transmit(&huart3,( uint8_t *)Usart3String,strlen(( const  char  *)Usart3String),0xFFFF);//????????? strlen:???????	
//   
//   //mpu_dmp_get_data(&pitch,&roll,&yaw);//???:0,DMP???????
//    while(mpu_dmp_get_data(&pitch,&roll,&yaw)!=0){}  //????????????????
//	
//	
//	g_fMPU6050YawMovePidOut = PID_realize(&pidMPU6050YawMovement,yaw);//PID???????? ?????????????

//	g_fMPU6050YawMovePidOut1 = 1.5 + g_fMPU6050YawMovePidOut;//??????PID????
//	g_fMPU6050YawMovePidOut2 = 1.5 - g_fMPU6050YawMovePidOut;
//	if(g_fMPU6050YawMovePidOut1 >3.5) g_fMPU6050YawMovePidOut1 =3.5;//????
//	if(g_fMPU6050YawMovePidOut1 <0) g_fMPU6050YawMovePidOut1 =0;
//	if(g_fMPU6050YawMovePidOut2 >3.5) g_fMPU6050YawMovePidOut2 =3.5;
//	if(g_fMPU6050YawMovePidOut2 <0) g_fMPU6050YawMovePidOut2 =0;
//	motorPidSetSpeed(g_fMPU6050YawMovePidOut1,g_fMPU6050YawMovePidOut2);


////**************????********************//
////????
//	if(HC_SR04_Read() > 25)//??????
//	{
//		motorPidSetSpeed(1,1);//???
//		HAL_Delay(100);
//	}
//	else{	//??????
//		motorPidSetSpeed(-1,1);//???? ??	
//		HAL_Delay(500);
//		if(HC_SR04_Read() > 25)//??????
//		{
//			motorPidSetSpeed(1,1);//???
//			HAL_Delay(100);
//		}
//		else{//??????
//			motorPidSetSpeed(1,-1);//???? ??
//			HAL_Delay(1000);
//			if(HC_SR04_Read() >25)//??????
//			{
//				 motorPidSetSpeed(1,1);//???
//				HAL_Delay(100);
//			}
//			else{
//				motorPidSetSpeed(-1,-1);//???
//				HAL_Delay(1000);
//				motorPidSetSpeed(-1,1);//????
//				HAL_Delay(50);
//			}
//		}
//	}


////*************?PID????************//
//	if(HC_SR04_Read() > 25)
//	{
//		motorPidSetSpeed(1,1);//???
//		HAL_Delay(100);
//	}
//	if(HC_SR04_Read() < 20)
//	{
//		motorPidSetSpeed(-1,-1);//???
//		HAL_Delay(100);
//	}

////**********PID????***********//
//    g_fHC_SR04_Read=HC_SR04_Read();//?????????
//	if(g_fHC_SR04_Read < 60){  //???60cm ????????
//		g_fFollow_PID_Out = PID_realize(&pidFollow,g_fHC_SR04_Read);//PID???????? ?????????????
//		if(g_fFollow_PID_Out > 6) g_fFollow_PID_Out = 6;//???????
//		if(g_fFollow_PID_Out < -6) g_fFollow_PID_Out = -6;
//		motorPidSetSpeed(g_fFollow_PID_Out,g_fFollow_PID_Out);//????????
//	}
//	else motorPidSetSpeed(0,0);//????60cm ???????
//	HAL_Delay(10);//????????????

//	ANO_DT_Send_F2(Motor1Speed*100, 3.0*100,Motor2Speed*100,3.0*100);//????????F2???4?int16????? ????
//	if(Usart_WaitReasFinish() == 0)//??????
//	{
//		cJsonData  = cJSON_Parse((const char *)Usart1_ReadBuf);
//		if(cJSON_GetObjectItem(cJsonData,"p") !=NULL)
//		{
//			cJsonVlaue = cJSON_GetObjectItem(cJsonData,"p");	
//		    p = cJsonVlaue->valuedouble;
//			pidMotor1Speed.Kp = p;
//		}
//		if(cJSON_GetObjectItem(cJsonData,"i") !=NULL)
//		{
//			cJsonVlaue = cJSON_GetObjectItem(cJsonData,"i");	
//		    i = cJsonVlaue->valuedouble;
//			pidMotor1Speed.Ki = i;
//		}
//		if(cJSON_GetObjectItem(cJsonData,"d") !=NULL)
//		{
//			cJsonVlaue = cJSON_GetObjectItem(cJsonData,"d");	
//		    d = cJsonVlaue->valuedouble;
//			pidMotor1Speed.Kd = d;
//		}
//		if(cJSON_GetObjectItem(cJsonData,"a") !=NULL)
//		{
//		
//			cJsonVlaue = cJSON_GetObjectItem(cJsonData,"a");	
//		    a = cJsonVlaue->valuedouble;
//			pidMotor1Speed.target_val =a;
//		}
//		if(cJSON_GetObjectItem(cJsonData,"b") !=NULL)
//		{
//		
//			cJsonVlaue = cJSON_GetObjectItem(cJsonData,"b");	
//		    b = cJsonVlaue->valuedouble;
//			pidMotor2Speed.target_val =b;
//		}
//		if(cJsonData != NULL){
//		  cJSON_Delete(cJsonData);//???????????cJsonVlaue??? ??????
//		}
//		memset(Usart1_ReadBuf,0,255);//????buf?????????strlen	
//	}
//	printf("P:%.3f  I:%.3f  D:%.3f A:%.3f\r\n",p,i,d,a);
	
	
//	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 10);
//	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 10);
//	HAL_GPIO_WritePin(BIN1_GPIO_Port,BIN1_Pin,GPIO_PIN_SET);
//	HAL_GPIO_WritePin(AIN1_GPIO_Port,AIN1_Pin,GPIO_PIN_SET);
	
	
//	uint8_t c_Data[] = "??????:???VCC\r\n";
//	HAL_UART_Transmit(&huart1,c_Data,sizeof(c_Data),0xFFFF);
//	HAL_Delay(1000);
//	printf("printf:???VCC??\r\n");
//	HAL_GPIO_TogglePin(LED_GPIO_Port,LED_Pin);
//	HAL_Delay(500);
	
	
  }
  /* USER CODE END 3 */


/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */








