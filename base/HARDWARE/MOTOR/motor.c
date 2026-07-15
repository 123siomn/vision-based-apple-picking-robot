#include "motor.h"
#include "tim.h"
#include "pid.h"

#define MAX_SPEED_UP  3
#define MOTOR_PID_OUTPUT_LIMIT  30.0f
#define MOTOR_PID_ERR_SUM_LIMIT 3.0f

extern float Motor1Speed ;
extern float Motor2Speed ;
extern tPid pidMotor1Speed;
extern tPid pidMotor2Speed;

float motorSpeedUpCut = 0.5;//加减速速度变量
static int g_iMotorLastCmd1 = 0;
static int g_iMotorLastCmd2 = 0;

void Motor_Set(int Motor1,int Motor2)
{
	g_iMotorLastCmd1 = Motor1;
	g_iMotorLastCmd2 = Motor2;
	// 逻辑速度约定：正数表示小车向前。
	// 实测 Motor_Set(正,正) 时左右轮均向前，因此电机1、电机2在底层都取反。
	Motor1 = -Motor1;
	Motor2 = -Motor2;
	
	//1.先根据正负设置方向GPIO 高低电平
	if(Motor1 <0) BIN1_SET;
	else  BIN1_RESET;
	
	if(Motor2 <0) AIN1_SET;
	else AIN1_RESET;
	
	//2.然后设置占空比  
	if(Motor1 <0)
	{
		if(Motor1 <-99) Motor1 =-99;
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, (100+Motor1));
	}
	else 
	{
		if(Motor1 >99) Motor1 = 99;
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1,Motor1);
	}

	if(Motor2<0)
	{
		if(Motor2 <-99) Motor2=-99;
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, (100+Motor2));
	}
	else
	{
		if(Motor2 >99) Motor2 =99;
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, Motor2);
	}


}
int Motor_GetLastCmd1(void)
{
	return g_iMotorLastCmd1;
}

int Motor_GetLastCmd2(void)
{
	return g_iMotorLastCmd2;
}

/**
* @brief  清零左右电机 PID 运行状态
* @note   停车时调用，清除积分项和历史误差，避免下次启动继续沿用旧输出。
* @param  无
* @return 无
*/
void motorPidReset(void)
{
	PID_Reset(&pidMotor1Speed);
	PID_Reset(&pidMotor2Speed);
}

/**
* @brief  获取电机 PID 输出限幅
* @note   TIM1 电机闭环中使用，避免异常反馈导致 PWM 输出过大。
* @param  无
* @return 电机 PID 输出正负限幅
*/
float motorPidGetOutputLimit(void)
{
	return MOTOR_PID_OUTPUT_LIMIT;
}

/**
* @brief  获取电机 PID 积分限幅
* @note   TIM1 电机闭环中使用，避免积分项持续累加。
* @param  无
* @return 电机 PID 积分正负限幅
*/
float motorPidGetErrSumLimit(void)
{
	return MOTOR_PID_ERR_SUM_LIMIT;
}

/**
* @brief  设置左右电机 PID 目标速度
* @note   本函数只更新目标值，实际 PID 计算和 Motor_Set() 统一在 TIM1 定时中断中执行。
* @param  Motor1SetSpeed: 电机1目标速度
* @param  Motor2SetSpeed: 电机2目标速度
* @return 无
*/
void motorPidSetSpeed(float Motor1SetSpeed,float Motor2SetSpeed)
{
	pidMotor1Speed.target_val = Motor1SetSpeed;
	pidMotor2Speed.target_val = Motor2SetSpeed;
}
//向前加速函数
void motorPidSpeedUp(void)
{
	
	if(motorSpeedUpCut <= MAX_SPEED_UP) motorSpeedUpCut +=0.5;//如果没有超过最大值就增加0.5
	motorPidSetSpeed(motorSpeedUpCut,motorSpeedUpCut);//设置到电机
}
//向前减速函数
void motorPidSpeedCut(void)
{
	
	if(motorSpeedUpCut >=0.5)motorSpeedUpCut-=0.5;//判断是否速度太小
	motorPidSetSpeed(motorSpeedUpCut,motorSpeedUpCut);//设置到电机
}






