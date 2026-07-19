#include "motor.h"
#include "tim.h"


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


