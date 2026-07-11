#include "include.h"

/*
 * 函数功能：机械臂主程序入口
 * 说明：初始化 PWM 舵机、系统定时、USART1、ADC、LED 和蜂鸣器，随后进入机械臂主循环任务。
 */
int main(void)
{
	SystemInit(); 			 //系统时钟初始化为72M
	InitDelay(72);	     //延时初始化
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	InitPWM();           //初始化普通 PWM 舵机输出
	InitTimer2();        //用于产生100us的定时中断
	InitUart1();         //USART1 用于与树莓派通信
	InitADC();
	InitLED();
	InitBuzzer();
	LED = LED_ON;
	ArmControl_SetHome(1000);//上电后让 PWM 舵机回到中位，等待树莓派命令
	while(1)
	{
		TaskRun();
	}
}