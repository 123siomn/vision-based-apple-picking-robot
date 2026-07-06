#include "include.h"

/*
 * 函数功能：机械臂主程序入口
 * 说明：当前工程固定为树莓派通过 USART1 控制 PWM 舵机，不再初始化 PS2 手柄、蓝牙、总线舵机、按键动作组和 Flash 动作组。
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

