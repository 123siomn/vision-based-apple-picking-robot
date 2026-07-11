#include "mpuiic.h"
#include "main.h"

/*
 * 本文件实现 MPU6050 使用的软件 IIC。
 * 当前硬件连接沿用 CubeMX 的普通 GPIO 配置：
 * SCL_6050 -> PB8，SDA_6050 -> PB9。
 * 这里不使用 STM32 硬件 I2C 外设，只通过 GPIO 拉高、拉低来模拟 IIC 时序。
 */

/**
 * @brief  软件 IIC 微秒级延时
 * @note   用空指令做短延时，给 SCL/SDA 电平变化留出稳定时间。
 * @param  usdelay: 需要延时的微秒数
 * @return 无
 */
void mpuiic_Delayus(uint32_t usdelay)
{
	__IO uint32_t delay = usdelay * (SystemCoreClock / 8U / 1000U / 1000U);

	while(delay-- != 0U)
	{
		__NOP();
	}
}

/**
 * @brief  MPU6050 软件 IIC 基础延时
 * @note   每次 SCL/SDA 翻转后调用，当前约 2us。
 * @param  无
 * @return 无
 */
void MPU_IIC_Delay(void)
{
	mpuiic_Delayus(2);
}

/**
 * @brief  初始化 MPU6050 软件 IIC 总线
 * @note   GPIO 已由 MX_GPIO_Init() 配置，这里只释放总线到空闲高电平。
 * @param  无
 * @return 无
 */
void MPU_IIC_Init(void)
{
	MPU_SDA_OUT();
	MPU_IIC_SCL_Hige;
	MPU_IIC_SDA_Hige;
	MPU_IIC_Delay();
}

/**
 * @brief  产生 IIC 起始信号
 * @note   起始条件：SCL 为高电平时，SDA 从高电平跳变到低电平。
 * @param  无
 * @return 无
 */
void MPU_IIC_Start(void)
{
	MPU_SDA_OUT();
	MPU_IIC_SDA_Hige;
	MPU_IIC_SCL_Hige;
	MPU_IIC_Delay();
	MPU_IIC_SDA_Low;
	MPU_IIC_Delay();
	MPU_IIC_SCL_Low;
}

/**
 * @brief  产生 IIC 停止信号
 * @note   停止条件：SCL 为高电平时，SDA 从低电平跳变到高电平。
 * @param  无
 * @return 无
 */
void MPU_IIC_Stop(void)
{
	MPU_SDA_OUT();
	MPU_IIC_SCL_Low;
	MPU_IIC_SDA_Low;
	MPU_IIC_Delay();
	MPU_IIC_SCL_Hige;
	MPU_IIC_Delay();
	MPU_IIC_SDA_Hige;
	MPU_IIC_Delay();
}

/**
 * @brief  等待从机 ACK 应答
 * @note   主机释放 SDA 后拉高 SCL，若从机把 SDA 拉低，表示 ACK 成功。
 * @param  无
 * @return 0: 收到 ACK  1: 等待超时或无 ACK
 */
uint8_t MPU_IIC_Wait_Ack(void)
{
	uint8_t timeout = 0;

	MPU_SDA_IN();
	MPU_IIC_SDA_Hige;
	MPU_IIC_Delay();
	MPU_IIC_SCL_Hige;
	MPU_IIC_Delay();

	while(MPU_READ_SDA)
	{
		timeout++;
		if(timeout > 250U)
		{
			MPU_IIC_Stop();
			return 1;
		}
	}

	MPU_IIC_SCL_Low;
	return 0;
}

/**
 * @brief  主机发送 ACK
 * @note   读多个字节时，除最后一个字节外，主机需要发送 ACK 继续读取。
 * @param  无
 * @return 无
 */
void MPU_IIC_Ack(void)
{
	MPU_IIC_SCL_Low;
	MPU_SDA_OUT();
	MPU_IIC_SDA_Low;
	MPU_IIC_Delay();
	MPU_IIC_SCL_Hige;
	MPU_IIC_Delay();
	MPU_IIC_SCL_Low;
}

/**
 * @brief  主机发送 NACK
 * @note   读取最后一个字节后发送 NACK，通知从机本次读取结束。
 * @param  无
 * @return 无
 */
void MPU_IIC_NAck(void)
{
	MPU_IIC_SCL_Low;
	MPU_SDA_OUT();
	MPU_IIC_SDA_Hige;
	MPU_IIC_Delay();
	MPU_IIC_SCL_Hige;
	MPU_IIC_Delay();
	MPU_IIC_SCL_Low;
}

/**
 * @brief  软件 IIC 发送 1 个字节
 * @note   从最高位开始发送，每发送 1 位产生 1 个 SCL 脉冲。
 * @param  txd: 待发送字节
 * @return 无
 */
void MPU_IIC_Send_Byte(uint8_t txd)
{
	uint8_t bit;

	MPU_SDA_OUT();
	MPU_IIC_SCL_Low;

	for(bit = 0; bit < 8U; bit++)
	{
		if((txd & 0x80U) != 0U)
		{
			MPU_IIC_SDA_Hige;
		}
		else
		{
			MPU_IIC_SDA_Low;
		}

		txd <<= 1;
		MPU_IIC_Delay();
		MPU_IIC_SCL_Hige;
		MPU_IIC_Delay();
		MPU_IIC_SCL_Low;
	}
}

/**
 * @brief  软件 IIC 读取 1 个字节
 * @note   从最高位开始读取，读取完成后按 ack 参数发送 ACK 或 NACK。
 * @param  ack: 1 发送 ACK，0 发送 NACK
 * @return 读取到的字节
 */
uint8_t MPU_IIC_Read_Byte(unsigned char ack)
{
	uint8_t bit;
	uint8_t receive = 0;

	MPU_SDA_IN();

	for(bit = 0; bit < 8U; bit++)
	{
		MPU_IIC_SCL_Low;
		MPU_IIC_Delay();
		MPU_IIC_SCL_Hige;
		receive <<= 1;
		if(MPU_READ_SDA)
		{
			receive++;
		}
		MPU_IIC_Delay();
	}

	if(ack == 0U)
	{
		MPU_IIC_NAck();
	}
	else
	{
		MPU_IIC_Ack();
	}

	return receive;
}

/**
 * @brief  向指定 IIC 设备的指定寄存器写 1 个字节
 * @note   保留旧头文件中的函数名，当前主要供兼容旧代码使用。
 * @param  daddr: 7 位设备地址
 * @param  addr:  寄存器地址
 * @param  data:  待写入数据
 * @return 无
 */
void IMPU_IC_Write_One_Byte(uint8_t daddr, uint8_t addr, uint8_t data)
{
	MPU_IIC_Start();
	MPU_IIC_Send_Byte((daddr << 1) | 0U);
	if(MPU_IIC_Wait_Ack() != 0U)
	{
		MPU_IIC_Stop();
		return;
	}
	MPU_IIC_Send_Byte(addr);
	if(MPU_IIC_Wait_Ack() != 0U)
	{
		MPU_IIC_Stop();
		return;
	}
	MPU_IIC_Send_Byte(data);
	(void)MPU_IIC_Wait_Ack();
	MPU_IIC_Stop();
}

/**
 * @brief  从指定 IIC 设备的指定寄存器读取 1 个字节
 * @note   保留旧头文件中的函数名，当前主要供兼容旧代码使用。
 * @param  daddr: 7 位设备地址
 * @param  addr:  寄存器地址
 * @return 读取到的数据；通信失败时返回 0xFF
 */
uint8_t MPU_IIC_Read_One_Byte(uint8_t daddr, uint8_t addr)
{
	uint8_t data;

	MPU_IIC_Start();
	MPU_IIC_Send_Byte((daddr << 1) | 0U);
	if(MPU_IIC_Wait_Ack() != 0U)
	{
		MPU_IIC_Stop();
		return 0xFFU;
	}
	MPU_IIC_Send_Byte(addr);
	if(MPU_IIC_Wait_Ack() != 0U)
	{
		MPU_IIC_Stop();
		return 0xFFU;
	}

	MPU_IIC_Start();
	MPU_IIC_Send_Byte((daddr << 1) | 1U);
	if(MPU_IIC_Wait_Ack() != 0U)
	{
		MPU_IIC_Stop();
		return 0xFFU;
	}
	data = MPU_IIC_Read_Byte(0);
	MPU_IIC_Stop();

	return data;
}