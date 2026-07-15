#include "mpu6050.h"

/*
 * 本文件只负责 MPU6050 寄存器级操作和原始数据读取。
 * 底层 IIC 时序由 mpuiic.c 提供，对外函数名保持不变，避免影响现有底盘代码。
 */

/**
 * @brief  将两个 8 位寄存器值合成为 signed short
 * @note   MPU6050 传感器数据均为高字节在前、低字节在后。
 * @param  high: 高 8 位
 * @param  low:  低 8 位
 * @return 合成后的 16 位有符号原始值
 */
static short MPU_Combine_Bytes(uint8_t high, uint8_t low)
{
	return (short)(((uint16_t)high << 8) | low);
}

/**
 * @brief  初始化 MPU6050
 * @note   使用软件 IIC 访问寄存器，初始化陀螺仪、加速度计、采样率和滤波器。
 * @param  无
 * @return 0: 初始化成功  1: 设备 ID 错误或通信失败
 */
uint8_t MPU_Init(void)
{
	uint8_t deviceId;

	MPU_IIC_Init();

	(void)MPU_Write_Byte(MPU_PWR_MGMT1_REG, 0x80U);
	HAL_Delay(100);
	(void)MPU_Write_Byte(MPU_PWR_MGMT1_REG, 0x00U);
	(void)MPU_Set_Gyro_Fsr(3);
	(void)MPU_Set_Accel_Fsr(0);
	(void)MPU_Set_Rate(50);
	(void)MPU_Write_Byte(MPU_INT_EN_REG, 0x00U);
	(void)MPU_Write_Byte(MPU_USER_CTRL_REG, 0x00U);
	(void)MPU_Write_Byte(MPU_FIFO_EN_REG, 0x00U);
	(void)MPU_Write_Byte(MPU_INTBP_CFG_REG, 0x80U);

	deviceId = MPU_Read_Byte(MPU_DEVICE_ID_REG);
	if(deviceId != MPU_ADDR)
	{
		return 1;
	}

	(void)MPU_Write_Byte(MPU_PWR_MGMT1_REG, 0x01U);
	(void)MPU_Write_Byte(MPU_PWR_MGMT2_REG, 0x00U);
	(void)MPU_Set_Rate(50);

	return 0;
}

/**
 * @brief  设置陀螺仪满量程范围
 * @param  fsr: 0=+-250dps，1=+-500dps，2=+-1000dps，3=+-2000dps
 * @return 0: 成功  其他: 失败
 */
uint8_t MPU_Set_Gyro_Fsr(uint8_t fsr)
{
	return MPU_Write_Byte(MPU_GYRO_CFG_REG, fsr << 3);
}

/**
 * @brief  设置加速度计满量程范围
 * @param  fsr: 0=+-2g，1=+-4g，2=+-8g，3=+-16g
 * @return 0: 成功  其他: 失败
 */
uint8_t MPU_Set_Accel_Fsr(uint8_t fsr)
{
	return MPU_Write_Byte(MPU_ACCEL_CFG_REG, fsr << 3);
}

/**
 * @brief  设置数字低通滤波器
 * @param  lpf: 低通滤波频率，单位 Hz
 * @return 0: 成功  其他: 失败
 */
uint8_t MPU_Set_LPF(uint16_t lpf)
{
	uint8_t data;

	if(lpf >= 188U)
	{
		data = 1;
	}
	else if(lpf >= 98U)
	{
		data = 2;
	}
	else if(lpf >= 42U)
	{
		data = 3;
	}
	else if(lpf >= 20U)
	{
		data = 4;
	}
	else if(lpf >= 10U)
	{
		data = 5;
	}
	else
	{
		data = 6;
	}

	return MPU_Write_Byte(MPU_CFG_REG, data);
}

/**
 * @brief  设置 MPU6050 采样率
 * @note   假定内部采样基准为 1kHz，同时把低通滤波频率设置为采样率的一半。
 * @param  rate: 目标采样率，范围 4~1000Hz
 * @return 0: 成功  其他: 失败
 */
uint8_t MPU_Set_Rate(uint16_t rate)
{
	uint8_t div;
	uint8_t result;

	if(rate > 1000U)
	{
		rate = 1000U;
	}
	if(rate < 4U)
	{
		rate = 4U;
	}

	div = (uint8_t)(1000U / rate - 1U);
	result = MPU_Write_Byte(MPU_SAMPLE_RATE_REG, div);
	if(result != 0U)
	{
		return result;
	}

	return MPU_Set_LPF(rate / 2U);
}

/**
 * @brief  读取 MPU6050 温度原始值并换算为摄氏度
 * @note   返回值扩大 100 倍，例如 3653 表示 36.53 摄氏度。
 * @param  无
 * @return 温度值 x100
 */
short MPU_Get_Temperature(void)
{
	uint8_t buf[2];
	short raw;
	float temp;

	if(MPU_Read_Len(MPU_ADDR, MPU_TEMP_OUTH_REG, 2, buf) != 0U)
	{
		return 0;
	}

	raw = MPU_Combine_Bytes(buf[0], buf[1]);
	temp = 36.53f + ((float)raw / 340.0f);

	return (short)(temp * 100.0f);
}

/**
 * @brief  读取陀螺仪三轴原始数据
 * @param  gx: X 轴陀螺仪原始值
 * @param  gy: Y 轴陀螺仪原始值
 * @param  gz: Z 轴陀螺仪原始值
 * @return 0: 成功  其他: 失败
 */
uint8_t MPU_Get_Gyroscope(short *gx, short *gy, short *gz)
{
	uint8_t buf[6];
	uint8_t result;

	result = MPU_Read_Len(MPU_ADDR, MPU_GYRO_XOUTH_REG, 6, buf);
	if(result == 0U)
	{
		*gx = MPU_Combine_Bytes(buf[0], buf[1]);
		*gy = MPU_Combine_Bytes(buf[2], buf[3]);
		*gz = MPU_Combine_Bytes(buf[4], buf[5]);
	}

	return result;
}

/**
 * @brief  读取加速度计三轴原始数据
 * @param  ax: X 轴加速度原始值
 * @param  ay: Y 轴加速度原始值
 * @param  az: Z 轴加速度原始值
 * @return 0: 成功  其他: 失败
 */
uint8_t MPU_Get_Accelerometer(short *ax, short *ay, short *az)
{
	uint8_t buf[6];
	uint8_t result;

	result = MPU_Read_Len(MPU_ADDR, MPU_ACCEL_XOUTH_REG, 6, buf);
	if(result == 0U)
	{
		*ax = MPU_Combine_Bytes(buf[0], buf[1]);
		*ay = MPU_Combine_Bytes(buf[2], buf[3]);
		*az = MPU_Combine_Bytes(buf[4], buf[5]);
	}

	return result;
}

/**
 * @brief  连续写 MPU6050 寄存器
 * @param  addr: MPU6050 7 位设备地址
 * @param  reg:  起始寄存器地址
 * @param  len:  写入字节数
 * @param  buf:  待写入数据缓冲区
 * @return 0: 成功  1: 通信失败
 */
uint8_t MPU_Write_Len(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *buf)
{
	uint8_t i;

	MPU_IIC_Start();
	MPU_IIC_Send_Byte((addr << 1) | 0U);
	if(MPU_IIC_Wait_Ack() != 0U)
	{
		MPU_IIC_Stop();
		return 1;
	}

	MPU_IIC_Send_Byte(reg);
	if(MPU_IIC_Wait_Ack() != 0U)
	{
		MPU_IIC_Stop();
		return 1;
	}

	for(i = 0; i < len; i++)
	{
		MPU_IIC_Send_Byte(buf[i]);
		if(MPU_IIC_Wait_Ack() != 0U)
		{
			MPU_IIC_Stop();
			return 1;
		}
	}

	MPU_IIC_Stop();
	return 0;
}

/**
 * @brief  连续读 MPU6050 寄存器
 * @param  addr: MPU6050 7 位设备地址
 * @param  reg:  起始寄存器地址
 * @param  len:  读取字节数
 * @param  buf:  读取数据保存缓冲区
 * @return 0: 成功  1: 通信失败
 */
uint8_t MPU_Read_Len(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *buf)
{
	MPU_IIC_Start();
	MPU_IIC_Send_Byte((addr << 1) | 0U);
	if(MPU_IIC_Wait_Ack() != 0U)
	{
		MPU_IIC_Stop();
		return 1;
	}

	MPU_IIC_Send_Byte(reg);
	if(MPU_IIC_Wait_Ack() != 0U)
	{
		MPU_IIC_Stop();
		return 1;
	}

	MPU_IIC_Start();
	MPU_IIC_Send_Byte((addr << 1) | 1U);
	if(MPU_IIC_Wait_Ack() != 0U)
	{
		MPU_IIC_Stop();
		return 1;
	}

	while(len > 0U)
	{
		if(len == 1U)
		{
			*buf = MPU_IIC_Read_Byte(0);
		}
		else
		{
			*buf = MPU_IIC_Read_Byte(1);
		}
		len--;
		buf++;
	}

	MPU_IIC_Stop();
	return 0;
}

/**
 * @brief  写 MPU6050 单个寄存器
 * @param  reg:  寄存器地址
 * @param  data: 待写入数据
 * @return 0: 成功  1: 通信失败
 */
uint8_t MPU_Write_Byte(uint8_t reg, uint8_t data)
{
	MPU_IIC_Start();
	MPU_IIC_Send_Byte((MPU_ADDR << 1) | 0U);
	if(MPU_IIC_Wait_Ack() != 0U)
	{
		MPU_IIC_Stop();
		return 1;
	}

	MPU_IIC_Send_Byte(reg);
	if(MPU_IIC_Wait_Ack() != 0U)
	{
		MPU_IIC_Stop();
		return 1;
	}

	MPU_IIC_Send_Byte(data);
	if(MPU_IIC_Wait_Ack() != 0U)
	{
		MPU_IIC_Stop();
		return 1;
	}

	MPU_IIC_Stop();
	return 0;
}

/**
 * @brief  读 MPU6050 单个寄存器
 * @param  reg: 寄存器地址
 * @return 读取到的数据；通信失败时返回 0xFF
 */
uint8_t MPU_Read_Byte(uint8_t reg)
{
	uint8_t data;

	MPU_IIC_Start();
	MPU_IIC_Send_Byte((MPU_ADDR << 1) | 0U);
	if(MPU_IIC_Wait_Ack() != 0U)
	{
		MPU_IIC_Stop();
		return 0xFFU;
	}

	MPU_IIC_Send_Byte(reg);
	if(MPU_IIC_Wait_Ack() != 0U)
	{
		MPU_IIC_Stop();
		return 0xFFU;
	}

	MPU_IIC_Start();
	MPU_IIC_Send_Byte((MPU_ADDR << 1) | 1U);
	if(MPU_IIC_Wait_Ack() != 0U)
	{
		MPU_IIC_Stop();
		return 0xFFU;
	}

	data = MPU_IIC_Read_Byte(0);
	MPU_IIC_Stop();

	return data;
}
